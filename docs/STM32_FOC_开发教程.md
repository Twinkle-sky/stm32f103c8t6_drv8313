# STM32 无刷电机 FOC（磁场定向控制）开发教程

> 本教程基于工作区中两个实战例程编写：
>
> - **example1**（`example/foc_example1/20251211_FOC`）：STM32F103C8T6 + AS5600 磁编码器（I2C）+ 定时器触发 ADC 电流采样，定点数（Q10/Q15）实现的三闭环（电流环 + 速度环）FOC，Keil MDK 工程。
> - **example3**（`example/foc_example3/stm32_foc-main`）：STM32F103C8T6 + MT6701 磁编码器（SSI/SPI）+ TIM1 40kHz PWM + ADC1/ADC2 双重同步注入采样，基于 CMSIS-DSP（`arm_math.h`）浮点实现，支持力矩/速度/位置/串级五种控制模式，Keil MDK 工程（要求 AC6 编译器、O2 优化）。
>
> 两例程均出自同一作者系列教程：https://blog.csdn.net/qq570437459/category_12672491

---

## 目录

1. [项目概述](#1-项目概述)
2. [硬件准备](#2-硬件准备)
3. [FOC 基本原理](#3-foc-基本原理)
4. [工程配置流程（CubeMX）](#4-工程配置流程cubemx)
5. [关键代码模块说明](#5-关键代码模块说明)
6. [调试方法与注意事项](#6-调试方法与注意事项)
7. [example1 与 example3 可复用代码结构分析](#7-example1-与-example3-可复用代码结构分析)

---

## 1. 项目概述

### 1.1 什么是 FOC

FOC（Field Oriented Control，磁场定向控制，又称矢量控制）是永磁同步电机（PMSM）/无刷直流电机（BLDC）最主流的控制方式。其核心思想是：**通过坐标变换，把三相定子电流投影到跟随转子旋转的 d-q 坐标系上，将三相交流电流的控制问题转化为两个直流量的控制问题**——

- **d 轴（直轴）电流 Id**：与转子磁场对齐方向，正常出力时控制为 0（Id=0 控制策略）；
- **q 轴（交轴）电流 Iq**：与转子磁场垂直，直接正比于电磁转矩（T ∝ Iq）。

控制了 Iq 就等于控制了转矩，进而可以套用经典控制理论（PID）搭建电流环、速度环、位置环的串级结构。

### 1.2 两个例程的功能与差异对比

| 项目 | example1 | example3 |
|---|---|---|
| MCU | STM32F103C8T6，HSE 8MHz×9 = 72MHz | 相同 |
| 位置传感器 | AS5600，12bit，I2C1 + DMA 链式循环读取 | MT6701，14bit，SPI1（SSI 模式）+ DMA 循环读取 |
| PWM 驱动 | TIM2，中心对齐模式1，ARR=3600（约 10kHz），4 通道 | TIM1，中心对齐模式3，40kHz，高级定时器，TRGO=更新事件 |
| ADC 电流采样 | ADC1 常规双通道（IN4/IN5）DMA 循环，由 TIM2 CH2（OC4REF→T2_CC2）触发 | ADC1/ADC2 双重同步**注入**采样（IN0/IN1），由 TIM1 更新事件 TRGO 触发，ADC 中断处理 |
| 数值格式 | 定点数：角度×1000 表示、Q10 正弦表、Q15 PID | 浮点 + CMSIS-DSP（`arm_sin_f32`/`arm_park_f32`/`arm_pid_f32`） |
| 控制结构 | 速度环 + dq 电流环（Id 目标 0），或开环 Uq 电压控制 | 力矩（电流）环、速度环、位置环、速度+力矩、位置+速度+力矩，共 5 种模式 |
| 电机参数 | 7 对极，AS5600 方向系数 `-1` | 7 对极，采样电阻 0.02Ω、运放增益 50 倍、最大电流 2A |
| 零位校准 | 输出固定电压矢量（Uq=100, θ=3π/2）锁定 1s 后记录编码器角度 | 输出基础矢量 1（0.5,0,0 占空比）保持 400ms 记录 `rotor_zero_angle` |
| 保护 | ADC 值越界（<96 或 >4000）时拉低 PA6 关断驱动 | 占空比上限 0.9 留出下桥导通窗口保证采样时间 |
| 典型参数 | 速度环 Kp=5000/Ki=20；电流环 Kp=10000/Ki=500（定点） | 位置环 P=3.5/I=0/D=7；速度环 0.02/0.001；电流环 1.2/0.02（浮点） |

**学习路线建议**：先用 example1 理解"FOC 最小系统"的完整数据流（定点实现帮助理解每一步的数学本质），再读 example3 学习工程化的架构（模块分层、模式分发、DSP 库加速、多环串级）。

---

## 2. 硬件准备

### 2.1 通用硬件清单

| 部件 | 要求 | 说明 |
|---|---|---|
| MCU 核心板 | STM32F103C8T6 最小系统 | 72MHz，需 HSE 晶振 |
| 云台电机/无刷电机 | 7 对极，带 3 根相线（如 2804/4008 云台电机） | 低感量云台电机最适合低压小电流入门 |
| 三相驱动板 | 6 路 MOSFET 半桥 + 三相桥，支持电机使能引脚、低边/相线电流采样放大输出 | example1 适配硬件见例程内"硬件配套.txt"；example3 板载采样电阻 0.02Ω + 50 倍运放 |
| 位置传感器 | AS5600（I2C，12bit）或 MT6701（SSI/SPI，14bit） | 必须与电机轴刚性同轴安装，同轴度直接影响换相平顺性 |
| 电流采样 | 两路相电流（下桥电阻采样 + 运放放大后进 ADC） | 只采两相即可，第三相由 ia+ib+ic=0 推出 |
| 电源 | 12V DC（驱动级），5V/3.3V（逻辑） | example3 板子不提供 5V 输出以防损坏电脑 USB，供电方式为 DC12V 或右侧 5V |
| 调试器/烧录器 | ST-LINK V2 / DAPLink | 串口（USART）用于打印调参曲线 |

### 2.2 关键硬件连接（从例程代码提取）

**example1 引脚分配**：

| 引脚 | 功能 |
|---|---|
| TIM2 CH1/CH3/CH4 | 三相 PWM（U/V/W），CH2 保留用作 ADC 触发（OC4REF→TRGO 链） |
| PA4 / PA5 | ADC1_IN4 / IN5，两相电流采样电压 |
| PA7 | GPIO 输出，电机驱动芯片使能 |
| PA6 | GPIO 输出，过流时拉低关断驱动 |
| PB3 | GPIO 输出，采样周期指示（示波器测中断频率用） |
| I2C1（PB6/PB7） | AS5600，器件地址 0x36，角度寄存器 0x0C |
| USART1 | 调试打印 |
| PC13 | LED 心跳灯 |

**example3 引脚分配**：

| 引脚 | 功能 |
|---|---|
| TIM1 CH1/CH2/CH3 及互补输出 | 三相 PWM，40kHz 中心对齐 |
| PA0 / PA1 | ADC1_IN0 / ADC2_IN1，两相电流采样（注入同步采样） |
| SPI1（PA4 片选 + SCK/MISO/MOSI） | MT6701，14bit 角度 + 4bit 磁场强度 + 6bit CRC |
| USART2 | 调试打印 |
| PB15 | LED 心跳灯 |

### 2.3 传感器安装要点

1. **磁铁必须正对芯片中心且与轴同心**，偏心会造成角度谐波，表现为低速抖动。
2. 上电后用串口打印原始角度，手转一圈确认 0→4095（AS5600）/0→16383（MT6701）线性变化、无跳变。
3. 例程中均有"CRC 校验失败丢弃本次数据"（MT6701）与"I2C 错误回调重启读取"（AS5600）的容错设计，务必保留。

---

## 3. FOC 基本原理

### 3.1 坐标变换体系

三相定子电流 i_a、i_b、i_c 互差 120°。FOC 的两级变换：

**① Clarke 变换（三相静止 → 两相静止 α-β）**

利用 ia+ib+ic=0，只需采样两相：

```
i_alpha = i_a
i_beta  = (i_a + 2·i_b) / √3
```

**② Park 变换（两相静止 α-β → 旋转 d-q，θ 为电角度）**

```
i_d =  i_alpha·cosθ + i_beta·sinθ
i_q = -i_alpha·sinθ + i_beta·cosθ
```

**③ 逆 Park 变换（电压指令 d-q → α-β）**

```
v_alpha = v_d·cosθ - v_q·sinθ
v_beta  = v_d·sinθ + v_q·cosθ
```

**④ SVPWM（α-β 电压矢量 → 三相 PWM 占空比）**

用 6 个基本电压矢量 + 零矢量在扇区内线性合成目标电压矢量，比 SPWM 多出约 15% 的母线电压利用率。

### 3.2 电角度与机械角度

```
电角度 θe = (机械角度 θm − 零位偏角 θ0) × 极对数 p   （再对 2π 取模）
```

example3 中用宏直接表达（`motor_runtime_param.h`）：

```c
#define rotor_phy_angle  (encoder_angle - rotor_zero_angle) // 转子机械角度
#define rotor_logic_angle (rotor_phy_angle * POLE_PAIRS)    // 转子电角度
```

> 注：电角度超过 2π 不必显式归一化，`arm_sin_f32` 等三角函数与 SVPWM 的扇区判断对任意实数均成立（example3 做法）；example1 则统一在"角度×1000"定点域取模到 0–6283。

### 3.3 零位校准（电角度对齐）——闭环能否跑通的关键

目标是求出"转子 d 轴与 SVPWM 基础矢量 1（α 轴正方向）重合时编码器的读数"，即 `zero_electric_angle` / `rotor_zero_angle`：

- **example1**：输出 `FOC_VoltageOutput(0, 100, 3π/2)` —— 给定纯 q 轴电压，电压矢量被放在 3π/2 方向，转子被拖到该方向锁定 1 秒，然后读编码器作为零位：

  ```c
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); // 使能驱动
  FOC_VoltageOutput(0, 100, _3PI_2_1000);
  HAL_Delay(1000);
  zero_shaft_angle_1000 = AS5600GetAngle_DMA(as5600_angle);
  FOC_VoltageOutput(0, 0, 0); // 松开
  ```

- **example3**：直接输出占空比 (0.5, 0, 0)，即基础矢量 1（对应转子 0°位置），保持 400ms 记录：

  ```c
  set_pwm_duty(0.5, 0, 0);        // d 轴强拖，对应转子零度位置
  HAL_Delay(400);
  rotor_zero_angle = encoder_angle;
  set_pwm_duty(0, 0, 0);          // 松开电机
  ```

**要点**：
1. 电压矢量方向与转子 d 轴的夹角决定转矩 T ∝ sin(矢量方向 − θd)。若零位偏了 90°，转矩恒为零、电机"卡死不转"；若方向反了（g_sensor_dir），电机只会抖动或反向飞车。这是历史上闭环跑不通的最常见根因。
2. 锁定期间负载不能卡死电机；校准角度应重新上电复测 2~3 次，波动应在几度以内。

### 3.4 控制环结构

```
位置环 PID ──输出──► 速度环 PID ──输出──► 电流环(Id=0, Iq) PID ──输出──► 逆Park ──► SVPWM ──► PWM
    ▲                     ▲                        ▲
 motor_logic_angle     motor_speed            i_d / i_q（Park 后）
```

- **电流环**：带宽最高，放在 PWM/ADC 同步中断里执行（example3 在 ADC 注入转换完成中断；example1 在 TIM3 1kHz 中断里结合最近一次采样值）；
- **速度环**：可用较低频率（example3 在 TIM3 @930Hz 中断里由角度差分计算转速并低通滤波；example1 在 TIM3 中断里对 0.01RPM 单位转速做 0.9/0.1 一阶滤波）；
- **位置环**：多圈逻辑角度由编码器差分累加获得（见 §5.4）。

---

## 4. 工程配置流程（CubeMX）

两个例程都是 CubeMX 生成工程（.ioc 在例程根目录），以下是复现配置的完整步骤。

### 4.1 时钟

- HSE 8MHz → PLL ×9 → **SYSCLK = 72MHz**，APB1 = 36MHz（定时器时钟×2 = 72MHz），APB2 = 72MHz；
- ADC 时钟 = PCLK2/6 = 12MHz（不超过 14MHz 上限）。

### 4.2 PWM 定时器（核心配置）

**example1（TIM2 通用定时器）**：
- 时钟源内部，`Prescaler=0`，`CounterMode=Center aligned mode 1`，`Period=3600-1` → PWM 频率 72MHz/(2×3600) = 10kHz；
- CH1/CH3/CH4 输出三相 PWM；CH2 不输出，仅作为 ADC 触发源；
- `TRGO = OC4REF`（主从模式使能），配合 `ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC2`，让 ADC 在 PWM 周期中的固定相位采样；
- 代码中还会动态设置：`__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_Period-10);` 精确控制采样点位置。

**example3（TIM1 高级定时器，推荐做法）**：
- `Prescaler=0`，`CounterMode=Center aligned mode 3`，`Period = 72M/40k = 1800` → **40kHz**；
- `TRGO = Update`（更新事件触发 ADC）——中心对齐模式 3 下，上溢与下溢各触发一次，采样点落在计数器峰值/谷值（对应下桥臂导通中点），是标准的"PWM 同步电流采样"方案；
- `RepetitionCounter = 4`：每 5 个计数周期产生一次更新事件，将 ADC 中断频率降为 40k/5 = **8kHz** 的控制频率；
- 三通道均 PWM1 模式，死区 0（云台电机小电流可不设，大功率需设死区）。

### 4.3 ADC 电流采样

**example3（双重同步注入采样，标准做法）**：
- ADC1/ADC2 配置为 `ADC_DUALMODE_INJECSIMULT`（双重同步注入模式），两路电流**同一瞬间**采样（相差几微秒的轮流采样会造成电流矢量畸变）；
- 注入组通道：ADC1→IN0（i_u），ADC2→IN1（i_v），7.5 周期采样时间；
- 触发源 `ADC_EXTERNALTRIGINJECCONV_T1_TRGO`；
- 使能 ADC1_2 中断（抢占优先级 1），在 `HAL_ADCEx_InjectedConvCpltCallback` 中处理；
- 上电先执行 `HAL_ADCEx_Calibration_Start()` 校准两个 ADC。

**example1（常规组 + DMA）**：
- ADC1 扫描模式 2 通道（IN4/IN5），外部触发 `T2_CC2`，DMA 循环传输到 `adc_buffer[2]`；
- 在 DMA 完成回调 `HAL_ADC_ConvCpltCallback` 里做越界保护判断。

### 4.4 位置传感器接口

- **AS5600（example1）**：I2C1 Fast Mode + DMA，采用"读完成回调里立刻再读"的链式循环读取，无需 CPU 轮询：
  ```c
  HAL_I2C_Mem_Read_DMA(&hi2c1, (0x36 << 1)|1, 0x0C, I2C_MEMADD_SIZE_8BIT, as5600_rx_data, 2);
  ```
  错误回调中重启读取，防止总线异常后数据链断掉。
- **MT6701（example3）**：SPI1 全双工 3 字节 DMA，片选 PA4 手动控制；回调中校验 CRC6 后重启下一次传输；优先级设最高（抢占级 0），保证角度数据流永不断流。

### 4.5 控制中断与 NVIC 优先级建议

| 中断 | 频率 | 建议抢占优先级 |
|---|---|---|
| 编码器 DMA/读取链 | 连续 | 0（最高，数据不能丢） |
| ADC 注入转换完成（电流环） | 8kHz | 1 |
| TIM3 速度环/心跳 | ~1kHz | 2 |
| 串口/调试 | 低 | 3 |

### 4.6 其他外设

- USART1/USART2 异步，115200，用于 `printf` 重定向（Keil 需实现 `fputc`）与 VOFA+ 等上位机画曲线；
- DMA：ADC、I2C/SPI 均为循环模式。

---

## 5. 关键代码模块说明

以下代码摘自两个例程，可直接复用（注明了出处）。

### 5.1 Clarke / Park / 逆 Park 变换

**example1 定点版**（`MDK-ARM/foc.c`，Q10 格式，`FOC_MulQ10` 做 (a×b)>>10）：

```c
void FOC_ClarkTransform(Currents* currents)
{
    currents->c = -currents->a - currents->b;        // ia+ib+ic=0
    currents->alpha = currents->a;
    volatile int32_t numerator = currents->a + (currents->b << 1); // a+2b
    currents->beta = FOC_MulQ10(numerator, ONE_OVER_SQRT3_Q10);    // β=(a+2b)/√3
}

void FOC_ParkTransform(Currents* currents, uint16_t angle)  // angle 0~6283
{
    volatile int32_t sin_val = _sin_q10(angle);
    volatile int32_t cos_val = _cos_q10(angle);
    volatile int32_t d1 = FOC_MulQ10(currents->alpha, cos_val);
    volatile int32_t d2 = FOC_MulQ10(currents->beta,  sin_val);
    currents->d = 0.9*currents->d + 0.1*(d1 + d2);   // 一阶低通滤波
    volatile int32_t q1 = FOC_MulQ10(-currents->alpha, sin_val);
    volatile int32_t q2 = FOC_MulQ10( currents->beta,  cos_val);
    currents->q = 0.9*currents->q + 0.1*(q1 + q2);
}
```

**example3 浮点版**（`Core/Src/adc.c`，直接用 CMSIS-DSP）：

```c
float i_alpha, i_beta;
arm_clarke_f32(motor_i_u, motor_i_v, &i_alpha, &i_beta);
float sin_value = arm_sin_f32(rotor_logic_angle);
float cos_value = arm_cos_f32(rotor_logic_angle);
arm_park_f32(i_alpha, i_beta, &_motor_i_d, &_motor_i_q, sin_value, cos_value);
motor_i_d = low_pass_filter(_motor_i_d, motor_i_d, 0.1f);
motor_i_q = low_pass_filter(_motor_i_q, motor_i_q, 0.1f);
```

> F103 无 FPU，浮点 `arm_park_f32` 纯软件模拟约需几微秒；8kHz 控制频率下 72MHz 主频仍可承受。若要更高的控制频率或更低的 CPU 占用，参考 example1 的定点方案。

### 5.2 SVPWM 生成

**example1（角度扇区法）**（`MDK-ARM/foc.c`）——先合成电压矢量幅值与角度，再按扇区计算基本矢量作用时间 T1/T2/零矢量 T0：

```c
PWM_Duty FOC_VoltageOutput(int32_t vd, int32_t vq, uint16_t angle_el)
{
    int32_t Uout; uint8_t sector;
    if (vd != 0) {                       // 合成矢量 + atan2 求方向
        Uout = _sqrt_fast(vd*vd + vq*vq);
        angle_el = _normalizeAngle(angle_el + fast_atan2_int(vq, vd));
    } else {                             // 纯 q 轴：直接加 90°
        Uout = vq;
        angle_el = _normalizeAngle(angle_el + _PI_2_1000);
    }
    if (Uout >  U_MAX_LIMIT_Q10) Uout =  U_MAX_LIMIT_Q10;  // 限幅（留采样时间）
    if (Uout < -U_MAX_LIMIT_Q10) Uout = -U_MAX_LIMIT_Q10;

    sector = (angle_el * 6) / _2PI_1000;                 // 扇区 0~5
    uint16_t theta = angle_el - sector * _PI_3_1000;     // 扇区内角度
    // T1 = √3·sin(π/3−θ)·Uout，T2 = √3·sin(θ)·Uout，T0 = 1−T1−T2（归一化到1024）
    volatile int32_t T1 = ((SQRT3_Q10 * _sin_q10(_PI_3_1000 - theta)) >> 10) * Uout >> 10;
    volatile int32_t T2 = ((SQRT3_Q10 * _sin_q10(theta))         ) >> 10 * Uout >> 10;
    volatile int32_t T0 = 1024 - T1 - T2;

    switch (sector) {                    // 各扇区三相导通时间表
    case 0: duty.a = T1+T2+(T0>>1); duty.b = T2+(T0>>1); duty.c = (T0>>1); break;
    case 1: duty.a = T1+(T0>>1);    duty.b = T1+T2+(T0>>1); duty.c = (T0>>1); break;
    /* ... case 2~5 同理 ... */
    }
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, dutyPeriod.a);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, dutyPeriod.b);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, dutyPeriod.c);
    return duty;
}
```

**example3（矢量差值法，更简洁）**（`Drivers/motor/foc.c`）——用 A/B/C 三个布尔量直接判断扇区：

```c
static void svpwm(float phi, float d, float q, float *d_u, float *d_v, float *d_w)
{
    d = min(max(d, -1), 1);  q = min(max(q, -1), 1);
    const int v[6][3] = {{1,0,0},{1,1,0},{0,1,0},{0,1,1},{0,0,1},{1,0,1}}; // 6 基本矢量开关状态
    const int K_to_sector[] = {4, 6, 5, 5, 3, 1, 2, 2};

    float sin_phi = arm_sin_f32(phi), cos_phi = arm_cos_f32(phi);
    float alpha, beta;
    arm_inv_park_f32(d, q, &alpha, &beta, sin_phi, cos_phi);  // 逆 Park

    bool A = beta > 0;                          // 扇区判断三要素
    bool B = fabs(beta) > SQRT3 * fabs(alpha);
    bool C = alpha > 0;
    int sector = K_to_sector[4*A + 2*B + C];

    // 扇区内两基本矢量的作用时间（归一化）
    float t_m = arm_sin_f32(sector*rad60)*alpha - arm_cos_f32(sector*rad60)*beta;
    float t_n = beta*arm_cos_f32(sector*rad60 - rad60) - alpha*arm_sin_f32(sector*rad60 - rad60);
    float t_0 = 1 - t_m - t_n;

    *d_u = t_m*v[sector-1][0] + t_n*v[sector%6][0] + t_0/2;
    *d_v = t_m*v[sector-1][1] + t_n*v[sector%6][1] + t_0/2;
    *d_w = t_m*v[sector-1][2] + t_n*v[sector%6][2] + t_0/2;
}

void foc_forward(float d, float q, float rotor_rad)
{
    float d_u, d_v, d_w;
    svpwm(rotor_rad, d, q, &d_u, &d_v, &d_w);
    set_pwm_duty(d_u, d_v, d_w);   // 弱符号函数，由 main.c 重写
}
```

**写占空比（`main.c`）** —— 注意 0.9 限幅保证每周期下桥导通 ≥10%，给电流采样留出时间；临界区写入防止与中断竞争：

```c
void set_pwm_duty(float d_u, float d_v, float d_w)
{
    d_u = min(d_u, 0.9);  d_v = min(d_v, 0.9);  d_w = min(d_w, 0.9);
    __disable_irq();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, d_u * htim1.Instance->ARR);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, d_v * htim1.Instance->ARR);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, d_w * htim1.Instance->ARR);
    __enable_irq();
}
```

### 5.3 PID 控制

**example1 定点 PID**（`MDK-ARM/pid.c`，Q15 增益 + 输出限幅 + 抗积分饱和）：

```c
void PID_Init(PID_Controller* pid, int32_t kp, int32_t ki, int32_t kd,
              int32_t out_max, int32_t out_min)
{
    pid->Kp = kp;  pid->Ki = ki;  pid->Kd = kd;
    pid->integral = 0;  pid->prev_error = 0;
    pid->out_max = out_max << Q;  pid->out_min = out_min << Q;
    pid->integral_max = 0.8f * (pid->out_max - pid->out_min);  // 积分限幅
}

int32_t PID_Calculate(PID_Controller* pid, int32_t setpoint, int32_t feedback)
{
    int32_t error = setpoint - feedback;
    int32_t p_term = pid->Kp * error;
    pid->integral += pid->Ki * error;                    // 积分累加
    if (pid->integral >  pid->integral_max) pid->integral =  pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    int32_t d_term = pid->Kd * (error - pid->prev_error);
    pid->prev_error = error;
    int32_t output = p_term + pid->integral + d_term;
    if (output > pid->out_max) {                         // 输出饱和时回退积分（抗饱和）
        output = pid->out_max;
        pid->integral -= pid->Ki * error;
    } else if (output < pid->out_min) {
        output = pid->out_min;
        pid->integral -= pid->Ki * error;
    }
    pid->out = output >> Q;
    return pid->out;
}
```

**example3 浮点 PID**（`Drivers/motor/foc.c`，直接复用 CMSIS-DSP 的 `arm_pid_f32`）：

```c
static float speed_loop(float speed_rad)
{
    float diff = speed_rad - motor_speed;
    float out = arm_pid_f32(&pid_speed, diff);
    pid_speed.state[2] = fmaxf(fminf(pid_speed.state[2], 1.0f), -1.0f); // 积分溢出截断
    return out;
}

void set_motor_pid(   // 上电一次性配置四组 PID
    float position_p, float position_i, float position_d,
    float speed_p,    float speed_i,    float speed_d,
    float torque_d_p, float torque_d_i, float torque_d_d,
    float torque_q_p, float torque_q_i, float torque_q_d)
{ /* 给四个 arm_pid_instance_f32 赋 Kp/Ki/Kd 后调用 arm_pid_init_f32() */ }
```

实测定参参考：电流环 Kp≈1.2、Ki≈0.02（归一化电流域）；速度环 Kp≈0.02、Ki≈0.001；位置环 Kp≈3.5、D≈7（微分起阻尼作用）。example1 定点域：速度环 5000/20，电流环 10000/500。

### 5.4 编码器读取、多圈角度与测速

**MT6701 SPI 链式读取 + CRC 校验（example3，`Core/Src/spi.c`）**：

```c
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);          // 片选拉高锁存
        int angle_raw = (mt6701_rx_data[1] >> 2) | (mt6701_rx_data[0] << 6); // 14bit
        uint8_t crc_raw = mt6701_rx_data[2] & ((1<<6)-1);            // CRC6 校验
        if (calculate_crc(...) != crc_raw) { /* 丢弃坏数据，重读 */ }
        encoder_angle = 2*PI*angle_raw / (1<<14);                    // 0~2π

        static float encoder_angle_last; static int once = 1;
        if (once) { once = 0; encoder_angle_last = encoder_angle; }  // 首次赋初值
        float diff = cycle_diff(encoder_angle - encoder_angle_last, 2*PI);
        encoder_angle_last = encoder_angle;
        motor_logic_angle = cycle_diff(motor_logic_angle + diff, position_cycle); // 多圈累加
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);        // 启动下一次
        HAL_SPI_TransmitReceive_DMA(&hspi1, mt6701_rx_data, mt6701_rx_data, 3);
    }
}
```

其中多圈折叠函数（增量折叠到 (-cycle/2, cycle/2]，用于跨 0 点累计）：

```c
float cycle_diff(float diff, float cycle)
{
    if (diff >  (cycle/2)) diff -= cycle;
    else if (diff < -(cycle/2)) diff += cycle;
    return diff;
}
```

测速（example3，`tim.c` TIM3 @930Hz 中断）：

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        float diff_angle = cycle_diff(encoder_angle - encoder_angle_last, 2*PI);
        encoder_angle_last = encoder_angle;
        float _motor_speed = diff_angle * motor_speed_calc_freq;   // rad/s
        motor_speed = low_pass_filter(_motor_speed, motor_speed, 0.07f); // 低通降噪
    }
}
```

AS5600 的 I2C 链式读取见 §4.4；example1 中转速同样由相邻角度差分 + 归一化（±π 折叠）+ 一阶低通得到，输出单位 0.01RPM。

### 5.5 主控制流程（示例：example1 的 TIM3 中断）

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3 && as5600_data_ready) {
        as5600_data_ready = 0;
        // 1. 机械角度（定点×1000）与电角度
        shaft_angle_1000 = AS5600GetAngle_DMA(as5600_angle);
        electrical_angle_1000 = ((AS5600_DIR*(shaft_angle_1000 - zero_shaft_angle_1000)
                                  + _2PI_1000) * MOTOR_POLE_PAIR) % _2PI_1000;
        // 2. 测速（差分折叠 + 低通）
        /* ...见 §5.4... */
        // 3. 电流采样去偏置 + Clarke/Park
        currents.a = adc_buffer[0] - 1976;      // 偏置标定值
        currents.b = adc_buffer[1] - 1978;
        FOC_CurrentProcessing(&currents, electrical_angle_1000);
        // 4. 三环串级 or 开环电压
        if (pid_flag) {
            PID_Calculate(&pid_speed, speed_tar, shaft_speed_1000);
            current_tar_q = pid_speed.out;
            PID_Calculate(&pid_d, 0, currents.d);              // Id 目标 0
            PID_Calculate(&pid_q, current_tar_q, currents.q);
            FOC_VoltageOutput(pid_d.out, pid_q.out, electrical_angle_1000);
        } else {
            FOC_VoltageOutput(voltage_d, voltage_q, electrical_angle_1000);
        }
    }
}
```

example3 的对应流程在 `HAL_ADCEx_InjectedConvCpltCallback`（8kHz）：采样换算 → Clarke/Park/滤波 → 按 `motor_control_context.type` 分发到 `lib_torque_control` / `lib_speed_control` / `lib_position_control` / `lib_speed_torque_control` / `lib_position_speed_torque_control`，各函数内部完成对应环 PID 后调用 `foc_forward()` 输出 SVPWM。切换控制模式只需在主循环改两个成员变量：

```c
motor_control_context.torque_norm_q = 0.4;      // 力矩模式 40% 强度
motor_control_context.type = control_type_torque;
// 或
motor_control_context.speed = 30;               // 速度模式 30 rad/s
motor_control_context.type = control_type_speed;
```

### 5.6 电流采样的标定（易被忽略）

电流 = (ADC 电压 − 中点偏置) / R_shunt / 运放增益：

```c
float u_1 = 3.3f * (raw/4095.0f - 0.5f);   // 采样电阻接在运放输入中点，输出 1.65V 对应 0A
float i_1 = u_1 / 0.02 / 50;               // R_SHUNT=0.02Ω, OP_GAIN=50
```

example1 用固定偏置（1976/1978）直接相减，工程上更稳妥的做法是：上电且 PWM 全 50% 时多次采样求平均作为零电流偏置。

---

## 6. 调试方法与注意事项

### 6.1 分阶段点亮流程（强烈建议按顺序）

1. **只测 PWM**：不开中断，示波器测三桥输出，确认频率、中心对齐、使能引脚有效；占空比写 (0.5,0,0) 应能用万用表量出 U 相对 V/W 的电压。
2. **只测编码器**：打印角度，手动转轴确认线性、方向；记录 `AS5600_DIR`（+1/-1）。
3. **开环电压控制**（`pid_flag=0`）：给定 Uq，手转电机感受"齿感"（均匀的阻力凸极），说明零位与方向正确；再让 Uq 缓慢扫描角度，电机应能缓慢旋转（开环拖动）。
4. **零位校准**：按 §3.3 流程执行，多次上电复测偏差 < 几度。
5. **电流环单独调试**：固定 θe（开环锁定），观察 i_d/i_q 波形是否收敛到给定值，调 PID。
6. **速度环**：小目标速度起步（example1 的 speed_tar 单位是 0.01RPM），逐步加大，用串口/VOFA+ 画阶跃响应曲线。
7. **位置环**：最后加入，注意多圈周期 `position_cycle` 的设置与限位。

### 6.2 常见问题排查表

| 现象 | 可能原因 |
|---|---|
| 电机完全不转、无齿感 | 驱动使能未拉高；PWM 未启动；Uq 太小 |
| 有齿感但电机抖动不转 | 零位偏差接近 90°（转矩恒为零）；极对数错误 |
| 电机猛转一下就堵死/飞车 | 编码器方向 `AS5600_DIR` 反了；电角度符号错 |
| 转速上不去、发热 | U_MAX 限幅太小；电流采样偏置错误导致 Id 不为 0 |
| 低速抖动、啸叫 | 测速低通系数太大（滤波太弱）；PWM 频率太低 |
| 偶发失控跳变 | 编码器读数 CRC/I2C 错误未丢弃（参考 §5.4 容错代码） |
| AS5600 数据流中断 | I2C 错误回调必须重启读取，否则数据链永久断开 |

### 6.3 关键注意事项

1. **编译优化**：example1 明确要求至少开 **O2**，否则定点运算的中断耗时可能超出 PWM 周期；example3 要求 Keil **AC6** 编译器（代码里有 `#error` 提示）。
2. **中断耗时控制**：电流环全部计算（采样→变换→PID→SVPWM）必须在一个控制周期内完成。可在中断入口拉高 GPIO、出口拉低（example1 的 PB3 就是干这个的），示波器测占空比。
3. **PWM 占空比限幅**：永远不要输出 100% 占空比，低边采样方案必须留出下桥导通窗口（example3 的 0.9 限幅）。
4. **变量共享与临界区**：中断与主循环共享的变量加 `volatile`；写 TIM 比较寄存器时用 `__disable_irq()` 临界区（见 §5.2）。
5. **printf 不进高频中断**：串口打印极慢，放主循环节流输出（example1 用 `ADC_data_ready` 标志 + 主循环打印）。
6. **上电安全**：调试时把电机轴上不带负载/夹爪，电源加限流；校准锁定阶段电机可能突然跳动。
7. **过流保护**：example1 在 DMA 回调里检查 ADC 原始值越界并立即关断使能（PA6），建议保留该保护。

### 6.4 调参顺序（每个环）

1. Ki=0，逐步加大 Kp 直到响应出现轻微振荡，回退 30~50%；
2. 加 Ki 消除稳态误差，过大引起超时振荡；
3. 位置环用 Kd 提供阻尼（example3 位置环 D=7 就是典型用法）；
4. 每个环用"阶跃目标 + 串口打波形（VOFA+ JustFloat）"观察超调与调节时间。

---

## 7. example1 与 example3 可复用代码结构分析

### 7.1 example1 的模块划分（定点方案）

```
20251211_FOC/
├── Core(CubeMX): main.c / adc.c / tim.c / i2c.c / dma.c / usart.c / gpio.c
└── MDK-ARM/                        ← 用户算法层（可直接搬走复用）
    ├── foc_math.c/h    正弦表 _sin_q10/_cos_q10、fast_atan2、_sqrt_fast、
    │                   角度归一化 _normalizeAngle、Q10 乘法 FOC_MulQ10
    ├── foc.c/h         Clarke、Park、逆Park、电流处理、SVPWM（FOC_VoltageOutput）
    ├── pid.c/h         Q15 定点 PID（限幅 + 抗饱和），通用，可脱离 FOC 使用
    ├── foc_current.c/h ADC 缓冲区、DMA 回调、过流保护
    ├── foc_as5600.c/h  I2C DMA 链式读角度、数据就绪标志、错误重启
    └── foc_config.c/h  （预留）电机参数识别
```

**特点与复用建议**：
- 角度统一用"×1000 定点整数"表示（0~6283 ↔ 0~2π），正弦用 1572 点查表 + 象限折叠，适合无 FPU、需要极致实时性的场合；
- `foc_math` 与 `pid` 模块零依赖（不包含 HAL），**可以直接拷贝到任何 F103 工程使用**；
- `FOC_VoltageOutput` 与 TIM2 强耦合（直接写寄存器），移植时把 `__HAL_TIM_SET_COMPARE` 换成你的定时器即可；
- 主控制逻辑集中在 `HAL_TIM_PeriodElapsedCallback`，数据流一目了然，适合作为教学参考。

### 7.2 example3 的模块划分（浮点 + DSP 方案）

```
stm32_foc-main/
├── Core/Src:  main.c（模式设定、校准、set_pwm_duty 重写）
│              adc.c（电流采样 + 变换 + 模式分发 = 电流环执行点）
│              spi.c（MT6701 链式读取 + CRC + 多圈累计）
│              tim.c（PWM 配置 + 测速中断）
├── Drivers/motor/
│    ├── foc.c/h                五种控制模式 lib_xxx_control + svpwm + PID 封装
│    ├── conf.h                 电机/电路/软件参数集中定义（极对数、采样电阻、
│    │                          PWM 频率、多圈周期 position_cycle）
│    ├── motor_runtime_param.h  运行时角度/速度变量 + 电角度宏定义
│    └── motor_runtime_param.c  变量定义
├── algorithm/filter.c          一阶低通 + 简化卡尔曼
└── Middlewares: CMSIS-DSP (arm_math)
```

**特点与复用建议**：
- **架构上做了干净分层**：算法层（`Drivers/motor/foc.c`）通过 `set_pwm_duty()` 弱符号回调与硬件层解耦——移植到别的 MCU 只需重写这一个函数；
- `conf.h` 集中管理全部可调参数，一个文件适配不同电机；
- 五种控制模式形成"串级可拆装"结构：`lib_position_speed_torque_control` 就是位置环→限速→速度环→限矩→电流环的完整串联，各环独立可测，这是**最值得借鉴的架构**；
- `cycle_diff()` 折叠函数 + 多圈逻辑角 `motor_logic_angle` 是位置控制的标准实现，可直接复用；
- 依赖 CMSIS-DSP：`arm_clarke_f32 / arm_park_f32 / arm_inv_park_f32 / arm_pid_f32 / arm_sin_f32`，代码量少、精度高，但需注意 F103 无 FPU 的运算耗时。

### 7.3 面向本项目（stm32f103c8t6_drv8313，DRV8313 方案）的落地建议

结合两个例程的优点，推荐的复用组合：

1. **硬件层**：DRV8313 为集成三相驱动，参考 example3 的 TIM1 中心对齐 + TRGO 更新事件触发 + ADC 双注入同步采样方案（标准做法），使能/复位引脚参考 example1 的 PA7/PA6 关断保护逻辑；
2. **传感器**：AS5600 走 example1 的 I2C DMA 链式读取 + 错误重启；角度→电角度换算用 example3 的宏方案；
3. **算法层**：使用 example3 的分层架构（`foc.c` + `conf.h` + 弱符号 `set_pwm_duty`），控制模式先实现 `lib_speed_control`（本项目电压模式速度环已验证：矢量方向 = θd + zero_electric_angle + 90°，转矩 ∝ sin(矢量方向 − θd)）；
4. **零位校准**：采用 example1 的两阶段思路——开环拖动自动判向 + 电角度 0 处静态锁定读零位，勿漏掉 90° 补偿；
5. **数值格式**：8kHz 以下控制频率、F103 无 FPU 也能接受 example3 的浮点方案；若要更高频率（≥16kHz）或更低 CPU 占用，切换 example1 的定点实现。

---

## 附录 A：example3 的五种控制模式速查

| 模式 | 枚举 | 关键参数 | 内部链路 |
|---|---|---|---|
| 力矩 | `control_type_torque` | `torque_norm_d/q`（归一化 0~1） | dq 电流环 → SVPWM |
| 速度 | `control_type_speed` | `speed`（rad/s） | 速度环 → Uq → SVPWM |
| 位置 | `control_type_position` | `position`（rad，多圈 ±3π） | 位置环 → Uq → SVPWM |
| 速度+力矩 | `control_type_speed_torque` | `speed`, `max_torque_norm` | 速度环→限幅→电流环 |
| 位置+速度+力矩 | `control_type_position_speed_torque` | `position`, `max_speed`, `max_torque_norm` | 位置→速度→电流三级 |

## 附录 B：关键参数标定值汇总

| 参数 | example1 | example3 |
|---|---|---|
| 极对数 | 7 | 7 |
| PWM 频率 | 10kHz（TIM2） | 40kHz（TIM1），控制中断 8kHz |
| 测速环频率 | TIM3（约 1kHz） | TIM3 @930Hz |
| 速度环 PID | Kp=5000, Ki=20（定点） | Kp=0.02, Ki=0.001 |
| 电流环 PID | Kp=10000, Ki=500（定点） | Kp=1.2, Ki=0.02 |
| 位置环 PID | — | Kp=3.5, Kd=7 |
| 电流滤波 α | 0.1（Park 后 d/q） | 0.1（d/q）、0.07（速度） |
| 采样电阻/增益 | 偏置法（1976/1978 LSB） | 0.02Ω / 50 倍 |
| 输出限幅 | U_MAX_LIMIT（留采样时间） | 占空比 ≤0.9，PID 积分 ±1.0 |
| 多圈周期 | — | 6π（可改） |
