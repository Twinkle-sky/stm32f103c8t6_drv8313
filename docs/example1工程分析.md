# example1 工程详细分析（foc_example1 / 20251211_FOC）

> 分析对象：`example/foc_example1/20251211_FOC`（Keil MDK，STM32F103C8T6）。
> 定位：**定点数（Q10/Q15）实现的三相 FOC**（电流环+速度环），AS5600 磁编码器，
> "FOC 最小系统"的完整参考实现。配套硬件与本项目 DRV8313 板同系列。

---

## 1. 工程组织

```
20251211_FOC/
├── .ioc                      CubeMX 配置
├── Inc/ Src/                 CubeMX 外设层
│   ├── main.c                ★ 主控制逻辑（TIM3 中断回调、零位校准、初始化）
│   ├── tim.c                 TIM2（PWM+触发）、TIM3（1kHz 控制环）
│   ├── adc.c                 ADC1 双通道 + DMA
│   ├── i2c.c / dma.c / gpio.c / usart.c
├── MDK-ARM/                  ★ 用户算法层（可整体移植）
│   ├── foc_math.c/h          Q10 数学库（正弦表/atan2/sqrt）
│   ├── foc.c/h               Clarke/Park/逆Park/SVPWM
│   ├── pid.c/h               Q15 PID（限幅+抗饱和）
│   ├── foc_current.c/h       ADC 回调、过流保护
│   ├── foc_as5600.c/h        AS5600 I2C DMA 链式读取
│   └── foc_config.c/h        电机参数自动识别
```

分层特点：`MDK-ARM/` 六个模块除 `foc.c`（写 TIM2 寄存器）和 `foc_current.c`（引用 adc_buffer）外不依赖 HAL，移植成本低。

## 2. CubeMX 外设配置（从生成代码反推）

| 外设 | 配置 | 说明 |
|---|---|---|
| 时钟 | HSE 8M ×9 | 72MHz，ADC=PCLK2/6=12MHz |
| TIM2 | 中心对齐1，ARR=3600-1 | **PWM≈10kHz**；CH1/3/4 三相输出 |
| ADC 触发 | `ExternalTrigConv=T2_CC2`，CH2 比较=3600-10 | 采样点固定在周期尾部 |
| TIM3 | PSC=72-1，ARR=1000-1 | **1kHz 控制环中断** |
| ADC1 | 扫描 2 通道 IN4/IN5，DMA 循环 | 两相电流，7.5 周期采样 |
| I2C1 | 400kHz，PB6/PB7 | AS5600（0x36），RX-DMA 链式 |
| GPIO | PA6=过流关断、PA7=使能、PB3=耗时探针、PC13=LED | |

> ⚠️ **移植差异**：example1 板 PA6 是关断 GPIO；本项目 DRV8313 板 PA6 是 ADC1_IN6（W 相电流）。过流保护逻辑移植时需改为 ADC 数值判断（PA7 使能一致）。

## 3. 数据流总览

```
后台持续链：I2C DMA 链式读 AS5600 → as5600_angle → data_ready=1
            ADC1(T2_CC2 触发) → DMA 循环 → adc_buffer[2] → 过流检查

TIM3 中断 @1kHz（main.c HAL_TIM_PeriodElapsedCallback）：
  1. shaft_angle = AS5600GetAngle_DMA()                    机械角(0.001rad)
  2. electrical  = (DIR×(shaft−zero)+2π)×7 % 2π            电角度
  3. 转速：差分 ±π 折叠 → 0.9/0.1 一阶低通                 单位 0.01RPM
  4. currents.a/b = adc_buffer − 偏置(1976/1978)
     → Clarke → Park(0.9/0.1 滤波) → i_d / i_q
  5. 闭环：速度环PID → Iq给定 → dq电流环PID(Id目标0) → SVPWM
     开环：FOC_VoltageOutput(vd, vq, θ) 直接输出
  → TIM2 CH1/3/4 比较寄存器
```

主循环 `while(1)` 只做两件事：PC13 心跳、`ADC_data_ready` 标志节流打印调参数据。

## 4. 核心模块分析

### 4.1 foc_math —— Q10 定点数学库

**约定**：角度=弧度×1000（0~6283↔0~2π），函数值=×1024（Q10），乘法 `(a*b)>>10` 用 int64 中间量防溢出。

| 函数 | 实现 | 用途 |
|---|---|---|
| `_sin_q10` | 1572 点表（0~π/2）+四象限折叠 | 全部三角运算 |
| `_cos_q10` | sin(θ+π/2) | 同上 |
| `fast_atan2_int` | \|y/x\| 三段线性近似 | SVPWM 合成矢量方向 |
| `_sqrt_fast` | 16 次逐位试探 | 电压矢量幅值 |
| `_normalizeAngle` | 取模+负补偿 | 角度归一 |

关键常量：`U_MAX_LIMIT_Q10=307`（矢量幅值限 0.3，保 ≥10µs 下桥采样窗口）、`SQRT3_Q10=1773`。
**评估**：零依赖直接拷贝；正弦表占 3.1KB Flash。

### 4.2 foc —— 变换与 SVPWM

**Clarke**（ia+ib+ic=0，两相采样）：

```c
alpha = ia;  beta = FOC_MulQ10(ia + (ib<<1), ONE_OVER_SQRT3_Q10);
```

**Park**：结果带 0.9/0.1 一阶低通（滤采样噪声，代价约 1 周期滞后）。

**`FOC_VoltageOutput(vd, vq, θ)` 五步**：

1. 合成矢量：vd≠0 → `Uout=√(vd²+vq²)`、方向=θ+atan2(vq,vd)；vd==0 → 方向=θ+90°、Uout=vq（常见路径，免 sqrt/atan2）；
2. `Uout` 限幅 ±307；
3. 扇区 `sector=(θ×6)/2π`（0~5），扇区内角 `θr=θ−sector·π/3`；
4. 基本矢量作用时间 `T1=√3·sin(π/3−θr)·Uout`、`T2=√3·sin(θr)·Uout`、`T0=1024−T1−T2`（归一化 1024=满周期），6 个 sector 的导通时间查表分配；
5. 占空比再过 0.1/0.9 低通（`×3.51` 系数把 0~1024 域映射到 0~3600 ARR 域，3.51=3600/1024）后写 TIM2 CH1/3/4 比较寄存器。

**细节观察**：
- T0/2 用整数右移，扇区表存在量化误差（±1 LSB 占空比），10kHz 下影响微小；
- vq 为负时 Uout<0，靠 T1/T2 变负等效反相，能工作但 T0=1024−T1−T2 可能越界，属实现上的取巧；
- 占空比低通（0.1/0.9）平滑了转矩纹波/噪音，代价是输出滞后约 1 个控制周期。

### 4.3 pid —— Q15 定点 PID

结构：增益 Q15（`Kp*error` 直接乘）、积分限幅 `integral_max=0.8×(out_max−out_min)`、输出限幅时回退一次积分（简化版抗饱和）。速度环 5000/20、电流环 10000/500，输出限 ±500。

**注意**：输出饱和后的抗饱和判断条件写法有瑕疵（限幅后比较的是已钳位的值），极端情况积分仍可能轻微累积——闭环整定时输出限幅给了余量，实际影响小，但移植时建议改成标准的 back-calculation。

### 4.4 foc_current —— 电流采样与保护

- `adc_buffer[2]`（DMA 循环，2 通道），完成回调置 `ADC_data_ready=1` 供主循环节流打印；
- **过流保护**：任一通道 <96 或 >4000（≈0.077V/3.2V）→ 拉低 PA6 关断驱动。阈值覆盖了"运放输出饱和/断线"两种故障；
- PB3 置位/复位脉冲 = 采样中断耗时探针（示波器测占空比）。

### 4.5 foc_as5600 —— 编码器链式读取

```c
AS5600_Read_Angle() → HAL_I2C_Mem_Read_DMA(0x36, 0x0C, 2字节)
回调：拼角度 → data_ready=1 → 立即再发起下一次（链式循环，CPU 零轮询）
错误回调：HAL_Delay(1000) 后重启读取（保证总线异常后数据流自愈）
```

**观察**：`AS5600GetAngle_DMA()` 只做 0~4095→0~6283 线性换算，**没有多圈累计**——机械角始终是单圈值。转速靠相邻差分 ±π 折叠（阈值 ±1000），闭环速度环对此够用，但**位置控制必须另加多圈逻辑**（参考 example3 的 cycle_diff 方案）。

### 4.6 foc_config —— 电机参数自动识别（亮点）

`foc_config_main()` 流程：ADC/编码器读数 sanity check → PA7 关断 → `ADC_ZeroCalibration()`（1000 样本平均求零偏）→ `MotorParameterIdentification()`。

识别算法（正反转各一轮）：

1. 用 `FOC_VoltageOutput(0, VQ_MAX, 0/π/2/π/3π/2)` 依次锁定 4 个电角度位置，Vq 斜坡升降防冲击；
2. 每次回到 0 电角度记录编码器读数，**当第 N 次读数与首次差 <0.05rad → 极对数=N**（机械圈闭环自证）；
3. 正反转各记录 20 个 0 电角度点，对每点做 `fmod(θ, 2π/7)` 求余数取平均 → **零位偏置**（余数法比单点锁定精度高，等效多次测量去噪）；
4. 由正转时 0 电角度点的增减判断**编码器方向**；
5. 直接打印三行可粘贴的宏：`AS5600_DIR / AS5600_OFFSET / MOTOR_POLE_PAIR`。

这套"极对数+零位+方向"一次性识别是工程上很实用的设计，main.c 里的 `AS5600_OFFSET=791`、`AS5600_DIR=-1` 就是这样标出来的。

### 4.7 main.c —— 初始化与控制主流程

上电序列：外设 Init → AS5600 首读 → 启动 4 路 PWM → 启动 ADC DMA → 设置 CH2 比较值（采样点）→ **零位校准**（PA7 使能 → `FOC_VoltageOutput(0,100,3π/2)` 锁定 1s → 读编码器存 `zero_shaft_angle_1000`）→ PID 初始化 → 启动 TIM3。

TIM3 中断（1kHz）即 §3 的五步；`pid_flag` 切换开环/闭环；speed_tar=30000（0.01RPM 单位=300RPM，注释"100转"已过时）。

## 5. 单位与量纲速查

| 变量 | 单位 | 换算 |
|---|---|---|
| 角度（*_1000） | 0.001 rad | ÷1000 → rad |
| 电角度 | 0.001 rad，0~6283 | ×p=7 与机械角互转 |
| 转速 shaft_speed_1000 | 0.01 RPM | d(0.001rad)/1ms ×954.9 |
| 电流 currents.a/b/q/d | ADC LSB（去偏置后） | 未换算成安培，PID 在 LSB 域整定 |
| 电压 vd/vq/Uout | Q10（1024=1.0 归一化） | ×VM=实际电压，限幅 0.3×VM≈3.7V |
| 占空比 duty | 0~1024 | dutyPeriod 域 0~3600（×3.51） |

## 6. 代码质量观察（移植前必读）

**值得学习的**：
1. 全定点化在无 FPU 的 F103 上把控制环开销压到最低；
2. AS5600 链式 DMA + 错误自愈、TIM2 同步触发 ADC 采样点，数据链路设计标准；
3. `foc_config.c` 参数自动识别（极对数/零位/方向 + 余数法精化）大幅降低上手门槛；
4. 过流硬件级保护 + 调试探针 GPIO 的工程习惯。

**需要警惕的**：
1. **TIM3 中断里 printf**（`data_ready==0` 分支）：阻塞式 UART 最坏 65ms/字节级，1kHz 中断里绝不能留，移植时删掉或改标志位；
2. 电流单位停留在 ADC LSB，换电机/换运放增益后 PID 参数全部要重整定——建议移植时换算成安培（`I=(u−1.65)/R/G`，参考 example3 conf.h）；
3. 零位校准是**单点 1s 锁定**，负载摩擦会引入误差，正式产品用 foc_config 的余数法；
4. `AS5600GetAngle_DMA` 无多圈，位置控制需补 `cycle_diff` 累计；
5. PID 抗饱和写法有瑕疵（见 4.3）；
6. 阻塞式 `HAL_Delay` 散布在初始化/校准流程，移植到 RTOS 需改造。

## 7. 移植到本项目（DRV8313 板）的差异清单

| 项 | example1 | 本项目板 | 动作 |
|---|---|---|---|
| PWM | TIM2 CH1/3/4（PA0/PA2/PA3） | 相同 | 直接复用 |
| 使能 | PA7 高有效 | 相同（EN1/2/3 并联） | 直接复用 |
| 电流采样 | 2 通道 PA4/PA5，偏置 1976/1978 | **3 通道 PA4/PA5/PA6**，REF=1.65V | 补第三通道，偏置用运行时校准，换算成安培 |
| 过流保护 | PA6 GPIO 关断 | PA6 被 ADC 占用 | 改为 ADC 越界判断 + PA7 关断 |
| 编码器 | AS5600 I2C1 PB6/PB7 | 相同（板上有两个封装位，只焊一个） | 直接复用 |
| 供电 | 12V | USB 5V 升压 ≈12.4V | VM 宏用实测值 |
| 控制频率 | TIM3 1kHz | 待定（当前 SPWM 用 1kHz） | 电流环建议同步采样触发而非软定时 |

## 8. 一句话总结

example1 = **"最小可跑的定点 FOC 教科书"**：Q10/Q15 数学库、Clarke/Park/SVPWM、双环 PID、编码器链、参数自识别一应俱全，数据流清晰适合逐行学习；它的短板（LSB 电流域、单点校准、中断 printf、无多圈）恰好都能在 example3 的工程化架构里找到答案，两者对照阅读收益最大。
