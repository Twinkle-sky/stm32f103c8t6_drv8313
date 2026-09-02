# 开环实验：三相 SPWM 正弦调制（方案四）

> 原理：不经过 Clarke/Park/SVPWM，直接输出三相互差 120° 的正弦占空比，合成一个以 θ_cmd 匀速旋转的电压矢量，同步拖动转子。目的是用最少的代码验证"三相驱动 + 相序 + 同步拖动"整条功率链路。
>
> 前提：已完成《开环点亮任务清单.md》阶段一~二（硬件检查 + CubeMX 的 TIM2/PA7/USART1 配置）。

---

## 1. 数学模型

三相对称正弦电压（互差 120° 电角度）：

```
duty_u = 0.5 + k·sin(θ_cmd)
duty_v = 0.5 + k·sin(θ_cmd − 2π/3)
duty_w = 0.5 + k·sin(θ_cmd + 2π/3)     （等价于 −4π/3）
```

- `k = U_相幅值 / Vm`，是调制深度（0~1）。本板 Vm ≈ 12.4V，**U 取 1~2V → k ≈ 0.08~0.16**，非常小；
- `θ_cmd += ω × dt`，ω 是**电角速度**（rad/s，电角度域），建议 2~10 rad/s 起步；
- 三个占空比均值都是 50%，所以共模分量抵消，线电压是干净的三相正弦。

> 与 SVPWM 的差别：线性调制区 SVPWM 相幅值上限 = Vm/√3 ≈ 7.2V，SPWM 只有 Vm/2 ≈ 6.2V。开环小电压下无所谓。

## 2. 需要的外设（对照 CubeMX）

| 外设 | 配置 | 状态 |
|---|---|---|
| TIM2 CH1/CH3/CH4（PA0/PA2/PA3） | 中心对齐，Period=3600-1，约 10kHz | 清单阶段二已列 |
| PA7 `MOTOR_EN` | GPIO 输出，默认低 | 清单阶段二已列 |
| USART1 | 115200 | 清单阶段二已列 |
| **TIM3**（本实验新增） | Prescaler=72-1，Period=1000-1 → **1kHz 更新中断**，NVIC 使能 | 本次加 |

> 不需要 ADC、不需要 I2C——这是本方案"野路子"的价值：不接编码器也能转。

## 3. 代码（新建 `drivers/motor_ctrl/spwm.c/h`，自行加入 CMake）

### spwm.h

```c
#ifndef SPWM_H
#define SPWM_H
#include <stdint.h>
#include <stdbool.h>

void  spwm_init(void);          // 配置完 TIM2 后调用一次
void  spwm_start(float omega, float volt);   // omega: 电角速度 rad/s; volt: 相电压幅值 V
void  spwm_stop(void);          // 归零占空比 + 关断驱动
void  spwm_set_speed(float omega);           // 运行中调
float spwm_get_angle(void);                  // 调试用，读当前 θ_cmd

// 由 TIM3 1kHz 中断调用（在 stm32f1xx_it.c 或 HAL 回调里）
void  spwm_tick(void);

#endif
```

### spwm.c

```c
#include "spwm.h"
#include "main.h"      // htim2, MOTOR_EN 标签
#include <math.h>

extern TIM_HandleTypeDef htim2;

#define PWM_PERIOD   3600.0f    // 与 CubeMX ARR 一致
#define VM           12.4f      // 母线电压（MT3608 实测值），影响 k 换算
#define TICK_FREQ    1000.0f    // TIM3 中断频率

static volatile float theta_cmd;      // 电角度 rad
static volatile float omega;          // 电角速度 rad/s
static volatile float k_mod;          // 调制深度 = U/VM
static volatile bool  running;

void spwm_init(void)
{
    theta_cmd = 0.0f;
    omega = 0.0f;
    k_mod = 0.0f;
    running = false;
    // 三相 50%（零矢量），先启动 PWM，驱动仍是关的
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)(PWM_PERIOD * 0.5f));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)(PWM_PERIOD * 0.5f));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint32_t)(PWM_PERIOD * 0.5f));
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

void spwm_start(float omega_rad, float volt)
{
    theta_cmd = 0.0f;
    omega = omega_rad;
    k_mod = volt / VM;
    if (k_mod > 0.3f) k_mod = 0.3f;     // 安全限幅：≤ ~3.7V 相幅值
    HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_SET);   // 拉高 PA7
    running = true;
}

void spwm_stop(void)
{
    running = false;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)(PWM_PERIOD * 0.5f));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)(PWM_PERIOD * 0.5f));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint32_t)(PWM_PERIOD * 0.5f));
    HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET); // 关断
}

void spwm_set_speed(float omega_rad) { omega = omega_rad; }

float spwm_get_angle(void) { return theta_cmd; }

void spwm_tick(void)
{
    if (!running) return;

    // 1. 角度推进
    theta_cmd += omega / TICK_FREQ;
    if (theta_cmd > 2.0f * 3.14159265f) theta_cmd -= 2.0f * 3.14159265f;

    // 2. 三相正弦占空比（一次性算好，缩短中断耗时）
    const float TWO_PI_3 = 2.0943951f;
    float s_u = sinf(theta_cmd);
    float s_v = sinf(theta_cmd - TWO_PI_3);
    float s_w = sinf(theta_cmd + TWO_PI_3);

    __disable_irq();    // 临界区：三相必须同时更新
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,
        (uint32_t)((0.5f + k_mod * s_u) * PWM_PERIOD));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,
        (uint32_t)((0.5f + k_mod * s_v) * PWM_PERIOD));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4,
        (uint32_t)((0.5f + k_mod * s_w) * PWM_PERIOD));
    __enable_irq();
}
```

### 接入点（`application/src/app.c` 或 main.c 的 USER CODE 区）

```c
// 初始化（app_init / USER CODE BEGIN 2）：
MX_TIM3_Init();          // CubeMX 已生成
spwm_init();
HAL_TIM_Base_Start_IT(&htim3);

// TIM3 中断回调（全局只此一份，若已有 HAL_TIM_PeriodElapsedCallback 则在 TIM3 分支里加一行）：
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) spwm_tick();
}

// app_run() 里做远程控制实验（串口命令或临时变量控制）：
spwm_start(2.0f, 1.5f);   // 2 rad/s、1.5V，起步组合
// 停止：spwm_stop();
```

## 4. 实验步骤（每步有验收点）

1. **静态 50%**：只调 `spwm_init()` + PA7 拉高。示波器量 PA0/PA2/PA3 均为 10kHz 对称波形；`OUTx` 对 GND ≈ 6.2V 直流。→ 验证载波与使能。
2. **单相正弦**：手动写固定 θ=0 的三相比值（k=0.1），示波器（AC 耦合）看 OUT1，应为正弦且 U-V、U-W 相位差 120°。→ 验证调制公式。
3. **慢速旋转**：`spwm_start(2.0f, 1.5f)`，电机应低速均匀转动（低速顿挫正常）。
   - 完全不动 → PA7 / k 太小 / ω 太小；
   - 猛抖后堵死 → 相序错，**把 V/W 两根电机线对调，或把代码里 `−2π/3` 与 `+2π/3` 互换**；
   - 啸叫明显 → 正常（10kHz 在人耳范围），后续可升 PWM 频率到 16~20kHz（Period 改 1800-1）。
4. **加速与失步演示**（重要认知实验）：逐步加 ω 到 30~50 rad/s，观察在某一点电机突然失步抖动/堵转——这就是开环的失步边界，闭环没有这个问题。
5. **换向验证**：`spwm_set_speed(-2.0f)`，θ 递减，电机应反向（若不反，同样说明相序处理在别处）。

## 5. 与闭环方案的衔接

| 验证项 | SPWM 阶段获得 | 闭环阶段复用方式 |
|---|---|---|
| 相序/方向 | 正反都试过 | 决定 `AS5600_DIR` 与电角度符号 |
| 功率链路 | 使能/载波/三相全通 | 直接复用 |
| PWM 更新临界区写法 | 已验证 | SVPWM 输出同样写法 |
| 局限 | 转矩不受控、电压利用率低 15% | 闭环换回 SVPWM（`FOC_VoltageOutput`） |

验证完成后 `spwm.c` 可保留作为调试工具，正式闭环走 `drivers/motor_ctrl/foc.c` 的 SVPWM 路径，两者共用同一个 TIM2。

## 6. 注意事项

1. **PA7 上电默认低**：PWM 未就绪前绝不能使能，`spwm_start` 里先写好 50% 占空比再拉高 EN 的顺序不要颠倒；
2. **异常停机**：调试中任何异常（异响、发烫）立即 `spwm_stop()`；更稳妥可在断点前直接拔电机线；
3. **sinf 耗时**：72MHz 下软件 `sinf` 约几 µs，1kHz 中断完全够用；若日后把调制频率升到 10kHz 以上，换 example1 的 `sine_array` 查表或 CMSIS-DSP `arm_sin_f32`；
4. **VM 实测**：k_mod 用实测母线电压换算，USB 供电下 MT3608 输出可能偏离 12.4V，必要时串口打印供电电压。
