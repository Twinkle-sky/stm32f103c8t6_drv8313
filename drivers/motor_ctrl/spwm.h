/**
 * @file    spwm.h
 * @brief   三相 SPWM 开环驱动接口（方案四：正弦脉宽调制，无编码器开环旋转）
 *
 * 原理：输出三相互差 120° 电角度的正弦占空比，
 *       duty_x = 0.5 + k·sin(θ_cmd ± 2π/3)，
 *       合成一个以 ω 匀速旋转的电压矢量，以同步拖动方式带动转子旋转。
 *       θ_cmd 由 TIM3 周期中断（spwm_tick）自主推进，不依赖编码器。
 *
 * 硬件依赖：TIM2 CH1/CH3/CH4（PA0/PA2/PA3 三相 PWM）、PA7（MOTOR_EN 三相使能）、
 *           TIM3 周期中断（调用 spwm_tick）。
 */
#ifndef SPWM_H
#define SPWM_H
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief  初始化 SPWM 模块
 * @note   三相占空比置 50%（零矢量，输出等效 0V）并启动 TIM2 PWM，
 *         但不拉高使能（PA7 保持低，驱动关断），电机不通电。
 * @note   必须在 spwm_start() 之前调用，且只能调用一次（CubeMX 的
 *         MX_TIM2_Init() 之后）。
 */
void  spwm_init(void);

/**
 * @brief  启动开环旋转
 * @param  omega_rads  目标机械角速度，单位 rad/s（内部已自动 × 极对数
 *                     换算为电角速度，调用者传机械量即可。
 *                     例：6.28 = 2π rad/s = 1 机械圈/秒）
 * @param  volt        目标相电压幅值，单位 V（建议 1~2V 起步，
 *                     内部限幅 3.7V / 调制深度 0.3）
 * @note   会将 θ_cmd 清零并拉高 PA7 使能驱动。
 * @note   ★ 只能调用一次！重复调用会清零 θ_cmd 导致电机锁死不转。
 *         运行中改转速请用 spwm_set_speed()。
 */
void  spwm_start(float omega_rads, float volt);

/**
 * @brief  停止开环旋转
 * @note   三相占空比回 50%（零矢量）→ 再拉低 PA7 关断驱动。
 *         先归零后关断的顺序保证 MOS 无冲击。
 */
void  spwm_stop(void);

/**
 * @brief  运行中修改目标机械角速度（单位与 spwm_start 的 omega_rads 相同：rad/s 机械）
 * @note   只改变推进速度，不重置角度，电机平滑变速，用于验证正反转。
 */
void  spwm_set_speed(float omega_rads);

/**
 * @brief  读取当前电角度指令 θ_cmd（0~2π），调试用
 * @return 电角度，单位 rad
 * @note   验证方法：主循环每 1s 打印一次，增量应 = 设定机械角速度 × 7。
 */
float spwm_get_angle(void);

/**
 * @brief  SPWM 周期节拍，由 TIM3 更新中断（1kHz）调用
 * @note   在 HAL_TIM_PeriodElapsedCallback 的 TIM3 分支中调用，
 *         每次调用推进一次 θ_cmd 并刷新三相占空比。
 * @note   执行频率必须与 spwm.c 中 TICK_FREQ 宏一致，否则实际转速
 *         会按比例偏离设定值。
 */
void  spwm_tick(void);

#endif
