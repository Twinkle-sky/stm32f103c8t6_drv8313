#pragma once
#include "global_def.h"
// 电机物理参数：
#define POLE_PAIRS 7 // 极对数

// 电路参数：
#define R_SHUNT 0.02           // 电流采样电阻，欧姆
#define OP_GAIN 50             // 运放放大倍数
#define MAX_CURRENT 2          // 最大q轴电流，安培A
#define ADC_REFERENCE_VOLT 3.3 // 电流采样adc参考电压，伏
#define ADC_BITS 12            // ADC精度，bit

// 单片机配置参数：
#define motor_pwm_freq 40000      // 驱动桥pwm频率，Hz
#define motor_speed_calc_freq 930 // 电机速度计算频率，Hz

// 软件参数：

// 电机软件上的多圈周期，位置模式下能控制的范围，等于正半周期+负半周期，可以任意修改(比如改为1234*pi)
#define position_cycle (6 * PI)
