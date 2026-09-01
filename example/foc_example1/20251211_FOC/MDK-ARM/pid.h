#ifndef __PID_H
#define __PID_H

//#ifdef __cplusplus
//extern "C" {
//#endif

//#include "stm32f10x.h"
#include <stdint.h>

/* Q格式定义 */
#define Q 16
#define FLOAT_TO_Q15(x) ((int32_t)((x) * (1 << Q)))
#define Q15_TO_FLOAT(x) ((float)(x) / (1 << Q))

/* PID参数结构体 - 使用定点数(Q15格式) */
typedef struct {
    volatile int32_t Kp;          // 比例增益 (Q15)
    volatile int32_t Ki;          // 积分增益 (Q15) 
    volatile int32_t Kd;          // 微分增益 (Q15)
    volatile int32_t integral;    // 积分项累积 (Q15)
    volatile int32_t prev_error;  // 上次误差 (Q15)
    volatile int32_t out_max;     // 输出上限 (实际值)
    volatile int32_t out_min;     // 输出下限 (实际值)
    volatile int32_t integral_max; // 积分限幅 (Q15)
		volatile int32_t out; // 积分限幅 (Q15)
} PID_Controller;

/* 函数声明 */

/**
  * @brief  PID控制器初始化
  * @param  pid: PID控制器结构体指针
  * @param  kp: 比例增益
  * @param  ki: 积分增益
  * @param  kd: 微分增益
  * @param  out_max: 输出最大值
  * @param  out_min: 输出最小值
  * @retval 无
  */
void PID_Init(PID_Controller* pid, int32_t kp, int32_t ki, int32_t kd, 
              int32_t out_max, int32_t out_min);

/**
  * @brief  PID计算函数
  * @param  pid: PID控制器结构体指针
  * @param  setpoint: 设定值
  * @param  feedback: 反馈值
  * @retval PID控制器输出
  */
int32_t PID_Calculate(PID_Controller* pid, int32_t setpoint, int32_t feedback);



#ifdef __cplusplus
}
#endif

#endif /* __PID_H */

