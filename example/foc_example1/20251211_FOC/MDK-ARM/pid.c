#include "pid.h"
#include <stdint.h>

/* 内部函数声明 */
static inline int32_t Q15_Multiply(int32_t a, int32_t b);

/**
  * @brief  PID控制器初始化
  */
void PID_Init(PID_Controller* pid, int32_t kp, int32_t ki, int32_t kd, 
              int32_t out_max, int32_t out_min) 
{
    pid->Kp = (kp);
    pid->Ki = (ki);
    pid->Kd = (kd);
    pid->integral = 0;
    pid->prev_error = 0;
    pid->out_max = (out_max<<Q);
    pid->out_min = (out_min<<Q);
	
    // 设置积分限幅，防止积分饱和
    pid->integral_max = 0.8f*((out_max<<Q) - (out_min<<Q));
}

/**
  * @brief  PID计算函数
  */
int32_t PID_Calculate(PID_Controller* pid, int32_t setpoint, int32_t feedback) 
{
    // 计算误差 (实际值)
    int32_t error = setpoint - feedback;
    
    
    // 比例项
    int32_t p_term = pid->Kp*error;//pid->Kp是乘以Q15的
    
    // 积分项 - 使用抗饱和积分
    pid->integral += (pid->Ki* error);
		
    
    // 积分限幅
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < -pid->integral_max) {
        pid->integral = -pid->integral_max;
    }
    
    int32_t i_term = pid->integral;
    
    // 微分项
    int32_t derivative = error - pid->prev_error;
    int32_t d_term =pid->Kd*derivative;
    pid->prev_error = error;
    
    // 计算PID输出 (Q15格式)
    int32_t output_q15 = p_term + i_term + d_term;
    
    // 转换为实际值 (右移0位)
    int32_t output = output_q15;
    
    // 输出限幅
    if (output > pid->out_max) {
        output = pid->out_max;
        // 抗饱和：如果输出饱和，停止积分累积
        if ((error > 0 && output > (pid->out_max)) ||
            (error < 0 && output < (pid->out_min))) {
            pid->integral -= (pid->Ki* error);
        }
    } else if (output < pid->out_min) {
        output = pid->out_min;
        // 抗饱和处理
        if ((error > 0 && output > (pid->out_max )) ||
            (error < 0 && output < (pid->out_min ))) {
            pid->integral -= (pid->Ki* error);
        }
    }
    pid->out=(output>>Q);
    return pid->out;
}

///**
//  * @brief  PID计算函数
//  */
//int32_t PID_Calculate(PID_Controller* pid, int32_t setpoint, int32_t feedback) 
//{
//    // 计算误差 (实际值)
//    int32_t error = setpoint - feedback;
//    
//    // 转换为Q15格式进行运算
//    int32_t error_q15 = error; //error<< Q;
//    
//    // 比例项
//    int32_t p_term = Q15_Multiply(pid->Kp, error_q15);
//    
//    // 积分项 - 使用抗饱和积分
//    pid->integral += Q15_Multiply(pid->Ki, error_q15);
//    
//    // 积分限幅
//    if (pid->integral > pid->integral_max) {
//        pid->integral = pid->integral_max;
//    } else if (pid->integral < -pid->integral_max) {
//        pid->integral = -pid->integral_max;
//    }
//    
//    int32_t i_term = pid->integral;
//    
//    // 微分项
//    int32_t derivative = error_q15 - pid->prev_error;
//    int32_t d_term = Q15_Multiply(pid->Kd, derivative);
//    pid->prev_error = error_q15;
//    
//    // 计算PID输出 (Q15格式)
//    int32_t output_q15 = p_term + i_term + d_term;
//    
//    // 转换为实际值 (右移Q位)
//    int32_t output = output_q15 >> Q;
//    
//    // 输出限幅
//    if (output > pid->out_max) {
//        output = pid->out_max;
//        // 抗饱和：如果输出饱和，停止积分累积
//        if ((error > 0 && output_q15 > (pid->out_max << Q)) ||
//            (error < 0 && output_q15 < (pid->out_min << Q))) {
//            pid->integral -= Q15_Multiply(pid->Ki, error_q15);
//        }
//    } else if (output < pid->out_min) {
//        output = pid->out_min;
//        // 抗饱和处理
//        if ((error > 0 && output_q15 > (pid->out_max << Q)) ||
//            (error < 0 && output_q15 < (pid->out_min << Q))) {
//            pid->integral -= Q15_Multiply(pid->Ki, error_q15);
//        }
//    }
//    pid->out=output;
//    return output;
//}



/* 内部函数实现 */

/**
  * @brief  定点数乘法 - 保持Q格式
  */
static inline int32_t Q15_Multiply(int32_t a, int32_t b) 
{
    int64_t result = (int64_t)a * b;
    return (int32_t)(result >> Q);
}

//// 使用示例main.c
//#include "pid.h"
//#include "stm32f10x.h"

//int main(void) {
//    PID_Controller motor_pid;
//    
//    // 初始化PID：Kp=1.0, Ki=0.1, Kd=0.05, 输出范围-1000~1000
//    PID_Init(&motor_pid, 1.0f, 0.1f, 0.05f, 1000, -1000);
//    
//    int32_t target = 500;    // 目标值
//    int32_t feedback = 0;    // 反馈值
//    int32_t output;          // 控制输出
//    
//    while(1) {
//        // 读取传感器获取反馈值
//        // feedback = Read_Sensor();
//        
//        // 计算PID输出
//        output = PID_Calculate(&motor_pid, target, feedback);
//        
//        // 应用控制输出
//        // Set_Output(output);
//        
//        // 延时，控制周期1ms
//        // Delay_ms(1);
//    }
//}
