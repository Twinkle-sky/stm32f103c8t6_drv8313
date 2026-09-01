#pragma once

typedef enum
{
    control_type_null,                  // 不进行控制
    control_type_position,              // 位置控制
    control_type_speed,                 // 速度控制
    control_type_torque,                // 力矩控制
    control_type_speed_torque,          // 速度-力矩控制
    control_type_position_speed_torque, // 位置-速度-力矩控制
} motor_control_type;

typedef struct
{
    motor_control_type type;
    float position;        // 目标角度，单位rad
    float speed;           // 目标速度，单位rad/s
    float torque_norm_d;   // 目标d轴强度，0~1
    float torque_norm_q;   // 目标q轴强度，0~1
    float max_speed;       // 串级控制时的最大速度，单位rad/s
    float max_torque_norm; // 串级控制时的最大q轴力矩，0~1
} motor_control_context_t;

extern motor_control_context_t motor_control_context;

float cycle_diff(float diff, float cycle);
void foc_forward(float d, float q, float rotor_rad);

void lib_position_control(float rad);                                                           // 位置（角度）控制
void lib_speed_control(float speed);                                                            // 速度控制
void lib_torque_control(float torque_norm_d, float torque_norm_q);                              // 力矩（电流）控制
void lib_speed_torque_control(float speed, float max_torque_norm);                              // 速度-力矩控制
void lib_position_speed_torque_control(float position, float max_speed, float max_torque_norm); // 位置-速度-力矩控制

void set_motor_pid(
    float position_p, float position_i, float position_d,
    float speed_p, float speed_i, float speed_d,
    float torque_d_p, float torque_d_i, float torque_d_d,
    float torque_q_p, float torque_q_i, float torque_q_d
		);
