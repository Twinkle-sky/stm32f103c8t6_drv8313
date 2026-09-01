#ifndef __FOC_H
#define __FOC_H

#include <stdint.h>
#include "tim.h"
#include "foc_current.h"
/* 正弦表 (存储0-π/2的正弦值 * 1000) */
extern const int16_t sine_array[1572];


#define PWM_Period  3600




///* 角度结构体 */
//typedef struct {
//    volatile int32_t sin_v;  /* 正弦值 (Q10格式) */
//    volatile int32_t cos_v;  /* 余弦值 (Q10格式) */
//} Angle;

/* PWM占空比结构体 */
typedef struct {
    volatile int32_t a;     /* A相占空比 (0-1024) */
    volatile int32_t b;     /* B相占空比 (0-1024) */
    volatile int32_t c;     /* C相占空比 (0-1024) */
} PWM_Duty;



/* FOC核心函数 */
void FOC_ClarkTransform(Currents* currents);
void FOC_ParkTransform(Currents* currents, uint16_t angle);
void FOC_InverseParkTransform(int32_t vd, int32_t vq, uint16_t angle, int32_t* valpha, int32_t* vbeta);
void FOC_CurrentProcessing(Currents* currents, uint16_t angle);

PWM_Duty FOC_VoltageOutput(int32_t vd, int32_t vq, uint16_t angle);
uint16_t FOC_AngleNormalize(uint16_t angle);

#endif /* __FOC_H */