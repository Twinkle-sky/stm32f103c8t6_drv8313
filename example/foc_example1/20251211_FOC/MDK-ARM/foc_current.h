#ifndef __FOC_CURRENT_H
#define __FOC_CURRENT_H

#include <stdint.h>
#define ADC_BUFFER_SIZE 1
// 选项1：2相电流采样（U/V两相，W相通过计算得出）
#define CURRENT_PHASE_NUM  2
extern uint16_t adc_buffer[ADC_BUFFER_SIZE * 2];  // 2个通道的数据缓冲区

extern uint16_t ADC_data_ready;




/* 电流结构体 */
typedef struct {
    volatile int32_t a;      /* A相电流 (Q10格式) */
    volatile int32_t b;      /* B相电流 (Q10格式) */
    volatile int32_t c;      /* C相电流 (Q10格式)  */
    volatile int32_t alpha;  /* α轴电流 (Q10格式) */
    volatile int32_t beta;   /* β轴电流 (Q10格式) */
    volatile int32_t d;      /* d轴电流 (Q10格式) */
    volatile int32_t q;      /* q轴电流 (Q10格式) */
} Currents;


//void phase_current_init(Currents* currents);


#endif /* __FOC_CURRENT_H */