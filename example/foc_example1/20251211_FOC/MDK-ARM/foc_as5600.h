#ifndef __FOC_AS5600_H
#define __FOC_AS5600_H

#include <stdint.h>



extern volatile uint8_t as5600_data_ready;
extern volatile uint16_t as5600_angle ;
void AS5600_Read_Angle(void);
int AS5600GetAngle_DMA(uint16_t as5600_angle);



#endif /* __FOC_AS5600_H */