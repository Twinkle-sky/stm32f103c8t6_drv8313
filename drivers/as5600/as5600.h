#ifndef __AS5600_H
#define __AS5600_H

#include <stdint.h>

extern volatile uint8_t as5600_data_ready;
extern volatile uint16_t as5600_angle ;
extern volatile uint32_t as5600_data_count;
extern volatile uint32_t as5600_rd_status;
void AS5600_Read_Angle(void);
int AS5600GetAngle_DMA(uint16_t as5600_angle);



#endif