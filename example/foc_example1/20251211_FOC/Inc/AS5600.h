#ifndef __AS5600_H
#define __AS5600_H

#include "i2c.h"

#define AS5600_I2C_HANDLE hi2c1

#define I2C_TIME_OUT_BASE   10
#define I2C_TIME_OUT_BYTE   1

/*
注意:AS5600的地址0x36是指的是原始7位设备地址,而ST I2C库中的设备地址是指原始设备地址左移一位得到的设备地址
*/

#define AS5600_RAW_ADDR    0x36
#define AS5600_ADDR        (AS5600_RAW_ADDR << 1)
#define AS5600_WRITE_ADDR  (AS5600_RAW_ADDR << 1)
#define AS5600_READ_ADDR   ((AS5600_RAW_ADDR << 1) | 1)


#define AS5600_RESOLUTION 4096 //12bit Resolution 

#define AS5600_RAW_ANGLE_REGISTER  0x0C




#define PI														3.14159265358979f
#define AS5600_ADDRESS                0x36<<1		//设备从地址
#define Write_Bit                 		0	   			//写标记
#define Read_Bit                  		1    			//读标记
#define Angle_Hight_Register_Addr 		0x0C 			//寄存器高位地址
#define Angle_Low_Register_Addr   		0x0D 			//寄存器低位地址
#define DATA_SIZE 										2 				// 每次读取2字节数据


void AS5600Init(void);
uint16_t AS5600GetRawAngle(void);
float AS5600GetAngle(void);
void AS5600_Read_DMA(uint8_t regAddress, uint8_t* pData, uint16_t Size);
int i2cRead_DMA(uint8_t dev_addr, uint8_t *pData, uint32_t count);
int i2cWrite_DMA(uint8_t dev_addr, uint8_t *pData, uint32_t count);
int AS5600GetAngle_DMA(uint16_t as5600_angle);
//int AS5600GetAngle_DMA_init(uint16_t as5600_angle) ;
#endif /* __BSP_AS5600_H */


