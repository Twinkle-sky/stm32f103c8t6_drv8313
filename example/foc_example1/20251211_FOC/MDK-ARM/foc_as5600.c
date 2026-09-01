#include "main.h"
#include "i2c.h"
#include "foc_math.h"
#include "foc_as5600.h"
//AS5600 AS5600 AS5600 AS5600 AS5600 AS5600 AS5600 AS5600 AS5600 AS5600 AS5600 AS5600 AS5600 AS5600 AS5600 AS5600 
// 最优方案 - 专门用于读取寄存器设备
// 全局变量
uint8_t as5600_rx_data[2];
volatile uint16_t as5600_angle = 0;
volatile uint8_t as5600_data_ready = 0;
volatile  uint32_t as5600_data_count=0;


static int angle_data_prev; //上次位置
// 启动AS5600角度读取
void AS5600_Read_Angle(void)
{
    HAL_I2C_Mem_Read_DMA(&hi2c1,
                        ((0x36 << 1)|1),      // AS5600地址
                        0x0C,             // 角度寄存器
                        I2C_MEMADD_SIZE_8BIT,
                        as5600_rx_data,
                        2);
}

// 回调函数 - 这个会被HAL库自动调用！
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c->Instance == I2C1) {
        // 组合角度数据（高字节在前）
        as5600_angle = (as5600_rx_data[0] << 8) | as5600_rx_data[1];
        //as5600_angle &= 0x0FFF;  // 取12位有效数据
        
        as5600_data_ready = 1;   // 设置数据就绪标志
        as5600_data_count++;
        // 可以立即启动下一次读取（连续采样）
        AS5600_Read_Angle();
    }
}
// 错误处理回调
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c->Instance == I2C1) {
        // 发生错误时，延迟一段时间后重试
        HAL_Delay(1000);
        AS5600_Read_Angle();
    }
}


int AS5600GetAngle_DMA(uint16_t as5600_angle) {
  
  int angle_data = as5600_angle;
	
  angle_data_prev = angle_data;
	
	return angle_data*_2PI_1000/4096;
}
