#include "foc_current.h"
#include "adc.h"
//#include "usart.h"
//ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC ADC 
/* USER CODE BEGIN PV */

uint16_t adc_buffer[ADC_BUFFER_SIZE * 2];  // 2个通道的数据缓冲区
uint16_t ADC_data_ready=0;


// ADC转换完成回调函数（可选）
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  // 每次DMA传输完成时调用
  if(hadc->Instance == ADC1)
  {
		ADC_data_ready=1;
		if( adc_buffer[0]<96||adc_buffer[1]<96||adc_buffer[0]>4000||adc_buffer[1]>4000){
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);//过流关断PA6驱动芯片
			//UART_Print("!over current!\r\n");
		}
		//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);//
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
		
    //ProcessADCData();
  }
}


//void phase_current_init(Currents* currents){
//	
//}
