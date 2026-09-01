/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "AS5600.h"
#include <math.h>

#include <string.h>
#include <stdarg.h>  // 添加这个头文件
#include <stdio.h>   // 如果需要使用vsnprintf

#include "foc.h"
#include "pid.h"
#include "foc_as5600.h"
#include "foc_math.h"
#include "foc_current.h"
#include "foc_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PWM_Period  3600
#define AS5600_DIR -1
#define AS5600_OFFSET 791
#define MOTOR_POLE_PAIR 7
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


// 电机角度相关变量（单位：弧度×1000）
volatile int32_t zero_shaft_angle_1000;   // 零位机械角度
volatile int32_t shaft_angle_1000;        // 当前机械角度
volatile int32_t shaft_angle_pre_1000;    // 上次机械角度
volatile int32_t shaft_speed_1000;        // 机械转速
volatile int32_t electrical_angle_1000;   // 电角度
volatile int32_t d_shaft_angle_1000;

// 电压控制变量
volatile int32_t voltage_q = 0;  // q轴电压给定
volatile int32_t voltage_d = 0;    // d轴电压给定

// 电流和速度控制变量
volatile int32_t current_tar_q = 0;  // q轴电流给定
volatile int32_t current_tar_d = 0;  // d轴电流给定
volatile int32_t speed_tar = 30000;      // 速度给定，100转/分钟

// 电流采样结构体
Currents currents = {0};

// PID控制器
PID_Controller pid_d;      // d轴电流环PID
PID_Controller pid_q;      // q轴电流环PID
PID_Controller pid_speed;  // 速度环PID

volatile int32_t pid_flag = 1;  // PID控制使能标志：0=开环电压控制，1=闭环PID控制

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t whilecount=0;


// 串口发送字符串
void UART_Print(char* str)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);
}
// 串口发送多个整数
void UART_PrintMultipleInts(int a, int b, int c)
{
    char buffer[50];  // 用于存储转换后的字符串
    // 格式化多个整数，按照指定格式拼接
    sprintf(buffer, "%d,%d,%d\n", a, b, c);
    // 通过串口发送格式化后的字符串
    UART_Print(buffer);
}



//UART UART UART UART UART UART UART UART UART UART UART UART UART UART UART UART UART UART UART UART UART UART 
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
 
int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
  return ch;
}



//☆TIM3中断 ☆TIM3中断 ☆TIM3中断 ☆TIM3中断 ☆TIM3中断 ☆TIM3中断 ☆TIM3中断 ☆TIM3中断 ☆TIM3中断 ☆TIM3中断 ☆TIM3中断

// TIM3中断计数器，用于定时任务
volatile uint32_t tim3_interrupt_count = 0;
/**
  * @brief  TIM3周期中断回调函数 - FOC控制主循环
  * @param  htim: TIM句柄
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        tim3_interrupt_count++;
			
      // 检查AS5600角度数据是否就绪
        if(as5600_data_ready)
        {
            as5600_data_ready = 0;
            
            // 1. 获取机械角度（通过DMA）
            shaft_angle_1000 = AS5600GetAngle_DMA(as5600_angle);
            
            // 2. 计算电角度（7对极电机）
//            electrical_angle_1000 = ((-shaft_angle_1000 + _2PI_1000 + zero_shaft_angle_1000) * 7) % _2PI_1000;
//						electrical_angle_1000 = ((shaft_angle_1000 + _2PI_1000 - zero_shaft_angle_1000) * 7) % _2PI_1000;
						electrical_angle_1000 = ((AS5600_DIR*(shaft_angle_1000-zero_shaft_angle_1000) + _2PI_1000  ) * MOTOR_POLE_PAIR) % _2PI_1000;
            
            // 3. 计算转速（带角度归一化和低通滤波）
            d_shaft_angle_1000 = shaft_angle_1000 - shaft_angle_pre_1000;
						shaft_angle_pre_1000 = shaft_angle_1000;
            
            // 角度归一化处理
            if(d_shaft_angle_1000 < -1000)
                d_shaft_angle_1000 = _2PI_1000 + d_shaft_angle_1000;
            else if(d_shaft_angle_1000 > 1000)
                d_shaft_angle_1000 = -(_2PI_1000 - d_shaft_angle_1000);
            
            // 转速计算（一阶低通滤波：0.2旧值 + 0.8新值）
            shaft_speed_1000 = 0.9 * shaft_speed_1000 + 0.1 * AS5600_DIR*(d_shaft_angle_1000 * 1000 * 6000 / _2PI_1000);//单位是0.01rpm(每分钟圈数)
             //shaft_speed_1000 = AS5600_DIR*(d_shaft_angle_1000 * 1000 * 60 / _2PI_1000);//单位是rpm(每分钟圈数)
            
            // 4. 读取并处理ADC电流采样（去除偏置）
            currents.a = adc_buffer[0] - 1976;  // 通道A电流，1975为偏置值
            currents.b = adc_buffer[1] - 1978;  // 通道B电流
            FOC_CurrentProcessing(&currents, electrical_angle_1000);
            
            // 5. 根据pid_flag选择控制模式
            if(pid_flag)
            {
                // 闭环PID控制模式
                // 速度环控制
                PID_Calculate(&pid_speed, speed_tar, shaft_speed_1000);
                
                
                // 电流环控制
								current_tar_q = pid_speed.out;  // 速度环输出作为q轴电流给定
                PID_Calculate(&pid_d, 0, currents.d);      // d轴电流控制
                PID_Calculate(&pid_q, current_tar_q, currents.q);  // q轴电流控制
                
                // 输出SVPWM
                FOC_VoltageOutput(pid_d.out, pid_q.out, electrical_angle_1000);
            }
            else
            {
                // 开环电压控制模式
                FOC_VoltageOutput(voltage_d, voltage_q, electrical_angle_1000);
            }
        }
        else
        {
            // AS5600数据未就绪，输出调试信息
            UART_Print("as5600_data_ready is 0\r\n");
            printf("%d,%d,%d\r\n", shaft_angle_1000, electrical_angle_1000, as5600_data_ready);
        }

    }
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//我的代码1
	//ADC_ChannelConfTypeDef sConfig;		//通道初始化
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	//我的代码初始化
	//ADC_ChannelConfTypeDef sConfig;		//通道初始化
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	//我的代码开始

	HAL_Delay(1000);
	AS5600_Read_Angle();  // 启动第一次读取
	HAL_Delay(100);

	
	// 启动TIM2的4个PWM通道（用于三相逆变器控制）
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);


	// 启动ADC DMA转换
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE * 2) != HAL_OK)
	{
		Error_Handler();
	}
	//用定时器触发ADC采样
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_Period-10);
	

	//电机参数识别程序
	//foc_config_main();while(1){}

	//要么执行简单的电机零位识别，获取zero_shaft_angle_1000
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);// PA7使能电机驱动芯片
	FOC_VoltageOutput(0,100, _3PI_2_1000);
	HAL_Delay(1000);
	zero_shaft_angle_1000=AS5600GetAngle_DMA(as5600_angle);
	FOC_VoltageOutput(0,0, 0);
	//要么直接给zero_shaft_angle_1000
	//zero_shaft_angle_1000=AS5600_OFFSET;

	printf("Hello, STM32!\r\n");
	//printf("你好, STM32!\r\n");
	

	
	// PID控制器初始化
	// 速度环PID参数：Kp=10000, Ki=10, Kd=0, 输出限幅±1000
	PID_Init(&pid_speed, 5000, 20, 0, 500, -500);//高速10~800(300 1)
	// 电流环PID参数：Kp=10000, Ki=5000, Kd=0, 输出限幅±500
	PID_Init(&pid_d, 10000, 500, 0, 500, -500);
	PID_Init(&pid_q, 10000, 500, 0, 500, -500);
	

	// 启动TIM3定时器中断（FOC控制周期）
    HAL_TIM_Base_Start_IT(&htim3);
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		//whilecount++;
		if(tim3_interrupt_count>1000){
			tim3_interrupt_count=0;
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
			
			//UART_PrintMultipleInts(shaft_angle_1000,electrical_angle_1000, _cos(electrical_angle_1000));
			//UART_PrintMultipleInts(1000*Ta,1000*Tb,1000*Tc);
		}

		
//		
		if(ADC_data_ready){
			
			//whilecount++;
			ADC_data_ready=0;
			printf("%d,%d,%d,%d,%d,%d,%d\r\n", \
			currents.a,\
			currents.b,\
			current_tar_q, \
			currents.q,\
			currents.d,\
			speed_tar,\
			shaft_speed_1000);
			
			//ProcessADCData();
		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
