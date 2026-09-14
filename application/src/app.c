/**
 ******************************************************************************
 * @file    app.c
 * @brief   Application layer implementation — PC13 LED blink
 ******************************************************************************
 * @attention
 *
 * This is the application entry point. Currently implements a simple
 * PC13 LED blinker as a demonstration of the firmware/app separation.
 *
 * In production, replace this with your actual application logic
 * (sensor fusion, control loops, communication stacks, etc.).
 *
 ******************************************************************************
 */

#include "app.h"
#include "main.h"            /* BSP defines: LED_Pin, LED_GPIO_Port */
#include "stm32f1xx_hal.h"   /* HAL_Delay, HAL_GPIO_TogglePin */
#include <stdio.h>           /* printf */

#include "spwm.h"
#include "stm32f1xx_hal_uart.h"
#include "tim.h"
#include "usart.h"
#include "as5600.h"
#include <stdio.h>
/* Private Defines ------------------------------------------------------------*/

#define BLINK_PERIOD_MS  500U   /* 500ms on, 500ms off → 1 Hz */

/* Public Functions -----------------------------------------------------------*/

/**
 * @brief  Application initialization.
 */
void app_init(void)
{
    /* 顺序要求：先 init（启动 PWM、占空比 50%），再 start（拉高 EN），
     * 最后启动 TIM3 中断开始推进角度。 */
    // spwm_init();
    // HAL_TIM_Base_Start_IT(&htim3);
    // spwm_start(6.28f, 1.5f);   // 2 rad/s、1.5V，起步组合

    //5600的启动要比mcu慢，此处必须要延时，否则可能会卡死IIC总线
    //若要增强鲁棒性，最好实现AS5600_Bus_Recovery，出错后deinit IIC
    HAL_Delay(100);
    AS5600_Read_Angle();
    HAL_Delay(100);
}

/**
 * @brief  Application main loop iteration.
 *
 * Simple blinker: toggle PC13 every 500ms using blocking delay.
 * For a real application, replace HAL_Delay with a non-blocking
 * timer/scheduler to avoid wasting CPU cycles.
 */
void app_run(void)
{
    /* 链式 DMA 已由回调自动续读，这里只消费 as5600_angle */
    printf("as5600_angle=%u reads=%lu\r\n",
           as5600_angle, (unsigned long)as5600_data_count);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);   /* 心跳：LED 闪 = 主循环活着 */
    HAL_Delay(100);
}

// TIM3 中断回调（全局只此一份，若已有 HAL_TIM_PeriodElapsedCallback 则在 TIM3 分支里加一行）：
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // if (htim->Instance == TIM3) spwm_tick();
}

