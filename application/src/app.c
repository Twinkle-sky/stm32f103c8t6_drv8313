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

#include "spwm.h"
#include "tim.h"
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
    spwm_init();
    HAL_TIM_Base_Start_IT(&htim3);
    spwm_start(6.28f, 1.5f);   // 2 rad/s、1.5V，起步组合
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

}

// TIM3 中断回调（全局只此一份，若已有 HAL_TIM_PeriodElapsedCallback 则在 TIM3 分支里加一行）：
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) spwm_tick();
}

