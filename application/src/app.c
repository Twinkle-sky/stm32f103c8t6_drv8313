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

/* Private Defines ------------------------------------------------------------*/

#define BLINK_PERIOD_MS  500U   /* 500ms on, 500ms off → 1 Hz */

/* Public Functions -----------------------------------------------------------*/

/**
 * @brief  Application initialization.
 */
void app_init(void)
{
    /*
     * Ensure LED is off at startup.
     * GPIO is already configured by MX_GPIO_Init().
     */
    // HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
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
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(BLINK_PERIOD_MS);
}
