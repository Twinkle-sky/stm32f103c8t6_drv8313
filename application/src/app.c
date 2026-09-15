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

#include "adc.h"
/* Private Defines ------------------------------------------------------------*/

#define BLINK_PERIOD_MS  500U   /* 500ms on, 500ms off → 1 Hz */

/* Public Functions -----------------------------------------------------------*/
volatile uint32_t adc_isr_count = 0;   /* 注入中断计数：主循环算速率用 */

uint32_t value_adc1 = 0;
uint32_t value_adc2 = 0;

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    value_adc1 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    value_adc2 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    adc_isr_count++;
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);   /* ADC探针 */
  }
}
/**
 * @brief  Application initialization.
 */
void app_init(void)
{
    /* ---- 1) 传感器先起来：此时 ADC 还没使能、TIM2 还没启动，
     *         没有任何 20kHz 注入中断来打扰 I2C 时序 ----
     *  5600的启动要比mcu慢，此处必须要延时，否则可能会卡死IIC总线
     *  若要增强鲁棒性，最好实现AS5600_Bus_Recovery，出错后deinit IIC */
    HAL_Delay(100);
    AS5600_Read_Angle();
    HAL_Delay(100);

    /* ---- 2) ADC 双模式注入就绪，必须在 TIM2 载波启动（TRGO 开始输出）之前 ----
     * dual injected simultaneous 模式下 ADC1 的注入触发会同时带动 ADC2，
     * 所以从机 ADC2 必须先上电并使能，主机 ADC1 后开中断；否则 ADC1 收到的
     * 第一批触发沿会打在 ADC2 尚未准备好的时候。
     * !! hadc1.Init.ContinuousConvMode 必须保持 ENABLE !!
     *    HAL_ADC_IRQHandler 处理完 JEOC 后会判断
     *    (注入软件触发 || (JAUTO==0 && 规则组软件触发 && CONT==DISABLE)) 并关掉
     *    JEOC 中断；本工程注入是外部触发，正靠 CONT==ENABLE 让该分支不成立，
     *    改成 DISABLE 后注入中断只会进一次，探针直接停摆。 */
    HAL_ADCEx_Calibration_Start(&hadc2);
    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADCEx_InjectedStart(&hadc2);        /* 从机先使能 */
    HAL_ADCEx_InjectedStart_IT(&hadc1);     /* 主机后启动，JEOC 中断在此打开 */

    /* ---- 3) 最后启动 PWM 载波与节拍（顺序要求：先 init 占空比 50%，
     *         再 start 拉高 EN，最后 TIM3 推进角度）---- */
    spwm_init();
    HAL_TIM_Base_Start_IT(&htim3);
    // spwm_start(6.28f, 1.5f);   // 2 rad/s、1.5V，起步组合
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
    // static uint32_t last_adc_isr = 0, last_reads = 0, last_recov = 0;
    // uint32_t isr_rate  = (adc_isr_count - last_adc_isr) * 10U;   /* 每 100ms 一拍 → ×10 = 每秒 */
    // uint32_t read_rate = (as5600_data_count - last_reads) * 10U;
    // uint32_t recov     = as5600_recover_count - last_recov;
    // last_adc_isr = adc_isr_count;
    // last_reads   = as5600_data_count;
    // last_recov   = as5600_recover_count;

    // printf("adc_isr/s=%lu (expect 20000) reads/s=%lu recover=%lu\r\n",
    //        (unsigned long)isr_rate, (unsigned long)read_rate,
    //        (unsigned long)recov);
    printf("as5600_angle = %u\r\n",as5600_angle);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);   /* 心跳：LED 闪 = 主循环活着 */
    HAL_Delay(100);
}

// TIM3 中断回调（全局只此一份，若已有 HAL_TIM_PeriodElapsedCallback 则在 TIM3 分支里加一行）：
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // if (htim->Instance == TIM3) spwm_tick();
}

