#include "main.h"
#include "i2c.h"
#include "as5600.h"
#include <stdio.h>

uint8_t as5600_rx_data[2];
volatile uint16_t as5600_angle = 0;
volatile uint8_t as5600_data_ready = 0;
volatile  uint32_t as5600_data_count=0;
volatile uint32_t as5600_rd_status = 0;   /* last HAL_I2C_Mem_Read_DMA() return value */
volatile uint32_t as5600_recover_count = 0;  /* 总线恢复次数（诊断用） */

void AS5600_Read_Angle(void)
{
    as5600_rd_status = HAL_I2C_Mem_Read_DMA(&hi2c1,((0x36 << 1)|1),0x0C,I2C_MEMADD_SIZE_8BIT,as5600_rx_data,2);
}

/* µs 级忙等延时（ISR 上下文不能用 HAL_Delay） */
static void as5600_delay_us(uint32_t us)
{
    /* 72MHz 下约 12 个周期/循环，粗略校准 */
    uint32_t n = us * 6;
    while (n--) { __NOP(); }
}

/**
 * @brief  I2C 总线恢复：释放被从设备钳死的 SDA + 复位 I2C 外设。
 *
 * 冷上电时 AS5600 未就绪可能把 SDA 钳在低电平，此时任何 START 都发不出去，
 * 盲目重发永远失败。必须：
 *   1. DeInit I2C（释放引脚、复位外设状态机）
 *   2. 把 SCL/SDA 配成开漏 GPIO，手动打 9 个 SCL 时钟直到 SDA 释放
 *   3. 手动补一个 STOP 条件
 *   4. 重新 Init 并重启链式读取
 */
void AS5600_Bus_Recovery(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    int i;

    as5600_recover_count++;

    /* 1. 复位 I2C 外设并释放引脚（MspDeInit 会关时钟、GPIO 恢复默认） */
    HAL_I2C_DeInit(&hi2c1);

    /* 2. SCL/SDA 配成开漏输出，靠外部上拉保持高电平 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
    as5600_delay_us(10);

    /* 3. 最多 9 个时钟脉冲，直到 SDA 被从设备释放 */
    for (i = 0; i < 9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        as5600_delay_us(5);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        as5600_delay_us(5);
    }

    /* 4. 补一个 STOP：SCL 高时 SDA 由低到高 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    as5600_delay_us(5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    as5600_delay_us(5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    as5600_delay_us(10);

    /* 5. 重新初始化（MspInit 会重配 GPIO 复用、DMA、中断） */
    HAL_I2C_Init(&hi2c1);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c->Instance == I2C1) {
        /* RAW_ANGLE (reg 0x0C~0x0D): 12-bit value in the LOW 12 bits —
         * byte0 = RAW_ANGLE[11:8], byte1 = RAW_ANGLE[7:0].
         * (verified on real hardware: the 16-bit word spans 0~4095 over one
         * mechanical revolution; do NOT shift right by 4.) */
        as5600_angle = (uint16_t)(((uint16_t)as5600_rx_data[0] << 8) | as5600_rx_data[1]);
        as5600_data_ready = 1;
        as5600_data_count++;

        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);   /* 链路活性探针 */

        /* 链式续读：F1 HAL 对 DMA 状态复位有时序差异，若返回 BUSY 则
         * 强制 READY 后重试一次，保证链不断（否则只剩 app_init 那一次
         * 读取，后续全是旧值）。 */
        hi2c->hdmarx->State = HAL_DMA_STATE_READY;
        if (HAL_I2C_Mem_Read_DMA(&hi2c1, ((0x36 << 1) | 1), 0x0C,
                                 I2C_MEMADD_SIZE_8BIT,
                                 as5600_rx_data, 2) != HAL_OK) {
            as5600_rd_status = 1;   /* 链断裂标记，方便串口观察 */
        }
    }
}

/* I2C 出错（NACK/总线仲裁丢失/DMA 错误等）时 HAL 会停掉传输且不再
 * 触发完成回调。冷上电时 AS5600 未就绪会 NACK 甚至钳死 SDA，
 * 盲目重发永远失败 —— 必须做总线恢复（9 时钟释放 SDA + 外设复位），
 * 然后重启链式读取。 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        as5600_rd_status = 2;   /* 错误标记 */
        AS5600_Bus_Recovery();
        AS5600_Read_Angle();
    }
}
