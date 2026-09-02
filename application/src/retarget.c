/**
 ******************************************************************************
 * @file    retarget.c
 * @brief   Redirect C library I/O (printf, puts, ...) to USART1
 ******************************************************************************
 * @attention
 *
 * Kept in the application layer on purpose: the CubeMX-generated files
 * under firmware/ must stay untouched.
 *
 * CubeMX syscalls.c provides a *weak* _write() that loops over
 * __io_putchar(). Both are overridden here:
 *   - _write()        : strong override, sends the whole printf() buffer
 *                       in one blocking transfer (fewer HAL calls).
 *   - __io_putchar()  : fallback hook, used by the picolibc path in
 *                       syscalls.c and by any code calling it directly.
 *
 * NOTE: printf() shares huart1 with any HAL_UART_Transmit_IT() calls.
 * While an IT transmission is in progress the blocking transmit returns
 * HAL_BUSY and the printf() output of that call is dropped.
 *
 ******************************************************************************
 */

#include <stdint.h>

#include "usart.h"   /* huart1 — CubeMX-generated, lives in firmware/ */

/* Public Functions -----------------------------------------------------------*/

/**
 * @brief  Low-level character output hook (newlib / picolibc retarget).
 * @param  ch Character to transmit
 * @retval The character transmitted
 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

/**
 * @brief  Strong override of the weak _write() in firmware syscalls.c.
 * @param  file File descriptor (unused)
 * @param  ptr  Data buffer
 * @param  len  Number of bytes to transmit
 * @retval Number of bytes handed to the UART driver
 */
int _write(int file, char *ptr, int len)
{
    (void)file;
    if (len <= 0)
    {
        return len;
    }
    /* 0xFFFF ms covers any sane printf() buffer at 115200 baud. */
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len, 0xFFFF);
    return len;
}
