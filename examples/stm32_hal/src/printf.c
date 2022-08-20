/**
 * @file   printf.c
 * @author ZiTe (honmonoh@gmail.com)
 */

#include "printf.h"

extern UART_HandleTypeDef PRINTF_USART_HANDLE_TYPEDEF;

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&PRINTF_USART_HANDLE_TYPEDEF,
                    (uint8_t *)&ch,
                    1,
                    HAL_MAX_DELAY);
  return ch;
}