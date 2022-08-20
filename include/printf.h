/**
 * @file   printf.h
 * @author ZiTe (honmonoh@gmail.com)
 */

#ifndef __PRINTF_H
#define __PRINTF_H

#include <stdio.h>
#include "stm32f4xx_hal.h"

#define PRINTF_USART_HANDLE_TYPEDEF (huart2)

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

#endif /* __PRINTF_H */