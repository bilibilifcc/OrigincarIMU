#ifndef __DEBUG_UART_H
#define __DEBUG_UART_H

#include "stm32f10x.h"
#include <stdio.h>

void Debug_UART_Init(uint32_t baudrate);
void Debug_UART_SendString(char *str);
void Debug_UART_SendHex(uint8_t data);
int fputc(int ch, FILE *f);  /* 重定向printf */

#endif