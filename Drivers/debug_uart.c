#include "debug_uart.h"

void Debug_UART_Init(uint32_t baudrate) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    
    /* PA9(TX) 复用推挽输出 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* PA10(RX) 浮空输入 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);
    
    USART_Cmd(USART1, ENABLE);
}

void Debug_UART_SendString(char *str) {
    while (*str) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, *str++);
    }
}

void Debug_UART_SendArr(uint8_t *arr, size_t num) {
		size_t i = 0;
		for(i=0;i<num;i++)
		{
				while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
				USART_SendData(USART1, arr[i]);
		}

}

void Debug_UART_SendHex(uint8_t data) {
    char hex[3];
    hex[0] = "0123456789ABCDEF"[data >> 4];
    hex[1] = "0123456789ABCDEF"[data & 0x0F];
    hex[2] = '\0';
    Debug_UART_SendString(hex);
}

/* printf重定向 */
int fputc(int ch, FILE *f) {
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, (uint8_t)ch);
    return ch;
}