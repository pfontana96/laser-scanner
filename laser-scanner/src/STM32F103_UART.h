/*
 * UART.h
 *
 *  Created on: 5 abr. 2020
 *      Author: Pedro Fontana
 */

#ifndef STM32F103_UART_H_
#define STM32F103_UART_H_

#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#define UART USART2

void UART2Init(const int baudrate, const uint8_t rxinterr);
void UART2ClockEnable(void);
void UART2GPIOEnable(void);
void UART2InterruptEnable(void);

void UART2putc(const char c);
int UART2puts(const char* str);
char UART2getc(void);


#endif /* STM32F103_UART_H_ */
