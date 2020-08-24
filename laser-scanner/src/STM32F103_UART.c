/*
 * UART.c
 *
 *  Created on: 5 abr. 2020
 *      Author: Pedro Fontana
 */

#include <STM32F103_UART.h>

// UART2 Init
void UART2Init(const int baudrate, const uint8_t rxinterr)
{
	UART2ClockEnable();
	UART2GPIOEnable();

	USART_InitTypeDef USART_init_struct;
	USART_init_struct.USART_BaudRate = baudrate;
	USART_init_struct.USART_WordLength = USART_WordLength_8b;
	USART_init_struct.USART_StopBits = USART_StopBits_1;
	USART_init_struct.USART_Parity = USART_Parity_No;
	USART_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_init_struct);

	if (rxinterr)
	{
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
		UART2InterruptEnable();
	}

	USART_Cmd(USART2, ENABLE);
}

void UART2ClockEnable()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
}

void UART2GPIOEnable()
{
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	GPIO_InitTypeDef GPIO_init_struct;
	GPIO_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_init_struct.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_init_struct);
	GPIO_init_struct.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_init_struct);
}

void UART2InterruptEnable()
{
	NVIC_InitTypeDef NVIC_init_struct;
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // enable the USART2 receive interrupt
	NVIC_init_struct.NVIC_IRQChannel = USART2_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_init_struct.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_init_struct.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_init_struct.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_init_struct);
}

void UART2putc(const char c)
{
	USART_SendData(USART2, (uint8_t) c);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}

int UART2puts(const char* str)
{
	int i = 0;
	while(*str != 0)
	{
		UART2putc(*str++);
		i++;
	}
	return i;
}

char UART2getc()
{
	while(USART_GetFlagStatus(USART2, USART_IT_RXNE)== RESET);
	char c = USART2->DR;
	return c;
}

