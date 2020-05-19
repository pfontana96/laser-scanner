/*
 * timers.c
 *
 *  Created on: 1 abr. 2020
 *      Author: Pedro Fontana
 */

#include "timers.h"

void timerInit(const uint8_t id, const int prescaler, const int period)
{
	TIM_TimeBaseInitTypeDef timer_init_structure;

	switch(id)
	{
		case 1:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
			timer_init_structure.TIM_Prescaler = prescaler;
			timer_init_structure.TIM_CounterMode = TIM_CounterMode_Up;
			timer_init_structure.TIM_Period = period;
			timer_init_structure.TIM_ClockDivision = TIM_CKD_DIV1;
			timer_init_structure.TIM_RepetitionCounter = 0;
			TIM_TimeBaseInit(TIM1, &timer_init_structure);
			TIM_Cmd(TIM1, ENABLE);
			break;
		case 2:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
			timer_init_structure.TIM_Prescaler = prescaler;
			timer_init_structure.TIM_CounterMode = TIM_CounterMode_Up;
			timer_init_structure.TIM_Period = period;
			timer_init_structure.TIM_ClockDivision = TIM_CKD_DIV1;
			timer_init_structure.TIM_RepetitionCounter = 0;
			TIM_TimeBaseInit(TIM2, &timer_init_structure);
			TIM_Cmd(TIM2, ENABLE);
			break;
		case 3:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
			timer_init_structure.TIM_Prescaler = prescaler;
			timer_init_structure.TIM_CounterMode = TIM_CounterMode_Up;
			timer_init_structure.TIM_Period = period;
			timer_init_structure.TIM_ClockDivision = TIM_CKD_DIV1;
			timer_init_structure.TIM_RepetitionCounter = 0;
			TIM_TimeBaseInit(TIM3, &timer_init_structure);
			TIM_Cmd(TIM3, ENABLE);
			break;
		case 4:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
			timer_init_structure.TIM_Prescaler = prescaler;
			timer_init_structure.TIM_CounterMode = TIM_CounterMode_Up;
			timer_init_structure.TIM_Period = period;
			timer_init_structure.TIM_ClockDivision = TIM_CKD_DIV1;
			timer_init_structure.TIM_RepetitionCounter = 0;
			TIM_TimeBaseInit(TIM4, &timer_init_structure);
			TIM_Cmd(TIM4, ENABLE);
			break;
		default:
			break;
//			printf("Not valid timer\r\n");
	}
}

void timerInterruptEnable(const uint8_t id, const uint16_t event)
{
	NVIC_InitTypeDef nvic_structure;
	switch(id)
	{
		case 1:
			nvic_structure.NVIC_IRQChannel = TIM1_UP_IRQn; //TIM1 update interrupt
			TIM_ITConfig(TIM1, event, ENABLE);
			break;
		case 2:
			nvic_structure.NVIC_IRQChannel = TIM2_IRQn;
			TIM_ITConfig(TIM2, event, ENABLE);
			break;
		case 3:
			nvic_structure.NVIC_IRQChannel = TIM3_IRQn;
			TIM_ITConfig(TIM3, event, ENABLE);
			break;
		case 4:
			nvic_structure.NVIC_IRQChannel = TIM4_IRQn;
			TIM_ITConfig(TIM4, event, ENABLE);
			break;
		default:
			break;
	}
	nvic_structure.NVIC_IRQChannelPreemptionPriority = 0;
	nvic_structure.NVIC_IRQChannelSubPriority = 0;
	nvic_structure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_structure);

}

