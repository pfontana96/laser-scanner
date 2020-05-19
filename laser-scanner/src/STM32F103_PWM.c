/*
 * STM32F103_PWM.c
 *
 *  Created on: 28 abr. 2020
 *      Author: Pedro Fontana
 */

#include <stdint.h>
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "STM32F103_PWM.h"
#include "misc.h"

/*  Escribir Duty */
uint16_t PWM_Duty(uint8_t canal, uint16_t Duty)
{
	switch (canal)
	{
	case 1:
		TIM3->CCR1 =  Duty;
		break;
	case 2:
		TIM3->CCR2 =  Duty;
		break;
	case 3:
		TIM3->CCR3 =  Duty;
		break;
	case 4:
		TIM3->CCR4 =  Duty;
		break;
	default:
		return -1;
		break;
	}
	return Duty;
}

uint16_t PWM_Period(uint16_t Periodo)
{
		TIM3->ARR = Periodo;
		return Periodo;
}

uint16_t PWM_Prescaler(uint16_t Prescaler)
{
		TIM3->PSC = Prescaler;
		return Prescaler;
}

uint16_t PWM_Init(uint8_t canal, uint16_t iniduty, uint8_t polaridad)
{
	/* Mapeo de pines de PWM1 y PWM2 */
	// -------------QEI1
//GPIO_InitTypeDef GPIO_InitStructure;				// Estructuras paa inicializar pines
//NVIC_InitTypeDef NVIC_InitStructure;  			// e interrupciones del QEI

GPIO_InitTypeDef GPIO_InitStructure;
TIM_OCInitTypeDef PWM_Canal_Init;// = {0,};
PWM_Canal_Init.TIM_OCMode = TIM_OCMode_PWM1;
PWM_Canal_Init.TIM_Pulse = iniduty;
PWM_Canal_Init.TIM_OutputState = TIM_OutputState_Enable;
if (polaridad==0)
PWM_Canal_Init.TIM_OCPolarity = TIM_OCPolarity_Low;
else
PWM_Canal_Init.TIM_OCPolarity = TIM_OCPolarity_High;

GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;

switch (canal)
	{
	case 1:
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		TIM_OC1Init(TIM3, &PWM_Canal_Init);
		TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
//		GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
		break;
	case 2:
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		TIM_OC2Init(TIM3, &PWM_Canal_Init);
		TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
//		GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
		break;
	case 3:
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		TIM_OC3Init(TIM3, &PWM_Canal_Init);
		TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
//		GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
		break;
	case 4:
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		TIM_OC4Init(TIM3, &PWM_Canal_Init);
		TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
//		GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
		break;
	default:
		return -1;
		break;
	}
return iniduty;
}

