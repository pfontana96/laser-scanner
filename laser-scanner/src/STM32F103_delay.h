/*
 * STM32F1O3_delay.h
 *
 *  Created on: 20 abr. 2020
 *      Author: Pedro Fontana
 */

#ifndef STM32F103_DELAY_H_
#define STM32F103_DELAY_H_

#include "stm32f10x.h"

static __IO uint32_t us_ticks;
__IO uint32_t sysTick_Time;

void DelayInit(void);
void DelayUs(uint32_t us);
void DelayMs(uint32_t ms);
uint32_t getTime_us(void);
uint32_t getTime_ms(void);

#endif /* STM32F103_DELAY_H_ */
