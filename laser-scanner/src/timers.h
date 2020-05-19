/*
 * timers.h
 *
 *  Created on: 1 abr. 2020
 *      Author: Pedro Fontana
 */

#ifndef TIMERS_H_
#define TIMERS_H_

#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "misc.h"


void timerInit(const uint8_t id, const int prescaler, const int period);
void timerInterruptEnable(const uint8_t id, const uint16_t);

#endif /* TIMERS_H_ */
