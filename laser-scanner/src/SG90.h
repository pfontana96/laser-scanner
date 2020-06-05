/*
 * PWM.h
 *
 *  Created on: 15 abr. 2020
 *      Author: Pedro Fontana
 */

#ifndef SG90_H_
#define SG90_H_


#include <stdint.h>
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "timers.h"
#include "misc.h"

/* Servo SG-90*/
/* Period: 20 ms
 * Duty cycle: (1-2) ms
 * 	0° at 1.5 ms
 * 	90° at 2 ms
 * 	-90° at 1 ms
 **/

#define SERVO_PERIOD 20         // [ms]
#define SERVO_MAX_DUTY 2.5  	// [ms]
#define SERVO_MIN_DUTY 0.5		// [ms]

#define SERVO_TIMER             TIM3
#define SERVO_TIMER_ID			3
#define SERVO_TIMER_CLK         RCC_APB1Periph_TIM3

/*~~~~~~~~~~~~~~~~ Servo 1: Base ~~~~~~~~~~~~~~~~*/
#define SERVO1_Pin               GPIO_Pin_6
#define SERVO1_GPIO_PORT         GPIOA
#define SERVO1_GPIO_CLK          RCC_APB2Periph_GPIOA
#define SERVO_BASE_ID			 ((uint8_t) 1 )

/*~~~~~~~~~~~~~~~~ Servo 2: Top  ~~~~~~~~~~~~~~~~*/
#define SERVO2_Pin               GPIO_Pin_7
#define SERVO2_GPIO_PORT         GPIOA
#define SERVO2_GPIO_CLK          RCC_APB2Periph_GPIOA
#define SERVO_TOP_ID			 ((uint8_t) 2 )

/* Public Functions */
void ServosInit(void);
void ServoSetPos(const uint8_t id, const float angle);

/* Private Functions */
void ServosClockEnable(void);
void ServosGPIOEnable(void);
void ServosInterruptEnable(void);
void PWMInit(void);


#endif /* SG90_H_ */
