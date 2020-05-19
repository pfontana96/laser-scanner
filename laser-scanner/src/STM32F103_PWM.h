/*
 * STM32F103_PWM.h
 *
 *  Created on: 28 abr. 2020
 *      Author: Pedro Fontana
 */

#ifndef STM32F103_PWM_H_
#define STM32F103_PWM_H_
/* Mapeo de pines de PWM1 y PWM2 */
// -------------PWM1
#define PWM1_PIN               GPIO_Pin_6
#define PWM1_GPIO_PORT         GPIOA
#define PWM1_GPIO_CLK          RCC_APB2Periph_GPIOA
#define PWM1_SOURCE            GPIO_PinSource6

// -------------PWM2
#define PWM2_PIN               GPIO_Pin_7
#define PWM2_GPIO_PORT         GPIOA
#define PWM2_GPIO_CLK          RCC_APB2Periph_GPIOA
#define PWM2_SOURCE            GPIO_PinSource7

#define PWM1_TIMER              TIM3
#define PWM2_TIMER              TIM3

#define PWM1_TIMER_CLK          RCC_APB1Periph_TIM3
#define PWM2_TIMER_CLK          RCC_APB1Periph_TIM3



uint16_t PWM_Init(uint8_t, uint16_t, uint8_t);
uint16_t PWM_Duty(uint8_t, uint16_t);
uint16_t PWM_Period(uint16_t);
uint16_t PWM_Prescaler(uint16_t);


#endif /* STM32F103_PWM_H_ */
