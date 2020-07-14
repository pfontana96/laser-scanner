/*
 * PWM.c
 *
 *  Created on: 15 abr. 2020
 *      Author: Pedro Fontana
 */
#include "SG90.h"
//#define LOG_DBG

#ifdef LOG_DBG
#include "STM32F103_UART.h"
#include "stdio.h"
char UART_dbg_buff[100];
#endif

void initPWM(uint8_t channel);

uint16_t PWM_period = 10000;

void ServoSetPos(const uint8_t id, const float angle)
{
	// Angle to duty-cycle conversion
	// -90° --> 1 ms (1000 Hz)
	// 0° ----> 1.5 ms (666.67 Hz)
	// 90° ---> 2 ms (500 Hz)
	// Timer's clock is working at 180000 Hz
	// duty = ((y2 - y1)/(x2-x1))*(x - x1) + y1
	// duty_ms = ((2ms - 1.5ms)/(90° - 0°))*(angle - 0°) + 1.5ms   and 180 kHz = 180 cycles/ms
	// duty_cycles = 180*duty_ms
//	float duty_ms = (0.5f/90)*angle + 1.5f;
	float duty_ms = (1.1f/90)*angle + 1.5f;
	if(duty_ms > SERVO_MAX_DUTY)
		duty_ms = SERVO_MAX_DUTY;
	else if(duty_ms < SERVO_MIN_DUTY)
		duty_ms = SERVO_MIN_DUTY;
	// Timer's clock is working at 180000 Hz
	uint16_t duty_cycles = (uint16_t) (duty_ms*180);
#ifdef LOG_DBG
	sprintf(UART_dbg_buff, "angle: %f | duty_ms: %f | duty_cy: %d \r\n", angle, duty_ms, duty_cycles);
	UART2puts(UART_dbg_buff);
#endif
	switch(id)
	{
		case SERVO_BASE_ID:
//			UART2puts("Servo 1 PWM changed\r\n");
			SERVO_TIMER->CCR1 = duty_cycles;
			break;
		case SERVO_TOP_ID:
			SERVO_TIMER->CCR2 = duty_cycles;
			break;
	}

}

void ServosInit()
{
	timerInit(SERVO_TIMER_ID, 400, 3600); // 72 MHz / 400 = 180000 Hz / 3600 = 50 Hz (0.02 s period) // Datasheet says max frequency for APB1 is 36 MHz but TIM2 works as expected when considerating 72 MHz as frequency
//	timerInit(3, 7200, PWM_period);
	ServosGPIOEnable();
	PWMInit();

}

void ServosGPIOEnable()
{
	GPIO_InitTypeDef GPIO_init_struct;
	GPIO_init_struct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
	// Servo 1
	GPIO_init_struct.GPIO_Pin = SERVO1_Pin;
	GPIO_Init(SERVO1_GPIO_PORT, &GPIO_init_struct);
	// Servo 2
	GPIO_init_struct.GPIO_Pin = SERVO2_Pin;
	GPIO_Init(SERVO2_GPIO_PORT, &GPIO_init_struct);
}

void ServosClockEnable()
{
	RCC_APB2PeriphClockCmd(SERVO1_GPIO_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(SERVO2_GPIO_CLK, ENABLE);
}

void PWMInit()
{

	initPWM(SERVO_BASE_ID);
	initPWM(SERVO_TOP_ID);

//	GPIO_InitTypeDef GPIO_InitStructure;
//	TIM_OCInitTypeDef PWM_Canal_Init;// = {0,};
//	PWM_Canal_Init.TIM_OCMode = TIM_OCMode_PWM1;
//	PWM_Canal_Init.TIM_Pulse = 270;
//	PWM_Canal_Init.TIM_OutputState = TIM_OutputState_Enable;
//	PWM_Canal_Init.TIM_OCPolarity = TIM_OCPolarity_High;
//
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//
//	RCC_APB2PeriphClockCmd(SERVO1_GPIO_CLK, ENABLE);
//	GPIO_InitStructure.GPIO_Pin = SERVO1_Pin;
//	GPIO_Init(SERVO1_GPIO_PORT, &GPIO_InitStructure);
//
//	TIM_OC1Init(SERVO_TIMER, &PWM_Canal_Init);
//	TIM_OC1PreloadConfig(SERVO_TIMER, TIM_OCPreload_Enable);
//
//	RCC_APB2PeriphClockCmd(SERVO2_GPIO_CLK, ENABLE);
//	GPIO_InitStructure.GPIO_Pin = SERVO2_Pin;
//	GPIO_Init(SERVO2_GPIO_PORT, &GPIO_InitStructure);
//
//	TIM_OC2Init(SERVO_TIMER, &PWM_Canal_Init);
//	TIM_OC2PreloadConfig(SERVO_TIMER, TIM_OCPreload_Enable);
}

void initPWM(uint8_t channel)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_OCInitTypeDef PWM_Canal_Init;// = {0,};
	PWM_Canal_Init.TIM_OCMode = TIM_OCMode_PWM1;
	PWM_Canal_Init.TIM_Pulse = 270;
	PWM_Canal_Init.TIM_OutputState = TIM_OutputState_Enable;
	PWM_Canal_Init.TIM_OCPolarity = TIM_OCPolarity_High;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	switch(channel)
	{
		case SERVO_BASE_ID:

			RCC_APB2PeriphClockCmd(SERVO1_GPIO_CLK, ENABLE);
			GPIO_InitStructure.GPIO_Pin = SERVO1_Pin;
			GPIO_Init(SERVO1_GPIO_PORT, &GPIO_InitStructure);

			TIM_OC1Init(SERVO_TIMER, &PWM_Canal_Init);
			TIM_OC1PreloadConfig(SERVO_TIMER, TIM_OCPreload_Enable);
			break;

		case SERVO_TOP_ID:

			RCC_APB2PeriphClockCmd(SERVO2_GPIO_CLK, ENABLE);
			GPIO_InitStructure.GPIO_Pin = SERVO2_Pin;
			GPIO_Init(SERVO2_GPIO_PORT, &GPIO_InitStructure);

			TIM_OC2Init(SERVO_TIMER, &PWM_Canal_Init);
			TIM_OC2PreloadConfig(SERVO_TIMER, TIM_OCPreload_Enable);
			break;
	}
}

