/*
 * STM32F103_I2C.h
 *
 *  Created on: Jan 16, 2020
 *      Author: pedro
 */

#ifndef STM32F103_I2C_H_
#define STM32F103_I2C_H_

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#include "STM32F103_delay.h"
#include <STM32F103_UART.h>

#define I2Cx_RCC			RCC_APB1Periph_I2C2
#define I2Cx				I2C2
#define I2C_GPIO_RCC		RCC_APB2Periph_GPIOB
#define I2C_GPIO			GPIOB
#define I2C_PIN_SDA			GPIO_Pin_11
#define I2C_PIN_SCL			GPIO_Pin_10

// Error codes return by I2CWrite or I2CRead

#define I2C_BUS_ERROR 			(1<<0)
#define I2C_ARBITRATION_LOSS 	(1<<1)
#define I2C_ACKNOWLEDGE_FAILURE (1<<2)
#define I2C_OVERRUN_UNDERRUN	(1<<3)
#define I2C_PEC_ERROR			(1<<4)
#define I2C_TIMEOUT_TLOW_ERROR	(1<<5)
#define I2C_TIMEOUT_GENERAL		(1<<6)

#define I2C_TIMEOUT				100000   // timeout in us

#define WAIT_ON_TIMEOUT(condition) 							\
		({													\
			uint32_t _start_t = getTime_us();					\
			uint8_t _error = 0;								\
			while(!(condition))								\
			{												\
				if ((getTime_us() - _start_t) > I2C_TIMEOUT)	\
				{											\
					_error |= I2C_TIMEOUT_GENERAL;			\
					break;									\
				}											\
			}												\
			_error;											\
		})

/*		Private functions		*/

uint8_t _I2CCheckError(void);

/*		Public Functions		*/

void I2CEnablePeriphClock(void);

uint8_t I2CWrite(uint8_t address, uint8_t* data, uint8_t count);
uint8_t I2CRead(uint8_t address, uint8_t* data, uint8_t count);


#endif /* STM32F103_I2C_H_ */
