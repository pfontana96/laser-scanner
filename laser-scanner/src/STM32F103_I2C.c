/*
 * STM32F103_I2C.c
 *
 *  Created on: Jan 16, 2020
 *      Author: pedro
 */
#include "STM32F103_I2C.h"
#include <stdio.h>

#define I2C_DBG

#ifdef I2C_DBG

#include <string.h>

void sprintf_i2c_error(char* buf, uint8_t error)
{
	if(error & I2C_TIMEOUT_TLOW_ERROR)
		strcat(buf, "I2C_TIMEOUT_TLOW_ERROR\r\n");
	if(error & I2C_PEC_ERROR)
		strcat(buf, "I2C_PEC_ERROR\r\n");
	if(error & I2C_OVERRUN_UNDERRUN)
		strcat(buf, "I2C_OVERRUN_UNDERRUN\r\n");
	if(error & I2C_ACKNOWLEDGE_FAILURE)
		strcat(buf, "I2C_ACKNOWLEDGE_FAILURE\r\n");
	if(error & I2C_ARBITRATION_LOSS)
		strcat(buf, "I2C_ARBITRATION_LOSS\r\n");
	if(error & I2C_BUS_ERROR)
		strcat(buf, "I2C_BUS_ERROR\r\n");
}

#endif

char UART_dbg_buff[100];

void I2CEnablePeriphClock()
{
	// Initialization struct
	I2C_InitTypeDef I2C_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;

	// Step 1: Initialize I2C
	RCC_APB1PeriphClockCmd(I2Cx_RCC, ENABLE);

	// Reset the I2Cx Peripheral
	RCC_APB1PeriphResetCmd(I2Cx_RCC, ENABLE);
	RCC_APB1PeriphResetCmd(I2Cx_RCC, DISABLE);

	I2C_InitStruct.I2C_ClockSpeed = 400000;
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2Cx, &I2C_InitStruct);
	I2C_Cmd(I2Cx, ENABLE);

	// Step 2: Initialize GPIO as open drain alternate function
	RCC_APB2PeriphClockCmd(I2C_GPIO_RCC, ENABLE);
	GPIO_InitStruct.GPIO_Pin = I2C_PIN_SCL | I2C_PIN_SDA;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(I2C_GPIO, &GPIO_InitStruct);

	// Enable ERRORs interrupt
	I2C_ITConfig(I2Cx, I2C_IT_ERR, ENABLE);

//	DelayInit();
	UART2puts("I2C Initialised..\r\n");
}

uint8_t _I2CCheckError()
{
	uint8_t error = 0;
	if (I2Cx->SR1 & I2C_SR1_TIMEOUT){
		error |= I2C_TIMEOUT_TLOW_ERROR;
		I2Cx->SR1 &=~ I2C_SR1_TIMEOUT;
	}
	if (I2Cx->SR1 & I2C_SR1_PECERR){
		error |= I2C_PEC_ERROR;
		I2Cx->SR1 &=~ I2C_SR1_PECERR;
	}
	if (I2Cx->SR1 & I2C_SR1_OVR){
		error |= I2C_OVERRUN_UNDERRUN;
		I2Cx->SR1 &=~ I2C_SR1_OVR;
	}
	if (I2Cx->SR1 & I2C_SR1_AF){
		error |= I2C_ACKNOWLEDGE_FAILURE;
		I2Cx->SR1 &=~ I2C_SR1_AF;
	}
	if (I2Cx->SR1 & I2C_SR1_ARLO){
		error |= I2C_ARBITRATION_LOSS;
		I2Cx->SR1 &=~ I2C_SR1_ARLO;
	}
	if (I2Cx->SR1 & I2C_SR1_BERR){
		error |= I2C_BUS_ERROR;
		I2Cx->SR1 &=~ I2C_SR1_BERR;
	}
	return error;
}

uint8_t I2CWrite(uint8_t address, uint8_t* data, uint8_t count)
{
	// Write count bytes to I2Cx
	uint8_t i2c_error = 0;
	// Wait until I2Cx is not busy anymore
	if(WAIT_ON_TIMEOUT(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY) == RESET))
		goto error;
	// Generate start condition
	I2C_GenerateSTART(I2Cx, ENABLE);

	// Wait for I2C EV5.
	// It means that the start condition has been correctly released
	// on the I2C bus (the bus is free, no other devices is communicating))
	if(WAIT_ON_TIMEOUT(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT) == SUCCESS))
		goto error;

	// Send slave address
	I2C_Send7bitAddress(I2Cx, (address << 1), I2C_Direction_Transmitter);

	// Wait for I2C EV6
	// It means that a slave acknowledges his address
	if(WAIT_ON_TIMEOUT(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == SUCCESS))
		goto error;

	for(uint8_t i = 0; i < count; i++)
	{
		// Send data byte
		I2C_SendData(I2Cx, data[i]);
		// Wait for I2C EV8_2.
		// It means that the data has been physically shifted out and
		// output on the bus)
		if(WAIT_ON_TIMEOUT(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == SUCCESS))
			goto error;
	}

	// Generate I2C stop condition
	I2C_GenerateSTOP(I2Cx, ENABLE);

	// Wait until I2C stop condition has been sent
	if(WAIT_ON_TIMEOUT(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF) == RESET))
		goto error;

	return 0;

error:
	i2c_error |= (_I2CCheckError() | I2C_TIMEOUT_GENERAL);

#ifdef I2C_DBG
	UART2puts("ATTENTION!! ERRORS OCCURED DURING I2C_Write\r\n");
	sprintf(UART_dbg_buff, "I2C encountered errors:\r\n");
	sprintf_i2c_error(UART_dbg_buff, i2c_error);
	UART2puts(UART_dbg_buff);
#endif

	// Generate I2C stop condition
	I2C_GenerateSTOP(I2Cx, ENABLE);

	// Wait until I2C stop condition has been sent
	WAIT_ON_TIMEOUT(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF) == RESET);

	return i2c_error;
}



uint8_t I2CRead(uint8_t address, uint8_t* data, uint8_t count)
{
//	_I2CStart();
//	_I2CSendAddress(address << 1, I2C_Direction_Transmitter);
//	for(uint8_t i = 0; i < count; i++)
//	{
//		_I2CReceiveACK();
//		data[i] = I2C_ReceiveData(I2Cx);
//	}
//	_I2CReceiveNACK();
//	_I2CStop();

	// I2C_NACKPositionConfig

	// Read count bytes from I2Cx
	// As specified in reference RM0008 (p.763)
	uint8_t i2c_error = 0;
	// Wait until I2C is free
	if(WAIT_ON_TIMEOUT(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY) == RESET))
		goto error;

	// Generate start condition
	I2C_GenerateSTART(I2Cx, ENABLE);

	// Wait for I2C EV5.
	// It means that the start condition has been correctly released
	// on the I2C bus (the bus is free, no other devices is communicating))
	if(WAIT_ON_TIMEOUT(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT) == SUCCESS))
		goto error;

	// Send slave address (7-bit)
	I2C_Send7bitAddress(I2Cx, (address << 1), I2C_Direction_Receiver);

	// Wait for I2C EV6
	// It means that a slave acknowledges his address
	if(WAIT_ON_TIMEOUT(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == SUCCESS))
		goto error;

	switch(count)
	{
		case 0:
			break;

		case 1:
			// Only 1 byte to receive
			I2C_AcknowledgeConfig(I2Cx, DISABLE); // No acknowledge returned
			// Wait for I2C EV7
			// It means that the data has been received in I2C data register
			if(WAIT_ON_TIMEOUT(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) == SUCCESS))
				goto error;

			// Generate I2C stop condition
			I2C_GenerateSTOP(I2Cx, ENABLE);

			// Read I2Cx->DR register
			*data = I2C_ReceiveData(I2Cx);

			// Wait until I2C stop condition has been sent
			if(WAIT_ON_TIMEOUT(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF) == RESET))
				goto error;

			I2C_AcknowledgeConfig(I2Cx, ENABLE); 							// Acknowledge returned
			break;

		case 2:
			//	2 Bytes to receive
			// ACK bit controls the (N)ACK of the next byte which will be received in the shift register
			I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Next);
			I2C_AcknowledgeConfig(I2Cx, DISABLE); 							// No acknowledge returned

			// Wait for I2C EV7
			// It means that the data has been received in I2C data register
			if(WAIT_ON_TIMEOUT(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) == SUCCESS))
				goto error;

			// Generate I2C stop condition
			I2C_GenerateSTOP(I2Cx, ENABLE);

			// Read I2Cx->DR register (2 times)
			*data = I2C_ReceiveData(I2Cx);
			*(++data) = I2C_ReceiveData(I2Cx);

			// Wait until I2C stop condition has been sent
			if(WAIT_ON_TIMEOUT(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF) == RESET))
				goto error;

			I2C_AcknowledgeConfig(I2Cx, ENABLE); 							// Acknowledge returned

			break;

		default:
			// More than 2 bytes to receive
			while((count--) >= 3)
			{
				// Wait for I2C EV7
				// It means that the data has been received in I2C data register
				if(WAIT_ON_TIMEOUT(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) == SUCCESS))
					goto error;

				// Read I2Cx->DR register
				*(data++) = I2C_ReceiveData(I2Cx);
			}

			// Wait for I2C EV7
			// It means that the data has been received in I2C data register
			if(WAIT_ON_TIMEOUT(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) == SUCCESS))
				goto error;

			I2C_AcknowledgeConfig(I2Cx, DISABLE); 							// No acknowledge returned

			// Read I2Cx->DR register (second last byte)
			*(data++) = I2C_ReceiveData(I2Cx);

			// Wait for I2C EV7
			// It means that the data has been received in I2C data register
			if(WAIT_ON_TIMEOUT(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) == SUCCESS))
				goto error;

			// Generate I2C stop condition
			I2C_GenerateSTOP(I2Cx, ENABLE);

			// Read I2Cx->DR register (last byte)
			*data = I2C_ReceiveData(I2Cx);

			// Wait until I2C stop condition has been sent
			if(WAIT_ON_TIMEOUT(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF) == RESET))
				goto error;

			I2C_AcknowledgeConfig(I2Cx, ENABLE); 							// Acknowledge returned

			break;
	}

	return 0; 														// No error (i2c_error = 0)

error:
	i2c_error |= (_I2CCheckError() | I2C_TIMEOUT_GENERAL);

#ifdef I2C_DBG
	UART2puts("ATTENTION!! ERRORS OCCURED DURING I2C_Read\r\n");
	sprintf(UART_dbg_buff, "I2C encountered errors:\r\n");
	sprintf_i2c_error(UART_dbg_buff, i2c_error);
	UART2puts(UART_dbg_buff);
#endif

	// Generate I2C stop condition
	I2C_GenerateSTOP(I2Cx, ENABLE);

	// Wait until I2C stop condition has been sent
	WAIT_ON_TIMEOUT(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF) == RESET);

	return i2c_error;
}
