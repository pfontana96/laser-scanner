/*
******************************************************************************
File:     main.c
Info:     Generated by Atollic TrueSTUDIO(R) 7.1.2   2020-04-03

The MIT License (MIT)
Copyright (c) 2009-2017 Atollic AB

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

******************************************************************************
*/

/* Includes */
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>


#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include <STM32F103_UART.h>
#include "STM32F103_I2C.h"
#include "STM32F103_PWM.h"
#include "STM32F103_VL53L0X.h"
#include "timers.h"
#include "SG90.h"





/* Private typedef */
typedef enum STATE {IDLE, MEASURING, INIT, FINISHED}state_t;

/* Private define  */
//#define CHRONO       // Measure elapsed time in sending measurement (use to calculate sampling rate)

#define NB_OF_MEASURES 5

//#define DEBUG_LOG

#define VERSION_REQUIRED_MAJOR 1
#define VERSION_REQUIRED_MINOR 0
#define VERSION_REQUIRED_BUILD 2

#define PI 3.14159265


/* Private macro */
/* Private variables */
char UART_buffer[200];
volatile float angle_base, delta_base = 2.0f, angle_top, delta_top = 2.0f;
const float angle_base_init = -70.0f, angle_top_init = -60.0f;
state_t state = IDLE;

/* Private function prototypes */
/* Private functions */
void initLED(void);
void toggleLED(void);
void updateAngles(void);

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/

int main(void)
{
  /**
  *  IMPORTANT NOTE!
  *  The symbol VECT_TAB_SRAM needs to be defined when building the project
  *  if code has been located to RAM and interrupts are used. 
  *  Otherwise the interrupt table located in flash will be used.
  *  See also the <system_*.c> file and how the SystemInit() function updates 
  *  SCB->VTOR register.  
  *  E.g.  SCB->VTOR = 0x20000000;  
  */

  /* TODO - Add your application code here */

  SystemInit();
  SystemCoreClockUpdate();

  // Configure the SysTick timer to overflow every 1 ms
#ifdef USE_OTHER_I2C
  SysTick_Config(SystemCoreClock / 1000);
#else
  // Configure the SysTick timer to overflow every 1 us
  SysTick_Config(SystemCoreClock / 1000000);
#endif
  initLED();

  ServosInit();
  UART2Init(9600, 0);
  UART2InterruptEnable();

  UART2puts("UART2 Initialised..\r\n");

  I2CEnablePeriphClock();

  UART2puts("I2C Initialised..\r\n");

  /*	Official ST API		*/

  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  VL53L0X_Dev_t MyDevice;
  VL53L0X_Dev_t *pMyDevice = &MyDevice;
  VL53L0X_Version_t                   Version;
  VL53L0X_Version_t                  *pVersion   = &Version;

  int32_t status_int;

// Initialize Comms
  pMyDevice->I2cDevAddr      = 0b0101001;
  pMyDevice->comms_type      =  1;
  pMyDevice->comms_speed_khz =  400;

  /*
   * Disable VL53L0X API logging if you want to run at full speed
   */
#ifdef VL53L0X_LOG_ENABLE
  VL53L0X_trace_config("test.log", TRACE_MODULE_ALL, TRACE_LEVEL_ALL, TRACE_FUNCTION_ALL);
#endif

  /*
   *  Get the version of the VL53L0X API running in the firmware
   */

  if(Status == VL53L0X_ERROR_NONE)
  {
	  status_int = VL53L0X_GetVersion(pVersion);
	  if (status_int != 0)
		  Status = VL53L0X_ERROR_CONTROL_INTERFACE;
  }

  /*
   *  Verify the version of the VL53L0X API running in the firmrware
   */

  if(Status == VL53L0X_ERROR_NONE)
  {
	  if( pVersion->major != VERSION_REQUIRED_MAJOR ||
		  pVersion->minor != VERSION_REQUIRED_MINOR ||
		  pVersion->build != VERSION_REQUIRED_BUILD )
	  {
#ifdef DEBUG_LOG
		  printf("VL53L0X API Version Error: Your firmware has %d.%d.%d (revision %d). This example requires %d.%d.%d.\r\n",
			  (int) pVersion->major, (int) pVersion->minor, (int) pVersion->build, (int) pVersion->revision,
			  VERSION_REQUIRED_MAJOR, VERSION_REQUIRED_MINOR, VERSION_REQUIRED_BUILD);
#endif
	  }
  }


  /*	Own implementation		*/
#ifdef DEBUG_LOG
  UART2puts("Call of VL53L0XInit...\r\n");
#endif

  if(Status == VL53L0X_ERROR_NONE)
  {
//	  Status = VL53L0XInit(pMyDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
	  Status = VL53L0XInit(pMyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING);
	  print_pal_error(Status);
  }

#ifdef CHRONO
  uint32_t start_t, end_t;
#endif

//  uint32_t n = NB_OF_MEASURES;
//  uint16_t measures[n], avg;
  uint16_t data;
  float x, y, z;

  state = IDLE;

  while(true)
  {
	  switch(state)
	  {
	  	  case MEASURING:

#ifdef CHRONO
	  		 start_t = getTime_ms();
#endif

	  		  updateAngles();
	  		  ServoSetPos(SERVO_BASE_ID, angle_base);
//	  		  DelayMs(1);
	  		  ServoSetPos(SERVO_TOP_ID, angle_top);

//	  		  sprintf(UART_buffer, "top: %f | base: %f\r\n", angle_top, angle_base);
//			  UART2puts(UART_buffer);

			  Status = VL53L0XGetSingleMeasure(pMyDevice, &data);
//			  data = 1000;
			  // VL53L0X_PollingDelay(pMyDevice);

			  if(Status == VL53L0X_ERROR_NONE)
			  {
#ifdef DEBUG_LOG
				  sprintf(UART_buffer, "measure = %d mm\r\n", data);
				  UART2puts(UART_buffer);
#endif
//				  if(data == 65535)
//					  data = 1300; // max measurable distance

				  // Message trame MX#Y#Z
				  float angle_top_rad = (angle_top + 90.0f)*(PI/180.0f);
				  float angle_base_rad = (angle_base + 90.0f)*(PI/180.0f);

				  x = (data * sin(angle_top_rad) * cos(angle_base_rad)) / 10; 	// Coordinate X in cm
				  y = (data * sin(angle_top_rad) * sin(angle_base_rad)) / 10; 	// Coordinate Y in cm
				  z = (data * cos(angle_top_rad)) / 10;			       		 	// Coordinate Z in cm

				  sprintf(UART_buffer, "M%d#%d#%d\n", (int) x, (int) y, (int) z);
				  UART2puts(UART_buffer);
#ifdef CHRONO
				  end_t = getTime_ms();
				  sprintf(UART_buffer, "Elapsed time: %d ms\r\n", (int) (end_t - start_t));
				  UART2puts(UART_buffer);
#endif
			  }
			  else
			  {
				  Status = VL53L0X_ERROR_NONE;
			  }

//			  DelayMs(2);
			  break;

	  	  case INIT:

	  		  angle_top = angle_top_init;
	  		  ServoSetPos(SERVO_TOP_ID, angle_top);
	  		  angle_base = angle_base_init;
	  		  ServoSetPos(SERVO_BASE_ID, angle_base);
	  		  state = MEASURING;
	  		  break;

	  	  case FINISHED:

	  		  UART2putc('E');
	  		  state = IDLE;
	  		  break;

	  	  case IDLE:
	  		  // Do nothing
	  		  break;
	  }
  }

  return Status;

}

void USART2_IRQHandler()
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE))
	{
		char data = USART2->DR;

		switch(data)
		{
			case 's':
				// Start measuring condition
				state = INIT;
				break;
			case 'p':
				// Pause measuring condition
				state = IDLE;
				break;
			case 'r':
				// Reanude condition
				state = MEASURING;
				break;
			default:
				break;
		}
	}
}

void updateAngles()
{
	angle_base = angle_base + delta_base;
	if(abs(angle_base) > abs(angle_base_init))
	{
		delta_base = - delta_base;
		angle_base = angle_base + delta_base;

		angle_top = angle_top + delta_top;

		if(angle_top >= abs(angle_top_init))
			state = FINISHED; // We've reached the end of the measurement
	}
}

void initLED()
{
	GPIO_InitTypeDef GPIO_init_struct;
	// Enable clock for GPIOC

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	// Configure PC13 (LED) as push-pull output
	GPIO_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_init_struct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_init_struct.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIO_init_struct);

}

void toggleLED()
{
	// If led is set we reset it, we set it otherwise
	if(GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13))
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	}
	else
	{
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
	}
}


/*
 * Minimal __assert_func used by the assert() macro
 * */
void __assert_func(const char *file, int line, const char *func, const char *failedexpr)
{
  while(1)
  {}
}

/*
 * Minimal __assert() uses __assert__func()
 * */
void __assert(const char *file, int line, const char *failedexpr)
{
   __assert_func (file, line, NULL, failedexpr);
}

