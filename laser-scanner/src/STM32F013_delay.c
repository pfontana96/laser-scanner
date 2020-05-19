/*
 * STM32F013_delay.c
 *
 *  Created on: 20 abr. 2020
 *      Author: Pedro Fontana
 */

#include "STM32F103_delay.h"

__IO uint32_t sysTick_Time = 0;

void DelayInit()
{
	// Configure the SysTick timer to overflow every 1 us
	SysTick_Config(SystemCoreClock / 1000000);
}

// SysTick_Handler function will be called every 1 us
void SysTick_Handler()
{
	  sysTick_Time++;
    if (us_ticks != 0)
        us_ticks--;
}

void DelayUs(uint32_t us)
{
    // Reload us value
    us_ticks = us;
    // Wait until usTick reach zero
    while (us_ticks);
}

void DelayMs(uint32_t ms)
{
    // Wait until ms reach zero
    while (ms--)
        DelayUs(1000);
}

uint32_t getTime_us()
{
	// Returns Time in us (defined by SysTick_Config())
	return sysTick_Time;
}

uint32_t getTime_ms()
{
	// Returns Time in ms (defined by SysTick_Config())
	return (uint32_t) (sysTick_Time / 1000);
}
