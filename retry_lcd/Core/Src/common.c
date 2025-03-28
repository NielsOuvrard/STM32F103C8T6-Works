/*
 * common.c
 *
 *  Created on: Feb 24, 2025
 *      Author: User123
 */

#include "common.h"

#define COMMON_US_FACTOR ((SystemCoreClock + 5000000UL - 1) / (10UL*1000000UL))

void delay_ms(uint32_t ms)
{
	// Declared as volatile to avoid compiler optimization
	volatile uint32_t cycles = 0;
	while(ms--)
	{
		/*
		 * This for loop takes 10 cycles per iteration
		 * The outer while takes 4 cycles per iteration
		 * Ideally, for a delay of 1ms we need a for loop
		 * from 0 to Fclk / 1000 clock cycles if we assume
		 * that each iteration takes 1 clock cycle.
		 *
		 * If Fclk = 8MHz -> 8MHz/1000 = 8000
		 *
		 * However, at low-level, each iteration of the for
		 * loop takes around 10 clock cycles, therefore
		 * instead of iterating up to Fclk / 1000, it
		 * should be Fclk / (1000*CYCLES_PER_ITER), i.e.,
		 * 8MHz/(1000*10) = 800
		 */
		for (cycles = 0; cycles < SystemCoreClock/(1000*10); cycles++)
		{
		}
	}
}

void delay_10us(uint32_t usx10)
{
	while(usx10--)
	{
		for(volatile uint32_t i = 0; i < SystemCoreClock/(100000*10); i++)
		{
		}
	}
}

void delay_us(uint32_t us)
{
#if (COMMON_US_FACTOR==1)
	/*
	 * This for loop takes 10 cycles per iteration
	 * The outer while takes 4 cycles per iteration
	 * Ideally, for a delay of 1ms we need a for loop
	 * from 0 to Fclk / 1000 clock cycles if we assume
	 * that each iteration takes 1 clock cycle.
	 *
	 * If Fclk = 8MHz -> 8MHz/1000 = 8000
	 *
	 * However, at low-level, each iteration of the for
	 * loop takes around 10 clock cycles, therefore
	 * instead of iterating up to Fclk / 1000, it
	 * should be Fclk / (1000*CYCLES_PER_ITER), i.e.,
	 * 8MHz/(1000*10) = 800
	 */
	for (volatile unsigned int cycles = 0; cycles < us; cycles++)
	{
	}
#else
	while(us--)
	{
		for (volatile unsigned int cycles = 0; cycles < COMMON_US_FACTOR; cycles++)
		{
		}
	}
#endif
}
