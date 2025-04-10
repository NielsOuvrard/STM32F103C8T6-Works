/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "clock_control.h"
#include "stm32f103xb.h"


// PA0 -> PZC -> input
// PA1 -> Po -> output 50

int main(void)
{
	// Enable GPIO A & C clock
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    // for the clock
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	GPIOC->CRH &= ~(0xF << 20); // Clear CNF13 & MODE13
    GPIOC->CRH |= (0x01 << 20); // output mode, max speed 10 MHz

	set_clk_speed(CLOCK_72MHz, true);

    GPIOA->CRL &= ~(0xFF << 0); // Clear CNF0 & MODE0
    GPIOA->CRL |= (0x00 << 0); // input mode, analog mode
    GPIOA->CRL |= (0x11 << 4); // output mode, max speed 50 MHz

	while (1)
	{
		GPIOC->ODR ^= (1 << 13);
		delay_ms(1000);
	}
}
