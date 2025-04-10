/*
 * clock_configuration.c
 *
 *  Created on: Feb 23, 2025
 *      Author: nielsouvrard
 */

#include "clock_configuration.h"

void configure_PLL(spped_clock speed, bool is_HSE)
{
	if (speed == CLOCK_8MHz) {
		return;
	}
	RCC->CFGR &= ~RCC_CFGR_PLLSRC;  // Clear PLL source bits
	RCC->CFGR &= ~RCC_CFGR_PLLMULL;  // Clear PLL multiplier bits
	if (is_HSE) {
		RCC->CFGR &= ~RCC_CFGR_PLLXTPRE;  // Clear HSE divider
		RCC->CFGR |= RCC_CFGR_PLLSRC;
		if (speed % 2) {
			// if speed is odd, divide by 2 the HSE
			RCC->CFGR |= (0x01 << 17); // PLLXTPRE on so we divide by 2 the HSE
		} else {
			speed = speed / 2 - 1; // 8MHz begin so / 2, and -1 because PLLMUL begins at 2
		}
		RCC->CFGR |= (speed << 18); // PLLMUL
	} else {
		// PLLSRC = 0, HSI by default
		RCC->CFGR |= (speed << 18); // PLLMUL
	}
}

void set_clk_speed(spped_clock speed, bool is_HSE)
{
	// Step 1: Enable HSE or HSI
	if (is_HSE) {
		RCC->CR &= ~RCC_CR_HSION; // HSI OFF
		RCC->CR |= RCC_CR_HSEON;
		while (!(RCC->CR & RCC_CR_HSERDY)); // HSERDY flag polling
	} else {
		RCC->CR &= ~RCC_CR_HSEON; // HSE OFF
		RCC->CR |= RCC_CR_HSION;
		while (!(RCC->CR & (0x01 << 1))); // HSIRDY flag polling
	}

	// Step 2: Configure PLL
	configure_PLL(speed, is_HSE);
	
	// Step 3: Enable PLL
	if (speed == CLOCK_8MHz) {
		RCC->CR &= ~RCC_CR_PLLON; // PLL OFF
		while (RCC->CR & RCC_CR_PLLRDY); // Polling PLLRDY flag
	} else {
		RCC->CR |= RCC_CR_PLLON; // PLL ON
		while (!(RCC->CR & RCC_CR_PLLRDY)); // Polling PLLRDY flag
	}
	
	// Step 4: Configure Flash
	if (speed <= CLOCK_24MHz) {
		FLASH->ACR &= ~(0x07); // 0 wait states
		FLASH->ACR &= ~(0x01 << 4); // Disable pre-fetch
		while (FLASH->ACR & (0x00 << 5)); // Polling for pre-fetch buffer disable
	} else {
		if (speed <= CLOCK_48MHz) {
			FLASH->ACR |= (0x01); // 1 wait states
		} else {
			FLASH->ACR |= (0x02); // 2 wait states
		}
		FLASH->ACR |= (0x01 << 4); // Enable pre-fetch
		while (!(FLASH->ACR & (0x01 << 5)));// Polling for pre-fetch buffer enable
	}

	if (speed != CLOCK_8MHz) {
		// Step 5: Select PLL as system clock
		RCC->CFGR |= 0x02; // SW = 2, PLL
		while (!(RCC->CFGR & (0x02 << 2))); // Polling SW status
	} else if (is_HSE) {
		RCC->CFGR |= 0x01; // SW = 1, HSE
		while (!(RCC->CFGR & (0x01 << 2))); // Polling SW status
	} else {
		RCC->CFGR &= ~(0x03); // SW = 0, HSI
		while (RCC->CFGR & (0x03 << 2)); // Polling SW status
	}

	// Step 6: Configure AHB, APB1, and APB2 prescalers
	// TODO
	// RCC->CFGR &= ~(0x07 << 8); // Clear APB1 prescaler
	// RCC->CFGR &= ~(0x07 << 11); // Clear APB2 prescaler
	// RCC->CFGR &= ~(0x03 << 14); // Clear ADC prescaler
	// RCC->CFGR &= ~(0x01 << 22); // Clear USB prescaler
	// RCC->CFGR &= ~(0x03 << 24); // Clear MCO
}

