/*
Fuente de reloj: HSE (Cristal externo)
SYSCLK: 48 MHz
AHBCLK: 48 MHz
APB1CLK: 12 MHz
APB2CLK: 24 MHz

// This one is the one
RCC->CR |= (1U << 16);
while (!(RCC->CR & (1U << 17)));
RCC->CFGR |= (1U << 16) | (0x7U << 18);  // 8MHz x 9 = 72MHz
RCC->CFGR |= (0x0U << 4) | (0x4U << 8) | (0x4U << 11);  // HPRE = 0, PPRE1 = 0x4, PPRE2 = 0x4
RCC->CR |= (1U << 24);  // PLLON
while (!(RCC->CR & (1U << 25)));  // Polling for PLLRDY
RCC->CFGR |= (0x2U << 0);  // PLL as system clock
while ((RCC->CFGR & (0x3U << 2)) != (0x2U << 2)); // polling for PLL as system clock









Diseña un controlador para leer datos de un sensor de celda de carga usando el HX711

1. Configuración del Reloj [1 pts]:

Fuente: Derivada de HSE.
SYSCLK = 48MHz
AHBCLK = 48MHz
APB1CLK = 12MHz
APB2CLK = 24MHz
2. Implementar Driver para HX711

hx711_Init: Configura GPIOs, y enciende el sensor
        * PA1 como salida push-pull a 2MHz (HX711_SCK) y ponerla a 1 lógico
        * PB2 como entrada flotante (HX711_DOUT)
hx711_read_raw: Realiza una lectura de datos crudos del sensor.
      * Los datos se transmiten de manera serial
      * Cada bit se lee en HX711_DOUT en cada flanco ascendente de la señal HX711_SCK
      * La señal de reloj HX711_SCK se envía desde el microcontrolador hasta el HX711
      * La señal de datos HX711_DOUT se transmite desde el HX711 hasta el microcontrolador


Enlace de la hoja de datos del HX711: https://cdn.sparkfun.com/datasheets/Sensors/ForceFlex/hx711_english.pdf







Deberá emplear la siguiente plantilla:
*/

#include "stm32f103xb.h"

#define __NOP() __asm volatile ("nop")  // Lasts 1 clock cycle of the STM32f103

void SystemClock_Config(void)
{
    // 1. Enable HSE (External Crystal)
	RCC->CR |= (1U << 16);
	while (!(RCC->CR & (1U << 17)));

    // 2. Set FLASH wait-states
	// ! not
	FLASH->ACR &= ~(0x07);			// Reset wait states
	FLASH->ACR |= (0x01 << 0);			// 1 wait states

    // 3. Configure prescalers (APB1, APB2)
	// ! not
	RCC->CFGR |= (0x0U << 4) | (0x4U << 8) | (0x4U << 11);  // HPRE = 0, PPRE1 = 0x4, PPRE2 = 0x4

    // 4. Configure PLL multiplier
	RCC->CFGR |= (1U << 16); // PLLSRC = HSE
	RCC->CFGR |= (0x07 << 18);	// PLLMUL = x9 (8MHz * 9 = 72MHz)

    // 5. Activate PLL and check PLLRDY flag
	RCC->CR |= (1U << 24);  // PLLON
	while (!(RCC->CR & (1U << 25)));  // Polling for PLLRDY

    // 6. Select PLL input as SYSCLK and check SWS field
	RCC->CFGR |= (0x2U << 0);  // PLL as system clock
	while ((RCC->CFGR & (0x3U << 2)) != (0x2U << 2)); // polling for PLL as system clock
}


void hx711_init(void)
{
    // 1. Enable GPIO clocks for HX711_DOUT and HX711_SCK
	RCC->APB2ENR |= BIT(2) | BIT(4);  // GPIOA, GPIOB

    // 2. Configure HX711_SCK as push-pull output
	GPIOA->CRL &= ~(0x0F << 4); // Clear MODE1 & CNF1
	GPIOA->CRL |= (0b10 << 4);  // MODE1[1:0] = 0b10

    // 3. Configure HX711_DOUT as floating input
	GPIOB->CRL &= ~(0x0F << 8); // Clear MODE2 & CNF2
	GPIOB->CRL |= (0b01 << 8);  // MODE2[1:0] = 0b01

    // 4. Set HX711_SCK low for 1ms
	GPIOA->ODR &= ~(0x01 << 1);
}



uint32_t hx711_read_raw(void)
{
    uint32_t data = 0;
    // Wait until HX711_DOUT changes to low level (indicates that data is available)
	while (GPIOB->IDR & (0x01 << 2)); // << 2 because HX711_DOUT is connected to PB2

    // Read 24 bits of data
    for (int i = 0; i < 24; i++)
    {
        // Generate rising edge on HX711_SCK and wait HX711_SCK High Time (T3, page 5)
		GPIOA->ODR |= (0x01 << 1);
		// 200ns por NOP
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();

        // Shift the "data" variable 1 bit to the left
		data <<= 1;

        // If the data read on HX711_DOUT is 1, set the LSB of the "data" variable to logical 1
		if (GPIOB->IDR & (0x01 << 2)) {
			data |= 0x01;
		}

        // Generate falling edge on HX711_SCK and wait HX711_SCK Low Time (T4, page 5)
		GPIOA->ODR &= ~(0x01 << 1);
		// 200ns por NOP
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
    }



    /*
    * Generate N additional pulses on HX711_SCK
    * to configure the gain to 64
    */
	for (int i = 0; i < 3; i++) {
        GPIOA->ODR |= (0x01 << 1);
        // 200ns por NOP
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();

        GPIOA->ODR &= ~(0x01 << 0);
	}



    // Check if bit 24 is logical 1, if so,
    // set the 8 most significant bits of the data variable to 1
	if (data & (0x01 << 24)) {
		data |= 0xFF000000;
	}

    // Return the read data
    return data;
}
