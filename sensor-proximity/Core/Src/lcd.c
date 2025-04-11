/*
 * lcd.c
 *
 *  Created on: Feb 20, 2025
 *      Author: nielsouvrard
 */


#include "lcd.h"

#define BIT(n)				(1UL << (n))
#define __NOP() __asm volatile ("nop")  // Lasts 1 clock cycle of the STM32f103


void delay_ms (uint32_t ms)
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
        * If Fclk = XMHz -> XMHz/1000 = X000
        *
        * However, at low-level, each iteration of the for
        * loop takes around 10 clock cycles, therefore
        * instead of iterating up to Fclk / 1000, it
        * should be Fclk / (1000*CYCLES_PER_ITER), i.e.,
        * XMHz/(1000*10) = X00
        */
        // should be hard-coded because of the number of cycles
        for (cycles = 0; cycles < 800; cycles++)
        {
        }
    }
}

static void LCD_trigger_enable_pin(void)
{
	// put E for 1ms
	GPIOB->ODR |= (0x01 << 5); // E = 1

	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();

	GPIOB->ODR &= ~(0x01 << 5); // E = 0
}

void LCD_Init(uint8_t dbWidth)
{
	// Configure GPIOs ============
	RCC->APB2ENR |= BIT(2); // Enable GPIOA clock
	RCC->APB2ENR |= BIT(3); // Enable GPIOB clock

	// PB5 - PB7, output mode, max speed 10 MHz, push-pull
	GPIOB->CRL &= ~(0xFFF << 20); // Clear CNF5, MODE5, CNF6, MODE6, CNF7, MODE7
	GPIOB->CRL |= (0b10 << 20); // output mode, max speed 2 MHz, 5
	GPIOB->CRL |= (0b10 << 24); // output mode, max speed 2 MHz, 6
	GPIOB->CRL |= (0b10 << 28); // output mode, max speed 2 MHz, 7

	// PA0 - PA7, output mode, max speed 10 MHz, push-pull
	GPIOA->CRL &= ~(0xFFFFFFFF); // Clear CNF0, MODE0, CNF1, MODE1, CNF2, MODE2, CNF3, MODE7
	GPIOA->CRL |= (0b10 << 0); // output mode, max speed 2 MHz, 0
	GPIOA->CRL |= (0b10 << 4); // output mode, max speed 2 MHz, 1
	GPIOA->CRL |= (0b10 << 8); // output mode, max speed 2 MHz, 2
	GPIOA->CRL |= (0b10 << 12); // output mode, max speed 2 MHz, 3
	GPIOA->CRL |= (0b10 << 16); // output mode, max speed 2 MHz, 4
	GPIOA->CRL |= (0b10 << 20); // output mode, max speed 2 MHz, 5
	GPIOA->CRL |= (0b10 << 24); // output mode, max speed 2 MHz, 6
	GPIOA->CRL |= (0b10 << 28); // output mode, max speed 2 MHz, 7

	// Perform init sequence: see https://cdn.sparkfun.com/assets/9/5/f/7/b/HD44780.pdf
	// See pag.
	if(dbWidth == LCD_8B_INTERFACE)
	{
		// 8-bit interface: see pag. 22, and 45
		// rise VCC to 4.5V, wait 40ms
		delay_ms(50);

		// RS = 0, R/W = 0, DB7 = 0, DB6 = 0, DB5 = 1, DB4 = 1...
		// B7 = 0, B6 = 0,  A7 = 0,  A6 = 0,  A5 = 1,  A4 = 1
		GPIOB->ODR &= ~(0x01 << 7); // RS = 0
		GPIOB->ODR &= ~(0x01 << 6); // R/W = 0
        // PB5 -> E
		GPIOA->ODR &= ~(0x01 << 7); // DB7 = 0
		GPIOA->ODR &= ~(0x01 << 6); // DB6 = 0
		GPIOA->ODR |= (0x01 << 5); // DB5 = 1
		GPIOA->ODR |= (0x01 << 4); // DB4 = 1
		GPIOA->ODR &= ~(0x01 << 3); // DB3 = 0
		GPIOA->ODR &= ~(0x01 << 2); // DB2 = 0
		GPIOA->ODR &= ~(0x01 << 1); // DB1 = 0
		GPIOA->ODR &= ~(0x01 << 0); // DB0 = 0

		LCD_trigger_enable_pin();
	
		// Wait for more than 4.1 ms
		delay_ms(5);

		GPIOB->ODR &= ~(0x01 << 7); // RS = 0
		GPIOB->ODR &= ~(0x01 << 6); // R/W = 0
		GPIOA->ODR &= ~(0x01 << 7); // DB7 = 0
		GPIOA->ODR &= ~(0x01 << 6); // DB6 = 0
		GPIOA->ODR |= (0x01 << 5); // DB5 = 1
		GPIOA->ODR |= (0x01 << 4); // DB4 = 1
		GPIOA->ODR |= (0x01 << 3); // DB3 = 0
		GPIOA->ODR |= (0x01 << 2); // DB2 = 0
		GPIOA->ODR |= (0x01 << 1); // DB1 = 0
		GPIOA->ODR |= (0x01 << 0); // DB0 = 0

		LCD_trigger_enable_pin();

		// Wait for more than 100 µs
		delay_ms(1);

		GPIOB->ODR &= ~(0x01 << 7); // RS = 0
		GPIOB->ODR &= ~(0x01 << 6); // R/W = 0
		GPIOA->ODR &= ~(0x01 << 7); // DB7 = 0
		GPIOA->ODR &= ~(0x01 << 6); // DB6 = 0
		GPIOA->ODR |= (0x01 << 5); // DB5 = 1
		GPIOA->ODR |= (0x01 << 4); // DB4 = 1
		GPIOA->ODR |= (0x01 << 3); // DB3 = 0
		GPIOA->ODR |= (0x01 << 2); // DB2 = 0
		GPIOA->ODR |= (0x01 << 1); // DB1 = 0
		GPIOA->ODR |= (0x01 << 0); // DB0 = 0

		LCD_trigger_enable_pin();

		// Wait for more than 100 µs
		delay_ms(1);

        // *  NOW THE SETTINGS
     
		GPIOB->ODR &= ~(0x01 << 7); // RS = 0
		GPIOB->ODR &= ~(0x01 << 6); // R/W = 0

		GPIOA->ODR &= ~(0x01 << 7); // DB7 = 0
		GPIOA->ODR &= ~(0x01 << 6); // DB6 = 0
		GPIOA->ODR |= (0x01 << 5); // DB5 = 1
		GPIOA->ODR |= (0x01 << 4); // DB4 = 1

		GPIOA->ODR |= (0x01 << 3); // DB3 = 1 // 1 because 2*16
		GPIOA->ODR &= ~(0x01 << 2); // DB2 = 0 // 0 Because 5 * 8 chars
		GPIOA->ODR &= ~(0x01 << 1); // DB1 = 0
		GPIOA->ODR &= ~(0x01 << 0); // DB0 = 0

		LCD_trigger_enable_pin();

        
		// Wait for more than 100 µs
		delay_ms(1);
        
		GPIOB->ODR &= ~(0x01 << 7); // RS = 0
		GPIOB->ODR &= ~(0x01 << 6); // R/W = 0
		GPIOA->ODR &= ~(0x01 << 7); // DB7 = 0
		GPIOA->ODR &= ~(0x01 << 6); // DB6 = 0
		GPIOA->ODR &= ~(0x01 << 5); // DB5 = 0
		GPIOA->ODR &= ~(0x01 << 4); // DB4 = 0
		GPIOA->ODR |= (0x01 << 3); // DB3 = 1
		GPIOA->ODR |= (0x01 << 2); // DB2 = 0 // D -> Set the entire display
		GPIOA->ODR &= ~(0x01 << 1); // DB1 = 0
		GPIOA->ODR &= ~(0x01 << 0); // DB0 = 0

        
		LCD_trigger_enable_pin();

        
		// Wait for more than 100 µs
		delay_ms(1);
        
		GPIOB->ODR &= ~(0x01 << 7); // RS = 0
		GPIOB->ODR &= ~(0x01 << 6); // R/W = 0
		GPIOA->ODR &= ~(0x01 << 7); // DB7 = 0
		GPIOA->ODR &= ~(0x01 << 6); // DB6 = 0
		GPIOA->ODR &= ~(0x01 << 5); // DB5 = 0
		GPIOA->ODR &= ~(0x01 << 4); // DB4 = 0
		GPIOA->ODR &= ~(0x01 << 3); // DB3 = 0
		GPIOA->ODR &= ~(0x01 << 2); // DB2 = 0
		GPIOA->ODR &= ~(0x01 << 1); // DB1 = 0
		GPIOA->ODR |= (0x01 << 0); // DB0 = 1

        
		LCD_trigger_enable_pin();

        
		// Wait for more than 100 µs
		delay_ms(1);
        
		GPIOB->ODR &= ~(0x01 << 7); // RS = 0
		GPIOB->ODR &= ~(0x01 << 6); // R/W = 0
		GPIOA->ODR &= ~(0x01 << 7); // DB7 = 0
		GPIOA->ODR &= ~(0x01 << 6); // DB6 = 0
		GPIOA->ODR &= ~(0x01 << 5); // DB5 = 0
		GPIOA->ODR &= ~(0x01 << 4); // DB4 = 0
		GPIOA->ODR &= ~(0x01 << 3); // DB3 = 0
		GPIOA->ODR |= (0x01 << 2); // DB2 = 1
		GPIOA->ODR |= (0x01 << 1); // DB1 = 1 // I/D = 1: Increment
		GPIOA->ODR &= ~(0x01 << 0); // DB0 = 1 // S = 1: Accompanies display shift

		LCD_trigger_enable_pin();

		// Wait for more than 100 µs
		delay_ms(1);
	}
	else if(dbWidth == LCD_4B_INTERFACE)
	{
		// 4-bit interface: see pag. 22, and 46
	}
}

// Write sequence
void LCD_Write(uint8_t data, uint8_t isCmd)
{
    if (isCmd) {
        GPIOB->ODR &= ~(0x01 << 7); // RS = 0 for command
    } else {
        GPIOB->ODR |= (0x01 << 7); // RS = 1 for data
    }
    GPIOB->ODR &= ~(0x01 << 6); // R/W = 0

	GPIOA->ODR &= ~(0xFF);
	GPIOA->ODR |= (data);

	LCD_trigger_enable_pin();
	// Wait for more than 100 µs
	delay_ms(1);

	// Write data/instruction (RS = 1/0)

	// Configure DB pins as outputs

	// Write data into DB pins

	// Send EN pulse: see Bus Timing Characteristics on pag. 49

	// Set DB pins to high
}

// Read sequence
uint8_t LCD_Read(uint8_t isData)
{
	uint8_t dout = 0x00;

	// Read mode (RW = 1)

	// Read data/busy&DDRAM (RW = 1/0)

	// Configure DB pins as floating inputs

	// Pull EN to HIGH: see Bus Timing Characteristics on pag. 49

	// Read data from DB pins

	// Pull EN to LOW: see Bus Timing Characteristics on pag. 49

	// Set DB pins to high

	return dout;
}


void LCD_Clear(void)
{
	// RS 0
	// RW 0
	GPIOB->ODR &= ~(0x01 << 7);
	GPIOB->ODR &= ~(0x01 << 6);
	GPIOA->ODR &= ~(0xFF);
	GPIOA->ODR |= (0x01 << 0); // 00000001

	LCD_trigger_enable_pin();
}

void move_cursor(uint8_t direction)
{
	// RS 0
	// RW 0
	GPIOB->ODR &= ~(0x01 << 7);
	GPIOB->ODR &= ~(0x01 << 6);
	GPIOA->ODR &= ~(0xFF);
	GPIOA->ODR |= (0x01 << 2); // 00000100

	LCD_trigger_enable_pin();
}

void shift_display(uint8_t direction)
{
	// RS 0
	// RW 0
	GPIOB->ODR &= ~(0x01 << 7);
	GPIOB->ODR &= ~(0x01 << 6);
	GPIOA->ODR &= ~(0xFF);
	GPIOA->ODR |= (0x03 << 2); // 00001100

	LCD_trigger_enable_pin();
}

void return_home(void)
{
	// RS 0
	// RW 0
	GPIOB->ODR &= ~(0x01 << 7);
	GPIOB->ODR &= ~(0x01 << 6);
	GPIOA->ODR &= ~(0xFF);
	GPIOA->ODR |= (0x01 << 0); // 00000001

	LCD_trigger_enable_pin();
}

void LCD_Goto_XY (int x, int y)
{
	uint8_t address = (y == 0) ? 0x80 : 0xC0;
	address += x;
	LCD_Write(address, 1);
}

void LCD_Print(const uint8_t *str)
{
	for (int i = 0; str[i] != '\0'; i++)
	{
		LCD_Write(str[i], 0);
	}
}
