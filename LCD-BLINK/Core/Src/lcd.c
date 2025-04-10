/*
 * lcd.c
 *
 *  Created on: Feb 20, 2025
 *      Author: nielsouvrard
 */


#include "lcd.h"
#include "common.h"

#define BIT(n)				(1UL << (n))
#define __NOP() __asm volatile ("nop")  // Lasts 1 clock cycle of the STM32f103

static void LCD_trigger_enable_pin(void)
{
	// put E for 1ms
	GPIOA->ODR |= (0x01 << 10); // E = 1

	__NOP();
	__NOP();
	__NOP();
	__NOP();

	__NOP();
	__NOP();
	__NOP();
	__NOP();

	GPIOA->ODR &= ~(0x01 << 10); // E = 0
}


void LCD_Init(uint8_t dbWidth)
{
	// PA8 - PA10, output mode, max speed 10 MHz, push-pull
	GPIOA->CRH &= ~(0xFFF); // Clear CNF8, MODE8, CNF9, MODE9, CNF10, MODE10
	GPIOA->CRH |= (0b10 << 0); // output mode, max speed 2 MHz, 8
	GPIOA->CRH |= (0b10 << 4); // output mode, max speed 2 MHz, 9
	GPIOA->CRH |= (0b10 << 8); // output mode, max speed 2 MHz, 10

	// PA0 - PA7, output mode, max speed 2 MHz, push-pull
	GPIOA->CRL &= ~(0xFFFFFFFF); // Clear CNF0, MODE0, CNF1, MODE1, CNF2, MODE2, CNF3, MODE3
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
		uint16_t db_value = 0x3F;

		// RS = 0, R/W = 0, DB7 = 0, DB6 = 0, DB5 = 1, DB4 = 1...
		// B7 = 0, B6 = 0,  A7 = 0,  A6 = 0,  A5 = 1,  A4 = 1
		GPIOA->ODR &= ~(0x01 << 8); // RS = 0
		GPIOA->ODR &= ~(0x01 << 9); // R/W = 0
        // PB5 -> E
		GPIOA->ODR &= ~(0xFF);
		GPIOA->ODR |= db_value; // 0x30

		LCD_trigger_enable_pin();

		// Wait for more than 4.1 ms
		delay_ms(5);

		GPIOA->ODR &= ~(0x01 << 8); // RS = 0
		GPIOA->ODR &= ~(0x01 << 9); // R/W = 0

		GPIOA->ODR &= ~(0xFF);
		GPIOA->ODR |= db_value; // 0x30

		LCD_trigger_enable_pin();

		// Wait for more than 100 µs
		delay_ms(1);

		GPIOA->ODR &= ~(0x01 << 8); // RS = 0
		GPIOA->ODR &= ~(0x01 << 9); // R/W = 0

		GPIOA->ODR &= ~(0xFF);
		GPIOA->ODR |= db_value; // 0x30

		LCD_trigger_enable_pin();

		// Wait for more than 100 µs
		delay_ms(1);

        // ! NOW THE SETTINGS

        // LCD_Write(0x38, 1);

		GPIOA->ODR &= ~(0x01 << 8); // RS = 0
		GPIOA->ODR &= ~(0x01 << 9); // R/W = 0

		db_value = 0x30; // function set
		uint8_t n = 1; // N = 1: 2 lines, N = 0: 1 line
		uint8_t f = 0; // F = 0: 5x8 dots, F = 1: 5x10 dots
		db_value |= (n << 3);
		db_value |= (f << 2);

		GPIOA->ODR &= ~(0xFF);
		GPIOA->ODR |= db_value;

		LCD_trigger_enable_pin();


		// * Display off
		// Wait for more than 100 µs
		delay_ms(1);

		GPIOA->ODR &= ~(0x01 << 8); // RS = 0
		GPIOA->ODR &= ~(0x01 << 9); // R/W = 0

		db_value = 0x08; // display off
		uint8_t d = 0; // D = 1: Display on, D = 0: Display off
		uint8_t c = 0; // C = 1: Cursor on, C = 0: Cursor off
		uint8_t b = 0; // B = 1: Cursor blink, B = 0: Cursor doesn't blink
		db_value |= (d << 2);
		db_value |= (c << 1);
		db_value |= b;
		GPIOA->ODR &= ~(0xFF);
		GPIOA->ODR |= db_value;

		LCD_trigger_enable_pin();

		// Wait for more than 100 µs
		delay_ms(1);

		GPIOA->ODR &= ~(0x01 << 8); // RS = 0
		GPIOA->ODR &= ~(0x01 << 9); // R/W = 0

		// * Display clear
		db_value = 0x01; // clear display

		GPIOA->ODR &= ~(0xFF);
		GPIOA->ODR |= 1;

		LCD_trigger_enable_pin();


		// Wait for more than 100 µs
		delay_ms(1);

		GPIOA->ODR &= ~(0x01 << 8); // RS = 0
		GPIOA->ODR &= ~(0x01 << 9); // R/W = 0

		// * Entry mode set
		db_value = 0x04; // entry mode set
		uint8_t id = 1; // I/D = 1: Increment, I/D = 0: Decrement
		uint8_t s = 1; // S = 1: Accompanies display shift, S = 0: No display shift
		db_value |= (id << 1);
		db_value |= s;

		GPIOA->ODR &= ~(0xFF);
		GPIOA->ODR |= db_value;

		LCD_trigger_enable_pin();

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
	// Write mode (RW = 0)
	GPIOA->ODR &= ~(0x01 << 9); // R/W = 0

    // Set RS = isCmd (0 for command, 1 for data)
    if (isCmd) {
        GPIOA->ODR &= ~(0x01 << 8); // RS = 0 (command)
    } else {
        GPIOA->ODR |= (0x01 << 8); // RS = 1 (data)
    }

	// Configure DB pins as outputs
	GPIOA->CRL &= ~(0xFFFFFFFF); // Clear CNF0, MODE0, CNF1, MODE1, CNF2, MODE2, CNF3, MODE7
	GPIOA->CRL |= (0x11111111); // General purpose output push-pull

	// Write data into DB pins
	GPIOA->ODR &= ~(0xFF);
	GPIOA->ODR |= data;

	// Send EN pulse: see Bus Timing Characteristics on pag. 49
	LCD_trigger_enable_pin();

	delay_ms(1);

	// Set DB pins to high
	//GPIOA->ODR |= 0xFF;
}

// Read sequence
uint8_t LCD_Read(uint8_t isData)
{
	uint8_t dout = 0x00;

	// Read mode (RW = 1)
	GPIOA->ODR |= (0x01 << 9); // R/W = 1

    // Set RS = isData (1 for data, 0 for command/busy flag)
    if (isData) {
        GPIOA->ODR |= (0x01 << 8); // RS = 1
    } else {
        GPIOA->ODR &= ~(0x01 << 8); // RS = 0
    }

	// Read data/busy&DDRAM (RW = 1/0)
	GPIOA->ODR &= ~(0x01 << 8); // RS = 0

	// Configure DB pins as floating inputs
	GPIOA->CRL &= ~(0xFFFFFFFF); // Clear CNF0, MODE0, CNF1, MODE1, CNF2, MODE2, CNF3, MODE7

	// Pull EN to HIGH: see Bus Timing Characteristics on pag. 49
	GPIOA->ODR |= (0x01 << 10); // E = 1

	// Read data from DB pins
	dout = GPIOA->IDR;

	// Pull EN to LOW: see Bus Timing Characteristics on pag. 49
	GPIOA->ODR &= ~(0x01 << 10); // E = 0

	// Set DB pins to high
	GPIOA->ODR &= ~(0xFF);

	return dout;
}


void LCD_Goto_XY (int x, int y)
{
    // RS 0
    // RW 0
    GPIOA->ODR &= ~(0x01 << 8);
    GPIOA->ODR &= ~(0x01 << 9);

    // DB7 1
    // adress to the cursor (0x00 -> 0x0F) (0x40 -> 0x4F)
    uint8_t value = 0b10000000;
    value &= x;
    if (y) {
        value &= 0x40;
    }

}

void LCD_Print(const char *str)
{

}
