/*
 * lcd.h
 *
 *  Created on: Feb 20, 2025
 *  @Author: User123
 *  @Notes: https://cdn.sparkfun.com/assets/9/5/f/7/b/HD44780.pdf
 */

#ifndef LCD_H_
#define LCD_H_

/* Exported includes -----------------------------------------------*/
#include "stm32f103xb.h"
#include "time.h"

/* Exported defines ------------------------------------------------*/
// Pinout
#define LCD_DB_Port	GPIOA
#define LCD_EN_Port GPIOB
#define LCD_RS_Port GPIOB
#define LCD_RW_Port GPIOB
#define LCD_DB_Pin	(BIT() )
#define LCD_EN_Pin	BIT(0)
#define LCD_RS_Pin 	BIT(1)
#define LCD_RW_Pin 	BIT(2)

// Flags
#define LCD_COMMAND
#define LCD_DATA
#define LCD_REQUEST


// Instructions: see pag. 24
#define LCD_CMD_CLEAR_DISP	0x01
#define LCD_CMD_RETURN_HOME 0x02

// I/D, S
#define LCD_CMD_ENTRY_MODE		0x04
#define LCD_DISPLAY_SHIFT		0x01		// Shift display to right
#define LCD_DISPLAY_NO_SHIFT	0x00		// Display no shift
#define LCD_CURSOR_INC 			0x02		// Cursor increment
#define LCD_CURSOR_DEC 			0x00		// Cursor decrement

// D, C, B
#define LCD_CMD_DISPLAY_CONTROL 0x08
#define LCD_DISPLAY_ON			0x04		// Turn on display
#define LCD_DISPLAY_OFF			0x00		// Turn off display
#define LCD_CURSOR_ON			0x02		// Turn on cursor
#define LCD_CURSOR_OFF			0x00		// Turn off cursor
#define LCD_CURSOR_BLINK 		0x01		// Turn on blink cursor
#define LCD_CURSOR_NO_BLINK 	0x00		// Turn off blink cursor

// S/C, R/L: Moves cursor and shifts display without changing DDRAM contents
#define LCD_CURSOR_DISP_SHIFT 	0x10		// Move and shift cursor
#define LCD_DISPLAY_SHIFT 		0x08
#define LCD_CURSOR_MOVE 		0x00
#define LCD_RIGHT_SHIFT 		0x04
#define LCD_LEFT_SHIFT 			0x00

// DL, N, F: see pag. 10-11
#define LCD_CMD_FUNCTION_SET 	0x20
#define LCD_8B_INTERFACE 		0x10		// Set interface data length
#define LCD_4B_INTERFACE 		0x00		// Set interface data length
#define LCD_TWO_LINES 			0x08		// Set number of display lines
#define LCD_ONE_LINE 			0x00		// Set number of display lines
#define LCD_5X10_FONT 			0x04		// Set font size
#define LCD_5X8_FONT 			0x00		// Set font size

#define LCD_CMD_SET_CGRAM_ADDR 	0x40		// Set CGRAM (Character Generator) address
#define LCD_CGRAM_ADDR_MASK		0x3F

#define LCD_CMD_SET_DDRAM_ADDR 	0x80		// Set DDRAM (Data) address
#define LCD_DDRAM_ADDR_MASK		0x7F

#define LCD_CMD_READ_BUSY_ADDR	0x00
#define LCD_BUSY_MASK			0x80
#define LCD_BUSY				0x80
#define LCD_IDLE				0x00
#define LCD_ADDR_COUNTER_MASK	0x7F

/* Exported macros -------------------------------------------------*/

/* Exported typedefs ------------------------------------------------*/

/* Exported variables -----------------------------------------------*/

/* Exported prototype function --------------------------------------*/
// See pag. 32-33, 45-46
void LCD_Init(uint8_t dbWidth);

// See pag. 24-25
void LCD_Write(uint8_t data, uint8_t isCmd);

// See pag. 24-25
uint8_t LCD_Read(uint8_t isData);

void LCD_Print(const uint8_t *str);

void LCD_Clear(void);
void move_cursor(uint8_t direction);
void shift_display(uint8_t direction);
void return_home(void);
void LCD_Goto_XY (int x, int y);

#endif /* LCD_H_ */
