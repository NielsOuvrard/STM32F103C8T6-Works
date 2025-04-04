/*
 * main.c
 *
 *  Created on: Feb 23, 2025
 *      Author: nielsouvrard
 */

#include "main.h"
#include <stdio.h>
#include <string.h>

typedef struct {
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
} Time;


void increment_time_and_write(Time *time)
{
	char new_buffer[16];
	char last_buffer[16];
	sprintf(last_buffer, "%02d:%02d:%02d", time->hours, time->minutes, time->seconds);

	time->seconds++;
	if (time->seconds >= 60) {
		time->seconds = 0;
		time->minutes++;
		if (time->minutes >= 60) {
			time->minutes = 0;
			time->hours++;
			if (time->hours >= 24) {
				time->hours = 0;
			}
		}
	}

	sprintf(new_buffer, "%02d:%02d:%02d", time->hours, time->minutes, time->seconds);
	for (int i = 0; i < strlen(new_buffer); i++) {
		if (new_buffer[i] != last_buffer[i]) {
			LCD_Goto_XY(i, 0);
			LCD_Write(new_buffer[i], 0);
		}
	}

}

void fill_str_with_time(char *str, Time time)
{
	sprintf(str, "%02d:%02d:%02d", time.hours, time.minutes, time.seconds);
}

int main(void)
{
	RCC->APB2ENR |= (0x01 << 4); // Enable GPIOC clock
	GPIOC->CRH &= ~(0xF << 20); // Clear CNF13 & MODE13
    GPIOC->CRH |= (0x01 << 20); // output mode, max speed 10 MHz

	set_clk_speed(CLOCK_8MHz, true);
    change_clk(8);
    LCD_Init(LCD_8B_INTERFACE);

	Time time = {0, 0, 0};
	{
		char buffer[16];
		fill_str_with_time(buffer, time);
		LCD_Print(buffer);
	}



	/* Application loop forever */
	while (1)
	{
		// LCD_Clear();
		increment_time_and_write(&time);

		// shift_display(-1);
		GPIOC->ODR ^= (1 << 13);
		delay_ms(1000);
	}
}



// volatile int8_t state = 0; // BIT0 = ENC_A, BIT1 = ENC_B
// volatile int32_t contador = 0;

// const uint8_t table[] = {
// 	0b00,
//     0b01,
//     0b11,
//     0b10
// };

// uint8_t get_index(uint8_t state)
// {
//     for (int i = 0; i < 4; i++)
//     {
//         if (table[i] == state)
//         {
//             return i;
//         }
//     }
//     assert(0); // should never reach here
// }

// void EXTI15_10_IRQHandler(void)
// {
//     uint8_t newState = 0;
//     if (GPIOB->IDR & GPIO_PIN_10)
//         newState |= 0b01;
//     if (GPIOB->IDR & GPIO_PIN_11)
//         newState |= 0b11;

//     uint8_t state_index = get_index(state);
//     uint8_t new_index = get_index(newState);
//     if (state_index < new_index || (state_index == 3 && new_index == 0))
//     {
//         contador++;
//     }
//     else if (state_index > new_index || (state_index == 0 && new_index == 3))
//     {
//         contador--;
//     }
//     else
//     {
//         assert(0); // should never reach here
//     }
//     state = newState;
// }
