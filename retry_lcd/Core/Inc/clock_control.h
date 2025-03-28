/*
 * clock_configuration.h
 *
 *  Created on: Feb 23, 2025
 *      Author: nielsouvrard
 */

#include "stm32f103xb.h"
#include <stdbool.h>

typedef enum spped_clock
// from 8MHz to 72MHz
// Due to hardware limitation, the Blue Pill board cannot run at more than 72MHz
{
    CLOCK_8MHz,
    CLOCK_12MHz, // divide by 2 the 8MHz, then multiply by 3
    CLOCK_16MHz,
    CLOCK_20MHz,
    CLOCK_24MHz,
    CLOCK_28MHz,
    CLOCK_32MHz,
    CLOCK_36MHz,
    CLOCK_40MHz,
    CLOCK_44MHz,
    CLOCK_48MHz,
    CLOCK_52MHz,
    CLOCK_56MHz,
    CLOCK_60MHz,
    CLOCK_64MHz,
    CLOCK_68MHz, // unavailable, as the PLL can't multiply by 17. It's 64MHz instead
    CLOCK_72MHz
} spped_clock;

void set_clk_speed(spped_clock speed, bool is_HSE);
