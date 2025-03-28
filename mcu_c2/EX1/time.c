/*
 * time.c
 *
 *  Created on: Feb 23, 2025
 *      Author: nielsouvrard
 */

#include "time.h"

volatile uint32_t mhz_clk = 800; // by default, 8MHz

void change_clk (int clk)
{
    mhz_clk = clk * 100;
}

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
