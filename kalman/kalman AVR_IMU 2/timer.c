/*
    Copyright (c) 2007 Michael P. Thompson <mpthompson@gmail.com>

    Permission is hereby granted, free of charge, to any person 
    obtaining a copy of this software and associated documentation 
    files (the "Software"), to deal in the Software without 
    restriction, including without limitation the rights to use, copy, 
    modify, merge, publish, distribute, sublicense, and/or sell copies 
    of the Software, and to permit persons to whom the Software is 
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be 
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
    DEALINGS IN THE SOFTWARE.

    $Id$
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "timer.h"

volatile uint8_t timer_count;
volatile uint8_t timer_ready;
volatile uint8_t timer_rand;
volatile uint16_t timer_wait[2];

// Define the compare register value to generate a 100 Hz timer interrupt.
// #define CRVALUE	   78      // 8 MHz
// #define CRVALUE     156     // 16 MHz
#define CRVALUE     195     // 20 MHz

void timer_init(void)
{
    // Clear the timer count and ready flag.
    timer_count = 0;
    timer_ready = 0;

    // Set the compare match A value to yield an interrupt every 1/100th of a second.
    TCNT0 = 0;
    OCR2A = CRVALUE;
    OCR2B = 0;

    // Set timer/counter0 control register A.
    TCCR2A = (0<<COM2A1) | (0<<COM2A0) |                    // Disconnect OC2A.
             (0<<COM2B1) | (0<<COM2B0) |                    // Disconnect OC2B.
             (1<<WGM21) | (0<<WGM20);                       // Mode 2 - clear timer on compare match.

    // Set timer/counter0 control register B.
    TCCR2B = (0<<FOC2A) | (0<<FOC2B) |                      // No force output compare A or B.
             (0<<WGM22) |                                   // Mode 2 - clear timer on compare match.
             (1<<CS02) | (1<<CS01) | (1<<CS00);				// Clk/1024 prescale.

    // Set the timer/counter0 interrupt masks.
    TIMSK2 = (1<<OCIE2A) |                                  // Interrupt on compare match A.
             (0<<OCIE2B) |                                  // No interrupt on compare match B.
             (0<<TOIE2);                                    // No interrupt on overflow.
}


SIGNAL(SIG_OUTPUT_COMPARE2A)
// Handles timer/counter0 overflow.
{
    // Increment the timer count.
    ++timer_count;

    // Increment the timer random.
    ++timer_rand;

    // Have we reached 1/10th of a second?
    if (timer_count > 9)
    {
        // Set the timer ready flag.
        timer_ready = 1;

        // Decrement the timer wait.
        if (timer_wait[0]) --timer_wait[0];
        if (timer_wait[1]) --timer_wait[1];

        // Reset the timer count.
        timer_count = 0;
    }
}
