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

#ifndef _KA_TIMER_H_
#define _KA_TIMER_H_ 1

// Declare externally so in-lines work.
extern volatile uint8_t timer_ready;
extern volatile uint8_t timer_rand;
extern volatile uint16_t timer_wait[2];


void timer_init(void);

inline static uint8_t timer_random(void)
// Return the timer psuedo random value.
{
    return timer_rand;
}


inline static uint8_t timer_is_ready(void)
// Return the timer ready flag.
{
    return timer_ready;
}


inline static void timer_clear_ready(void)
// Clear the timer ready flag.
{
    timer_ready = 0;
}


inline static void timer_wait_set(uint8_t timer, uint16_t wait_time)
// Set the wait timer.
{
    timer_wait[timer & 1] = wait_time;
}


inline static uint8_t timer_wait_done(uint8_t timer)
// Return true if the timer wait is finished.
{
    return timer_wait[timer & 1] ? 0 : 1;
}


#endif // _KA_TIMER_H_
