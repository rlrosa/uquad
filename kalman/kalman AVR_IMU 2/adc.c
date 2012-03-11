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

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "adc.h"

//
// ATmega168
// =========
//
// The 10-bit Analog to Digital Converter (ADC) on the ATmega MCU is used
// to provide power and position feedback from the servo circuitry.  The
// analog inputs are assigned as follows:
//
//  ADC0 (PC0) - Accelerometer X
//  ADC1 (PC1) - Accelerometer Y
//  ADC2 (PC2) - Accelerometer Z
//  ADC3 (PC3) - Gyroscope X
//  ADC4 (PC4) - Gyroscope Y
//  ADC5 (PC5) - Gyroscope Reference
//

// Defines for the power and position channels.
#define ADC_CHANNEL_ACCEL_X     0
#define ADC_CHANNEL_ACCEL_Y     1
#define ADC_CHANNEL_ACCEL_Z     2
#define ADC_CHANNEL_GYRO_X      3
#define ADC_CHANNEL_GYRO_Y      4
#define ADC_CHANNEL_GYRO_REF    5
#define ADC_CHANNEL_MAX         ADC_CHANNEL_GYRO_REF
#define ADC_CHANNEL_COUNT       (ADC_CHANNEL_MAX + 1)

//
// NOTE: The constants chosen below are assuming a 20 MHz system clock
// to the AVR MCU.  If a different clock is used these constants will
// need to be adjusted.
//

// The ADC clock prescaler of 128 is selected to yield a 156.25 KHz ADC clock
// from an 20 MHz system clock.  The ADC clock must be between 50 KHz and
// 200 KHz for maximum resolution.
#define ADPS        ((1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0))

// The timer clock prescaler of 1024 is selected to yield a 19.53 KHz clock
// from a 20 MHz system clock.
#define CSPS        ((1<<CS02) | (0<<CS01) | (1<<CS00))

// Define the compare register value to generate a 100 Hz timer interrupt from
// a 20 MHz system clock. The timer interrupt drives another software controlled
// counter to initiate ADC samples at a 10 Hz rate.
#define CRVALUE     195

// Globals used to maintain ADC state and values.
volatile uint8_t adc_ready;
volatile uint16_t adc_count;
volatile uint16_t adc_accel_x;
volatile uint16_t adc_accel_y;
volatile uint16_t adc_accel_z;
volatile uint16_t adc_gyro_x;
volatile uint16_t adc_gyro_y;
volatile uint16_t adc_gyro_ref;
volatile uint16_t adc_channel;

void adc_init(void)
// Initialize ADC conversion for reading current monitoring and position.
{
    // Read from position first.
    adc_channel = ADC_CHANNEL_ACCEL_X;

    // Initialize flags and values.
    adc_ready = 0;
    adc_count = 0;
    adc_accel_x = 0;
    adc_accel_y = 0;
    adc_accel_z = 0;
    adc_gyro_x = 0;
    adc_gyro_y = 0;
    adc_gyro_ref = 0;

    //
    // Initialize ADC registers.
    //

    // Make sure ports PC0 (ADC0) thru PC5 (ADC5) are set low.
    PORTC &= ~((1<<PC0) | (1<<PC1) | (1<<PC2) | (1<<PC3) | (1<<PC4) | (1<<PC5));

    // Disable digital input for ADC0 true ADC5 to reduce power consumption.
    DIDR0 |= (1<<ADC0D) | (1<<ADC1D) | (1<<ADC2D) | (1<<ADC3D) | (1<<ADC4D) | (1<<ADC5D);

    // Set the ADC multiplexer selection register.
    ADMUX = (0<<REFS1) | (1<<REFS0) |                       // Select AVCC as voltage reference.
            (0<<ADLAR) |                                    // Keep high bits right adjusted.
            0;                                              // Select first channel.

    // Set the ADC control and status register B.
    ADCSRB = (0<<ADTS2) | (1<<ADTS1) | (1<<ADTS0);          // Timer/Counter0 Compare Match A.

    // Set the ADC control and status register A.
    ADCSRA = (1<<ADEN) |                                    // Enable ADC.
             (0<<ADSC) |                                    // Don's start yet, will be auto triggered.
             (0<<ADATE) |                                   // Disable auto triggering.
             (1<<ADIE) |                                    // Activate ADC conversion complete interrupt.
             ADPS;                                          // Prescale -- see above.

    // Set timer/counter0 control register A.
    TCCR0A = (0<<COM0A1) | (0<<COM0A0) |                    // Disconnect OCOA.
             (0<<COM0B1) | (0<<COM0B0) |                    // Disconnect OCOB.
             (1<<WGM01) | (0<<WGM00);                       // Mode 2 - clear timer on compare match.

    // Set timer/counter0 control register B.
    TCCR0B = (0<<FOC0A) | (0<<FOC0B) |                      // No force output compare A or B.
             (0<<WGM02) |                                   // Mode 2 - clear timer on compare match.
             CSPS;                                          // Timer clock prescale -- see above.

    // Set the timer/counter0 interrupt masks.
    TIMSK0 = (1<<OCIE0A) |                                  // Interrupt on compare match A.
             (0<<OCIE0B) |                                  // No interrupt on compare match B.
             (0<<TOIE0);                                    // No interrupt on overflow.

    // Set the compare match A value which initiates an ADC sample.
    OCR0A = CRVALUE;
}


SIGNAL(SIG_OUTPUT_COMPARE0A)
// Handles timer/counter0 compare match A.
{
    // We want to sample at 10 Hz rate.
    if (++adc_count == 10)
    {
        // Initiate an ADC sample.
        ADCSRA |= (1<<ADSC);

        // Reset the adc count.
        adc_count = 0;
    }
}


SIGNAL(SIG_ADC)
// Handles ADC interrupt.
{
    // Which channel is being read?
    switch (adc_channel)
    {
        case ADC_CHANNEL_ACCEL_X:

            // Store the ADC channel value.
            adc_accel_x = ADCW;

            break;

        case ADC_CHANNEL_ACCEL_Y:

            // Store the ADC channel value.
            adc_accel_y = ADCW;

            break;

        case ADC_CHANNEL_ACCEL_Z:

            // Store the ADC channel value.
            adc_accel_z = ADCW;

            break;

        case ADC_CHANNEL_GYRO_X:

            // Store the ADC channel value.
            adc_gyro_x = ADCW;

            break;

        case ADC_CHANNEL_GYRO_Y:

            // Store the ADC channel value.
            adc_gyro_y = ADCW;

            break;

        case ADC_CHANNEL_GYRO_REF:

            // Store the ADC channel value.
            adc_gyro_ref = ADCW;

            break;
    }

    // Increment the ADC channel.
    ++adc_channel;

    // If we have finished then set the channel flag.
    if (adc_channel > ADC_CHANNEL_MAX)
    {
        // The ADC channels are ready.
        adc_ready = 1;

        // Reset the ADC channel.
        adc_channel = 0;
    }
    else
    {
        // The ADC channels are not ready.
        adc_ready = 0;
    }

    // Set the channel for the next sample.
    ADMUX = (0<<REFS1) | (1<<REFS0) |                       // Select AVCC as voltage reference.
            (0<<ADLAR) |                                    // Keep high bits right adjusted.
            adc_channel;                                    // Select analog input.

    // Start the next channel if we haven't wrapped.
    if (adc_channel > 0) ADCSRA |= (1<<ADSC);
}

