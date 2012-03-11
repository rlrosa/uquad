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

#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include "imu.h"
#include "adc.h"
#include "kalman.h"
#include "tilt.h"
#include "usart.h"

// Fix some lameness in the latest AVR Studio and GCC tools.
uint8_t ee __attribute__ ((section(".eeprom")));

float gyro_x;
float gyro_y;
float accel_x;
float accel_y;
float accel_z;
float roll;
float pitch;
float roll_rate;
float pitch_rate;
float roll_est;
float pitch_est;

#if USE_KALMAN

// Kalman state structures.
kalman roll_kalman_state;
kalman pitch_kalman_state;

#endif

#if USE_TILT

// Tilt state structures.
tilt roll_tilt_state;
tilt pitch_tilt_state;

#endif


int main(void)
{
    // Initialize the USART.
    usart_init(BAUD2UBRR_57600);

    // Initialize the ADC.
    adc_init();

#if USE_KALMAN
    // Initialize the Kalman states for pitch and roll.  The measurement noise
    // covariance (Sz_00) is the noise in the accelerometer measurements.  Data
    // read from 1000 samples of the accelerometer had a variance of 0.00002.
    // The process noise covariance (Sw_00, Sw_11) values below were pulled from
    // the autopilot site.  The proper settings for these values needs to be
    // explored further, but apparently Sw_00 is how much we trust the accelerometer
    // and Sw_11 is how much we trust the gyro.
    kalman_init(&roll_kalman_state, 0.0998, 0.00002, 0.001, 0.003);
    kalman_init(&pitch_kalman_state, 0.0998, 0.00002, 0.001, 0.003);
#endif

#if USE_TILT
    // Initialize the tilt state for pitch and roll.
    tilt_init(&roll_tilt_state, 0.0998, 0.3, 0.003, 0.001);
    tilt_init(&pitch_tilt_state, 0.0998, 0.3, 0.003, 0.001);
#endif

    // Enable interrupts.
    sei();

    // Wait until adc samples are ready.
    while (!adc_is_ready());

    // Get the x, y and z accelerometer values.
    accel_x = (float) adc_get_accel_x();
    accel_y = (float) adc_get_accel_y();
    accel_z = (float) adc_get_accel_z();

    // Reset the ready flag.
    adc_reset_ready();

    // Initialize the pitch and roll estimate.
    roll_est = atan2(accel_y, accel_z);
    pitch_est = atan2(accel_x, accel_z);

    // Loop forever.
    for (;;)
    {
        // Get the gyro values.
        if (adc_is_ready())
        {
            // Get the x and y gyro values.
            gyro_x = (float) adc_get_gyro_x();
            gyro_y = (float) adc_get_gyro_y();

            // Get the x, y and z accelerometer values.
            accel_x = (float) adc_get_accel_x();
            accel_y = (float) adc_get_accel_y();
            accel_z = (float) adc_get_accel_z();

            // Reset the ready flag.
            adc_reset_ready();

            // Zero adjust the gyro values.  A better way of dynamically determining
            // these values must be found rather than using hard coded constants.
            gyro_x -= 470.5;
            gyro_y -= 440.5;

            // Zero adjust the accelerometer values.  A better way of dynamically determining
            // these values must be found rather than using hard coded constants.
            accel_x -= 506.0;
            accel_y -= 506.0;
            accel_z -= 506.0;

            // Determine the pitch and roll in radians.  X and Z acceleration determines the
            // pitch and Y and Z acceleration determines roll.  Note the accelerometer vectors
            // that are perpendicular to the rotation of the axis are used.  We can compute the
            // angle for the full 360 degree rotation with no linearization errors by using the
            // arctangent of the two accelerometer readings.  The accelerometer values do not
            // need to be scaled into actual units, but must be zeroed and have the same scale.
            // Note that we manipulate the sign of the acceleration so the sign of the accelerometer
            // derived angles match the gyro rates.
            roll = atan2(accel_y, accel_z);
            pitch = atan2(-accel_x, accel_z);

            // Determine gyro angular rate from raw analog values.
            // Each ADC unit: 3300 / 1024 = 3.222656 mV
            // Gyro measures rate: 2.0 mV/degrees/second
            // Gyro measures rate: 114.591559 mV/radians/second
            // Each ADC unit equals: 3.222656 / 2.0 = 1.611328 degrees/sec
            // Each ADC unit equals: 3.222656 / 114.591559 = 0.0281230 radians/sec
            // Gyro rate: adc * 0.0281230 radians/sec
            roll_rate = gyro_x * 0.0281230;
            pitch_rate = gyro_y * 0.0281230;

#if USE_KALMAN
            // Pass the measured pitch and roll values and rates through the Kalman filter to
            // determine the estimated pitch and roll values in radians.
            roll_est = kalman_update(&roll_kalman_state, roll_rate, roll);
            pitch_est = kalman_update(&pitch_kalman_state, pitch_rate, pitch);
#endif

#if USE_TILT
            // Pass the measured roll values and rates through the Kalman filter to
            // determine the estimated roll values in radians.
            tilt_state_update(&roll_tilt_state, roll_rate);
            tilt_kalman_update(&roll_tilt_state, roll);
            roll_est = tilt_get_angle(&roll_tilt_state);

            // Pass the measured pitch values and rates through the Kalman filter to
            // determine the estimated pitch values in radians.
            tilt_state_update(&pitch_tilt_state, pitch_rate);
            tilt_kalman_update(&pitch_tilt_state, pitch);
            pitch_est = tilt_get_angle(&pitch_tilt_state);
#endif

            // Adjust pitch and roll to compensate for yaw rotation component.
            // XXX pitch_est += roll_est * sin(pitch_est) * tan(roll_est);
            // XXX roll_est *= cos(pitch_est);

            // Skip output if the previous values are not yet finished.
            if (usart_xmit_buffer_ready())
            {
                // usart_xmit_printf("{%f, %f},\n\r", roll_rate, roll);
                // usart_xmit_printf("{%f, %f},\n\r", pitch_rate, pitch);
                // usart_xmit_printf("%f\n\r", roll);
                // usart_xmit_printf("%f %f\n\r", gyro_x, gyro_y);

                // Report the estimate pitch and roll values in degrees.
                usart_xmit_printf("pitch = %-+6.1f  roll = %-+6.1f   \r\n", pitch_est * 57.2957795, roll_est * 57.2957795);
            }
        }
    }

    return 0;
}

