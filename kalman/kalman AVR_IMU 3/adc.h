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

#ifndef _IMU_ADC_H_
#define _IMU_ADC_H_ 1

// Initialize ADC conversion.
void adc_init(void);

// Declare externally so in-lines work.
extern volatile uint8_t adc_ready;
extern volatile uint16_t adc_count;
extern volatile uint16_t adc_accel_x;
extern volatile uint16_t adc_accel_y;
extern volatile uint16_t adc_accel_z;
extern volatile uint16_t adc_gyro_x;
extern volatile uint16_t adc_gyro_y;
extern volatile uint16_t adc_gyro_ref;
extern volatile uint16_t adc_timer_count;

// In-lines for fast access to power flags and values.

inline static uint8_t adc_is_ready(void)
// Return the ADC ready flag.
{
    return adc_ready;
}

inline static void adc_reset_ready(void)
// Reset the ADC flag.
{
    adc_ready = 0;
}

inline static uint16_t adc_get_count(void)
// Return the ADC count.
{
    return adc_count;
}

inline static uint16_t adc_get_accel_x(void)
// Return the accelerometer X value.
{
    return adc_accel_x;
}

inline static uint16_t adc_get_accel_y(void)
// Return the accelerometer Y value.
{
    return adc_accel_y;
}

inline static uint16_t adc_get_accel_z(void)
// Return the accelerometer Z value.
{
    return adc_accel_z;
}

inline static uint16_t adc_get_gyro_x(void)
// Return the gyroscope X value.
{
    return adc_gyro_x;
}

inline static uint16_t adc_get_gyro_y(void)
// Return the gyroscope Y value.
{
    return adc_gyro_y;
}

inline static uint16_t adc_get_gyro_ref(void)
// Return the gyroscope reference value.
{
    return adc_gyro_ref;
}

#endif // _IMU_ADC_H_
