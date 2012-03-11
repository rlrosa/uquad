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

#ifndef _IMU_KALMAN_H_
#define _IMU_KALMAN_H_ 1

typedef struct _kalman kalman;

struct _kalman
{
    // State estimation matrix.
    float x_00;
    float x_10;

    // Covariance of estimation error matrix.
    float P_00;
    float P_01;
    float P_10;
    float P_11;

    // State constants.
    float A_01;
    float B_00;
    float Sz_00;
    float Sw_00;
    float Sw_11;
};

// Initialize the kalman state.
void kalman_init(kalman *self, float dt, float Sz_00, float Sw_00, float Sw_11);

// Update the kalman state estimate and return the estimated angle.
float kalman_update(kalman *self, float gyro_rate, float accel_angle);

#endif // _IMU_KALMAN_H_
