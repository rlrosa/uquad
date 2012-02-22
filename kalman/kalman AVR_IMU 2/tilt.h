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

#ifndef _IMU_TILT_H_
#define _IMU_TILT_H_ 1

typedef struct _tilt tilt;

struct _tilt
{
    // Two states, angle and gyro bias. Unbiased angular rate is a byproduct.
    float bias;
    float rate;
    float angle;

    // Covariance of estimation error matrix.
    float P_00;
    float P_01;
    float P_10;
    float P_11;

    // State constants.
    float dt;
    float R_angle;
    float Q_gyro;
    float Q_angle;
};

void tilt_init(tilt *self, float dt, float R_angle, float Q_gyro, float Q_angle);
void tilt_state_update(tilt *self, float gyro_rate);
void tilt_kalman_update(tilt *self, float angle_measured);

inline static float tilt_get_bias(tilt *self)
// Get the bias.
{
    return self->bias;
}

inline static float tilt_get_rate(tilt *self)
// Get the rate.
{
    return self->rate;
}

inline static float tilt_get_angle(tilt *self)
// Get the angle.
{
    return self->angle;
}

#endif // _IMU_TILT_H_
