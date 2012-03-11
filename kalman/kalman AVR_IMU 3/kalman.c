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
#include "imu.h"
#include "kalman.h"

#if USE_KALMAN

// Temp values used during Kalman update.
static float s_00;
static float inn_00;
static float K_00;
static float K_10;
static float AP_00;
static float AP_01;
static float AP_10;
static float AP_11;
static float APAT_00;
static float APAT_01;
static float APAT_10;
static float APAT_11;
static float KCPAT_00;
static float KCPAT_01;
static float KCPAT_10;
static float KCPAT_11;

void kalman_init(kalman *self, float dt, float Sz_00, float Sw_00, float Sw_11)
// Initialize the kalman state.
{
    // Initialize outputs.
    self->x_00 = 0.0;
    self->x_10 = 0.0;

    // Initialize covariance of estimate state.
    self->P_00 = 1.0;
    self->P_01 = 0.0;
    self->P_10 = 0.0;
    self->P_11 = 1.0;

    // Set the delta in seconds between samples.
    self->A_01 = -dt;
    self->B_00 = dt;

    // Set the measurement noise covariance matrix values.
    self->Sz_00 = Sz_00;

    // Set the process noise covariance matrix values.
    self->Sw_00 = Sw_00;
    self->Sw_11 = Sw_11;
}

float kalman_update(kalman *self, float gyro_rate, float accel_angle)
// Update the state estimation and compute the Kalman gain.
// The estimated angle from the output matrix is returned.
{
    // Update the state estimate by extrapolating current state estimate with input u.
    // x = A * x + B * u
    self->x_00 += (self->A_01 * self->x_10) + (self->B_00 * gyro_rate);

    // Compute the innovation -- error between measured value and state estimate.
    // inn = y - c * x
    inn_00 = accel_angle - self->x_00;

    // Compute the covariance of the innovation.
    // s = C * P * C' + Sz
    s_00 = self->P_00 + self->Sz_00;

    // Compute AP matrix for use below.
    // AP = A * P
    AP_00 = self->P_00 + self->A_01 * self->P_10;
    AP_01 = self->P_01 + self->A_01 * self->P_11;
    AP_10 = self->P_10;
    AP_11 = self->P_11;

    // Compute the kalman gain matrix.
    // K = A * P * C' * inv(s)
    K_00 = AP_00 / s_00;
    K_10 = AP_10 / s_00;

    // Update the state estimate.
    // x = x + K * inn
    self->x_00 += K_00 * inn_00;
    self->x_10 += K_10 * inn_00;

    // Compute the new covariance of the estimation error.
    // P = A * P * A' - K * C * P * A' + Sw
    APAT_00 = AP_00 + (AP_01 * self->A_01);
    APAT_01 = AP_01;
    APAT_10 = AP_10 + (AP_11 * self->A_01);
    APAT_11 = AP_11;
    KCPAT_00 = (K_00 * self->P_00) + (K_00 * self->P_01) * self->A_01;
    KCPAT_01 = (K_00 * self->P_01);
    KCPAT_10 = (K_10 * self->P_00) + (K_10 * self->P_01) * self->A_01;
    KCPAT_11 = (K_10 * self->P_01);
    self->P_00 = APAT_00 - KCPAT_00 + self->Sw_00;
    self->P_01 = APAT_01 - KCPAT_01;
    self->P_10 = APAT_10 - KCPAT_10;
    self->P_11 = APAT_11 - KCPAT_11 + self->Sw_11;

    // Return the estimate.
    return self->x_00;
}

#endif // USE_KALMAN
