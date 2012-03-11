//
// This code is from the Autopilot project and implements an
// Extended Kalman Filter (EKF) where the model isn't linear
// functions of the state, but are differentiable functions.
// Note that a matrix of partial derivatives (the Jacobian) is
// computed in the code below.
//
// Original code from: http://www.rotomotion.com/downloads/tilt.c
//
// The original comments from the code have been preserved to
// the extend possible.  Also note that this code is licensed
// under GPL.
//
// Original comments below:
//
// 1 dimensional tilt sensor using a dual axis accelerometer
// and single axis angular rate gyro.  The two sensors are fused
// via a two state Kalman filter, with one state being the angle
// and the other state being the gyro bias.
//
// Gyro bias is automatically tracked by the filter.  This seems
// like magic.
//
// Please note that there are lots of comments in the functions and
// in blocks before the functions.  Kalman filtering is an already complex
// subject, made even more so by extensive hand optimizations to the C code
// that implements the filter.  I've tried to make an effort of explaining
// the optimizations, but feel free to send mail to the mailing list,
// autopilot-devel@lists.sf.net, with questions about this code.
//
// (c) 2003 Trammell Hudson <hudson@rotomotion.com>
//
//  This file is part of the autopilot onboard code package.
//
//  Autopilot is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  Autopilot is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with Autopilot; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include <inttypes.h>
#include "imu.h"
#include "tilt.h"

#if USE_TILT

void tilt_init(tilt *self, float dt, float R_angle, float Q_gyro, float Q_angle)
// Initialize the kalman state.
{
    // Initialize the two states, the angle and the gyro bias.  As a
    // byproduct of computing the angle, we also have an unbiased
    // angular rate available.
    self->bias = 0.0;
    self->rate = 0.0;
    self->angle = 0.0;

    // Initialize the delta in seconds between gyro samples.
    self->dt = dt;

    // Initialize the measurement noise covariance matrix values.
    // In this case, R is a 1x1 matrix tha represents expected
    // jitter from the accelerometer.  Original Autopilot software
    // had this value set to 0.3.
    self->R_angle = R_angle;

    // Initialize the process noise covariance matrix values.
    // In this case, Q indicates how much we trust the acceleromter
    // relative to the gyros.  Original Autopilot software had
    // Q_gyro set to 0.003 and Q_angle set to 0.001.
    self->Q_gyro = Q_gyro;
    self->Q_angle = Q_angle;

    // Initialize covariance of estimate state.  This is updated
    // at every time step to determine how well the sensors are
    // tracking the actual state.
    self->P_00 = 1.0;
    self->P_01 = 0.0;
    self->P_10 = 0.0;
    self->P_11 = 1.0;
}

void tilt_state_update(tilt *self, float gyro_rate)
// tilt_state_update() is called every dt with a biased gyro
// measurement by the user of the module.  It updates the current
// angle and rate estimate.
//
// The pitch gyro measurement should be scaled into real units, but
// does not need any bias removal.  The filter will track the bias.
//
// Our state vector is:
//
//  X = [ angle, gyro_bias ]
//
// It runs the state estimation forward via the state functions:
//
//  Xdot = [ angle_dot, gyro_bias_dot ]
//
//  angle_dot = gyro - gyro_bias
//  gyro_bias_dot = 0
//
// And updates the covariance matrix via the function:
//
//  Pdot = A*P + P*A' + Q
//
// A is the Jacobian of Xdot with respect to the states:
//
//  A = [ d(angle_dot)/d(angle)     d(angle_dot)/d(gyro_bias) ]
//      [ d(gyro_bias_dot)/d(angle) d(gyro_bias_dot)/d(gyro_bias) ]
//
//    = [ 0 -1 ]
//      [ 0  0 ]
//
// Due to the small CPU available on the microcontroller, we've
// hand optimized the C code to only compute the terms that are
// explicitly non-zero, as well as expanded out the matrix math
// to be done in as few steps as possible.  This does make it
// harder to read, debug and extend, but also allows us to do
// this with very little CPU time.
{
    // Static so these are kept off the stack.
    static float gyro_rate_unbiased;
    static float Pdot_00;
    static float Pdot_01;
    static float Pdot_10;
    static float Pdot_11;

    // Unbias our gyro.
    gyro_rate_unbiased = gyro_rate - self->bias;

    // Compute the derivative of the covariance matrix
    //
    // Pdot = A*P + P*A' + Q
    //
    // We've hand computed the expansion of A = [ 0 -1, 0 0 ] multiplied
    // by P and P multiplied by A' = [ 0 0, -1, 0 ].  This is then added
    // to the diagonal elements of Q, which are Q_angle and Q_gyro.
    Pdot_00 = self->Q_angle - self->P_01 - self->P_10;
    Pdot_01 = -self->P_11;
    Pdot_10 = -self->P_11;
    Pdot_11 = self->Q_gyro;

    // Store our unbiased gyro estimate.
    self->rate = gyro_rate_unbiased;

    // Update the angle estimate.
    //
    // angle += angle_dot * dt
    //       += (gyro - gyro_bias) * dt
    //       += q * dt
    //
    self->angle += gyro_rate_unbiased * self->dt;

    // Update the covariance matrix.
    self->P_00 += Pdot_00 * self->dt;
    self->P_01 += Pdot_01 * self->dt;
    self->P_10 += Pdot_10 * self->dt;
    self->P_11 += Pdot_11 * self->dt;
}

void tilt_kalman_update(tilt *self, float angle_measured)
// tilt_kalman_update() is called by a user of the module when a
// new accelerometer angle measurement is available.
//
// This does not need to be called every time step, but can be if
// the accelerometer data are available at the same rate as the
// rate gyro measurement.
//
// As commented in state_update, the math here is simplified to
// make it possible to execute on a small microcontroller with no
// floating point unit.  It will be hard to read the actual code and
// see what is happening.
//
// The C matrix is a 1x2 (measurements x states) matrix that
// is the Jacobian matrix of the measurement value with respect
// to the states.  In this case, C is:
//
//  C = [ d(angle_measured)/d(angle)  d(angle_measured)/d(gyro_bias) ]
//    = [ 1 0 ]
//
// because the angle measurement directly corresponds to the angle
// estimate and the angle measurement has no relation to the gyro
// bias.
{
    // Static so these are kept off the stack.
    static float angle_error;
    static float C_0;
    static float PCt_0;
    static float PCt_1;
    static float E;
    static float K_0;
    static float K_1;
    static float t_0;
    static float t_1;

    // Compute the error in the estimate.
    angle_error = angle_measured - self->angle;

    // C_0 shows how the state measurement directly relates to
    // the state estimate.
    //
    // The C_1 shows that the state measurement does not relate
    // to the gyro bias estimate.  We don't actually use this, so
    // we comment it out.
    C_0 = 1.0;
    // C_1 = 0.0;

    // PCt<2,1> = P<2,2> * C'<2,1>, which we use twice.  This makes
    // it worthwhile to precompute and store the two values.
    // Note that C[0,1] = C_1 is zero, so we do not compute that term.
    PCt_0 = C_0 * self->P_00; /* + C_1 * P_01 = 0 */
    PCt_1 = C_0 * self->P_10; /* + C_1 * P_11 = 0 */

    // Compute the error estimate.  From the Kalman filter paper:
    //
    // E = C P C' + R
    //
    // Dimensionally,
    //
    //  E<1,1> = C<1,2> P<2,2> C'<2,1> + R<1,1>
    //
    // Again, note that C_1 is zero, so we do not compute the term.
    E = self->R_angle + C_0 * PCt_0; /* + C_1 * PCt_1 = 0 */

    // Compute the Kalman filter gains.  From the Kalman paper:
    //
    // K = P C' inv(E)
    //
    // Dimensionally:
    //
    // K<2,1> = P<2,2> C'<2,1> inv(E)<1,1>
    //
    // Luckilly, E is <1,1>, so the inverse of E is just 1/E.
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    //
    // Update covariance matrix.  Again, from the Kalman filter paper:
    //
    // P = P - K C P
    //
    // Dimensionally:
    //
    // P<2,2> -= K<2,1> C<1,2> P<2,2>
    //
    // We first compute t<1,2> = C P.  Note that:
    //
    // t[0,0] = C[0,0] * P[0,0] + C[0,1] * P[1,0]
    //
    // But, since C_1 is zero, we have:
    //
    // t[0,0] = C[0,0] * P[0,0] = PCt[0,0]
    //
    // This saves us a floating point multiply.
    t_0 = PCt_0; /* C_0 * P[0][0] + C_1 * P[1][0] */
    t_1 = C_0 * self->P_01; /* + C_1 * P[1][1]  = 0 */

    self->P_00 -= K_0 * t_0;
    self->P_01 -= K_0 * t_1;
    self->P_10 -= K_1 * t_0;
    self->P_11 -= K_1 * t_1;

    //
    // Update our state estimate.  Again, from the Kalman paper:
    //
    // X += K * err
    //
    // And, dimensionally,
    //
    // X<2> = X<2> + K<2,1> * err<1,1>
    //
    // err is a measurement of the difference in the measured state
    // and the estimate state.  In our case, it is just the difference
    // between the two accelerometer measured angle and our estimated
    // angle.
    self->bias  += K_1 * angle_error;
    self->angle += K_0 * angle_error;
}

#endif // USE_TILT
