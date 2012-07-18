/**
 * uquad_types: General definitions used all over the code.
 * Copyright (C) 2012  Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @file   uquad_types.c
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 11:20:03 2012
 *
 * @brief general definitions used all over the code.
 *
 */
#ifndef UQUAD_TYPES_H
#define UQUAD_TYPES_H

#ifndef UQUAD_BOOL
  #define UQUAD_BOOL int
  #ifndef true
    #define true 1
    #define false 0
  #endif //true
#endif //UQUAD_BOOL
typedef UQUAD_BOOL uquad_bool_t;

typedef enum STATE_VECTOR{
    /// Cartesian coordinates
    SV_X = 0, /* [m]     */
    SV_Y,     /* [m]     */
    SV_Z,     /* [m]     */
    /// Euler angles
    SV_PSI,   /* [rad]   */
    SV_PHI,   /* [rad]   */
    SV_THETA, /* [rad]   */
    /// Velocity - Non-inertial
    SV_VQX,   /* [m/s]   */
    SV_VQY,   /* [m/s]   */
    SV_VQZ,   /* [m/s]   */
    /// Angular velocity - Non-inertial
    SV_WQX,   /* [rad/s] */
    SV_WQY,   /* [rad/s] */
    SV_WQZ,   /* [rad/s] */
    /// Accelerometer bias estimation
    SV_BAX,   /* [m/s^2] */
    SV_BAY,   /* [m/s^2] */
    SV_BAZ    /* [m/s^2] */
}STATE_VECTOR_T;

#define GRAVITY           9.81 	      // Gravity
#define IXX               0.0232      // Quad intertia tensor - x
#define IYY               0.0232      // Quad intertia tensor - y
#define IZZ               0.0437      // Quad intertia tensor - z
#define IZZM              0.0000154   // Motor Inertia - z
#define LENGTH            0.29	      // Length of quadcopter arm [m]
#define D_CENTER_MASS_M   0.07        // Distance from center to center of mass [m]
#define PI                3.1415926
#define F_B1              4.60160135072435e-05     // Coeficiente cuadrático de la fuerza
#define F_B2              -0.00103822726273726     // Coeficiente lineal de la fuerza
#define M_D1              3.4734e-6   // Torque = w^2*M_D1 + w*M_D2
#define M_D2              -1.3205e-4  // Torque = w^2*M_D1 + w*M_D2
#define DRAG_A1           0.0000034734// drag = w^2*A2 + w*A1
#define DRAG_A2           -0.00013205 // drag = w^2*A2 + w*A1
#define DRIVE_A1          4.60160135072435e-05 // drive = w^2*A2 + w*A1
#define DRIVE_A2          -0.00103822726273726 // drive = w^2*A2 + w*A1
#define MASA_DEFAULT      (1.700)     // Weight of the quadcopter [kg]
#define STATE_COUNT       12          // State vector length
#define LENGTH_INPUT      4           // Input vector length
#define GPS_RAND_PP       1.0         // Random values of +-GPS_RAND_PP/2.0

/**
 * Definition of default sampling period, in microseconds.
 * Should match code running on IMU.
 */
#define TS_DEFAULT_US     10000L
#define TS_DEFAULT_US_DBL ((double)TS_DEFAULT_US)

/**
 *
 * Sampling time within [TS_MIN,TS_MAX] will be used for
 * kalman filtering. If out of range, using it within the kalman filter
 * would force a violent reaction, and the stabilization time would
 * be unacceptable.
 *
 */
#define TS_JITTER          2000L // Max jitter accepted
#define TS_ERROR_WAIT      10    // Wait 10 errors before logging again
#define TS_MAX             (TS_DEFAULT_US + TS_JITTER)
#define TS_MIN             (TS_DEFAULT_US - TS_JITTER)

/**
 * Limits for sanity check.
 * If the state estimation is out of bounds for SANITY_MAX samples, then
 * some action should be taken (abort, etc). If out of bound, our linear
 * model will not be valid, and the chances of recovering are very low, so
 * it's probably better to fall with the motores turned off...
 * Possibles causes:
 *     - psi/phi: Diverging controller, wind, impact against something.
 *     - temp   : short in the ESCs, fire, etc.
 */
#define SANITY_MAX      20
#define SANITY_MAX_TEMP 55.0 // [°C]
#define SANITY_MAX_PSI  1.0  // [rad]
#define SANITY_MAX_PHI  1.0  // [rad]

#endif
