/**
 * uquad_config: main configuration.
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
 * @file   uquad_config.h
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 11:20:03 2012
 *
 * @brief main configuration.
 */
#ifndef UQUAD_CONFIG_H
#define UQUAD_CONFIG_H

#include <uquad_types.h>

/**
 * Show debug information on ssh session, and log
 * data to log files.
 * Should not disturb timing, but may be disabled
 * if more computation time is required.
 */
#define DEBUG             1

/**
 * Use fake GPS information to help state estimation.
 * If GPS_RAND is set to 0, the quadcopter will think
 * it is at [x,y,z] = [0,0,0], otherwise it will use
 * a random value centered around it.
 */
#define GPS_FAKE          1 // Simulate GPS data
#define GPS_RAND          0 // Use random values centered at [0,0,0]

/**
 * Use kalman estimation of acc bias.
 */
#define KALMAN_BIAS       1

/**
 * Use PI control on:
 *   [SV_X, SV_Y, SV_Z,SV_THETA]
 */
#define CTRL_INTEGRAL     1

/**
 * Apply a PI on:
 *   [SV_PSI, SV_PHI, SV_Z, SV_THETA]
 * Must enable CTRL_INTEGRAL to be able to use this, and will disable
 * proportional control on:
 *   [SV_X, SV_Y, SV_Z, SV_VQX, SV_VQY, SV_VQZ]
 * This is a requirement, otherwise system will not be controllable.
 */
#define CTRL_INTEGRAL_ANG 1

/**
 * Apply control on all 12 states, as documented in uquad_kalman.h
 *
 */
#define FULL_CONTROL      1

/**
 * Ratio of [samples]/[control action].
 * Setting to 1 will apply a control action every time a new sample
 * is received from the IMU.
 */
#define CTRL_TS           1

/**
 * Setting the following will allow main to run with no connection ping
 * to server (laptop).
 * This is not recommended, since it will NOT be possible to stop the motors
 * if wifi dies.
 */
#define CHECK_NET_BYPASS  0

// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
// Do NOT edit the following lines!
// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
#if FULL_CONTROL
#define STATES_CONTROLLED     STATE_COUNT
#define STATES_INT_CONTROLLED 4
#else
#define STATES_CONTROLLED     8
#define STATES_INT_CONTROLLED 2
#endif

#if !KALMAN_BIAS
#error								\
    "Kalman bias removal not implemented correctly for H()"
#endif // !KALMAN_BIAS

#if (!USE_GPS && GPS_ZERO)
#error							\
    "Cannot use fake GPS without USE_GPS enabled!"	\
    "Edit src/CMakelists.txt to enable USE_GPS"
#endif // (!USE_GPS && GPS_ZERO)

#if (!USE_GPS && FULL_CONTROL)
#error						\
    "Cannot use FULL_CONTROL without USE_GPS!"
#endif // (!USE_GPS && FULL_CONTROL)

#if (USE_GPS && !FULL_CONTROL)
#error						\
    "Without FULL_CONTROL, GPS is useless!"
#endif // (!USE_GPS && FULL_CONTROL)

#if (CTRL_INTEGRAL_ANG && !CTRL_INTEGRAL)
#error							\
    "Cannot use PI on angles without CTRL_INTEGRAL "
#endif // (CTRL_INTEGRAL_ANG && !CTRL_INTEGRAL)

#if CHECK_NET_BYPASS
#warning  "check net bypassed!"
#endif // CHECK_NET_BYPASS

#if KALMAN_BIAS
#define STATE_BIAS        3
#else
#define STATE_BIAS        0
#endif

#if (CTRL_TS < 1)
#error						\
    "Must be CTRL_TS >= 1"
#endif // (CTRL_TS < 1)

#endif // UQUAD_CONFIG_H
