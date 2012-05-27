/**
 * manual_mode: macros for controlling main from keyboard.
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
 * @file   manual_mode.h
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 11:20:03 2012
 *
 * @brief macros for controlling main from keyboard.
 */
#ifndef MANUAL_MODE_H
#define MANUAL_MODE_H

#define MANUAL_MODE          'm'
#define MANUAL_PSI_INC       'j'
#define MANUAL_PSI_DEC       'g'
#define MANUAL_PHI_INC       'y'
#define MANUAL_PHI_DEC       'h'
#define MANUAL_THETA_INC     'q'
#define MANUAL_THETA_DEC     'w'
#define MANUAL_EULER_STEP    (0.045) // [rad]
#define MANUAL_WEIGHT        '0'     // reset motor speed to MOT_W_HOVER
#define MANUAL_WEIGHT_INC    'i'
#define MANUAL_WEIGHT_DEC    'k'
#define MANUAL_WEIGHT_STEP   (0.05)   // [kg]
#define MANUAL_Z_INC         'e'
#define MANUAL_Z_DEC         'd'
#define MANUAL_Z_STEP        (0.2)   // [m]

#endif // MANUAL_MODE_H
