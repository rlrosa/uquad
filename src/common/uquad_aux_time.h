/**
 * uquad_time: lib with aux functions to help timevale handling.
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
 * @file   uquad_aux_time.h
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 11:20:03 2012
 *
 * @brief lib with aux functions to help timevale handling.
 *
 * Examples:
 *   - src/main/main.c
 *   - test/uquad_aux_time_test/uquad_aux_time_test.c
 */
#ifndef UQUAD_AUX_TIME_H
#define UQUAD_AUX_TIME_H

#include <sys/time.h>
#include <unistd.h> // for usleep()

#define double2tv(tv,db)						\
    {									\
	tv.tv_sec  = (long int) floor(db);				\
	tv.tv_usec = (long int) (1e6*(db - sign(db)*floor(db)));	\
    }

#define tv2double(db,tv)					\
    {								\
	db = ((double) tv.tv_sec) + ((double) tv.tv_usec)/1e6;	\
    }								\

#define sleep_ms(ms) usleep(1000*ms)

/** 
 * Calculate the difference between two time vals, without losing precision.
 * 
 * Source: http://www.cs.utah.edu/dept/old/texinfo/glibc-manual-0.02/library_19.html
 *
 * @param result: x-y
 * @param x 
 * @param y 
 * 
 * @return -1 if the diff is negative, 0 if diff is zero, 1 if the diff is positive.
 */
int uquad_timeval_substract (struct timeval * result, struct timeval x, struct timeval y);

/**
 * Verifies that a time diff falls within a given range, in microseconds.
 *
 * @param tv_diff Difference calculated using uquad_timeval_substract()
 * @param min_us Range min in microseconds.
 * @param max_us Range max in microseconds.
 *
 * @return 0 iif in range, -1 iif less than min_us, 1 iif more than max_us
 */
int in_range_us(struct timeval tv_diff, long int min_us, long int max_us);

#endif
