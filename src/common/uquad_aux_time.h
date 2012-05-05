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
