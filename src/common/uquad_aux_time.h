#ifndef UQUAD_AUX_TIME_H
#define UQUAD_AUX_TIME_H

#include <sys/time.h>
#include <unistd.h> // for usleep()

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

#define sleep_ms(ms) usleep(1000*ms)

#endif
