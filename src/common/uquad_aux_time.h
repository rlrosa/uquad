#ifndef UQUAD_AUX_TIME_H
#define UQUAD_AUX_TIME_H

#include <sys/time.h>

/** 
 * Calculate the difference between two time vals, without loosing precision.
 * 
 * @param result 
 * @param x 
 * @param y 
 * 
 * @return error code.
 */
int uquad_timeval_substract (struct timeval * result, struct timeval x, struct timeval y);

#endif
