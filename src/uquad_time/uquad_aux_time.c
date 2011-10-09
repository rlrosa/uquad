#include <uquad_aux_time.h>

/** 
 * Subtract the `struct timeval' values X and Y, result in RESULT.
 *
 * Source: http://www.cs.utah.edu/dept/old/texinfo/glibc-manual-0.02/library_19.html
 * 
 * @param result 
 * @param x 
 * @param y 
 * 
 * @return -1 if the diff is negative, 0 if diff is zero, 1 if the diff is positive.
 */
int uquad_timeval_substract (struct timeval * result, struct timeval x, struct timeval y){
    /* Perform the carry for the later subtraction by updating y. */
    if (x.tv_usec < y.tv_usec) {
	int nsec = (y.tv_usec - x.tv_usec) / 1000000 + 1;
	y.tv_usec -= 1000000 * nsec;
	y.tv_sec += nsec;
    }
    if (x.tv_usec - y.tv_usec > 1000000) {
	int nsec = (y.tv_usec - x.tv_usec) / 1000000;
	y.tv_usec += 1000000 * nsec;
	y.tv_sec -= nsec;
    }
    
    /* Compute the time remaining to wait.
       tv_usec is certainly positive. */
    result->tv_sec = x.tv_sec - y.tv_sec;
    result->tv_usec = x.tv_usec - y.tv_usec;
    
    if(x.tv_sec < y.tv_sec)
	// -1 if diff is negative
	return -1;
    if(x.tv_sec > y.tv_sec)
	// 1 if diff is positive
	return 1;
    // 0 if equal
    return 0;
}
