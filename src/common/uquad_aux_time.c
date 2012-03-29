#include <uquad_aux_time.h>

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

int in_range_us(struct timeval tv_diff, long int min_us, long int max_us)
{
    int retval = 0;
    if(tv_diff.tv_sec > 0       ||
       tv_diff.tv_usec > max_us ||
       tv_diff.tv_usec < min_us)
    {
	if(tv_diff.tv_sec > 0 || tv_diff.tv_usec > max_us)
	    retval = 1;
	else
	    retval = -1;
    }
    return retval;
}
