#ifndef UQUAD_AUX_TIME_H
#define UQUAD_AUX_TIME_H

#include <sys/time.h>

int uquad_timeval_substract (struct timeval * result, struct timeval x, struct timeval y);

#endif
