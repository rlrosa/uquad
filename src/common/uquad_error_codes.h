#ifndef UQUAD_ERROR_CODES_H
#define UQUAD_ERROR_CODES_H

#include <stdio.h>

enum UQUAD_ERROR_CODES{
ERROR_OK = 0,
ERROR_FAIL,
ERROR_READ_TIMEOUT,
ERROR_READ_SYNC,
ERROR_READ_SKIP,
ERROR_OPEN,
ERROR_CLOSE,
ERROR_MALLOC,
ERROR_IMU_AVG_NOT_ENOUGH,
ERROR_IO,
ERROR_WRITE,
ERROR_INVALID_ARG,
ERROR_NULL_POINTER,
ERROR_IMU_STATUS,
ERROR_IO_DEV_NOT_FOUND,
ERROR_GPS_OPEN,
ERROR_GPS_STREAM,
ERROR_MATH_MAX_DIM,
ERROR_MATH_MAT_DIM,
ERROR_MATH_VEC_LEN,
ERROR_MATH_MAT_SING,
ERROR_MATH_DIV_0
};

/**
 * Print error message to stderr
 * 
 */
#define err_log(msg) fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,msg)

/**
 * If @retval is an error, then propagate error without printing anything.
 * 
 */
#define err_propagate(retval) if(retval!=ERROR_OK){fprintf(stderr,"backtrace:%s:%d\n",__FILE__,__LINE__);return retval;}

/**
 * If @retval is an error, then print @msg to stderr and propagate error.
 * 
 */
#define err_check(retval,msg) if(retval!=ERROR_OK){fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,msg);return retval;}


/**
 * Verifies that malloc succeeded.
 * 
 */
#define mem_alloc_check(pointer) if(pointer==NULL){fprintf(stderr,"%s:%d: malloc failed\n",__FILE__,__LINE__);return NULL;}

/// No functions

#endif //UQUAD_ERROR_CODES_H
