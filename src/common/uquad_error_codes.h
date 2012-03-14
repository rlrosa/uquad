#ifndef UQUAD_ERROR_CODES_H
#define UQUAD_ERROR_CODES_H

#include <stdio.h>
#include <errno.h>
#include <string.h>

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
ERROR_IMU_NOT_CALIB,
ERROR_IO_DEV_NOT_FOUND,
ERROR_GPS_OPEN,
ERROR_GPS_STREAM,
ERROR_MATH_MAX_DIM,
ERROR_MATH_MAT_DIM,
ERROR_MATH_VEC_LEN,
ERROR_MATH_MAT_SING,
ERROR_MATH_UNDERFLOWS,
ERROR_MATH_OVERFLOWS,
ERROR_MATH_DIV_0,
ERROR_MOTOR_CMD_START,
ERROR_MOTOR_CMD_KILL,
ERROR_MOTOR_SET,
ERROR_MOTOR_USAGE,
ERROR_MOTOR_W,
ERROR_KQ,
ERROR_KQ_ACK_NONE,
ERROR_KQ_ACK_TOO_MANY,
ERROR_KQ_ACK_MORE,
ERROR_KQ_NO_ACKS_AVAIL,
ERROR_KQ_SEND,
ERROR_MOT_SATURATE
};

/**
 * Print error message to stderr
 * 
 */
#define err_log(msg) fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,msg)
#define err_log_stderr(msg) fprintf(stderr,"%s:%d: %s: %s\n",__FILE__,__LINE__,msg, strerror(errno))

/**
 * Print error message with number to stderr
 * 
 */
#define err_log_num(msg,num) fprintf(stderr,"%s:%d: %s(%d)\n",__FILE__,__LINE__,msg,num)

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
 * If @retval is an error, call quit().
 * Usefull in test programs, allows cleaning up.
 * 
 */
#define quit_if(retval) if(retval!=ERROR_OK)quit()

/**
 * If @retval is an error, call quit().
 * Usefull in test programs, allows cleaning up.
 * 
 */
#define quit_log_if(retval,msg) if(retval!=ERROR_OK){err_log(msg);quit();}

/**
 * If @retval is an error, then got to the beginning of the loop
 * 
 */
#define log_n_continue(retval,msg) if(retval!=ERROR_OK){err_log(msg);continue;}

/**
 * Verifies that malloc succeeded.
 * 
 */
#define mem_alloc_check(pointer) if(pointer==NULL){fprintf(stderr,"%s:%d: malloc failed\n",__FILE__,__LINE__);return NULL;}

/// No functions

#endif //UQUAD_ERROR_CODES_H
