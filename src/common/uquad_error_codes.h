#ifndef UQUAD_ERROR_CODES_H
#define UQUAD_ERROR_CODES_H

#include <stdio.h>
#include <errno.h>
#include <string.h>

#define DEBUG 1

enum UQUAD_ERROR_CODES{
ERROR_OK = 0,
ERROR_FAIL,
ERROR_READ_TIMEOUT,
ERROR_READ_SYNC,
ERROR_READ_SKIP,
ERROR_READ,
ERROR_OPEN,
ERROR_CLOSE,
ERROR_MALLOC,
ERROR_IMU_AVG_NOT_ENOUGH,
ERROR_IMU_NO_UPDATES,
ERROR_IO,
ERROR_INVALID_PIPE_NAME,
ERROR_WRITE,
ERROR_INVALID_ARG,
ERROR_NULL_POINTER,
ERROR_IMU_STATUS,
ERROR_IMU_NOT_CALIB,
ERROR_IO_DEV_NOT_FOUND,
ERROR_GPS,
ERROR_GPS_OPEN,
ERROR_GPS_STREAM,
ERROR_GPS_NO_UPDATES,
ERROR_GPS_NO_VEL,
ERROR_GPS_SYS_REF,
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
ERROR_MOT_SATURATE,
ERROR_TIMING
};

/**
 * Print error message to stderr
 * 
 */
#define err_log(msg) fprintf(stderr,"%s:%s:%d: %s\n",__TIME__,__FILE__,__LINE__,msg)
#define err_log_stderr(msg) fprintf(stderr,"%s:%s:%d: %s: %s\n",__TIME__,__FILE__,__LINE__,msg, strerror(errno))

/**
 * Print error message with number to stderr
 * 
 */
#define err_log_num(msg,num) fprintf(stderr,"%s:%s:%d: %s(%d)\n",__TIME__,__FILE__,__LINE__,msg,num)

/**
 * Print error message with double to stderr
 *
 */
#define err_log_double(msg,dbl) fprintf(stderr,"%s:%s:%d: %s: %lf\n",__TIME__,__FILE__,__LINE__,msg,dbl)

/**
 * Print error message with double to log
 *
 */
#define log_double(log,msg,dbl) fprintf(log,"%s:%s:%d: %s: %lf\n",__TIME__,__FILE__,__LINE__,msg,dbl)

#define log_eol(log) fprintf(log,"\n")

/**
 * Print timeval message with unsigned long to stderr
 * 
 */
#define err_log_tv(msg,tv) fprintf(stderr,"%s:%s:%d: %s(%ld.%06ld)\n",__TIME__,__FILE__,__LINE__,msg,tv.tv_sec, tv.tv_usec)

/**
 * Print timeval to log with unsigned long to log
 * 
 */
#define log_tv(log,msg,tv) fprintf(log,"%s:%s:%d: %s(%ld.%06ld)\n",__TIME__,__FILE__,__LINE__,msg,tv.tv_sec, tv.tv_usec)

/**
 * Print timeval to log with unsigned long to log, no '\n'
 * 
 */
#define log_tv_only(log,tv) fprintf(log,"%ld.%06ld\t",tv.tv_sec, tv.tv_usec)

/**
 * Print int to log ended by tab
 * 
 */
#define log_int_only(log,db) fprintf(log,"%d\t",db)

/**
 * Print double to log ended by tab
 * 
 */
#define log_double_only(log,db) fprintf(log,"%lf\t",db)

/**
 * If @retval is an error, then propagate error without printing anything.
 * 
 */
#define err_propagate(retval) if(retval!=ERROR_OK){fprintf(stderr,"backtrace:%s:%d\n",__FILE__,__LINE__);return retval;}

/**
 * If @retval is an error, then print @msg to stderr and propagate error.
 * 
 */
#define err_check(retval,msg) if(retval!=ERROR_OK){fprintf(stderr,"%s:%s:%d: %s\n",__TIME__,__FILE__,__LINE__,msg);return retval;}

/**
 * If @retval is an error, then print strerr to stderr and propagate error.
 *
 */
#define err_check_std(retval) if(retval!=ERROR_OK){fprintf(stderr,"%s:%s:%d: %s\n",__TIME__,__FILE__,__LINE__,strerror(errno));return retval;}

/**
 * If @retval is an error, then print strerr to stderr.
 *
 */
#define err_log_std(retval) if(retval!=ERROR_OK){fprintf(stderr,"%s:%s:%d: %s\n",__TIME__,__FILE__,__LINE__,strerror(errno));}

/**
 * If @retval is an error, call quit().
 * Usefull in test programs, allows cleaning up.
 * 
 */
#define quit_if(retval) if(retval!=ERROR_OK)quit()

/**
 * If @retval is an error, jump to cleanup.
 * Usefull in init routines, allows cleaning up.
 *
 */
#define cleanup_if(retval) if(retval != ERROR_OK) {fprintf(stderr,"backtrace:%s:%d\n",__FILE__,__LINE__);goto cleanup;}

/**
 * If @retval is an error, jump to cleanup.
 * Usefull in init routines, allows cleaning up.
 *
 */
#define cleanup_if_null(ptr) if(ptr == NULL) {fprintf(stderr,"%s:%s:%d: Null pointer!\n",__TIME__,__FILE__,__LINE__);goto cleanup;}

/**
 * If @retval is an error, jump to cleanup.
 * Usefull in init routines, allows cleaning up.
 *
 */
#define cleanup_log_if(retval,msg) if(retval != ERROR_OK) {fprintf(stderr,"%s:%d:\t%s\n",__FILE__,__LINE__,msg);goto cleanup;}

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
 * If @retval is an error, then log and jump to label
 * 
 */
#define log_n_jump(retval,label,msg) if(retval!=ERROR_OK){err_log(msg);goto label;}

/**
 * Verifies that malloc succeeded.
 * 
 */
#define mem_alloc_check(pointer) if(pointer==NULL){fprintf(stderr,"%s:%s:%d: malloc failed\n",__TIME__,__FILE__,__LINE__);return NULL;}

/// No functions

#endif //UQUAD_ERROR_CODES_H
