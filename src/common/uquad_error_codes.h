/**
 * uquad_error_codes: macros for error handling
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
 * @file   uquad_error_codes.h
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 11:20:03 2012
 *
 * @brief macros for error handling
 */
#ifndef UQUAD_ERROR_CODES_H
#define UQUAD_ERROR_CODES_H

/**
 * If stderr has been re-routed, for example to
 * log to a file and still be able to use the
 * macros below, then enabling this will print
 * everything that is considered an error to both:
 *   - whatever stderr was re-reouted to
 *   - stdout
 * To re-route stderr, do something like:
 *   FILE *log = fopen("log_file.log","w");
 *   stderr    = log;
 * The err_log("something") will log to log.
 */
#define REROUTE_STDERR 1

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>

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
    ERROR_IMU_FILTER_LEN_NOT_ENOUGH,
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
    ERROR_GPS_NO_FIX,
    ERROR_GPS_SYS_REF,
    ERROR_MATH_MAX_DIM,
    ERROR_MATH_MAT_DIM,
    ERROR_MATH_VEC_LEN,
    ERROR_MATH_MAT_SING,
    ERROR_MATH_UNDERFLOWS,
    ERROR_MATH_OVERFLOWS,
    ERROR_MATH_DIV_0,
    ERROR_MATH_NEGATIVE,
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
    ERROR_IPC,
    ERROR_MOT_SATURATE,
    ERROR_TIMING
};

#define rerouted() (fileno(stderr)!=STDERR_FILENO)

/**
 * Print error message to stderr
 *
 */
#define err_log(msg)					\
    {							\
	log_msg(stderr,msg);				\
	if(REROUTE_STDERR && rerouted())		\
	    log_msg(stdout,msg);			\
    }

/**
 * Print error message to log
 *
 */
#define log_msg(log,msg)					\
    {								\
	fprintf(log,"%s:%d: %s\n\r",				\
		__FILE__,__LINE__,msg);				\
    }

/**
 * Print error message to stderr
 *
 */
#define err_log_only(msg)					\
    {								\
	log_msg_only(stderr,msg);				\
	if(REROUTE_STDERR && rerouted())			\
	    log_msg_only(stdout,msg);				\
    }

/**
 * Print error message to log
 *
 */
#define log_msg_only(log,msg)					\
    {								\
	fprintf(log,"%s:%d: %s\t",				\
		__FILE__,__LINE__,msg);				\
    }

#define err_log_stderr(msg)						\
    {									\
	fprintf(stderr,"%s:%d: %s: %s\n\r",				\
		__FILE__,__LINE__,msg, strerror(errno));		\
	if(REROUTE_STDERR && rerouted())				\
	    fprintf(stdout,"%s:%d: %s: %s\n\r",			\
		    __FILE__,__LINE__,msg, strerror(errno));	\
    }

/**
 * Print error message with number to stderr
 * 
 */
#define err_log_num(msg,num)					\
    {								\
	fprintf(stderr,"%s:%d: %s(%d)\n\r",			\
		__FILE__,__LINE__,msg,num);			\
	if(REROUTE_STDERR && rerouted())			\
	    fprintf(stdout,"%s:%d: %s(%d)\n\r",			\
		    __FILE__,__LINE__,msg,num);			\
    }

/**
 * Print error message with number to stderr
 *
 */
#define err_log_num_num(msg,num1,num2)				\
    {								\
	fprintf(stderr,"%s:%d: %s(%d/%d)\n\r",			\
		__FILE__,__LINE__,msg,num1,num2);		\
	if(REROUTE_STDERR && rerouted())			\
	    fprintf(stdout,"%s:%d: %s(%d/%d)\n\r",		\
		    __FILE__,__LINE__,msg,num1,num2);		\
    }

/**
 * Print error message with char to stderr
 *
 */
#define err_log_char(msg,ch)				\
    {							\
	fprintf(stderr,"%s:%d: %s(%c)\n\r",		\
		__FILE__,__LINE__,msg,ch);		\
	if(REROUTE_STDERR && rerouted())		\
	    fprintf(stdout,"%s:%d: %s(%c)\n\r",		\
		    __FILE__,__LINE__,msg,ch);		\
    }

/**
 * Print error message with number to stderr
 *
 */
#define err_log_str(msg,str)					\
    {								\
	fprintf(stderr,"%s:%d: %s(%s)\n\r",			\
		__FILE__,__LINE__,msg,str);			\
	if(REROUTE_STDERR && rerouted())			\
	    fprintf(stdout,"%s:%d: %s(%s)\n\r",			\
		    __FILE__,__LINE__,msg,str);			\
    }

/**
 * Print error message with double to stderr
 *
 */
#define err_log_double(msg,dbl)					\
    {								\
	fprintf(stderr,"%s:%d: %s(%lf)\n\r",			\
		__FILE__,__LINE__,msg,dbl);			\
	if(REROUTE_STDERR && rerouted())			\
	    fprintf(stdout,"%s:%d: %s(%lf)\n\r",		\
		    __FILE__,__LINE__,msg,dbl);			\
    }

/**
 * Print double ended by tab to stderr
 *
 */
#define err_log_double_only(dbl)					\
    {									\
	fprintf(stderr,"%lf\t", dbl);					\
	if(REROUTE_STDERR && rerouted())				\
	    fprintf(stdout,"%0.15f\t", dbl);				\
    }

/**
 * Print double ended by tab to stderr
 *
 */
#define err_log_double_only_tight(dbl)					\
    {									\
	fprintf(stderr,"%lf\t", dbl);					\
	if(REROUTE_STDERR && rerouted())				\
	    fprintf(stdout,"%0.3f\t", dbl);				\
    }

/**
 * Print error message with double to stderr
 *
 */
#define err_log_eol()							\
    {									\
	fprintf(stderr,"\n\r");						\
	if(REROUTE_STDERR && rerouted())				\
	    fprintf(stdout,"\n\r");					\
    }

/**
 * Print double ended by tab to log.
 *
 */
#define log_double_only(log, dbl) fprintf(log,"%0.15f\t", dbl);

/**
 * Print double ended by tab to log.
 *
 */
#define log_double_only_tight(log, dbl) fprintf(log,"%0.3f\t", dbl);

/**
 * Print error message with double to log
 *
 */
#define log_double(log,msg,dbl)				\
    {							\
	fprintf(log,"%s:%d: %s: %lf\n\r",		\
		__FILE__,__LINE__,msg,dbl);		\
    }

#define log_eol(log) fprintf(log,"\n\r")
#define log_tab(log) fprintf(log,"\t")

/**
 * Print timeval to log with unsigned long to log
 *
 */
#define log_tv(log,msg,tv)						\
    {									\
	fprintf(log,"%s:%d: %s(%ld.%06ld)\n\r",				\
		__FILE__,__LINE__,msg,tv.tv_sec, tv.tv_usec);		\
    }

/**
 * Print timeval to log with int to log
 *
 */
#define log_tv_num(log,msg,tv,num)					\
    {									\
	fprintf(log,"%s:%d: %s(%ld.%06ld) (%d)\n\r",			\
		__FILE__,__LINE__,					\
		msg,tv.tv_sec, tv.tv_usec, num);			\
    }

/**
 * Print timeval to log with int to stderr
 *
 */
#define err_log_tv(msg,tv)			\
    {						\
	log_tv(stderr,msg,tv);			\
	if(REROUTE_STDERR && rerouted())	\
	{					\
	    log_tv(stdout,msg,tv);		\
	}					\
    }

/**
 * Print timeval message with int to stderr
 *
 */
#define err_log_tv_num(msg,tv,num)		\
    {						\
	log_tv_num(stderr,msg,tv,num);		\
	if(REROUTE_STDERR && rerouted())	\
	{					\
	    log_tv_num(stdout,msg,tv,num);	\
	}					\
    }

/**
 * Print timeval to log with unsigned long to log, no '\n\r'
 * 
 */
#define log_tv_only(log,tv)					\
    {								\
	fprintf(log,"%ld.%06ld\t",tv.tv_sec, tv.tv_usec);	\
    }

/**
 * Print int to log ended by tab
 * 
 */
#define log_int_only(log,db) fprintf(log,"%d\t",db)

/**
 * If @retval is an error, then propagate error without printing anything.
 *
 */
#define err_propagate(retval)				\
    if(retval!=ERROR_OK)				\
    {							\
	fprintf(stderr,"backtrace:%s:%s:%d\n\r",	\
		__FILE__,__FUNCTION__,__LINE__);	\
	if(REROUTE_STDERR && rerouted())		\
	    fprintf(stdout,"backtrace:%s:%s:%d\n\r",	\
		    __FILE__,__FUNCTION__,__LINE__);	\
	return retval;					\
    }

/**
 * If @retval is an error, then print @msg to stderr and propagate error.
 * 
 */
#define err_check(retval,msg)			\
    {						\
	if(retval!=ERROR_OK)			\
	{					\
	    err_log(msg);			\
	    return retval;			\
	}					\
    }

/**
 * If @retval is an error, then print strerr to stderr.
 *
 */
#define err_log_std(retval)						\
    if(retval!=ERROR_OK)						\
    {									\
	fprintf(stderr,"%s:%d: %s\n\r",					\
		__FILE__,__LINE__,strerror(errno));			\
	if(REROUTE_STDERR && rerouted())				\
	    fprintf(stdout,"%s:%d: %s\n\r",				\
		    __FILE__,__LINE__,strerror(errno));			\
    }

/**
 * If @retval is an error, then print strerr to stderr and propagate error.
 *
 */
#define err_check_std(retval)			\
    if(retval!=ERROR_OK)			\
    {						\
	err_log_std(retval);			\
	return retval;				\
    }

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
#define cleanup_if(retval)				\
    if(retval != ERROR_OK)				\
    {							\
	fprintf(stderr,"backtrace:%s:%d\n\r",		\
		__FILE__,__LINE__);			\
	if(REROUTE_STDERR && rerouted())		\
	    fprintf(stdout,"backtrace:%s:%d\n\r",	\
		    __FILE__,__LINE__);			\
	goto cleanup;					\
    }

/**
 * If @retval is an error, jump to cleanup.
 * Usefull in init routines, allows cleaning up.
 *
 */
#define cleanup_if_null(ptr)			\
    if(ptr == NULL)				\
    {						\
	err_log("Null pointer!");		\
	goto cleanup;				\
    }

/**
 * If @retval is an error, jump to cleanup.
 * Usefull in init routines, allows cleaning up.
 *
 */
#define cleanup_log_if(retval,msg)		\
    if(retval != ERROR_OK)			\
    {						\
	err_log(msg);				\
	goto cleanup;				\
    }

/**
 * If @retval is an error, call quit().
 * Usefull in test programs, allows cleaning up.
 * 
 */
#define quit_log_if(retval,msg)			\
    if(retval!=ERROR_OK)			\
    {						\
	err_log(msg);				\
	quit();					\
    }

/**
 * If @retval is an error, then got to the beginning of the loop
 * 
 */
#define log_n_continue(retval,msg)		\
    if(retval!=ERROR_OK)			\
    {						\
	err_log(msg);				\
	continue;				\
    }

/**
 * If @retval is an error, then log and jump to label
 *
 */
#define log_n_jump(retval,label,msg)		\
    if(retval!=ERROR_OK)			\
    {						\
	err_log(msg);				\
	goto label;				\
    }

/**
 * If @retval is an error, then log and jump to label
 *
 */
#define jump_if(retval,label)			\
    if(retval!=ERROR_OK)			\
    {						\
	goto label;				\
    }

/**
 * Verifies that malloc succeeded, return NULL if fail
 * 
 */
#define mem_alloc_check(pointer)		\
    if(pointer==NULL)				\
    {						\
	err_log("malloc failed!");		\
	return NULL;				\
    }

/**
 * Verifies that malloc succeeded, returns ERROR_MALLOC.
 *
 */
#define mem_alloc_check_ret_err(pointer)	\
    if(pointer==NULL)				\
    {						\
	err_log("malloc failed!");		\
	return ERROR_MALLOC;			\
    }

/**
 * Reads a double from FILE* src, returns answer in db, checks
 * for errors and logs if so.
 *
 */
#define read_double(src,db)				\
    {							\
	if(fscanf(src, "%lf", &db) <= 0 )		\
	{						\
	    err_log_stderr("Failed to read double!");	\
	    err_propagate(ERROR_READ);			\
	}						\
    }

/**
 * Print names of elements of state vector to log.
 */
#define print_sv_name(log)						\
    fprintf(log,							\
	    "x\ty\tz\t"							\
	    "psi\tphi\tthe\t"						\
	    "vqx\tvqy\tvqz\t"						\
	    "wqx\twqy\twqz\t"						\
	    "ax\tay\taz\n");

/// No functions

#endif //UQUAD_ERROR_CODES_H
