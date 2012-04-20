#ifndef UQUAD_LOGGER_H
#define UQUAD_LOGGER_H

#include <stdio.h>

#define LOGGER_DEBUG 0
#define LOGGER_PERM  0666
/**
 * -- -- -- -- -- -- -- -- -- -- -- --
 * Logging via pipe, separates worker
 * from logger, avoiding I/O blocking.
 * -- -- -- -- -- -- -- -- -- -- -- --
 */
#define DEF_PERM 0666
FILE *uquad_logger_add(char *log_name, char *path);
void uquad_logger_remove(FILE *pipe_f);

/**
 * -- -- -- -- -- -- -- -- -- -- -- --
 * Aux logging functions
 * -- -- -- -- -- -- -- -- -- -- -- --
 */
#define UQUAD_LOG_UP_ONE_LINE "\033[A"
#define UQUAD_LOG_CLEAR_LINE "\033[2K"
#define UQUAD_LOG_UP_AND_CLEAR "\033[A\033[2K"
/** 
 * 
 * @param log_name String where log will be returned (mem must be malloc previously).
 * @param start_string Log name will start with this string. 
 * 
 * @return error code.
 */
int uquad_log_generate_log_name(char *log_name, char *start_string);

/** 
 * Rewinds a buffer. Clears NUM_LINES from STREAM. 
 * 
 * @param stream 
 * @param num_lines Should be >= 0
 * 
 * @return error code.
 */
int uquad_log_rew_and_clear(FILE *stream, int num_lines);


#endif // UQUAD_LOGGER_H
