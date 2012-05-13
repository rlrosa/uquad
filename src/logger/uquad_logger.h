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
/**
 * Launches a logger as a child process, returns the
 * file to which the parent should write.
 *
 * NOTE: Logging to ram will allocate ram MB of mem and save data there. If
 *       logger runs out of space, it will discard data new data.
 *
 * @param log_name Name to be used for new log.
 * @param path Path to where log file will be created.
 * @param ram If not 0, then logs will be saved in ram MB of RAM, and dumped when closed.
 *
 * @return File (pipe) to which parent should write, or NULL if error.
 */
FILE *uquad_logger_add(char *log_name, char *path, int ram);

/**
 * Closes parent side of pipe, indicating to child that it should
 * dump remaining data and die.
 * Argument must match the value previously returned by uquad_logger_add().
 *
 * @param pipe_f Name of file (pipe) to close.
 */
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
