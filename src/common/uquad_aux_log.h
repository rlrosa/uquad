#ifndef UQUAD_LOG_H
#define UQUAD_LOG_H

#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include <uquad_error_codes.h>
#include <stdio.h>

#define UQUAD_LOG_UP_ONE_LINE "\033[A"
#define UQUAD_LOG_CLEAR_LINE "\033[2K"
#define UQUAD_LOG_UP_AND_CLEAR "\033[A\033[2K"

/** 
 * 
 * 
 * @param log_name String where log will be returned (mem must be malloc previously).
 * @param start_string Log name will start with this string. 
 * 
 * @return error code.
 */
int uquad_log_generate_log_name(char * log_name, char * start_string);

/** 
 * Rewinds a buffer. Clears NUM_LINES from STREAM. 
 * 
 * @param stream 
 * @param num_lines Should be >= 0
 * 
 * @return error code.
 */
int uquad_log_rew_and_clear(FILE * stream, int num_lines);

#endif
