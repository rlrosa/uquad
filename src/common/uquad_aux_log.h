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

int uquad_log_generate_log_name(char * log_name, char * start_string);
int uquad_log_rew_and_clear(FILE * stream, int num_lines);

#endif
