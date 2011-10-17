#include <uquad_aux_log.h>

int uquad_log_generate_log_name(char * log_name, char * start_string){
    time_t rawtime;
    struct tm * tm;
    int retval;
    time(&rawtime);
    tm = localtime (&rawtime);
    retval = sprintf(log_name,"%04d_%02d_%02d_xx_%02d_%02d_%02d", 1900 + tm->tm_year, tm->tm_mon + 1,tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
    if(retval < 0)
	return ERROR_FAIL;
    if(start_string != NULL)
	retval = sprintf(log_name,"%s%s",start_string,log_name);
    return retval;
}

int uquad_log_rew_and_clear(FILE * stream, int num_lines){
    int retval,i;
    if(stream == NULL){
	err_check(ERROR_FAIL,"Cannot clear null stream");
    }
    for(i=0;i<num_lines;++i){
	retval = fputs(UQUAD_LOG_UP_ONE_LINE,stdout);
	if(retval == EOF){
	    err_check(ERROR_WRITE,"fputs() returned error!");
	}
    }
    return ERROR_OK;
}
