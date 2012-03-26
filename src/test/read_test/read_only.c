#include <stdlib.h>
#include <stdint.h> // for uint8_t
#include <unistd.h> // for usleep()
#include <fcntl.h> // for open()

#include <uquad_error_codes.h>
#include <uquad_aux_time.h>
#include <uquad_aux_io.h>

#define HOW_TO "./prueba </dev/tty##>"
#define EOFRAME_TIMING 1
#define ERROR_TIMING 0

#define USE_FOPEN 0

#if USE_FOPEN
static FILE *dev = NULL;
#else
static int dev = -1;
#endif

void quit()
{
#if USE_FOPEN
    if(dev != NULL)
	fclose(dev);
#else
    if(dev > 0)
	close(dev);
#endif
    err_log("Terminating...\n");
    exit(0);
}

void uquad_sig_handler(int signal_num){
    err_log_num("Caught signal %d!\n",signal_num);
    quit();
}

int main(int argc, char *argv[])
{
    char * device_imu;
    uint8_t buff_tmp_8;
    int retval, read_ok;
    struct timeval tv_prev, tv_diff, tv_tmp;

    if(argc<2)
    {
        err_log(HOW_TO);
        exit(1);
    }
    else
    {
        device_imu = argv[1];
    }

#if USE_FOPEN
    dev = fopen(device_imu,"wb+");
    if(dev == NULL)
    {
	err_log_stderr("fopen() failed!");
	quit();
    }
#else
    dev = open(device_imu,O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(dev < 0)
    {
	err_log_stderr("Failed to open dev!");
	quit();
    }
#endif

    while(1)
    {
#if USE_FOPEN
	retval = check_io_locks(-1, dev, &read_ok, NULL);
#else
	retval = check_io_locks(dev, NULL, &read_ok, NULL);
#endif
	if(retval < 0)
	{
	    err_log_stderr("check_io_locks() failed!");
	}
	if(read_ok)
	{
#if USE_FOPEN
	    retval = fread(& buff_tmp_8,1,1,dev);
	    if(retval < 0)
	    {
		err_log_stderr("fread() failed!");
		continue;
	    }
#else
	    retval = read(dev, & buff_tmp_8,1);
	    if(retval < 0)
	    {
#if ERROR_TIMING
		gettimeofday(&tv_tmp,NULL);
		uquad_timeval_substract(&tv_diff,tv_tmp,tv_prev);
		gettimeofday(&tv_prev,NULL);
		fprintf(stderr,"%ld\t",tv_diff.tv_usec);
#endif
		err_log_stderr("read() failed!");
		continue;
	    }
#endif
#if EOFRAME_TIMING
	    if (buff_tmp_8 == 'Z')
	    {
		gettimeofday(&tv_tmp,NULL);
		uquad_timeval_substract(&tv_diff,tv_tmp,tv_prev);
		gettimeofday(&tv_prev,NULL);
		printf("%ld\n",tv_diff.tv_usec);
		usleep(4000);
	    }
#endif
	}
    }
    
    quit();
}

