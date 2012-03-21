// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
#ifndef UQUAD_GPS_LIB_TEST // test the uquad interface to gpsd
// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#include <uquad_gps_comm.h>
#include <uquad_aux_io.h>
#include <unistd.h>

#define FREE_N_DIE_IF_ERROR(ret,msg) if(ret!=ERROR_OK){fprintf(stderr,"GPS test failed:%s:%d:%s\n",__FILE__,__LINE__,msg);goto kill_n_close;}

int main(void)
{
    int ret;
    gps_t *gps;
    io_t *io;
    
    gps = gps_comm_init();
    if(gps == NULL){
	err_log("GPS test failed.");
    }

    io = io_init();
    if(io == NULL){
	err_log("GPS test failed.");
    }

    // add GPS
    ret = io_add_dev(io, gps_comm_get_fd(gps));
    FREE_N_DIE_IF_ERROR(ret,"Failed to add device to io.");

    // add stdin to support clean exit
    ret = io_add_dev(io,STDIN_FILENO);
    FREE_N_DIE_IF_ERROR(ret,"Failed to add STDIN to dev list");

    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    /// Poll n read loop
    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    uquad_bool_t read = false,write = false;
    uquad_bool_t reg_gps = true, reg_stdin = true;
    unsigned char tmp_buff[2];
    struct gps_fix_t gps_fix;
    //    poll_n_read:
    while(1){
        ret = io_poll(io);
        FREE_N_DIE_IF_ERROR(ret,"io_poll() error");
        // gps
        if(reg_gps){
            ret = io_dev_ready(io,gps_comm_get_fd(gps),&read,&write);
            FREE_N_DIE_IF_ERROR(ret,"io_dev_ready() error");
            if(read){
                ret = gps_comm_read(gps);
                if(ret != ERROR_OK){
                    fprintf(stdout,"\nGPS missed frame?\n\n");
                }else{
                    ret = gps_comm_read(gps);
                    if(ret != ERROR_OK){
                        fprintf(stdout,"\nGPS had no data!\n\n");
		    }else{
			//gps_fix = gps_comm_get_data(gps);
			fprintf(stdout,"fix mode:%d\n",gps_comm_get_status(gps));
			if(gps_comm_get_status(gps)){
			    gps_fix = gps_comm_get_data(gps);
			    fprintf(stdout,"\tlat:%f\n\tlon:%f\n\talt:%f\n\ttimestamp:%f\n\n",
				   gps_fix.latitude,
				   gps_fix.longitude,
				   gps_fix.altitude,
				   gps_fix.time);
			}else{
			    printf("\n");
			}

		    }
                }
            }
        }
        // stdin
        if(reg_stdin){
            ret = io_dev_ready(io,STDIN_FILENO,&read,&write);
            FREE_N_DIE_IF_ERROR(ret,"io_dev_ready() error");
            if(read){
                ret = fread(tmp_buff,1,1,stdin);
                if(ret<=0)
                    fprintf(stdout,"\nNo user input!!\n\n");
                else
                    // Any user input will terminate program
                    goto kill_n_close;
            }
        }
        fflush(stdout);
    }

    // Failure...
    kill_n_close:
    ret = io_deinit(io);
    if(ret != ERROR_OK){
	err_log("Failed to deinit io.");
    }
    gps_comm_deinit(gps);
    return ERROR_OK;
}
// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
#else //UQUAD_GPS_LIB_TEST defined, testing gpsd library directly.
// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include <ctype.h>

#include "gpsd.h"

#ifndef S_SPLINT_S
#include <unistd.h>
#endif /* S_SPLINT_S */
#include <getopt.h>
#include <signal.h>

#define sleep_ms(time_ms) usleep(1000*time_ms)

static struct gps_data_t gpps_data;

int main(int argc, char *argv[])
{
    int ret;
    
    ret = gps_open(NULL, 0, &gpps_data);

    (void) gps_stream(&gpps_data, WATCH_ENABLE | WATCH_JSON, NULL);

    while(1)
    {
	/* Put this in a loop with a call to a high resolution sleep () in it. */
	if (gps_waiting (&gpps_data, 500)) {
	    errno = 0;
	    if (gps_read (&gpps_data) == -1) {
		//TODO die in a handsome way
		printf("ERROR!\n");
		goto close_and_die;	    
	    } else {
		/* Display data from the GPS receiver. */
		if (gpps_data.set)
		    printf("fix mode:%d\n",gpps_data.fix.mode);
	    }
	}
	sleep_ms(1000);
    }

    /* When you are done... */
    close_and_die:
    (void) gps_stream(&gpps_data, WATCH_DISABLE, NULL);
    (void) gps_close (&gpps_data);
}

#endif //UQUAD_GPS_LIB_TEST
