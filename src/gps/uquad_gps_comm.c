#include <uquad_gps_comm.h>
#include <gpsd.h>
#include <stdlib.h>
#include <uquad_error_codes.h>

#define GPS_COMM_STREAM_FLAGS_ENA WATCH_ENABLE | WATCH_JSON
#define GPS_COMM_STREAM_FLAGS_DIS WATCH_DISABLE

gps_t *  gps_comm_init(void){
    gps_t * gps;
    int ret;
    gps = (gps_t *)malloc(sizeof(gps_t));
    mem_alloc_check(gps);

    // Initialize data structure and open connection
    ret = gps_open(NULL, 0, gps);
    if(ret < 0)
    {
	err_log("GPS init failed, could not open connection to GPS, is daemon running?");
	cleanup_if(ERROR_GPS);
    }

    // Start TX from GPS
    ret = gps_stream(gps, GPS_COMM_STREAM_FLAGS_ENA, NULL);
    if(ret < 0)
    {
	err_log("GPS init failed, could not stream from GPS.\nAttempting to close connection...");
	// deinit, and ignore return value, since gps_comm_deinit takes care of logging.
	(void) gps_comm_deinit(gps);
	cleanup_if(ERROR_GPS);
    }
    return gps;

    cleanup:
    gps_comm_deinit(gps);
    return NULL;
}   

void  gps_comm_deinit(gps_t * gps){
    int ret;
    if(gps == NULL)
    {
	err_log("WARN: Nothing to free");
	return;
    }
    ret = gps_stream(gps, GPS_COMM_STREAM_FLAGS_DIS, NULL);
    if(ret < 0)
	err_log("WARN: ignoring error while terminating GPS stream...");
    ret = gps_close (gps);
    if(ret < 0)
	err_log("WARN: ignoring error while closing GPS...");
    free(gps);
}

int gps_comm_get_status(gps_t *gps){
    return gps->status;
}

int gps_comm_get_fd(gps_t *gps){
    return gps->gps_fd;
}

int gps_comm_read(gps_t *gps){
    int ret;
    ret = gps_read(gps);
    if(ret == -1){
	err_check(ERROR_IO,"New data from expected, but none found...\nWas select(gps_fd) called before gps_comm_read()?");
    }
    return ERROR_OK;
}

struct gps_fix_t gps_comm_get_data(gps_t *gps){
    return gps->fix;
}
