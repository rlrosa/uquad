/*
 * A simple command-line exerciser for the library.
 * Not really useful for anything but debugging.
 */
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
