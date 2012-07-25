/**
 * uquad_gps_comm: lib for communicating with gps over USB
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
 * @file   uquad_gps_comm.c
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 11:08:44 2012
 *
 * @brief  lib for communicating with gps over USB
 *
 */
#include <uquad_gps_comm.h>
#include <stdlib.h>
#include <math.h>
#include <uquad_error_codes.h>
#include <uquad_aux_time.h>
#include <uquad_aux_io.h>
#include <fcntl.h> // for open()

#define GPS_COMM_STREAM_FLAGS_ENA WATCH_ENABLE | WATCH_JSON
#define GPS_COMM_STREAM_FLAGS_DIS WATCH_DISABLE

static uquad_mat_t *m3x1 = NULL;
static uquad_mat_t *m3x3 = NULL;

int gps_comm_connect(gps_t *gps, const char *device)
{
    int retval;
    char str[256];
    gps->fd = open(device,O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(gps->fd < 0)
    {
	err_log_stderr("open()");
	retval = ERROR_OPEN;
	cleanup_if(retval);
    }

    if( (strlen(device) > 5) && (strncmp(device,"/dev/",5) == 0))
    {
	// Set default baudrate
	retval = sprintf(str,"stty -F %s 38400",device);
	if(retval < 0)
	{
	    err_log_stderr("sprintf()");
	    cleanup_if(ERROR_FAIL);
	}
	retval = system(str);
	if(retval != 0)
	{
	    err_log_stderr("system()");
	    cleanup_if(ERROR_IO);
	}
    }
    return ERROR_OK;

    cleanup:
    if(gps->fd > 0)
	if(close(gps->fd) < 0)
	{
	    err_log_stderr("close()");
	}
    return retval;
}

gps_t *  gps_comm_init(const char *device){
    gps_t * gps = NULL;
    int retval;

    if(device == NULL)
    {
	err_log("NULL pointer is invalid arg!");
	return NULL;
    }

    gps = (gps_t *)malloc(sizeof(gps_t));
    cleanup_if_null(gps);

    gps->pos = uquad_mat_alloc(3,1);
    cleanup_if_null(gps->pos);
    retval = uquad_mat_zeros(gps->pos);
    cleanup_if(retval);
    /* gps->pos_ep = uquad_mat_alloc(3,1); */
    /* cleanup_if_null(gps->pos_ep); */
    /* uquad_mat_zeros(gps->pos_ep); */
    /* cleanup_if(retval); */
    /* gps->gps_fix = (gps_fix_t *)malloc(sizeof(gps_fix_t)); */
    /* cleanup_if_null(gps->gps_fix); */

    m3x1 = uquad_mat_alloc(3,1);
    cleanup_if_null(m3x1);
    m3x3 = uquad_mat_alloc(3,3);
    cleanup_if_null(m3x3);

    gps->pos_0 = NULL;
    gps->tv_start = NULL;
    gps->tv_log_start = NULL;

    retval = gps_comm_connect(gps,device);
    cleanup_if(retval);
    return gps;

    cleanup:
    err_log("GPS init failed!");
    gps_comm_deinit(gps);
    return NULL;
}

int gps_comm_wait_fix(gps_t *gps, uquad_bool_t *got_fix, struct timeval *t_out)
{
    uquad_bool_t
	read_ok;
    int
	retval = ERROR_OK;
    struct timeval
	tv_in,
	tv_tmp,
	tv_diff;
    if(gps == NULL || got_fix == NULL)
    {
	err_check(ERROR_INVALID_ARG,"NULL pointer invalid arg!");
    }
    *got_fix = false;
    retval = gettimeofday(&tv_in,NULL);
    if(retval != 0)
    {
	err_check_std(ERROR_TIMING);
    }
    for(;;)
    {
	retval = check_io_locks(gps_comm_get_fd(gps),NULL,&read_ok,NULL);
	err_propagate(retval);
	if(read_ok)
	{
	    retval = gps_comm_read(gps, got_fix);
	    err_propagate(retval);
	    if(gps_comm_fix(gps))
	    {
		*got_fix = true;
		return ERROR_OK;
	    }
	    else
	    {
		//		sleep_ms(GPS_COMM_WAIT_FIX_SLEEP_MS);
	    }
	}

	if(t_out != NULL)
	{
	    /// Check if timeout was exceeded
	    retval = gettimeofday(&tv_tmp,NULL);
	    if(retval != 0)
	    {
		err_check_std(ERROR_TIMING);
	    }
	    retval = uquad_timeval_substract(&tv_diff,tv_tmp,tv_in);
	    if(retval <= 0)
	    {
		err_check(ERROR_TIMING,"Absurd timing!");
	    }
	    retval = uquad_timeval_substract(&tv_tmp,tv_diff,*t_out);
	    if(retval > 0)
	    {
		err_log("Timed out waiting for GPS!");
		return ERROR_OK;
	    }
	}
    }
}

int gps_comm_set_0(gps_t *gps, gps_comm_data_t *gps_dat)
{
    int retval;
    if(gps->pos_0 != NULL)
	err_check(ERROR_FAIL,"Already set 0!");
    gps->pos_0    = uquad_mat_alloc(3,1);
    if(gps->pos_0 == NULL)
    {
	err_propagate(ERROR_MALLOC);
    }
    retval = uquad_mat_copy(gps->pos_0,gps_dat->pos);
    err_propagate(retval);
    return retval;
}

int gps_comm_get_0(gps_t *gps, gps_comm_data_t *gps_dat)
{
    int retval;
    if(gps == NULL || gps_dat == NULL)
    {
	err_check(ERROR_INVALID_ARG,"Null pointer invalid arg!");
    }
    if(gps->pos_0 == NULL)
    {
	err_check(ERROR_GPS,"Start position not defined!");
    }
    retval = uquad_mat_copy(gps_dat->pos, gps->pos_0);
    err_propagate(retval);
    retval = uquad_mat_zeros(gps_dat->vel);
    err_propagate(retval);
    return ERROR_OK;
}

void gps_comm_deinit(gps_t * gps){
    if(gps == NULL)
    {
	err_log("WARN: Nothing to free");
	return;
    }
    uquad_mat_free(gps->pos);
    /* uquad_mat_free(gps->pos_ep); */
    /* free(gps->gps_fix); */
    uquad_mat_free(m3x1);
    uquad_mat_free(m3x3);

    uquad_mat_free(gps->pos_0);

    if(gps->dev == NULL)
    {
	if(gps->fd > 0)
	    if(close(gps->fd))
	    {
		err_log_stderr("close()");
	    }
    }
    else
    {
	fclose(gps->dev);
	gps->dev = NULL;
	if(gps->tv_start != NULL)
	{
	    free(gps->tv_start);
	    gps->tv_start = NULL;
	}
	if(gps->tv_log_start != NULL)
	{
	    free(gps->tv_log_start);
	    gps->tv_log_start = NULL;
	}
    }
    free(gps);
}

int gps_comm_deg2utm(utm_t *utm, double la, double lo)
{
    if(utm == NULL)
    {
	err_check(ERROR_NULL_POINTER,"Invalid argument!");
    }
    double lat,lon;
    double Huso,S,deltaS;
    double a,epsilon,nu,v,ta,a1,a2,j2,j4,j6,alfa,beta,gama,Bm;
    char let;
    lat = deg2rad(la);
    lon = deg2rad(lo);

    Huso = floor( ( lo / 6 ) + 31);
    S = ( ( Huso * 6 ) - 183 );
    deltaS = lon - deg2rad(S);

    if (la<-72)      let='C';
    else if (la<-64) let='D';
    else if (la<-56) let='E';
    else if (la<-48) let='F';
    else if (la<-40) let='G';
    else if (la<-32) let='H';
    else if (la<-24) let='J';
    else if (la<-16) let='K';
    else if (la<-8)  let='L';
    else if (la<0)   let='M';
    else if (la<8)   let='N';
    else if (la<16)  let='P';
    else if (la<24)  let='Q';
    else if (la<32)  let='R';
    else if (la<40)  let='S';
    else if (la<48)  let='T';
    else if (la<56)  let='U';
    else if (la<64)  let='V';
    else if (la<72)  let='W';
    else             let='X';

    a = cos(lat) * sin(deltaS);
    epsilon = 0.5 * log( ( 1 + a) / ( 1 - a ) );
    nu = atan( tan(lat) / cos(deltaS) ) - lat;
    v = ( deg2utm_c / sqrt( ( 1.0 + ( deg2utm_ee * uquad_square(cos(lat)) ) ) )) * 0.9996;
    ta = ( deg2utm_ee / 2 ) * uquad_square(epsilon) * uquad_square(cos(lat));
    a1 = sin( 2 * lat );
    a2 = a1 * uquad_square(cos(lat));
    j2 = lat + ( a1 / 2.0 );
    j4 = ( ( 3 * j2 ) + a2 ) / 4.0;
    j6 = ( ( 5 * j4 ) + ( a2 * uquad_square(cos(lat))) ) / 3.0;
    alfa = ( 0.75 ) * deg2utm_ee;
    beta = ( 1.6666666666667 ) * uquad_square(alfa);
    gama = ( 1.2962962962963 ) * (alfa * alfa * alfa);
    Bm = 0.9996 * deg2utm_c * ( lat - alfa * j2 + beta * j4 - gama * j6 );
    utm->easting  = epsilon * v * ( 1.0 + ( ta / 3.0 ) ) + 500000;
    utm->northing = nu * v * ( 1.0 + ta ) + Bm;

    if (utm->northing < 0)
	utm->northing += 9999999;

    utm->let  = let;
    utm->zone = Huso;

    return ERROR_OK;
}

int gps_comm_get_fix_mode(gps_t *gps)
{
    return gps->fix;
}

uquad_bool_t gps_comm_fix(gps_t *gps)
{
    return (gps_comm_get_fix_mode(gps) > GPS_FIX_INVALID)?
	true:
	false;
}

int gps_comm_get_fd(gps_t *gps)
{
    return gps->fd;
}

int gps_comm_set_tv_start(gps_t *gps, struct timeval tv_start)
{
    if(gps == NULL)
    {
	err_check(ERROR_INVALID_ARG,"Invalid arguments!");
    }
    if(gps->dev == NULL)
    {
	err_check(ERROR_FAIL, "Can only be used when reading from a log file!");
    }
    if(gps->tv_start == NULL)
    {
	gps->tv_start = (struct timeval *)malloc(sizeof(struct timeval));
	if(gps->tv_start == NULL)
	{
	    err_check(ERROR_MALLOC, "Failed to allocate mem for tv_log_start!");
	}
    }
    *(gps->tv_start) = tv_start;
    return ERROR_OK;
}

// This will be shared by every gp*** parsing function
static char buff[GPS_NMEA_MAX_LEN];

/**
 * Process a pair of latitude/longitude.
 * Field must be, for example:
 *   field[0] = "4124.8963"
 *   field[0] = "N"
 *   field[0] = "08151.6838"
 *   field[0] = "W"
 *
 * @param field
 * @param lat
 * @param lon
 */
static void do_lat_lon(char *field[], double *lat_ans, double* lon_ans)
/*  fields starting at field index BEGIN */
{
    /// Taken from driver_nmea0183.c - gpsd
    double d, m;
    char str[20], *p;

    if (*(p = field[0]) != '\0') {
	double lat;
	(void)strcpy(str, p);
	lat = atof(str);
	m = 100.0 * modf(lat / 100.0, &d);
	lat = d + m / 60.0;
	p = field[1];
	if (*p == 'S')
	    lat = -lat;
	*lat_ans = lat;
    }
    if (*(p = field[2]) != '\0') {
	double lon;
	(void)strcpy(str, p);
	lon = atof(str);
	m = 100.0 * modf(lon / 100.0, &d);
	lon = d + m / 60.0;

	p = field[3];
	if (*p == 'W')
	    lon = -lon;
	*lon_ans = lon;
    }
}

static char buff_tmp[GPS_NMEA_MAX_LEN];
int gps_comm_parse_gpgga(gps_t *gps, char *buff)
{
    char *fields[4];
    char *token;
    (void) strcpy(buff_tmp, buff);
    if(gps == NULL)
    {
	err_check(ERROR_FAIL,"Invalid arguments!");
    }

    // Discard time info
    token = strtok(buff_tmp,GPS_NMEA_DELIMS);
    if(token == NULL) goto token_error;

    // latitude
    token = strtok(NULL,GPS_NMEA_DELIMS);
    if(token == NULL) goto token_error;
    fields[0] = token;
    token = strtok(NULL,GPS_NMEA_DELIMS);
    if(token == NULL) goto token_error;
    fields[1] = token;
    // longitude
    token = strtok(NULL,GPS_NMEA_DELIMS);
    if(token == NULL) goto token_error;
    fields[2] = token;
    token = strtok(NULL,GPS_NMEA_DELIMS);
    if(token == NULL) goto token_error;
    fields[3] = token;
    // Convert to degrees
    do_lat_lon(fields, &gps->lat, &gps->lon);

    // fix
    token = strtok(NULL,GPS_NMEA_DELIMS);
    if(token == NULL) goto token_error;
    gps->fix = atoi(token);

    // sat count
    token = strtok(NULL,GPS_NMEA_DELIMS);
    if(token == NULL) goto token_error;
    gps->sats = atoi(token);

    // DOP - Discarded
    token = strtok(NULL,GPS_NMEA_DELIMS);
    if(token == NULL) goto token_error;
    gps->dop = atof(token);

    // Altitude
    token = strtok(NULL,GPS_NMEA_DELIMS);
    if(token == NULL) goto token_error;
    gps->altitude = atof(token);

    return ERROR_OK;

    token_error:
    err_check(ERROR_FAIL,"Not enough tokens!");
}

typedef enum gps_read_status{
    GPS_SEARCHING = 0,
    GPS_READING_TYPE,
    GPS_READING_DATA,
    GPS_READING_CHECKSUM_1,
    GPS_READING_CHECKSUM_2,
    GPS_DONE,
    GPS_READ_STATE_COUNT
} gps_read_status_t;
int gps_comm_read(gps_t *gps, uquad_bool_t *ok)
{
    int
	bytes_read,
	retval;
    //    gps_fix_t gps_fix;
    static int buff_index = 0;
    static gps_read_status_t status = GPS_SEARCHING;
    static uint8_t checksum = 0;
    uint8_t checksum_hex[2];
    if(gps == NULL || ok == NULL)
    {
	err_check(ERROR_INVALID_ARG,"Invalid arguments!");
    }
    *ok = false;
    if(gps->dev == NULL)
    {
	// reading from real GPS device
	bytes_read = read(gps->fd,buff + buff_index,1);
	if(bytes_read < 0)
	{
	    err_log_stderr("Read error: no data! Restarting...");
	    buff_index = 0;
	    err_propagate(ERROR_IO);
	}
	switch(status)
	{
	case GPS_SEARCHING:
	    if(buff[buff_index] == GPS_NMEA_START)
	    {
		// New frame starting
		status = GPS_READING_TYPE;
		checksum = 0;
	    }
	    buff_index = 0;
	    break;
	case GPS_READING_TYPE:
	    checksum^=buff[buff_index];
	    if(buff[buff_index] == GPS_NMEA_DELIM)
	    {
		// Done reading type, check if it is the one we want
		if((buff_index == GPS_NMEA_TYPE_LEN) &&
		   (strncmp(buff,GPS_NMEA_GPGGA,GPS_NMEA_TYPE_LEN) == 0))
		{
		    status = GPS_READING_DATA;
		}
		else
		{
		    if(buff_index != GPS_NMEA_TYPE_LEN)
		    {
			err_log("WARN: Wrong NMEA type length! Data is being lost!");
		    }
		    // Back to square 1
		    status = GPS_SEARCHING;
		}
		buff_index = 0;
	    }
	    else
	    {
		buff_index += bytes_read;
	    }
	    break;
	case GPS_READING_DATA:
	    if(buff[buff_index] == GPS_NMEA_EOS)
	    {
		status = GPS_READING_CHECKSUM_1;
	    }
	    else
	    {
		// update checksum
		checksum^=buff[buff_index];
	    }
	    buff_index += bytes_read;
	    break;
	case GPS_READING_CHECKSUM_1:
	    buff_index += bytes_read;
	    status = GPS_READING_CHECKSUM_2;
	    break;
	case GPS_READING_CHECKSUM_2:
	    // Now checksum should have been read
	    buff_index -= 1; // go back to start of checksum
	    /// Verify checksum
	    checksum_hex[0] = '0'+(checksum >> 4);
	    checksum_hex[1] = '0'+(checksum & 0xF);
	    if(memcmp((void *)buff + buff_index,(void *) checksum_hex,2) == 0)
	    {
		/// checksum ok, advance and discard checksum
		status = GPS_DONE;
		buff_index -= 1;
		buff[buff_index] = '\0'; // end string, without checksum
		/**
		 * If the sentence was:
		 *   $GPGGA,223725.133,3455.1055,S,05610.0335,W,1,03,3.1,0.0,M,11.3,M,,0000*52
		 * the buffer will have the following string:
		 *   223725.133,3455.1055,S,05610.0335,W,1,03,3.1,0.0,M,11.3,M,,0000
		 */
	    }
	    else
	    {
		status = GPS_SEARCHING;
		err_log("WARN: Checksum failed! Discarding data...");
	    }
	    // no break here
	default:
	    if(status == GPS_DONE)
	    {
		retval = gps_comm_parse_gpgga(gps,buff);
		if(retval == ERROR_OK)
		    *ok = true;
	    }
	    status = GPS_SEARCHING; // back to square 1
	    break;
	}
    }
    else
    {
	err_check(ERROR_FAIL,"NOT Implemented!");
	// struct timeval *tv_curr must be an argument
	/**
	 * Reading from log file
	 *
	 * Each line of a gps.log file has a timestamp referred to
	 * the beginning of the program that generated the log file.
	 * We don't care about that time, we want the time relative
	 * to the beginning of the program that is reading from the
	 * log file.
	 */
	/* struct timeval */
	/*     tv_tmp, */
	/*     tv_diff; */
	/* static uquad_bool_t read_tv = false; */
	/* static struct timeval tv_diff_log; */
	/* double dtmp; */
	/* if(tv_curr == NULL) */
	/* { */
	/*     err_check(ERROR_INVALID_ARG,"Timestamp required!"); */
	/* } */
	/* if(gps->tv_start == NULL) */
	/* { */
	/*     gps->tv_start = (struct timeval *)malloc(sizeof(struct timeval)); */
	/*     if(gps->tv_start == NULL) */
	/*     { */
	/* 	err_check(ERROR_MALLOC, "Failed to allocate mem for tv_log_start!"); */
	/*     } */
	/*     *(gps->tv_start) = *tv_curr; */
	/* }	 */
	/* if(!read_tv) */
	/* { */
	/*     // read time from log */
	/*     retval = fscanf(gps->dev, "%lf", &dtmp); */
	/*     if(retval <= 0) */
	/*     { */
	/* 	err_check(ERROR_READ,"Failed to read timeval from log!"); */
	/*     } */
	/*     tv_tmp.tv_sec  = (unsigned long) floor(dtmp); */
	/*     tv_tmp.tv_usec = (unsigned long) ((dtmp - floor(dtmp))*1e6); */
	/*     read_tv = true; */
	/*     if(gps->tv_log_start == NULL) */
	/*     { */
	/* 	gps->tv_log_start = (struct timeval *)malloc(sizeof(struct timeval)); */
	/* 	if(gps->tv_log_start == NULL) */
	/* 	{ */
	/* 	    err_check(ERROR_MALLOC, "Failed to allocate mem for tv_log_start!"); */
	/* 	} */
	/* 	*(gps->tv_log_start) = tv_tmp; */
	/*     } */
	/*     retval = uquad_timeval_substract(&tv_diff, tv_tmp, *(gps->tv_log_start)); */
	/*     if(retval < 0) */
	/*     { */
	/* 	err_check(ERROR_TIMING, "Absurd timing!"); */
	/*     } */
	/*     tv_diff_log = tv_diff; */
	/* } */
	/* retval = uquad_timeval_substract(&tv_diff, *tv_curr, *(gps->tv_start)); */
	/* if(retval < 0) */
	/* { */
	/*     err_check(ERROR_TIMING, "Absurd timing!"); */
	/* } */
	/* retval = uquad_timeval_substract(&tv_diff, tv_diff, tv_diff_log); */
	/* if(retval < 0) */
	/* { */
	/*     // not the right time yet */
	/*     return ERROR_OK; */
	/* } */
	/* else */
	/* { */
	/*     read_tv = false; */
	/*     read_double(gps->dev,dtmp); // timestamp */
	/*     read_double(gps->dev,dtmp); // fix */
	/*     gps->fix = (int)dtmp; */
	/*     for(i = 0; i < 6; ++i) */
	/*     { */
	/* 	 // easting, northing, alt, vx, vy, vz */
	/* 	read_double(gps->dev,dtmp); */
	/*     } */
	/*     read_double(gps->dev,dtmp); // lat */
	/*     gps_fix.latitude  = dtmp; */
	/*     read_double(gps->dev,dtmp); // lon */
	/*     gps_fix.longitude = dtmp; */
	/*     read_double(gps->dev,dtmp); // speed */
	/*     gps_fix.speed     = dtmp; */
	/*     read_double(gps->dev,dtmp); // climb */
	/*     gps_fix.climb     = dtmp; */
	/*     read_double(gps->dev,dtmp); // track */
	/*     gps_fix.track     = dtmp; */
	/*     read_double(gps->dev,dtmp); // vel_ok */
	/*     gps_fix.epx = 0.0; */
	/*     gps_fix.epy = 0.0; */
	/*     gps_fix.epv = 0.0; */
	/*     gps_fix.eps = 0.0; */
	/*     gps_fix.epc = 0.0; */
	/*     gps_fix.epd = 0.0; */
	/* } */
    }
    if(!(*ok))
	return ERROR_OK;

    retval = gps_comm_deg2utm(&gps->utm, gps->lat, gps->lon);
    err_propagate(retval);
    gps->pos->m_full[0] = gps->utm.northing;
    gps->pos->m_full[1] = -gps->utm.easting;
    gps->pos->m_full[2] = gps->altitude;//gps_fix.altitude;

    gps->unread_data = true;

    /* if(isnan(gps_fix.epx) || */
    /*    isnan(gps_fix.epy) || */
    /*    isnan(gps_fix.epv)) */
    /* { */
    /* 	gps->pos_ep_ok = false; */
    /* } */
    /* else */
    /* { */
    /* 	gps->unread_data = true; */
    /* 	gps->pos_ep_ok = true; */
    /* 	gps->pos_ep->m_full[0] = gps_fix.epx; */
    /* 	gps->pos_ep->m_full[1] = gps_fix.epy; */
    /* 	gps->pos_ep->m_full[2] = gps_fix.epv; */
    /* } */

    /* if(isnan(gps_fix.speed) || */
    /*    isnan(gps_fix.climb) || */
    /*    isnan(gps_fix.track)) */
    /* { */
    /* 	gps->vel_ok = false; */
    /* } */
    /* else */
    /* { */
    /* 	gps->vel_ok = true; */
    /* 	gps->speed = gps_fix.speed; */
    /* 	gps->climb = gps_fix.climb; */
    /* 	gps->track = deg2rad(gps_fix.track); */
    /* 	if(isnan(gps_fix.eps) || */
    /* 	   isnan(gps_fix.epc) || */
    /* 	   isnan(gps_fix.epd)) */
    /* 	{ */
    /* 	    gps->vel_ep_ok = false; */
    /* 	} */
    /* 	else */
    /* 	{ */
    /* 	    gps->vel_ep_ok = true; */
    /* 	    gps->speed_ep = gps_fix.eps; */
    /* 	    gps->climb_ep = gps_fix.epc; */
    /* 	    gps->track_ep = deg2rad(gps_fix.epd); */
    /* 	} */
    /* } */

    return ERROR_OK;
}

int gps_comm_get_data(gps_t *gps, gps_comm_data_t *gps_data)
{
    int retval = ERROR_OK;
    if(gps_data == NULL)
    {
	err_check(ERROR_NULL_POINTER,"Invalid argument!");
    }
    if(!gps_comm_fix(gps))
    {
	err_check(ERROR_GPS_NO_FIX,"Will not accept data, fix not available!");
    }

    retval = uquad_mat_copy(gps_data->pos, gps->pos);
    err_propagate(retval);
    gps_data->dop = gps->dop;
/*     if(gps->vel_ok) */
/*     { */
/* #if GPS_COMM_DATA_NON_INERTIAL_VEL */
/* imu_data_t *imu_data must be an argument */
/* 	if(imu_data != NULL) */
/* 	{ */
/* 	    m3x1->m_full[0] = gps->speed*sin(gps->track); */
/* 	    m3x1->m_full[1] = -gps->speed*cos(gps->track);//TODO verify! */
/* 	    m3x1->m_full[2] = gps->climb; */
/* 	    retval = uquad_mat_rotate(true, */
/* 				      gps_data->vel, */
/* 				      m3x1, */
/* 				      imu_data->magn->m_full[0], */
/* 				      imu_data->magn->m_full[1], */
/* 				      imu_data->magn->m_full[2], */
/* 				      m3x3); */
/* 	    err_propagate(retval); */
/* 	} */
/* 	else */
/* 	{ */
/* 	    err_check(ERROR_GPS_SYS_REF,"Cannot convert to non-inertial frame without IMU!"); */
/* 	} */
/* #else // GPS_COMM_DATA_NON_INERTIAL_VEL */
/* 	if(imu_data != NULL) */
/* 	{ */
/* 	    err_log("WARN: Vels are inertial, will ignore IMU!"); */
/* 	} */
/* 	gps_data->vel->m_full[0] = gps->speed*sin(gps->track); */
/* 	gps_data->vel->m_full[1] = -gps->speed*cos(gps->track);//TODO verify! */
/* 	gps_data->vel->m_full[2] = gps->climb; */
/* #endif // GPS_COMM_DATA_NON_INERTIAL_VEL */
/*     } */
    return ERROR_OK;
}

int gps_comm_get_data_unread(gps_t *gps, gps_comm_data_t *gps_data, imu_data_t *imu_data)
{
    int retval = ERROR_OK;
    if(!gps->unread_data)
    {
	err_check(ERROR_GPS_NO_UPDATES,"NO new data!");
    }
    if(imu_data != NULL)
    {
	err_check(ERROR_GPS_NO_UPDATES,"NOT implemented!");
    }
    retval = gps_comm_get_data(gps, gps_data);
    err_propagate(retval);
    gps->unread_data = false;
    return ERROR_OK;
}

void gps_comm_dump(gps_t *gps, gps_comm_data_t *gps_data, FILE *stream)
{
    int i;
    if(stream == NULL)
	stream = stdout;

    // timestamp
    log_tv_only(stream,gps->timestamp);
    /* // fix type */
    /* log_int_only(stream, gps->fix); */
    // position
    for(i = 0; i < 3; ++i)
	log_double_only(stream,gps_data->pos->m_full[i]);
    /* // vel */
    /* for(i = 0; i < 3; ++i) */
    /* 	log_double_only(stream,(gps->vel_ok)? */
    /* 			gps_data->vel->m_full[i]: */
    /* 			0); */
    // raw data
    log_double_only(stream, gps->lat);
    log_double_only(stream, gps->lon);
    log_double_only(stream, gps->dop);
    /* log_double_only(stream, (gps->vel_ok)?gps->speed:0); */
    /* log_double_only(stream, (gps->vel_ok)?gps->climb:0); */
    /* log_double_only(stream, (gps->vel_ok)?gps->track:0); */
    /* log_int_only(stream, gps->vel_ok); */
    log_eol(stream);
}

gps_comm_data_t *gps_comm_data_alloc(void)
{
    int retval;
    gps_comm_data_t *gps_data = (gps_comm_data_t *)malloc(sizeof(gps_comm_data_t));
    mem_alloc_check(gps_data);
    if(gps_data == NULL)
    {
	cleanup_log_if(ERROR_MALLOC, "Failed to allocate gps_comm_data_t!");
    }
    gps_data->pos = uquad_mat_alloc(3,1);
    gps_data->vel = uquad_mat_alloc(3,1);
    if(gps_data->pos == NULL || gps_data->vel == NULL)
    {
	cleanup_log_if(ERROR_MALLOC, "Failed to allocate gps_comm_data_t!");
    }
    retval = uquad_mat_zeros(gps_data->pos);
    cleanup_if(retval);
    retval = uquad_mat_zeros(gps_data->vel);
    return gps_data;

    cleanup:
    gps_comm_data_free(gps_data);
    return NULL;
}

void gps_comm_data_free(gps_comm_data_t *gps_data)
{
    if(gps_data != NULL)
    {
	uquad_mat_free(gps_data->pos);
	uquad_mat_free(gps_data->vel);
    }
    free(gps_data);
}
