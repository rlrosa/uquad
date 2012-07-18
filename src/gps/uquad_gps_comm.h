/**
 * uquad_gps_comm: lib for communicating with gpsd
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
 * @file   uquad_gps_comm.h
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 11:08:44 2012
 *
 * @brief  lib for communicating with gpsd
 *
 * Features;
 *   - Will put data from gpsd into matrices, making it compatible with the rest of the uquad
 *   system.
 *   - Converts from lat/lon to UTM.
 *   - Can convert to non-inertial reference system if GPS_COMM_DATA_NON_INERTIAL_VEL is set
 *   to 1 and IMU data is available.
 *
 * Examples:
 *   - src/main/main.c
 *   - src/test/gps/gps_test.c
 */
#ifndef UQUAD_GPS_COMM_H
#define UQUAD_GPS_COMM_H

#include <uquad_aux_math.h>
#include <stdio.h>
#include <imu_comm.h>

#define GPS_COMM_DATA_NON_INERTIAL_VEL 0 // Convert vel to non-inertial sys
#define GPS_COMM_WAIT_FIX_SLEEP_MS     300

#define GPS_INIT_TOUT_S                60// GPS init timeout in sec
#define GPS_INIT_TOUT_US               0 // GPS init timeout in sec

#define GPS_NMEA_MAX_LEN               82
#define GPS_NMEA_START                 '$'
#define GPS_NMEA_DELIM                 ','
#define GPS_NMEA_TYPE_LEN              5
#define GPS_NMEA_GPGGA                 "GPGGA"
#define GPS_NMEA_EOS                   '*'
#define GPS_NMEA_EOL1                  '\n'
#define GPS_NMEA_EOL2                  '\r'


typedef struct gps_data_t gpsd_t;

typedef struct utm{
    double easting, northing;
    char let;
    int zone;
}utm_t;

/**
 * This is the data that we are currently using.
 * GPSD offers more data, that may eventually be used.
 */
typedef struct gps_comm_data{
    uquad_mat_t *pos; // Position (inertial system) {x,y,z}     [m]
    uquad_mat_t *vel; // Speed    (inertial system) {vx,vy,vz}  [m/s]
}gps_comm_data_t;

typedef struct gps_fix_ {
    int    mode;	/* Mode of fix */
#define MODE_NOT_SEEN	0	/* mode update not seen yet */
#define MODE_NO_FIX	1	/* none */
#define MODE_2D  	2	/* good for latitude/longitude */
#define MODE_3D  	3	/* good for altitude/climb too */
    double ept;		/* Expected time uncertainty */
    double latitude;	/* Latitude in degrees (valid if mode >= 2) */
    double epy;  	/* Latitude position uncertainty, meters */
    double longitude;	/* Longitude in degrees (valid if mode >= 2) */
    double epx;  	/* Longitude position uncertainty, meters */
    double altitude;	/* Altitude in meters (valid if mode == 3) */
    double epv;  	/* Vertical position uncertainty, meters */
    double track;	/* Course made good (relative to true north) */
    double epd;		/* Track uncertainty, degrees */
    double speed;	/* Speed over ground, meters/sec */
    double eps;		/* Speed uncertainty, meters/sec */
    double climb;       /* Vertical speed, meters/sec */
    double epc;		/* Vertical speed uncertainty */
}gps_fix_t;

typedef struct gps{
    int fd;                      // File descriptor for reading from gps
    int fix;                     // Fix type.

    gps_fix_t *gps_fix;          // Information in gpsd format

    uquad_mat_t *pos;            // Position (inertial system)     {x,y,z}     [m]
    uquad_mat_t *pos_ep;         // Position uncertainty           {x,y,z}     [m]

    gps_comm_data_t *gps_data;   // GPS data
    gps_comm_data_t *gps_data_ep;// GPS data uncertainty

    struct timeval timestamp;    // Time of update
    uquad_bool_t unread_data;    // Unread data is available, at least pos
    uquad_bool_t pos_ep_ok;      // pos_ep has valid data
    uquad_bool_t vel_ok;         // speed/climb has valid data
    uquad_bool_t vel_ep_ok;      // speed/climb uncertainty is valid

    // Starting point
    uquad_mat_t *pos_0;          // Initial pos (inertial system)  {x,y,z}     [m]

    // Aux structures
    utm_t utm;                   // UTM coordinates - used to calculate pos[0:1]

    double lat;
    double lon;

    double speed;                // Speed over ground             [m/s]
    double speed_ep;             // Speed uncertainty             [m/s]
    double climb;                // Vertical speed                [m/s]
    double climb_ep;             // Vertical speed uncertainty    [m/s]

    double track;                // Course Made Good (rel. to N)  [rad]
    double track_ep;             // CMG uncertainty               [rad]

    // Log file reading
    FILE   *dev;                 // For reading GPS data from log file
    struct timeval *tv_start;    // Test/Client program start time.
    struct timeval *tv_log_start;// Start time according to log file.
    // log indicates pgm started
}gps_t;

/**
 * Initializes memory for gps data structure, and opens connection to daemon.
 * 
 * @return Pointer to structure, or NULL if something went wrong.
 */
gps_t *  gps_comm_init(const char *device);

/**
 * Frees memory used for gps data structure, and closes connection to daemon.
 *  
 * @param gps 
 *
 */
void  gps_comm_deinit(gps_t *gps);

/**
 * Waits for gps fix.
 * If successful, then gps should have data ready to be read.
 * Will set initial position.
 *
 * @param gps
 * @param got_fix Reports success/failure
 * @param t_out If NULL, then will wait forever, else will only wait for t_out
 *
 * @return error code
 */
int gps_comm_wait_fix(gps_t *gps, uquad_bool_t *got_fix, struct timeval *t_out);

/**
 * Sets start time of client/test program.
 * Should only be used when reading from a log file.
 *
 * @param gps
 * @param tv_start new start time
 *
 * @return error code
 */
int gps_comm_set_tv_start(gps_t *gps, struct timeval tv_start);

/**
 * Sets starting position.
 *
 * NOTE: Must be called at most ONCE.
 *
 * @param gps
 * @param gps_dat Position to set as 0
 *
 * @return error code
 */
int gps_comm_set_0(gps_t *gps, gps_comm_data_t *gps_dat);

/**
 * Returns position determined by GPS when pgm started.
 * Must be called after gps_comm_set_0() was successful.
 *
 * @param gps
 * @param gps_dat Answer
 *
 * @return error code
 */
int gps_comm_get_0(gps_t *gps, gps_comm_data_t *gps_dat);

/**
 * From gps.h, GPS fix mode.
 * We want 3D fix, for x,y,z to be valid.
 * 
 * @param gps 
 * 
 * @return status:
 *             0: MODE_NOT_SEEN
 *             1: MODE_NO_FIX
 *             2: MODE_2D
 *             3: MODE_3D
 */
int gps_comm_get_fix_mode(gps_t *gps);

/**
 * Returns true iif GPS has 3D fix
 * 
 * @param gps 
 * 
 * @return 
 */
uquad_bool_t gps_comm_3dfix(gps_t *gps);

/**
 * Return file descriptor to allow reading only if new data is available.
 * 
 * @param gps 
 * 
 * @return 
 */
int gps_comm_get_fd(gps_t *gps);

/**
 * Update data from GPS.
 * This function should only be called if new data is available, which can be checked
 * using select() with the file descriptor from gps_comm_get_fd()
 *
 * Will update status of gps->unread_data if successful.
 * NOTE: If NOT reading GPS data from a log file, then (tv_curr==NULL). If data is being
 * read from a log file, then "tv_curr" must be updated using gettimeofday() before calling
 * gps_comm_read()
 * Time in the log file will be used to determine whether a new line of data should be read,
 * or if not enough time has passed (this is specified by the timestamps in the log file).
 *
 * @param gps 
 * @param ok true iif update was successful
 * @param tv_curr Current time in client program or NULL (must be NULL if using real GPS) 
 * 
 * @return error code.
 */
int gps_comm_read(gps_t *gps, uquad_bool_t *ok, struct timeval *tv_curr);

/**
 * Returns info from the GPS.
 * Look at the declaration of gps_t for more info.
 * NOTE: If GPS_COMM_DATA_NON_INERTIAL_VEL, imu_data must be NULL.
 *
 * @param gps 
 * @param gps_data Answer. Vel only valid if gps->vel_ok
 * @param imu_data IMU info to convert climb/speed/true_north to vx,vy,vz
 *
 * @return error code
 */
int gps_comm_get_data(gps_t *gps, gps_comm_data_t *gps_data, imu_data_t *imu_data);

/**
 * Same as gps_comm_get_data, except that only unread data will be considered
 *
 * @param gps 
 * @param gps_data
 * @param imu_data 
 *
 * @return 
 */
int gps_comm_get_data_unread(gps_t *gps, gps_comm_data_t *gps_data, imu_data_t *imu_data);

/**
 * Prints data from last gps_read, with timestamp
 * Format:
 *     timestamp fix x y z vx vy vz lat lon speed climb track vel_ok
 *
 * Description:
 *   - timestamp: is the moment the beagleboard received the data from the GPS.
 *   - fix: Fix type, should be 3D.
 *   - x,y,z: Position relative to inertial frame.
 *   - vx, vy, vz: Speed relative to inertial frame.
 *   - lat, lon: Raw position data
 *   - speed, climb, track: Raw speed data
 *   - vel_ok: Speed data is valid
 *
 * @param gps 
 * @param stream 
 */
void gps_comm_dump(gps_t *gps, gps_comm_data_t *gps_data, FILE *stream);

//#define sa         6378137.000000L
//#define sb         6356752.314245L
#define deg2utm_e  0.082094437950043   // (sqrt((sa^2) - (sb^2))/sb)
#define deg2utm_ee 0.00673949674233346 // (deg2utm_e*deg2utm_e)
#define deg2utm_c  6399593.62575867    // ((sa*sa)/sb)
/**
 * Converts from Lat/Long to UTM Coordinates (WGS84)
 * C implementation of deg2utm.m
 * Author: Rafael Palacios
 *         Universidad Pontificia Comillas
 *         Madrid, Spain
 * 
 * @param utm answer
 * @param lat 
 * @param lon 
 * 
 * @return 
 */
int gps_comm_deg2utm(utm_t *utm, double lat, double lon);

/**
 * Allocates mem for gps_comm_data_t structure.
 *
 *
 * @return Pointer to data or NULL if failure.
 */
gps_comm_data_t *gps_comm_data_alloc(void);

/**
 * Allocates mem for gps_comm_data_t structure.
 *
 * @param gps_data Structure to free (may be NULL)
 *
 * @return
 */
void gps_comm_data_free(gps_comm_data_t *gps_data);

#endif //UQUAD_GPS_COMM_H
