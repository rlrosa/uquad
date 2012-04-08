#ifndef UQUAD_GPS_COMM_H
#define UQUAD_GPS_COMM_H

#include <gpsd.h>
#include <uquad_aux_math.h>
#include <stdio.h>

typedef struct gps_data_t gpsd_t;

typedef struct utm{
    int easting, northing;
    char let;
    int zone;
}utm_t;

typedef struct gps{
    gpsd_t *gpsd;             // GPSD interface
    int fix;                  // Fix type.
    uquad_mat_t *pos;         // Position {x,y,z}  [m]
    double speed;             // Speed over ground [m/s]
    double climb;             // Vertical speed    [m/s]
    utm_t utm;                // UTM coordinates (matches {x,y})
    struct timeval timestamp; // Time of update
    uquad_bool_t unread_data; // Unread data is available
}gps_t;

/** 
 * Initializes memory for gps data structure, and opens connection to daemon.
 * 
 * @return Pointer to structure, or NULL if something went wrong.
 */
gps_t *  gps_comm_init(void);

/** 
 * Frees memory used for gps data structure, and closes connection to daemon.
 *  
 * @param gps 
 *
 */
void  gps_comm_deinit(gps_t *gps);

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
 *using select() with the file descriptor from gps_comm_get_fd()
 * 
 * Will update "unread_data" if successful.
 *
 * @param gps 
 * 
 * @return error code.
 */
int gps_comm_read(gps_t *gps);

/** 
 * Returns info from the GPS.
 * Look at the declaration of gps_t for more info.
 *
 * @param gps 
 * @param pos
 * @param speed 
 * @param climb 
 * 
 * @return error code
 */
int gps_comm_get_data(gps_t *gps, uquad_mat_t *pos, double *speed, double *climb);

/** 
 * Prints data from last gps_read, with timestamp
 * Format:
 *     timestamp speed climb x y z
 * 
 * @param gps 
 * @param stream 
 */
void gps_comm_dump(gps_t *gps, FILE *stream);

//#define sa         6378137.000000L
//#define sb         6356752.314245L
#define deg2utm_e  0.082094437950043L   // (sqrt((sa^2) - (sb^2))/sb)
#define deg2utm_ee 0.00673949674233346L // (deg2utm_e*deg2utm_e)
#define deg2utm_c  6399593.62575867L    // ((sa*sa)/sb)
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

#endif //UQUAD_GPS_COMM_H
