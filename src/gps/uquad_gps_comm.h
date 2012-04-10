#ifndef UQUAD_GPS_COMM_H
#define UQUAD_GPS_COMM_H

#include <gpsd.h>
#include <uquad_aux_math.h>
#include <stdio.h>
#include <imu_comm.h>

typedef struct gps_data_t gpsd_t;

typedef struct utm{
    int easting, northing;
    char let;
    int zone;
}utm_t;

typedef struct gps{
    gpsd_t *gpsd;             // GPSD interface
    int fix;                  // Fix type.

    uquad_mat_t *pos;         // Position             {x,y,z}  [m]
    uquad_mat_t *pos_ep;      // Position uncertainty {x,y,z}  [m]

    uquad_mat_t *vel;         // Speed             {vx,vy,vz}  [m/s]
    uquad_mat_t *vel_ep;      // Speed uncertainty {vx,vy,vz}  [m/s]

    struct timeval timestamp; // Time of update
    uquad_bool_t unread_data; // Unread data is available, at least pos
    uquad_bool_t pos_ep_ok;   // pos_ep has valid data
    uquad_bool_t vel_ok;      // speed/climb has valid data
    uquad_bool_t vel_ep_ok;   // speed/climb uncertainty is valid

    // Aux structures
    utm_t utm;                // UTM coordinates - used to calculate pos[0:1]

    double speed;             // Speed over ground             [m/s]
    double speed_ep;          // Speed uncertainty             [m/s]
    double climb;             // Vertical speed                [m/s]
    double climb_ep;          // Vertical speed uncertainty    [m/s]

    double track;             // Course Made Good (rel. to N)  [rad]
    double track_ep;          // CMG uncertainty               [rad]
}gps_t;

/** 
 * Initializes memory for gps data structure, and opens connection to daemon.
 * 
 * @return Pointer to structure, or NULL if something went wrong.
 */
gps_t *gps_comm_init(void);

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
 * NOTE: Must only request vel data if gps->vel_ok, otherwise, vel must be NULL.
 *
 * @param gps 
 * @param pos answer, absolute position [m]
 * @param vel NULL or answer, relative to quadcopter frame (non-inertial) [m/s]
 * @param imu_data IMU info to convert climb/speed/true_north to vx,vy,vz
 * 
 * @return error code
 */
int gps_comm_get_data(gps_t *gps, uquad_mat_t *pos, uquad_mat_t *vel, imu_data_t *imu_data);

/** 
 * Same as gps_comm_get_data, except that only unread data will be considered
 * 
 * @param gps 
 * @param pos 
 * @param vel 
 * @param imu_data 
 * 
 * @return 
 */
int gps_comm_get_data(gps_t *gps, uquad_mat_t *pos, uquad_mat_t *vel, imu_data_t *imu_data);

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
