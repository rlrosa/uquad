#ifndef UQUAD_GPS_COMM_H
#define UQUAD_GPS_COMM_H

#include <gpsd.h>

typedef struct gps_data_t gps_t;

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
 * @return 
 */
int  gps_comm_deinit(gps_t *gps);

/** 
 * From gps.h
 * GPS status -- always valid. Do we have a fix?
 * 
 * @param gps 
 * 
 * @return status:
 *             0: STATUS_NO_FIX
 *             1: STATUS_FIX
 *             2: STATUS_DGPS_FIX
 */
int gps_comm_get_status(gps_t *gps);

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
 * @param gps 
 * 
 * @return error code.
 */
int gps_comm_read(gps_t *gps);

/** 
 * Returns info from the GPS.
 * Look for the data structure def in gps.h for fields available.
 * 
 * @param gps 
 * 
 * @return 
 */
struct gps_fix_t gps_comm_get_data(gps_t *gps);

#endif //UQUAD_GPS_COMM_H
