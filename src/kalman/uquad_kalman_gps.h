#ifndef UQUAD_KALMAN_GPS_H
#define UQUAD_KALMAN_GPS_H

int uquad_kalman_gps(kalman_io_t* kalman_io_data, gps_data_t* gps_i_data);
void kalman_gps_deinit(kalman_io_t *kalman_io_data);

#endif // UQUAD_KALMAN_GPS_H
