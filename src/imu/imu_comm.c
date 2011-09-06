#include "imu_comm.h"

static double grad2rad(double degrees){
  // PI/180 == 0.017453292519943295
  return degrees*0.017453292519943295;
}

static double rad2grad(double radians){
  // 180/PI == 57.29577951308232
  return radians*57.29577951308232;
}

#define IMU_SENS_OPT_COUNT 4
static double imu_sens_g = {1.5,2,4,6};
static int imu_sens_mv_per_g = {800,600,400,300};
int imu_get_sens(int sens){
  assert(sens<IMU_SENS_OPT_COUNT);
  return imu_sens_mv_per_g[sens];
}   

/** 
 * Reads data until a frame init char is found.
 * Keeps going until a end char is found. Then stops.
 * 
 * @param imu 
 * @param frame New frame is returned here.
 * 
 * @return error code
 */
static int imu_read_frame(struct imu * imu, struct imu_frame * frame){
  int retval;
  unsigned char tmp = '@';// Anything diff from IMU_FRAME_INIT_CHAR
  int watchdog = 0;
  while(watchdog < IMU_FRAME_SIZE_BYTES_DEFAULT){
    retval = fread(&tmp,2,1,imu->device);
    if(retval == 0){
      perror("Read error...");
      return ERROR_READ_TIMEOUT;
    }else{
      if(tmp == IMU_FRAME_INIT_CHAR)
	break;
    }
    watchdog++;
  }
  if(watchdog>=IMU_FRAME_SIZE_BYTES_DEFAULT)
    return ERROR_READ_SYNC;

  // At this point we shpoul
  
  int * var; FILE * file;
  int retval;
  
  byte_buff_filled = 7;
  *var = (((*var)<<1) | ((byte_buff&0x80)==0x80));
}

/** 
 * 
 * 
 * @param imu 
 * @param gyro_data Raw data from IMU
 * @param gyro_reading Rate in rad/sec
 * 
 * @return error code
 */
static int imu_gyro_read(struct imu * imu, unsigned char * gyro_data, double * gyro_reading){
  //TODO
}

/** 
 * 
 * 
 * @param imu 
 * @param acc_data Raw data from IMU
 * @param acc_reading Acceleration, in m/s^2
 * 
 * @return error code
 */
static int imu_acc_read(struct imu * imu, unsigned char * acc_data, double * acc_reading){
  //TODO
}

static int imu_parse_frame(struct imu_frame * frame, double * xyzrpy){
  // Parse a frame, pull out acc and gyro readings.
  //TODO
//            acc_x_str = words[2]
//            acc_y_str = words[3]
//            acc_z_str = words[4]
//            pitch_str = words[5]
//            roll_str = words[6]
//            yaw_str = words[7]            
//            pitch_sensor = gyro_read(pitch_str,pitch_zero)
//            roll_sensor = gyro_read(roll_str,roll_zero)
//            yaw_sensor = gyro_read(yaw_str,yaw_zero)
//            acc_x_sensor = acc_read(float(acc_x_str),acc_x_zero,sens)
//            acc_y_sensor = acc_read(float(acc_y_str),acc_y_zero,sens)
//            acc_z_sensor = acc_read(float(acc_z_str),acc_z_zero,sens)
}
