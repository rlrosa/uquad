#include "imu_comm.h"

imu_status_t imu_comm_get_status(imu_t * imu){
    return imu->status;
}

/** 
 * IMU fw accepts commands while in idle mode.
 * 
 * @param imu 
 * @param cmd to send to the imu
 * 
 * @return error code
 */
static int imu_comm_send_cmd(imu_t * imu, unsigned char cmd){
    uquad_bool_t ready = false;
    int retval;
    retval = imu_comm_check_io_locks(imu->device, NULL, &ready);
    err_propagate(retval);
    if(!ready){
	// cannot write
	err_check(ERROR_WRITE,"Write error: Writing command would lock.");
    }else{
	// issue command
	retval = fprintf(imu->device,"%c\n",cmd);
	if(retval<0){
	    err_check(ERROR_WRITE,"Write error: Failed to send cmd to IMU");
	}
    }
    return ERROR_OK;
}

static int imu_comm_go_idle(imu_t * imu){
    int retval;
    if(imu->status == IMU_COMM_STATE_IDLE){
	printf("IMU already idle.\n");
	return ERROR_OK;
    }
    // Stop IMU
    retval = imu_comm_send_cmd(imu,IMU_COMMAND_IDLE);
    err_propagate(retval);
    // Get out of menu
    retval = imu_comm_send_cmd(imu,IMU_COMMAND_EXIT);
    err_propagate(retval);
    imu->status = IMU_COMM_STATE_IDLE;
    return ERROR_OK;
}

static int imu_comm_resume(imu_t * imu){
    int retval;
    if(imu->status == IMU_COMM_STATE_RUNNING){
	printf("IMU already running.\n");
	return ERROR_OK;
    }
    // Run IMU
    retval = imu_comm_send_cmd(imu,IMU_COMMAND_RUN);
    err_propagate(retval);
    imu->status = IMU_COMM_STATE_RUNNING;
    return ERROR_OK;
}

// IMU options
// sens options
unsigned char imu_sens_opt[IMU_SENS_OPT_COUNT] = {IMU_COMMAND_ACC_1G, \
						  IMU_COMMAND_ACC_2G, \
						  IMU_COMMAND_ACC_4G, \
						  IMU_COMMAND_ACC_6G};
// sampling freq options
unsigned char imu_fs_opt[IMU_FS_OPT_COUNT] = {IMU_COMMAND_FS_50,	\
					      IMU_COMMAND_FS_100,	\
					      IMU_COMMAND_FS_150,	\
					      IMU_COMMAND_FS_200,	\
					      IMU_COMMAND_FS_250};
// sampling freq values
unsigned char imu_fs_values[IMU_FS_OPT_COUNT] = {50,100,150,200,250};

/** 
 * Set accelerometer sensitivity.
 * IMU must be idle, and will be left idle.
 * 
 * @param imu 
 * @param new_value 
 * 
 * @return error code
 */
int imu_comm_set_acc_sens(imu_t * imu, int new_value){
    int retval;
    if((new_value<0) || (new_value > IMU_SENS_OPT_COUNT)){
	err_check(ERROR_INVALID_ARG,"Invalid value for acc sensitivity");
    }
    // IMU should be idle
    if(imu->status != IMU_COMM_STATE_IDLE){
	err_check(ERROR_IMU_STATUS,"IMU must be idle to set acc sens");
    }

    // Set new acc sens
    retval = imu_comm_send_cmd(imu,imu_sens_opt[new_value]);
    err_propagate(retval);
    // Update struct value
    imu->settings.acc_sens = new_value;
    return retval;
}

int imu_comm_get_acc_sens(imu_t * imu, int * acc_index){
    if(acc_index == NULL){
	err_check(ERROR_NULL_POINTER,"Cannot return value, null pointer as argument.");
    }
    *acc_index = imu->settings.acc_sens;
    return ERROR_OK;
}

/** 
 * Set sampling frequency (fs)
 * IMU must be idle, and will be left idle.
 * 
 * @param imu 
 * @param new_value 
 * 
 * @return error code
 */
int imu_comm_set_fs(imu_t * imu, int new_value){
    int retval;
    if((new_value<0) || (new_value > IMU_FS_OPT_COUNT)){
	err_check(ERROR_INVALID_ARG,"Invalid value for sampling frequency");
    }
    // IMU should be idle
    if(imu->status != IMU_COMM_STATE_IDLE){
	err_check(ERROR_IMU_STATUS,"IMU must be idle to set fs");
    }

    // Set new fs
    retval = imu_comm_send_cmd(imu,imu_fs_opt[new_value]);
    err_propagate(retval);
    // Update struct value
    imu->settings.fs = new_value;
    imu->settings.T = (double)1/imu_fs_values[new_value];
    return retval;
}

int imu_comm_get_fs(imu_t * imu, int * fs_index){
    if(fs_index == NULL){
	err_check(ERROR_NULL_POINTER,"Cannot return value, null pointer as argument.");
    }
    *fs_index = imu->settings.fs;
    return ERROR_OK;
}

static int imu_comm_send_defaults(imu_t * imu){
    int retval;
    // Set sampling frequency
    retval = imu_comm_set_fs(imu,IMU_DEFAULT_FS);
    err_propagate(retval);
    // Set acc sensitivity
    retval = imu_comm_set_acc_sens(imu,IMU_DEFAULT_ACC_SENS);
    err_propagate(retval);
    return retval;
}

static void imu_comm_calibration_clear(imu_t * imu){
    imu->is_calibrated = false;
    imu->calibration_counter = -1;
    int i;
    for(i=0;i<IMU_SENSOR_COUNT;++i){
	imu->null_estimates.xyzrpy[i] = 0;
    }
    imu->null_estimates.timestamp.tv_sec = 0;
    imu->null_estimates.timestamp.tv_usec = 0;
}

/** 
 * Configure IMU to respond as expected, specifically, to answer to macros
 * in imu_comm.h
 * - Set binary mode.
 * - Disable auto run
 * 
 * @param imu 
 * 
 * @return error code
 */
static int imu_comm_configure(imu_t * imu){
    int retval;
    // The following sequence of commands should get to the main IMU menu, from
    //from any initial state. If the unit was running then it will ignore all of
    //this, no damage done.

    // Get out of channel select menu or freq select menu
    retval = imu_comm_send_cmd(imu,IMU_COMMAND_X);
    err_propagate(retval);

    // Get out of acc sens menu
    retval = imu_comm_send_cmd(imu,IMU_COMMAND_ONE);
    err_propagate(retval);

    // If started from main menu, then previous command would have ended in channel
    //select menu, so get out of it
    retval = imu_comm_send_cmd(imu,IMU_COMMAND_X);
    err_propagate(retval);

    // Assuming binary mode on && autorun off, i don't want to parse...
    //TODO parse to avoid assuming stuff! or modify imu code to be easier...

    // Get out of menu
    retval = imu_comm_send_cmd(imu,IMU_COMMAND_EXIT);
    err_propagate(retval);

    // Now IMU should be in idle state, where it will recieve commands
    // This is open loop, so let's set assume everything went ok.
    imu->status = IMU_COMM_STATE_IDLE;
    retval = imu_comm_send_defaults(imu);
    err_propagate(retval);

    // Mark IMU as not calibrated
    imu_comm_calibration_clear(imu);

    // Start running in binary mode with all channels ON
    retval = imu_comm_resume(imu);
    err_propagate(retval);
    return retval;
}

static int imu_comm_connect(imu_t * imu, const char * device){
    int retval;
    imu->device = fopen(device,"wb+");
    if(imu->device == NULL){
	fprintf(stderr,"Device %s not found.\n",device);
	return ERROR_OPEN;
    }
    return ERROR_OK;
}

static int imu_comm_disconnect(imu_t * imu){
    int retval = ERROR_OK;
    retval = fclose(imu->device);
    if(retval != ERROR_OK){
	fprintf(stderr,"Failed to close connection to device.\n");
	return ERROR_CLOSE;
    }
    imu->device = NULL;
    return ERROR_OK;
}

static void imu_comm_restart_sampling(imu_t * imu){
    imu->unread_data = 0;
    imu->frames_sampled = 0;
    imu->frame_next = 0;
    imu->avg_ready = false;
}

/** 
 * Initialize IMU struct and send default value to IMU, this
 * ensures starting from a know state.
 * 
 * @return error code
 */
imu_t * imu_comm_init(const char * device){
    imu_t * imu;
    int i,retval;
    imu = (imu_t *)malloc(sizeof(struct imu));
    if(imu == NULL){
	fprintf(stderr,"Failed to allocate mem. \n");
	return imu;
    }
    // Set default values
    imu_comm_restart_sampling(imu);
    imu->status = IMU_COMM_STATE_UNKNOWN;
    imu->settings.fs = IMU_DEFAULT_FS;
    imu->settings.T = (double)1/imu_fs_values[IMU_DEFAULT_FS];
    imu->settings.acc_sens = IMU_DEFAULT_ACC_SENS;
    imu->settings.frame_width_bytes = IMU_DEFAULT_FRAME_SIZE_BYTES;
    for(i=0;i<IMU_SENSOR_COUNT;++i){
	imu->null_estimates.xyzrpy[i] = (1<< (IMU_ADC_BITS - 1)); // Set to mid scale
    }
    imu->null_estimates.timestamp.tv_sec = 0;
    imu->null_estimates.timestamp.tv_usec = 0;

    // now connect to the imu
    retval = imu_comm_connect(imu,device);
    if(retval != ERROR_OK)
	return NULL;

    // Send default values to IMU, then get it running, just in case it wasn't
    retval = imu_comm_configure(imu);
    if(retval != ERROR_OK)
	return NULL;
    return imu;
}

int imu_comm_deinit(imu_t * imu){
    int retval = ERROR_OK;
    if(imu->device != NULL)
	retval = imu_comm_disconnect(imu);
    // ignore answer and keep dying, leftovers are not reliable
    free(imu);
    return retval;
}

static double grad2rad(double degrees){
    // PI/180 == 0.017453292519943295
    return degrees*0.017453292519943295;
}

#if 0
static double rad2grad(double radians){
    // 180/PI == 57.29577951308232
    return radians*57.29577951308232;
}
#endif

static double counts2volts(imu_t * imu, double * acc){
    // Convert from count to m/s^2
    // m/s^2 = counts*vref/counts_full_scale
    *acc = (*acc)*IMU_ADC_COUNTS_2_VOLTS;
    return ERROR_OK;
}

static int gyro_scale_adjust(imu_t * imu, double * gyro_reading){
    //TODO Implement scale calibration,
    // Note: Should be /300, but /450 seems to work better.
    // Will be sensor specific
    // Should get a true calibration instead of this.
    *gyro_reading *= 0.92955;
    return ERROR_OK;
}

static int acc_scale_adjust(imu_t * imu, double * acc_reading){
    //TODO Implement scale calibration,
    // Will be sensor specific
    // Should get a true calibration instead of this.
    return ERROR_OK;
}

//static double imu_sens_g[IMU_SENS_OPT_COUNT] = {1.5, 2, 4, 6};
//static int imu_sens_mv_per_g[IMU_SENS_OPT_COUNT] = {800,600,400,300};
static double imu_sens_mv_per_g[IMU_SENS_OPT_COUNT] = {1.8750,3.3333,13.3333,30};//1000*imu_sens_g/imu_sens_mv_per_g
int imu_get_sens(int sens){
    if(!(sens<IMU_SENS_OPT_COUNT))
	return ERROR_FAIL;
    return imu_sens_mv_per_g[sens];
}

static int volts2g(int sens, double * val_to_convert){
    if(!(sens<IMU_SENS_OPT_COUNT))
	return ERROR_FAIL;
    *val_to_convert = (*val_to_convert)*imu_sens_mv_per_g[sens];
    return ERROR_OK;
}

static unsigned short int swap_LSB_MSB_16(unsigned short int a){
    return (((a&0xFF)<<8)|(a>>8));
}

/** 
 * Returns index of the last frame that was read.
 * 
 * @param imu 
 * 
 * @return index
 */
int frame_circ_index(imu_t * imu){
    return (imu->frame_next + IMU_FRAME_SAMPLE_AVG_COUNT - 1) % IMU_FRAME_SAMPLE_AVG_COUNT;
}

/** 
 * Checks if samples used for avg fall withing a certain interval.
 * It makes no sense to average samples from completely diff moments.
 * 
 * @param imu 
 * 
 * @return true if ok to avg, otherwise false
 */
static uquad_bool_t imu_comm_avg_validate_time_interval(imu_t *imu){
    int retval;
    long double max_interval;
    struct timeval diff;
    retval = uquad_timeval_substract(&diff,				\
				     imu->frame_buffer[frame_circ_index(imu)].timestamp, \
				     imu->frame_buffer[imu->frame_next].timestamp);
    if(retval<=0)
	return false;
    max_interval = (long double)imu->settings.T*IMU_COMM_AVG_MAX_INTERVAL;
    max_interval -= diff.tv_sec;
    if(max_interval<=0)
	return false;	
    max_interval -= diff.tv_usec/1000000;
    return (max_interval > 0)? true:false;
}   

/** 
 * Generates an average for each sensor. Uses a fixed buff size.
 * This should only be called when sampling starts, of if data is discarded 
 * or RX is stopped for a while.
 * 
 * @param imu 
 * 
 * @return error code
 */
static int imu_comm_avg(imu_t * imu){
    int tmp,i,j;
    time_t sec_oldest, sec_new;
    suseconds_t usec_oldest, usec_new;
    
    if(imu->frame_buffer[frame_circ_index(imu)].timestamp.tv_sec)
    imu->avg.timestamp.tv_sec = imu->frame_buffer[frame_circ_index(imu)].timestamp.tv_sec;
    imu->avg.timestamp.tv_usec = imu->frame_buffer[frame_circ_index(imu)].timestamp.tv_usec;
    for(i=0;i<IMU_SENSOR_COUNT;++i){// loop sensors
	tmp = 0;
	for(j=0;j<IMU_FRAME_SAMPLE_AVG_COUNT;++j)// loop sensor data
	    tmp += (int)imu->frame_buffer[j].raw[i];
	imu->avg.xyzrpy[i] = ((double)tmp)/IMU_FRAME_SAMPLE_AVG_COUNT;
    }

    imu->avg_ready = 1;
    return ERROR_OK;
}

/** 
 * Reads 1 byte, expecting it to be IMU_FRAME_INIT_CHAR.
 * NOTE: Assumes device can be read without blocking.
 * 
 * @param imu 
 * 
 * @return error code
 */
static int imu_comm_get_sync(imu_t * imu, uquad_bool_t * in_sync){
    int retval,i;
    *in_sync = false;
    unsigned char tmp = '@';// Anything diff from IMU_FRAME_INIT_CHAR
    for(i=0;i<IMU_DEFAULT_FRAME_SIZE_BYTES;++i){
	retval = fread(&tmp,IMU_INIT_END_SIZE,1,imu->device);
	if(retval < 0){	
	    err_check(ERROR_IO,"Read error: failed to get sync char...");
	}else{
	    if(retval > 0){
		if(tmp == IMU_FRAME_INIT_CHAR){
		    // No error printing, leave that for upper level
		    *in_sync = true;
		    return ERROR_OK;
		}
	    }else{
		// retval == 0
		*in_sync = false;
		return ERROR_READ_SYNC;
	    }
	}
    }
    // If we read 0 then there is no data available, so no sync and no error.
    // Set retval to ERROR_OK, otherwise it'll be # of bytes read
    return ERROR_READ_TIMEOUT;
}

/** 
 * Attempts to sync with IMU, and read data.
 * 
 * @param imu 
 * @param success 
 * 
 * @return 
 */
int imu_comm_read(imu_t * imu,uquad_bool_t * success){
    int retval;
    retval = imu_comm_get_sync(imu,success);
    err_propagate(retval);
    if(*success){
	// sync worked, now get data
	retval = imu_comm_read_frame(imu);
	if(retval!=ERROR_OK)
	    *success = false;
	err_propagate(retval);
    }
    return ERROR_OK;
}

/** 
 * Assumes sync char was read, reads the rest of the data.
 * Keeps going until a end char is found. Then stops.
 * 
 * @param imu 
 * @param frame New frame is returned here.
 * 
 * @return error code
 */
int imu_comm_read_frame(imu_t * imu){
    int retval = ERROR_OK,watchdog,read,i;
    unsigned char tmp = '@';// Anything diff from IMU_FRAME_INIT_CHAR
    imu_frame_t * new_frame;
    new_frame = imu->frame_buffer+imu->frame_next;

    // Get count
    watchdog = 0;
    while(watchdog < READ_RETRIES){
	retval = fread(& new_frame->count,IMU_BYTES_COUNT,1,imu->device);
	if(retval == 1){
	    break;
	}else{
	    if(retval < 0){
		err_check(ERROR_IO,"Read error: Failed to read count...");
	    }else{
		++watchdog;
	    }
	}
    }
    if(watchdog>=READ_RETRIES){
	err_check(ERROR_READ_TIMEOUT,"Read error: Timed out waiting for count...");
    }
    
    // Fix LSB/MSB
    new_frame->count = swap_LSB_MSB_16(new_frame->count);

    // Generate timestamp
    gettimeofday(& new_frame->timestamp,NULL);

    // Read sensors RAW data
    watchdog = 0;
    read = 0;
    while(watchdog < READ_RETRIES){
	retval = fread(new_frame->raw + read,IMU_BYTES_PER_SENSOR,IMU_SENSOR_COUNT-read,imu->device);
	if(retval > 0){
	    read += retval;
	    if(read == IMU_SENSOR_COUNT)
		// done reading
		break;
	}else{
	    if(retval < 0){
		err_check(ERROR_IO,"Read error: Failed to read sensor data...");
	    }else{
		++watchdog;
	    }
	}
    }
    if(watchdog>=READ_RETRIES){
	err_check(ERROR_READ_TIMEOUT,"Read error: Timed out waiting for sensor data...");
    }

    // Change LSB/MSB
    for(i=0;i<IMU_SENSOR_COUNT;++i)
	new_frame->raw[i] = swap_LSB_MSB_16(new_frame->raw[i]);

    // Now read out the end char
    watchdog = 0;
    while(watchdog < READ_RETRIES){
	retval = fread(&tmp,IMU_INIT_END_SIZE,1,imu->device);
	if(retval > 0){
	    if(tmp == IMU_FRAME_END_CHAR){
		break;
	    }else{
		err_check(ERROR_READ_SYNC,"Unexpected end of frame char: Discarding frame...");
	    }
	}else{
	    if(retval < 0){
		err_check(ERROR_IO,"Read error: Failed to read end char...");
	    }else{
		++watchdog;
	    }
	}
    }
    if(watchdog>=READ_RETRIES){
	err_check(ERROR_READ_TIMEOUT,"Read error: Timed out waiting for end char...");
    }

    // Everything went ok, pat pat :)
    if(imu->frames_sampled < IMU_FRAME_SAMPLE_AVG_COUNT){
	++imu->frames_sampled;
    }else{
	if(imu->frames_sampled == IMU_FRAME_SAMPLE_AVG_COUNT){
	    // If we have enough samples then calculate an avg
	    retval = imu_comm_avg(imu);
	    if(retval != ERROR_OK){
		// Something went wrong with the avg, let's start over
		imu->frames_sampled == 0;
		fprintf(stderr,"avg failed, restarting frame count");
		imu_comm_restart_sampling(imu);
	    }
	}
    }
    if(imu->status == IMU_COMM_STATE_CALIBRATING){
	// Lets keep on working on the calibration
	retval = imu_comm_calibration_add_frame(imu,new_frame);
	err_propagate(retval);
    }

    ++imu->frame_next; imu->frame_next %= IMU_FRAME_SAMPLE_AVG_COUNT;
    ++imu->unread_data;

    return ERROR_OK;
}

/**
 * Convert data read from gyro to rad/s
 * 
 * Gyro outputs ~1.65v for 0deg/sec, then this goes through a
 * 10bit ADC on the Atmega which compares 0-3.3v.
 * The data received is the result of the ADC.
 * 
 * @param imu 
 * @param frame Contains RAW data from IMU
 * @param gyro_reading Rate in rad/sec
 * 
 * @return error code
 */
static int imu_comm_gyro_read(imu_t * imu, imu_frame_t * frame, double * gyro_reading){
    int retval = ERROR_OK, i;
    for(i = 0; i<IMU_GYROS; ++i){
	gyro_reading[i] = ((double) *(frame->raw + IMU_ACCS + i)) - imu->null_estimates.xyzrpy[IMU_ACCS + i];
	retval = gyro_scale_adjust(imu,gyro_reading+i);
	err_propagate(retval);
	gyro_reading[i] = grad2rad(gyro_reading[i]);
    }
    return retval;
}

/** 
 * 
 * 
 * @param imu 
 * @param frame Raw data from IMU
 * @param acc_reading Acceleration, in m/s^2
 * 
 * @return error code
 */
static int imu_comm_acc_read(imu_t * imu, imu_frame_t * frame, double * acc_reading){
    int retval = ERROR_OK, i;
    for(i = 0; i<IMU_ACCS; ++i){
	// Avoid math on char to be able to hanlde negative results
	acc_reading[i] = ((double) * (frame->raw + i)) - imu->null_estimates.xyzrpy[i];
	retval = counts2volts(imu,acc_reading+i);
	err_propagate(retval);
	retval = volts2g(imu->settings.acc_sens,acc_reading+i);
	err_propagate(retval);
	retval = acc_scale_adjust(imu,acc_reading+i);
	err_propagate(retval);
    }
    return retval;
}

/** 
 * Convert raw data from IMU to double
 * 
 * @param frame raw data
 * @param data converted data
 * 
 * @return error code
 */
static int imu_comm_raw2data(imu_t * imu, imu_frame_t * frame, imu_data_t * data){
    int retval;
    if(data == NULL || frame == NULL){
	err_check(ERROR_NULL_POINTER,"Non null pointers required as args...");
    }
    // Get timestamp
    data->timestamp = frame->timestamp;

    // Get ACC readings
    retval = imu_comm_acc_read(imu, frame, data->xyzrpy);
    err_propagate(retval);

    // Get gyro reading
    retval = imu_comm_gyro_read(imu, frame, data->xyzrpy + IMU_ACCS);
    err_propagate(retval);

    return ERROR_OK;
}

/** 
 * Calculates value of the sensor reading from the RAW data, using current imu calibration.
 * 
 * @param imu Current imu status
 * @param xyzrpy Answer is returned here
 * 
 * @return error code
 */
int imu_comm_get_data_latest(imu_t * imu, imu_data_t * data){
    int retval = ERROR_OK;

    imu_frame_t * frame = imu->frame_buffer + frame_circ_index(imu);
    retval = imu_comm_raw2data(imu,frame,data);
    err_propagate(retval);

    return retval;
}

/** 
 * If unread data exists, then calculates the latest value of the sensor reading from the RAW data, using current imu calibration.
 * 
 * @param imu Current imu status
 * @param xyzrpy Answer is returned here
 * 
 * @return error code
 */
int imu_comm_get_data_raw_latest_unread(imu_t * imu, imu_data_t * data){
    int retval = ERROR_OK;
    if(imu->unread_data <= 0){
	err_check(ERROR_FAIL,"No unread data available.");
    }

    imu_frame_t * frame = imu->frame_buffer + frame_circ_index(imu);
    data->timestamp = frame->timestamp;
    int i;
    for(i=0;i<IMU_SENSOR_COUNT;++i){
	data->xyzrpy[i] = frame->raw[i];
    }
    imu->unread_data -= 1;
    return retval;
}

/** 
 * If unread data exists, then calculates the latest value of the sensor reading from the RAW data, using current imu calibration.
 * 
 * @param imu Current imu status
 * @param xyzrpy Answer is returned here
 * 
 * @return error code
 */
int imu_comm_get_data_latest_unread(imu_t * imu, imu_data_t * data){
    int retval = ERROR_OK;
    if(imu->unread_data <= 0){
	err_check(ERROR_FAIL,"No unread data available.");
    }

    imu_frame_t * frame = imu->frame_buffer + frame_circ_index(imu);
    retval = imu_comm_raw2data(imu,frame,data);
    err_propagate(retval);

    imu->unread_data -= 1;
    return retval;
}

uquad_bool_t imu_comm_avg_ready(imu_t * imu){
    return imu->avg_ready;
}

int imu_comm_get_avg(imu_t * imu, imu_data_t * data){
    int i;
    if(imu_comm_avg_ready(imu)){
	for(i=0;i<IMU_SENSOR_COUNT;++i){
	    data->xyzrpy[i] = imu->avg.xyzrpy[i];
	}
	data->timestamp = imu->avg.timestamp;
	imu->avg_ready = 0;
	return ERROR_OK;
    }
    err_check(ERROR_IMU_AVG_NOT_ENOUGH,"Not enough samples to average");
}

/** 
 * Return file descriptor corresponding to the IMU.
 * This should be used when polling devices from the main control loop.
 * 
 * @param imu 
 * @param fds file descriptor is returned here
 * 
 * @return error code
 */
int imu_comm_get_fds(imu_t * imu,int * fds){
    if(imu->device == NULL){
	err_check(ERROR_NULL_POINTER,"Cannot get fds, device set to NULL");
    }
    if(fds == NULL){
	err_check(ERROR_NULL_POINTER,"Cannot return fds in NULL pointer");
    }
    *fds = fileno(imu->device);
    return ERROR_OK;
}

/** 
 * Checks if reading/writing will block.
 * Writing should not be a problem, hw buffers should handle it.
 * If attempting to read and there is no data available, we do not want to
 * lock up the sys, that is the purpose of 'select'.
 * 
 * @param device attemping to read or write to.
 * @param check_read if true then checks if reading locks, if false check writing.
 * @param ready answer returned here
 * 
 * @return error code
 */
int imu_comm_check_io_locks(FILE * device, uquad_bool_t * read_ok, uquad_bool_t * write_ok){
    fd_set rfds,wfds;
    struct timeval tv;
    int retval, fd = fileno(device);
    FD_ZERO(&rfds);
    FD_SET(fd,&rfds);
    FD_ZERO(&wfds);
    FD_SET(fd,&wfds);
    // Set time waiting time to zero
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    // Check if we read/write without locking
    retval = select(fd+1,				\
		    (read_ok != NULL)?&rfds:NULL,	\
		    (write_ok != NULL)?&wfds:NULL,	\
		    NULL,&tv);
    if(read_ok != NULL){
	*read_ok = ((retval > 0) && FD_ISSET(fd,&rfds)) ? true:false;
    }
    if(write_ok != NULL)
	*write_ok = ((retval >0) && FD_ISSET(fd,&wfds)) ? true:false;
    if (retval < 0){
	err_check(ERROR_IO,"select() failed!");
    }else{
	// retval == 0 <--> No data available
    }
    return ERROR_OK;
}

// -- -- -- -- -- -- -- -- -- -- -- --
// Calibration
// -- -- -- -- -- -- -- -- -- -- -- --

uquad_bool_t imu_comm_calibration_is_calibrated(imu_t * imu){
    return imu->is_calibrated;
}	

/** 
 * Get IMU calibration.
 * Currently only calibration is null estimation.
 * //TODO:
 *   - gain
 *   - non linearity
 * 
 * @param imu 
 * @param calibration return data here (check return error code before using)
 * 
 * @return error code
 */
int imu_comm_calibration_get(imu_t * imu, imu_null_estimates_t * calibration){

    int retval,i;
    if(!imu_comm_calibration_is_calibrated(imu)){
	err_check(ERROR_IMU_STATUS,"IMU is not calibrated");
    }
    if(calibration == NULL){
	err_check(ERROR_NULL_POINTER,"Cannot return data in null pointer!");
    }

    for(i=0;i<IMU_SENSOR_COUNT;++i){	
	calibration->xyzrpy[i] = imu->null_estimates.xyzrpy[i];
    }
    calibration->timestamp = imu->null_estimates.timestamp;
    return ERROR_OK;
}

static unsigned int calibration_tmp[IMU_SENSOR_COUNT];
static struct timeval calibration_start_time;
/** 
 * Initiate IMU calibration.
 * NOTE: Assumes sensors are not being excited, ie, imu is staying completely still.
 * 
 * @param imu 
 * 
 * @return 
 */
int imu_comm_calibration_start(imu_t * imu){
    if(imu->status != IMU_COMM_STATE_RUNNING){
	err_check(ERROR_IMU_STATUS,"IMU must be running to calibrate!");
    }
    imu->status = IMU_COMM_STATE_CALIBRATING;
    int i;
    // clear tmp data
    for(i=0;i<IMU_SENSOR_COUNT;++i){
	calibration_tmp[i] = 0;
    }
    imu->calibration_counter = IMU_COMM_CALIBRATION_NULL_SIZE;

    return ERROR_OK;
}

/** 
 * Abort current IMU calibration process. All progress will be lost.
 * If a previous calibration existed, it will be preserved.
 *
 * @param imu 
 * 
 * @return error code
 */
int imu_comm_calibration_abort(imu_t * imu){
    if(imu->status != IMU_COMM_STATE_CALIBRATING){
	err_check(ERROR_IMU_STATUS,"Cannot abort calibration, IMU is not calibrating!");
    }
    // Restore IMU status:
    // if we were calibrating then we were running before
    imu->status = IMU_COMM_STATE_RUNNING;

    return ERROR_OK;
}

/** 
 * Integrate calibration data into IMU.
 * Replaces previous calibration, if any existed.
 * 
 * @param imu 
 * @param timestamp on the last sample used for the calibration
 * 
 * @return error code.
 */
int imu_comm_calibration_finish(imu_t * imu, struct timeval calibration_end_time){
    int retval;
    if(imu->status != IMU_COMM_STATE_CALIBRATING){
	err_check(ERROR_IMU_STATUS,"Cannot finish calibration, IMU is not calibrating!");
    }
    if(imu->calibration_counter != 0){
	err_check(ERROR_IMU_STATUS,"Not enough samples gathered!");
    }

    struct timeval calibration_duration;
    retval = uquad_timeval_substract(&calibration_duration,calibration_end_time,calibration_start_time);
    if(retval < 0){
	err_check(ERROR_FAIL,"Calibration duration should not be negative!");
    }
    //TODO check if calib took too long, and refuse it if so
    //define "too long"

    imu->calibration_counter = -1;
    int i;
    for(i=0;i<IMU_SENSOR_COUNT;++i){
	imu->null_estimates.xyzrpy[i] = ((double)calibration_tmp[i])/IMU_COMM_CALIBRATION_NULL_SIZE;
    }
    imu->null_estimates.timestamp = calibration_end_time;
    imu->is_calibrated = true;
    imu->status = IMU_COMM_STATE_RUNNING;

    return ERROR_OK;
}

/** 
 * Add frame info to build up calibration.
 * Will divide by frame count after done gathering, to avoid loosing info.
 * 
 * @param imu 
 * @param new_frame frame to add to calib 
 * 
 * @return error code
 */
int imu_comm_calibration_add_frame(imu_t * imu, imu_frame_t * new_frame){
    int retval;
    if(imu->status != IMU_COMM_STATE_CALIBRATING){
	err_check(ERROR_IMU_STATUS,"Cannot add frames, IMU is not calibrating!");
    }
    if(imu->calibration_counter <= 0){
	err_check(ERROR_FAIL,"Invalid value for calibration_counter");
    }

    if(imu->calibration_counter == IMU_COMM_CALIBRATION_NULL_SIZE){
	calibration_start_time = new_frame->timestamp;
    }

    int i;
    for(i=0;i<IMU_SENSOR_COUNT;++i){
	calibration_tmp[i] += new_frame->raw[i];
    }
    if(--imu->calibration_counter == 0){
	retval = imu_comm_calibration_finish(imu,new_frame->timestamp);
	err_propagate(retval);
    }

    return ERROR_OK;
}

int imu_comm_calibration_print(imu_null_estimates_t * calibration, FILE * stream){
    return imu_comm_print_data((imu_data_t *)calibration, stream);
}

static int previous_frame_count = -1;
int imu_comm_print_frame(imu_frame_t * frame, FILE * stream){
    int i;
    if(previous_frame_count == -1)
	previous_frame_count = frame->count;
    if(stream == NULL){
	stream = stdout;
    }
    if(previous_frame_count != (frame->count - 1)){
	fprintf(stderr,"\n\n\t\tSkipped frame!!!\n\n");
    }
    previous_frame_count = frame->count;

    fprintf(stream,"%d\t%d\t",(int)frame->timestamp.tv_sec,(int)frame->timestamp.tv_usec);
    fprintf(stream,"%d\t",frame->count);
    for(i=0;i<IMU_SENSOR_COUNT;++i){
	fprintf(stream,"%d\t",frame->raw[i]);
    }
    fprintf(stream,"\n");
    return ERROR_OK;
}    

 int imu_comm_print_data(imu_data_t * data, FILE * stream){
    int i;
    if(stream == NULL){
	stream = stdout;
    }
    fprintf(stream,"%d\t%d\t",(int)data->timestamp.tv_sec,(int)data->timestamp.tv_usec);
    for(i=0;i<IMU_SENSOR_COUNT;++i){
	fprintf(stream,"%f\t",data->xyzrpy[i]);
    }
    fprintf(stream,"\n");
    return ERROR_OK;
}
