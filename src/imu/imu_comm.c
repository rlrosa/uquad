#include "imu_comm.h"

/** 
 * IMU fw accepts commands while in idle mode.
 * 
 * @param imu 
 * @param cmd to send to the imu
 * 
 * @return error code
 */
static int imu_comm_send_cmd(struct imu * imu, unsigned char cmd){
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

static int imu_comm_go_idle(struct imu * imu){
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

static int imu_comm_resume(struct imu * imu){
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
int imu_comm_set_acc_sens(struct imu * imu, int new_value){
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

int imu_comm_get_acc_sens(struct imu * imu, int * acc_index){
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
int imu_comm_set_fs(struct imu * imu, int new_value){
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

int imu_comm_get_fs(struct imu * imu, int * fs_index){
    if(fs_index == NULL){
	err_check(ERROR_NULL_POINTER,"Cannot return value, null pointer as argument.");
    }
    *fs_index = imu->settings.fs;
    return ERROR_OK;
}

static int imu_comm_send_defaults(struct imu * imu){
    int retval;
    // Set sampling frequency
    retval = imu_comm_set_fs(imu,IMU_DEFAULT_FS);
    err_propagate(retval);
    // Set acc sensitivity
    retval = imu_comm_set_acc_sens(imu,IMU_DEFAULT_ACC_SENS);
    err_propagate(retval);
    return retval;
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
static int imu_comm_configure(struct imu * imu){
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

    // Start running in binary mode with all channels ON
    retval = imu_comm_resume(imu);
    err_propagate(retval);
    return retval;
}

static int imu_comm_connect(struct imu * imu, const char * device){
    int retval;
    imu->device = fopen(device,"wb+");
    if(imu->device == NULL){
	fprintf(stderr,"Device %s not found.\n",device);
	return ERROR_OPEN;
    }
    return ERROR_OK;
}

static int imu_comm_disconnect(struct imu * imu){
    int retval = ERROR_OK;
    retval = fclose(imu->device);
    if(retval != ERROR_OK){
	fprintf(stderr,"Failed to close connection to device.\n");
	return ERROR_CLOSE;
    }
    imu->device = NULL;
    return ERROR_OK;
}

/** 
 * Initialize IMU struct and send default value to IMU, this
 * ensures starting from a know state.
 * 
 * @return error code
 */
struct imu * imu_comm_init(const char * device){
    struct imu * imu;
    int i,retval;
    imu = (struct imu *)malloc(sizeof(struct imu));
    if(imu == NULL){
	fprintf(stderr,"Failed to allocate mem. \n");
	return imu;
    }
    // Set default values
    imu->frames_sampled = 0;
    imu->unread_data = 0;
    imu->frame_next = 0;
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

int imu_comm_deinit(struct imu * imu){
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

static double counts2volts(struct imu * imu, double * acc){
    // Convert from count to m/s^2
    // m/s^2 = counts*vref/counts_full_scale
    *acc = (*acc)*IMU_ADC_COUNTS_2_VOLTS;
    return ERROR_OK;
}

static int gyro_scale_adjust(struct imu * imu, double * gyro_reading){
    //TODO Implement scale calibration,
    // Note: Should be /300, but /450 seems to work better.
    // Will be sensor specific
    // Should get a true calibration instead of this.
    *gyro_reading *= 0.92955;
    return ERROR_OK;
}

static int acc_scale_adjust(struct imu * imu, double * acc_reading){
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
int frame_circ_index(struct imu * imu){
    return (imu->frame_next + IMU_FRAME_SAMPLE_AVG_COUNT - 1) % IMU_FRAME_SAMPLE_AVG_COUNT;
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
static int imu_comm_avg(struct imu * imu){
    int tmp,i,j;
    for(i=0;i<IMU_SENSOR_COUNT;++i){// loop sensors
	tmp = 0;
	for(j=0;j<IMU_FRAME_SAMPLE_AVG_COUNT;++j)// loop sensor data
	    tmp += (int)imu->frame_buffer[j].raw[i];
	imu->avg.xyzrpy[i] = ((double)tmp)/IMU_FRAME_SAMPLE_AVG_COUNT;
    }
    imu->avg.timestamp.tv_sec = imu->frame_buffer[frame_circ_index(imu)].timestamp.tv_sec;
    imu->avg.timestamp.tv_usec = imu->frame_buffer[frame_circ_index(imu)].timestamp.tv_usec;

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
 */static int imu_comm_get_sync(struct imu * imu, uquad_bool_t * in_sync){
    int retval;
    *in_sync = false;
    unsigned char tmp = '@';// Anything diff from IMU_FRAME_INIT_CHAR
    retval = fread(&tmp,IMU_INIT_END_SIZE,1,imu->device);
    if(retval < 0){	
	err_check(ERROR_IO,"Read error: failed to get sync char...");
    }else{
	if(retval > 0){
	    if(tmp == IMU_FRAME_INIT_CHAR){
		// No error printing, leave that for upper level
		*in_sync = true;
	    }
	}
    }
    // If we read 0 then there is no data available, so no sync and no error.
    // Set retval to ERROR_OK, otherwise it'll be # of bytes read
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
int imu_comm_read_frame(struct imu * imu){
    int retval = ERROR_OK,watchdog,read,i;
    unsigned char tmp = '@';// Anything diff from IMU_FRAME_INIT_CHAR
    struct imu_frame * new_frame;
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
	}
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
static int imu_comm_gyro_read(struct imu * imu, struct imu_frame * frame, double * gyro_reading){
    int retval = ERROR_OK, i;
    for(i = 0; i<IMU_GYROS; ++i){
	gyro_reading[i] = ((double) *(frame->raw + IMU_ACCS + i)) - imu->null_estimates.xyzrpy[IMU_ACCS + i];
	gyro_reading[i] = grad2rad(gyro_reading[i]);
	retval = gyro_scale_adjust(imu,gyro_reading+i);
	err_propagate(retval);
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
static int imu_comm_acc_read(struct imu * imu, struct imu_frame * frame, double * acc_reading){
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
 * Calculates value of the sensor reading from the RAW data, using current imu calibration.
 * 
 * @param imu Current imu status
 * @param xyzrpy Answer is returned here
 * 
 * @return error code
 */
int imu_comm_get_data_latest(struct imu * imu, imu_data_t * data){
    int retval = ERROR_OK;

    struct imu_frame * frame = imu->frame_buffer + frame_circ_index(imu);
    data->timestamp = frame->timestamp;

    // Get ACC readings
    retval = imu_comm_acc_read(imu, frame, data->xyzrpy);
    err_propagate(retval);

    // Get gyro reading
    retval = imu_comm_gyro_read(imu, frame, data->xyzrpy + IMU_ACCS);
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
int imu_comm_get_data_raw_latest_unread(struct imu * imu, imu_data_t * data){
    int retval = ERROR_OK;
    if(imu->unread_data <= 0){
	err_check(ERROR_FAIL,"No unread data available.");
    }

    struct imu_frame * frame = imu->frame_buffer + frame_circ_index(imu);
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
int imu_comm_get_data_latest_unread(struct imu * imu, imu_data_t * data){
    int retval = ERROR_OK;
    if(imu->unread_data <= 0){
	err_check(ERROR_FAIL,"No unread data available.");
    }

    struct imu_frame * frame = imu->frame_buffer + frame_circ_index(imu);
    data->timestamp = frame->timestamp;

    // Get ACC readings
    retval = imu_comm_acc_read(imu, frame, data->xyzrpy);
    err_propagate(retval);

    // Get gyro reading
    retval = imu_comm_gyro_read(imu, frame, data->xyzrpy + IMU_ACCS);
    err_propagate(retval);

    imu->unread_data -= 1;
    return retval;
}

uquad_bool_t imu_comm_avg_ready(struct imu * imu){
    return imu->avg_ready;
}

int imu_comm_get_avg(struct imu * imu, imu_data_t * data){
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
int imu_comm_get_fds(struct imu * imu,int * fds){
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

int imu_comm_poll(struct imu * imu, uquad_bool_t * ready){
    int retval;
    retval = imu_comm_check_io_locks(imu->device, ready, NULL);
    err_propagate(retval);
    if(*ready){
	// Ready to read. Check if in sync
	retval = imu_comm_get_sync(imu,ready);
	err_propagate(retval);
    }// else: No data available
    return retval;
}

int imu_comm_calibrate(struct imu * imu){
    //TODO implement!
    err_check(ERROR_FAIL,"Not implemented!");
}

static int previous_frame_count = -1;
int imu_comm_print_frame(struct imu_frame * frame, FILE * stream){
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
