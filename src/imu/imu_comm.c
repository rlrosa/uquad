#include "imu_comm.h"
#include <uquad_aux_io.h>
#include <uquad_aux_time.h>
#include <math.h> // for pow()
#include <fcntl.h> // for open()
/// Aux mem
static uquad_mat_t *m3x3;
static uquad_mat_t *m3x1_0;
static uquad_mat_t *m3x1_1;

imu_status_t imu_comm_get_status(imu_t *imu){
    return imu->status;
}

/** 
 * Allocates memory required for matrices
 * used by struct
 *
 * @param imu_data
 *
 * @return error code
 */
int imu_data_alloc(imu_data_t *imu_data)
{
    imu_data->acc = uquad_mat_alloc(3,1);
    imu_data->gyro = uquad_mat_alloc(3,1);
    imu_data->magn = uquad_mat_alloc(3,1);
    if(imu_data->acc == NULL ||
       imu_data->acc == NULL ||
       imu_data->acc == NULL)
    {
	err_propagate(ERROR_MALLOC);
    }
    return ERROR_OK;
}

/**
 * Frees memory required for matrices
 * used by struct
 *
 * @param imu_data
 */
void imu_data_free(imu_data_t *imu_data)
{
    uquad_mat_free(imu_data->acc);
    uquad_mat_free(imu_data->gyro);
    uquad_mat_free(imu_data->magn);
}

/**
 * Sends command to the IMU over serial line.
 *
 *@param imu 
 *@param cmd to send to the imu
 *
 *@return error code
 */
static int imu_comm_send_cmd(imu_t *imu, unsigned char cmd){
#if !IMU_COMM_FAKE     // If reading from a log file, we have nothing to send to it
    static unsigned char buff[2] = "X\n";
    buff[0] = cmd;
    int retval;
    // issue command
    retval = write(imu->device,&cmd,2);
    //	retval = fprintf(imu->device,"%c\n",cmd);
    if(retval<0)
    {
	err_log_stderr("Write error: Failed to send cmd to IMU");
	err_propagate(ERROR_WRITE);
    }
#endif //!IMU_COMM_FAKE
    return ERROR_OK;
}

/** 
 * Sends serial command to the IMU to make it stop.
 * 
 * @param imu 
 * 
 * @return 
 */
int imu_comm_stop(imu_t *imu){
    int retval;
    if(imu->status == IMU_COMM_STATE_STOPPED){
	err_log("IMU already stopped.");
	return ERROR_OK;
    }
    // Stop IMU
    retval = imu_comm_send_cmd(imu,IMU_COMMAND_STOP);
    err_propagate(retval);
    imu->status = IMU_COMM_STATE_STOPPED;
    return ERROR_OK;
}

/** 
 * Sends serial command to the IMU to make it run.
 * 
 * @param imu 
 * 
 * @return 
 */
int imu_comm_resume(imu_t *imu){
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

/** 
 * Marks estimated calibration as NOT set.
 * 
 * @param imu 
 */
static void imu_comm_calibration_clear(imu_t *imu){
    imu->calib.calib_estim_ready = false;
    imu->calib.calibration_counter = -1;
    imu->calib.timestamp_estim.tv_sec = 0;
    imu->calib.timestamp_estim.tv_usec = 0;
}

/** 
 *Configures IMU to use default params and start sampling.
 *Depends on set_work_mode() in Output.pde (firmware)
 *
 *@param imu 
 *
 *@return error code
 */
static int imu_comm_run_default(imu_t *imu){
    int retval = ERROR_OK;
#if !IMU_COMM_FAKE
    // Set run
    retval = imu_comm_send_cmd(imu,IMU_COMMAND_DEF);
    err_propagate(retval);
#endif
    imu->status = IMU_COMM_STATE_RUNNING;
    return retval;
}

/** 
 *Configure IMU.
 *
 *@param imu 
 *
 *@return error code
 */
static int imu_comm_configure(imu_t *imu){
    int retval = ERROR_OK;
    if(imu == NULL)
    {
	err_check(ERROR_NULL_POINTER,"Invalid arg.");
    }
    //TODO configure stuff?
    return retval;
}

/** 
 * Open a connection to the IMU.
 * Can be either a serial port (read/write) using binary
 * transmission, or a log file in ascii (read only).
 * 
 * @param imu 
 * @param device Name of the device to open.
 * 
 * @return error code.
 */
static int imu_comm_connect(imu_t *imu, const char *device){
#if IMU_COMM_FAKE
    // we don't want to write to the log file, just read.
    imu->device = fopen(device,"rb+");
    if(imu->device == NULL){
	fprintf(stderr,"Device %s not found.\n",device);
	return ERROR_OPEN;
    }
#else
    imu->device = open(device,O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(imu->device < 0)
    {
	err_log_stderr("Failed to open dev!");
	err_propagate(ERROR_IO);
    }
#endif
    return ERROR_OK;
}

/** 
 * Closes any connections opened by the imu, if any.
 * 
 * @param imu 
 * 
 * @return error code.
 */
static int imu_comm_disconnect(imu_t *imu){
    int retval = ERROR_OK;
#if IMU_COMM_FAKE
    if(imu->device != NULL)
    {
	retval = fclose(imu->device);
	if(retval == 0)
	    imu->device = NULL;
    }
#else
    if(imu->device > 0)
	retval = close(imu->device);
#endif
    if(retval < 0)
    {
	err_log_stderr("Failed to close device!");
	err_propagate(ERROR_CLOSE);
    }
    return ERROR_OK;
}

/** 
 * Allocates memory for three linear calibration structures.
 * Each structure uses 2 matrices.
 * 
 * @param imu 
 * 
 * @return 
 */
int imu_comm_alloc_calib_lin(imu_t *imu)
{
    int i;
    for (i = 0; i < 3; ++i)
    {
	// TK_inv
	imu->calib.m_lin[i].TK_inv = uquad_mat_alloc(3,3);
	if (imu->calib.m_lin[i].TK_inv == NULL)
	{
	    err_check(ERROR_MALLOC, "Failed to allocate K");
	}
	// b
	imu->calib.m_lin[i].b = uquad_mat_alloc(3,1);
	if (imu->calib.m_lin[i].b == NULL)
	{
	    err_check(ERROR_MALLOC, "Failed to allocate K");
	}
    }
    return ERROR_OK;
}

/** 
 * Frees memory for three linear calibration structures.
 * Each structure uses 2 matrices.
 * 
 * @param imu 
 * 
 * @return 
 */
int imu_comm_free_calib_lin(imu_t *imu)
{
    int i;
    for (i = 0; i < 3; ++i)
    {
	// TK_inv
	uquad_mat_free(imu->calib.m_lin[i].TK_inv);
	// b
	uquad_mat_free(imu->calib.m_lin[i].b);
    }
    return ERROR_OK;
}

/** 
 * Will load calibration for linear model from text file.
 * Will ignore spaces, end of lines, tabs, etc.
 * Expect to find:
 *  - a matrix M = T*inv(K)
 *  - a matrix b
 * See imu_calib_lin_t for details.
 * 
 * @param imu 
 * @param path 
 * 
 * @return 
 */
int imu_comm_load_calib(imu_t *imu, const char *path)
{
    int i,retval = ERROR_OK;
    FILE *calib_file = fopen(path,"r+");
    if(calib_file == NULL)
    {
	err_check(ERROR_OPEN,"Failed to open calib file!");
    }

    for (i = 0; i < 3; ++i)
    {
	// TK_inv
	retval = uquad_mat_load(imu->calib.m_lin[i].TK_inv, calib_file);
	if(retval != ERROR_OK)
	{
	    err_log("Failed to load TK_inv matrix!");
	    break;
	}		    

	// b
	retval = uquad_mat_load(imu->calib.m_lin[i].b, calib_file);
	if(retval != ERROR_OK)
	{
	    err_log("Failed to load b matrix!");
	    break;
	}
    }
    fclose(calib_file);
    imu->calib.calib_file_ready = true;
    return retval;
}

/** 
 * Will alocate memory to store calibration data, and load
 * calibration data.
 * 
 * @param imu 
 * 
 * @return 
 */
int imu_comm_init_calibration(imu_t *imu)
{
    int retval = ERROR_OK;
    retval = imu_comm_alloc_calib_lin(imu);
    err_propagate(retval);
    /// load calibration parameters from file
    retval = imu_comm_load_calib(imu, IMU_DEFAULT_CALIB_PATH);
    err_propagate(retval);
    // Use default p0 until we get a calibration
    imu->calib.null_est.pres = IMU_P0_DEFAULT;
    return retval;
}

/** 
 *Initialize IMU struct and send default value to IMU, this
 *ensures starting from a know state.
 *
 *@return error code
 */
imu_t *imu_comm_init(const char *device){
    imu_t *imu;
    int retval = ERROR_OK;
    imu = (imu_t *)malloc(sizeof(imu_t));
    mem_alloc_check(imu);
    memset(imu,0,sizeof(imu_t));
    imu->status = IMU_COMM_STATE_UNKNOWN;

    // now connect to the imu
    retval = imu_comm_connect(imu,device);
    cleanup_if(retval);

    // Get aux memory
    retval = imu_data_alloc(&imu->tmp_avg);
    cleanup_if(retval);

    // Send default values to IMU, then get it running, just in case it wasn't
    retval = imu_comm_configure(imu);
    cleanup_if(retval);

    retval = imu_comm_run_default(imu);
    cleanup_if(retval);

    // Mark IMU as not calibrated
    imu_comm_calibration_clear(imu);
    // Load/estimate calibration
    retval = imu_comm_init_calibration(imu);
    cleanup_if(retval);

    m3x3 = uquad_mat_alloc(3,3);
    m3x1_0 = uquad_mat_alloc(3,1);
    m3x1_1 = uquad_mat_alloc(3,1);
    if(m3x3 == NULL || m3x1_0 == NULL || m3x1_1 == NULL)
	goto cleanup;

    // Wait 300ms + a bit more for IMU to reset
    sleep_ms(350);
    return imu;

    cleanup:
    imu_comm_deinit(imu);
    return NULL;
}

/** 
 * Free any memory allocated for imu (if any), and close any
 * open connections (if any).
 * Safe to call under any circunstance.
 * 
 * @param imu 
 * 
 * @return 
 */
int imu_comm_deinit(imu_t *imu){
    int retval = ERROR_OK;
    if(imu == NULL)
    {
	err_log("WARN: Nothing to free.");
	return ERROR_OK;
    }
    retval = imu_comm_disconnect(imu);
    // ignore answer and keep dying, leftovers are not reliable
    imu_data_free(&imu->tmp_avg);
    //TODO chec if more to free
    imu_comm_free_calib_lin(imu);
    uquad_mat_free(m3x3);
    uquad_mat_free(m3x1_0);
    uquad_mat_free(m3x1_1);
    free(imu);
    return retval;
}

/** 
 *Attemps to read end of frame character.
 *Reads until timed out, or end character is found.
 *NOTE: Assumes device can be read without blocking.
 *
 *@param imu 
 *
 *@return error code
 */
static int imu_comm_get_sync_end(imu_t *imu){
    // Now read out the end char
    int watchdog = 0, retval;
    unsigned char tmp = 'X';// Anything diff from IMU_FRAME_END_CHAR
    while(watchdog < READ_RETRIES)
    {
#if IMU_COMM_FAKE
	retval = fread(&tmp,IMU_INIT_END_SIZE,1,imu->device);
#else
	retval = read(imu->device, &tmp,IMU_INIT_END_SIZE);
#endif
	if(retval < 0)
	{
	    err_log_stderr("Read error: failed to get sync char...");
	    err_propagate(ERROR_CLOSE);
	}
	if(retval > 0)
	{
	    if(tmp == IMU_FRAME_END_CHAR)
	    {
#if IMU_COMM_FAKE
		// read out end of line
		retval = fread(&tmp,IMU_INIT_END_SIZE,1,imu->device);
#endif
		return ERROR_OK;
	    }
#if !IMU_COMM_FAKE // in ascii we have to read the separator char
	    else
	    {
		err_check(ERROR_READ_SYNC,"Unexpected end of frame char: Discarding frame...");
	    }
#endif
	}
	else
	{
	    ++watchdog;
	}
    }// while(watchdog < READ_RETRIES)
    err_check(ERROR_READ_TIMEOUT,"Read error: Timed out waiting for end char...");
}

static uint8_t previous_sync_char = IMU_FRAME_INIT_CHAR;
/** 
 *Reads 1 byte, expecting it to be one of the frame init chars.
 *Will log warning if same sync char is read twice, because 
 *this implies a frame was skipped.
 *NOTE: Assumes device can be read without blocking.
 *
 *@param imu 
 *
 *@return error code, sync achieved iif ERROR_OK
 */
static int imu_comm_get_sync_init(imu_t *imu){
    int retval;//,i;
    unsigned char tmp = 'X';// Anything diff from IMU_FRAME_INIT_CHAR
#if IMU_COMM_FAKE
    retval = fread(&tmp,IMU_INIT_END_SIZE,1,imu->device);
#else
    retval = read(imu->device,&tmp,IMU_INIT_END_SIZE);
#endif
    if(retval < 0)
    {
	err_log_stderr("Read error: failed to get sync char...");
	err_propagate(ERROR_CLOSE);
    }
    else
    {
	if(retval > 0)
	{
	    // Match either of the init chars
	    if((tmp|IMU_FRAME_INIT_DIFF) == IMU_FRAME_INIT_CHAR_ALT)
	    {
		// Check if skipped frame
		if(!IMU_FRAME_ALTERNATES_INIT ||
		   ((tmp ^ previous_sync_char) == IMU_FRAME_INIT_DIFF))
		{
		    previous_sync_char = tmp;
		}
		else
		{
		    err_log("Skipped frame!");
		}
		return ERROR_OK;
	    }
	    else
	    {
		// wrong init char
		err_check(ERROR_READ_SKIP,"Wrong sync init char!");
	    }
	}
	else
	{
	    // If here, then retval == 0
	    err_check(ERROR_READ_SYNC,"sync failed, out of data!");
	}
    }
    err_check(ERROR_READ_SYNC,"Timed out!");
}

/** 
 * Set IMU to calibration mode.
 * Will gather data to estimate null (offsets).
 * NOTE: Assumes sensors are not being excited, ie, imu is staying completely still.
 *
 *@param imu 
 *
 *@return 
 */
int imu_comm_calibration_start(imu_t *imu){
    if(imu->status != IMU_COMM_STATE_RUNNING){
	err_check(ERROR_IMU_STATUS,"IMU must be running to calibrate!");
    }
    imu->status = IMU_COMM_STATE_CALIBRATING;
    /// clear calibration data/
    memset(&imu->calib.null_est,0,sizeof(imu_raw_null_t));
    imu->calib.calibration_counter = IMU_CALIB_SIZE;
    return ERROR_OK;
}

/** 
 *Abort current IMU calibration process. All progress will be lost.
 *If a previous calibration existed, it will be preserved.
 *
 *@param imu 
 *
 *@return error code
 */
int imu_comm_calibration_abort(imu_t *imu){
    if(imu->status != IMU_COMM_STATE_CALIBRATING)
    {
	err_check(ERROR_IMU_STATUS,"Cannot abort calibration, IMU is not calibrating!");
    }
    // Restore IMU status:
    // if we were calibrating then we were running before
    imu->status = IMU_COMM_STATE_RUNNING;

    return ERROR_OK;
}

static struct timeval calibration_start_time;
/** 
 *Integrate calibration data into IMU.
 *Replaces previous calibration, if any existed.
 *
 *@param imu 
 *@param timestamp on the last sample used for the calibration
 *
 *@return error code.
 */
int imu_comm_calibration_finish(imu_t *imu){
    int retval = ERROR_OK, i;
    if(imu->status != IMU_COMM_STATE_CALIBRATING){
	err_check(ERROR_IMU_STATUS,"Cannot finish calibration, IMU is not calibrating!");
    }
    if(imu->calib.calibration_counter != 0){
	err_check(ERROR_IMU_STATUS,"Not enough samples gathered!");
    }

    struct timeval tv_end, tv_diff;
    gettimeofday(&tv_end,NULL);
    retval = uquad_timeval_substract(&tv_diff,tv_end,calibration_start_time);
    if(retval < 0){
	err_check(ERROR_FAIL,"Calibration duration should not be negative!");
    }
    //TODO check if calib took too long, and refuse it if so
    //requires defintion of "too long"

    for(i = 0; i < 3; ++i)
    {
	imu->calib.null_est.acc[i]  /= IMU_CALIB_SIZE;
	imu->calib.null_est.gyro[i] /= IMU_CALIB_SIZE;
	imu->calib.null_est.magn[i] /= IMU_CALIB_SIZE;
    }
    imu->calib.null_est.temp /= IMU_CALIB_SIZE;
    imu->calib.null_est.pres /= IMU_CALIB_SIZE;

    imu->calib.calibration_counter = -1;
    imu->calib.timestamp_estim = tv_end;
    imu->calib.calib_estim_ready = true;
    imu->status = IMU_COMM_STATE_RUNNING;

    //TODO implement
    //retval = imu_comm_calbration_fix(imu);

    return ERROR_OK;
}

/** 
 *Add frame info to build up calibration.
 *Will divide by frame count after done gathering, to avoid loosing info.
 *
 *@param imu 
 *
 *@return error code
 */
int imu_comm_calibration_continue(imu_t *imu){
    int retval, i;
    static imu_raw_t new_frame;
    if(imu->status != IMU_COMM_STATE_CALIBRATING)
    {
	err_check(ERROR_IMU_STATUS,"Cannot add frames, IMU is not calibrating!");
    }
    if(imu->calib.calibration_counter <= 0)
    {
	err_check(ERROR_FAIL,"Invalid value for calib.calibration_counter");
    }

    if(imu->calib.calibration_counter == IMU_CALIB_SIZE)
    {
	gettimeofday(&calibration_start_time,NULL);
    }

    /// add new frame to accumulated data
    retval = imu_comm_get_raw_latest_unread(imu, &new_frame);
    err_propagate(retval);

    for( i = 0; i < 3; ++i)
    {
	imu->calib.null_est.acc[i] += (int32_t) new_frame.acc[i];
	imu->calib.null_est.gyro[i] += (int32_t) new_frame.gyro[i];
	imu->calib.null_est.magn[i] += (int32_t) new_frame.magn[i];
    }
    imu->calib.null_est.temp += (uint32_t) new_frame.temp;
    imu->calib.null_est.pres += (uint64_t) new_frame.pres;

    if(--imu->calib.calibration_counter == 0){
	retval = imu_comm_calibration_finish(imu);
	err_propagate(retval);
    }

    return ERROR_OK;
}

/** 
 * Copies the data in src to dest.
 * Must previously allocate mem for dest.
 * 
 * @param src 
 * @param dest 
 * 
 * @return 
 */
int imu_comm_copy_frame(imu_raw_t *src, imu_raw_t *dest)
{
    int i;
    if(src == NULL || dest == NULL)
	err_check(ERROR_NULL_POINTER,"NULL pointer arg is not valid.");
    dest->T_us = src->T_us;
    for (i=0; i<3; ++i)
	dest->acc[i] = src->acc[i];
    for (i=0; i<3; ++i)
	dest->gyro[i] = src->gyro[i];
    for (i=0; i<3; ++i)
	dest->magn[i] = src->magn[i];
    dest->temp = src->temp;
    dest->pres = src->pres;
    dest->timestamp = src->timestamp;
    return ERROR_OK;
}

/** 
 * Add frame to buff
 * Updates unread data and frame_count.
 *
 *@param imu 
 *@param new_frame frame to add
 *
 *@return error code
 */
int imu_comm_add_frame(imu_t *imu, imu_raw_t *new_frame){
    int retval;

    retval = imu_comm_copy_frame(new_frame, imu->frame_buff + imu->frame_buff_next);
    imu->frame_buff_latest = imu->frame_buff_next;
    imu->frame_buff_next = (imu->frame_buff_next + 1)%IMU_FRAME_BUFF_SIZE;
    ++imu->unread_data;
    imu->frame_count = uquad_min(imu->frame_count + 1,IMU_AVG_COUNT);

    err_propagate(retval);

    return ERROR_OK;
}

#if !IMU_COMM_FAKE
/** 
 *Takes an array of bytes and parses them according to the format
 *of the frames sent by the IMU, generating an imu_raw_t structure.
 *
 *@param new_frame New frame is returned here. Must have been previously allocated.
 *@param data raw data.
 *
 *@return error code
 */
void imu_comm_parse_frame_binary(imu_raw_t *new_frame, uint8_t *data)
{
    int i;
    int16_t *buff_tmp_16;

    // Get sampling time
    memcpy(&new_frame->T_us, data, IMU_BYTES_T_US);
    data += IMU_BYTES_T_US;

    // Generate timestamp
    gettimeofday(& new_frame->timestamp,NULL);

    // Get sensor data from buff (acc,gyro,magn,temp)
    buff_tmp_16 = (int16_t *) data;
    i = 0;
    // acc
    for(;i<3;++i)
	new_frame->acc[i%3] = buff_tmp_16[i];
    // gyro
    for(;i<6;++i)
	new_frame->gyro[i%3] = buff_tmp_16[i];
    // magn
    for(;i<9;++i)
	new_frame->magn[i%3] = buff_tmp_16[i];
    // temp
    new_frame->temp = (uint16_t)buff_tmp_16[i++];
    // Change LSB/MSB for 32 bit sensors (pressure)
    // pres
    new_frame->pres = *((uint32_t*)(buff_tmp_16+i));

    // Everything went ok, pat pat :)
}

/** 
 * Will read data from imu->device.
 * Reading is non blocking.
 * Assumes sync char was previously read.
 * 
 * @param imu 
 * @param new_frame answer is returned here. Valid iff done. Mem for
 * new_frame must have been previously allocated.
 * @param done true iif a frame was completed.
 * 
 * @return error code
 */
int imu_comm_read_frame_binary(imu_t *imu, imu_raw_t *new_frame, uquad_bool_t *done)
{
    static uint8_t buff_tmp_8[IMU_DEFAULT_FRAME_SIZE_BYTES-2]; // no space for init/end
    static int buff_index = 0;
    int retval = ERROR_OK;
    uquad_bool_t read_ok = false;
    *done = false;
    retval = check_io_locks(imu->device, NULL, &read_ok, NULL);
    if(read_ok)
    {
	retval = read(imu->device,buff_tmp_8 + buff_index,1);
	if(retval < 0)
	{
		    err_log_stderr("Read error: no data! Restarting...");
		    buff_index = 0;
		    err_propagate(ERROR_IO);
	}

	if(++buff_index == IMU_DEFAULT_FRAME_SIZE_BYTES-2)
	{
	    // frame completed, now parse.
	    buff_index = 0;
	    imu_comm_parse_frame_binary(new_frame, buff_tmp_8);
	    *done = true;
	}
    }
    return ERROR_OK;
}
#else
/** 
 *Assumes sync char was read, reads the rest of the data.
 *Does not read end of frame.
 *NOTE: Will block if data is not available.
 *
 *@param imu 
 *@param new_frame New frame is returned here.
 *
 *@return error code
 */
int imu_comm_read_frame_ascii(imu_t *imu, imu_raw_t *new_frame)
{
    int retval = ERROR_OK,i, itmp;
    unsigned int uitmp;

    // Get sampling time
    retval = fscanf(imu->device,"%ud",&uitmp);
    new_frame->T_us = (uint16_t)uitmp;
    if(retval < 0)
    {
	err_check(ERROR_IO,"Read error: Failed to read T_us...");
    }

    // Generate timestamp
    gettimeofday(& new_frame->timestamp,NULL);

    // Read sensors RAW data
    // acc
    for(i=0; i < 3; ++i)
    {
	retval = fscanf(imu->device,"%d",&itmp);
	if(retval < 0)
	{
	    err_check(ERROR_IO,"Read error: Failed to read T_us...");
	}
	new_frame->acc[i] = (uint16_t) itmp;
    }
    // gyro
    for(i=0; i < 3; ++i)
    {
	retval = fscanf(imu->device,"%d",&itmp);
	if(retval < 0)
	{
	    err_check(ERROR_IO,"Read error: Failed to read T_us...");
	}
	new_frame->gyro[i] = (uint16_t) itmp;
    }
    // magn
    for(i=0; i < 3; ++i)
    {
	retval = fscanf(imu->device,"%d",&itmp);
	if(retval < 0)
	{
	    err_check(ERROR_IO,"Read error: Failed to read T_us...");
	}
	new_frame->magn[i] = (uint16_t) itmp;
    }
    // temp
    retval = fscanf(imu->device,"%d",&itmp);
    if(retval < 0)
    {
	err_check(ERROR_IO,"Read error: Failed to read T_us...");
    }
    new_frame->temp = (uint16_t) itmp;
    // pres
    retval = fscanf(imu->device,"%ud",&uitmp);
    if(retval < 0)
    {
	err_check(ERROR_IO,"Read error: Failed to read T_us...");
    }
    new_frame->pres = (uint32_t) uitmp;
    // Everything went ok, pat pat :)
    return ERROR_OK;
}
#endif

/// IMU read state machine support
typedef enum read_status{
    IDLE = 0,
    INIT_SYNC_DONE,
    READ_FRAME_DONE,
    END_SYNC_DONE,
    STATE_COUNT
} read_status_t;
#if TIMING_IMU
static struct timeval tv_init, tv_end, tv_diff;
#endif
/** 
 *Attempts to sync with IMU, and read data.
 *Reading is performed 1 byte at a time, so if select() has
 *been checked previously, reading will not block.
 *
 *@param imu 
 *@param success This will be true when last byte read is end of frame char.
 *
 *@return error code
 */
int imu_comm_read(imu_t *imu, uquad_bool_t *ready){
    int retval;
    static imu_raw_t new_frame;
    static read_status_t status = IDLE;
    uquad_bool_t ok;
    *ready = false;

    switch(status)
    {
    case IDLE:
	/// -- -- -- -- -- -- -- -- -- -- -- --
	/// 1 - sync init
	/// -- -- -- -- -- -- -- -- -- -- -- --
#if TIMING_IMU
	gettimeofday(&tv_init,NULL);//TODO testing
#endif
	// sync by getting init frame char
	retval = imu_comm_get_sync_init(imu);
	err_propagate(retval);
	status++;

#if TIMING_IMU
	gettimeofday(&tv_end,NULL);//TODO testing
	retval = uquad_timeval_substract(&tv_diff,tv_end,tv_init);//TODO testing
	if(retval < 0)//TODO testing
	{//TODO testing
	    err_check(ERROR_TIMING,"Absurd timing!");//TODO testing
	}//TODO testing
	printf("%ld\t", tv_diff.tv_usec);//TODO testing
#endif
	break;
    case INIT_SYNC_DONE:
	/// -- -- -- -- -- -- -- -- -- -- -- --
	/// 2 - frame
	/// -- -- -- -- -- -- -- -- -- -- -- --
	// sync worked, now get data
#if IMU_COMM_FAKE
	retval = check_io_locks(-1, imu->device, &ok, NULL);
	if(ok)
	{
	    retval = imu_comm_read_frame_ascii(imu, &new_frame);
	    err_propagate(retval);
	    status++;
	}
#else
	retval = imu_comm_read_frame_binary(imu, &new_frame, &ok);
	err_propagate(retval);
	if(ok)
	{
	    // frame completed!
	    status++;
#if TIMING_IMU
	    gettimeofday(&tv_end,NULL);//TODO testing
	    retval = uquad_timeval_substract(&tv_diff,tv_end,tv_init);//TODO testing
	    if(retval < 0)//TODO testing
	    {//TODO testing
		err_check(ERROR_TIMING,"Absurd timing!");//TODO testing
	    }//TODO testing
	    printf("%ld\t", tv_diff.tv_usec);//TODO testing
	    gettimeofday(&tv_init,NULL);//TODO testing
#endif
	}
#endif
	break;
    case READ_FRAME_DONE:
	/// -- -- -- -- -- -- -- -- -- -- -- --
	/// 3 - sync end
	/// -- -- -- -- -- -- -- -- -- -- -- --
#if TIMING_IMU
	gettimeofday(&tv_init,NULL);//TODO testing
#endif

	// verify sync by getting end of frame char
	retval = imu_comm_get_sync_end(imu);
	if(retval == ERROR_READ_SYNC)
	{
	    // sync failed, restart
	    status = IDLE;
	}
	err_propagate(retval);
	status++;

#if TIMING_IMU
	gettimeofday(&tv_end,NULL);//TODO testing
	retval = uquad_timeval_substract(&tv_diff,tv_end,tv_init);//TODO testing
	if(retval < 0)//TODO testing
	{//TODO testing
	    err_check(ERROR_TIMING,"Absurd timing!");//TODO testing
	}//TODO testing
	printf("%ld\t", tv_diff.tv_usec);//TODO testing
#endif
	//break; // Do not break here, keep going!
    case END_SYNC_DONE:
	/// -- -- -- -- -- -- -- -- -- -- -- --
	/// 4 - add
	/// -- -- -- -- -- -- -- -- -- -- -- --
#if TIMING_IMU
	gettimeofday(&tv_init,NULL);//TODO testing
#endif

	// add the frame to the buff
	retval = imu_comm_add_frame(imu, &new_frame);
	err_propagate(retval);

	if(imu_comm_get_status(imu) == IMU_COMM_STATE_CALIBRATING)
	{
	    // Add to calibration
	    retval = imu_comm_calibration_continue(imu);
	    err_propagate(retval);
	}

	*ready = true;
	status = IDLE; //restart

#if TIMING_IMU
	gettimeofday(&tv_end,NULL);//TODO testing
	retval = uquad_timeval_substract(&tv_diff,tv_end,tv_init);//TODO testing
	if(retval < 0)//TODO testing
	{//TODO testing
	    err_check(ERROR_TIMING,"Absurd timing!");//TODO testing
	}//TODO testing
	printf("%ld\t", tv_diff.tv_usec);//TODO testing
#endif
	break;
    default:
	status = IDLE;
	err_check(ERROR_FAIL,"Invalid status! Restarting...");
	break;
    }

#if DEBUG
    if(*ready)
    {
	/// -- -- -- -- -- -- -- -- -- -- -- --
	/// 5 - print
	/// -- -- -- -- -- -- -- -- -- -- -- --
#if TIMING_IMU
	gettimeofday(&tv_init,NULL);//TODO testing
	gettimeofday(&tv_end,NULL);//TODO testing
	retval = uquad_timeval_substract(&tv_diff,tv_end,tv_init);//TODO testing
	if(retval < 0)//TODO testing
	{//TODO testing
	    err_check(ERROR_TIMING,"Absurd timing!");//TODO testing
	}//TODO testing
	printf("%ld\t", tv_diff.tv_usec);//TODO testing
	printf("\n");//TODO testing
#endif
    }
#endif //DEBUG

    return ERROR_OK;
}

int convert_2_euler(imu_data_t *data)
{
    int retval;
    double psi;   // rad
    double phi;   // rad
    double theta; // rad
    if(uquad_abs(data->acc->m_full[0]) < IMU_TH_DEADLOCK_ACC)
    {
	phi = -asin(data->acc->m_full[0]/GRAVITY);
	psi = atan2(data->acc->m_full[1],data->acc->m_full[2]);
    }else if(data->acc->m_full[0]>0){
	phi=-PI/2;
	psi=0;
    }else{
	phi=PI/2;
	psi=0;
    }


    m3x3->m[0][0] = cos(phi)/(uquad_square(cos(phi)) + uquad_square(sin(phi)));
    m3x3->m[0][1] = (sin(phi)*sin(psi))/((uquad_square(cos(phi)) + uquad_square(sin(phi)))*(uquad_square(cos(psi)) + uquad_square(sin(psi))));
    m3x3->m[0][2] = (cos(psi)*sin(phi))/((uquad_square(cos(phi)) + uquad_square(sin(phi)))*(uquad_square(cos(psi)) + uquad_square(sin(psi))));
    m3x3->m[1][0] = 0;
    m3x3->m[1][1] = cos(psi)/(uquad_square(cos(psi)) + uquad_square(sin(psi)));
    m3x3->m[1][2] = -sin(psi)/(uquad_square(cos(psi)) + uquad_square(sin(psi)));
    m3x3->m[2][0] = -sin(phi)/(uquad_square(cos(phi)) + uquad_square(sin(phi)));
    m3x3->m[2][1] = (cos(phi)*sin(psi))/((uquad_square(cos(phi)) + uquad_square(sin(phi)))*(uquad_square(cos(psi)) + uquad_square(sin(psi))));
    m3x3->m[2][2] = (cos(phi)*cos(psi))/((uquad_square(cos(phi)) + uquad_square(sin(phi)))*(uquad_square(cos(psi)) + uquad_square(sin(psi))));

    retval = uquad_mat_prod(m3x1_0,m3x3,data->magn);
    err_propagate(retval);

    theta=atan2(m3x1_0->m_full[0],m3x1_0->m_full[1]) + IMU_TH_DEADLOCK_ANG;//9.78;

    data->magn->m_full[0]=psi;
    data->magn->m_full[1]=phi;
    data->magn->m_full[2]=theta;
    return ERROR_OK;
}

/** 
 * Uses linear model provided by calib to convert raw into valid
 * real world data.
 * Assumes raw is an array of length 3.
 * 
 * @param imu 
 * @param raw input.
 * @param conv Answer is returned here.
 * @param calib Calibration to use for conversion.
 * 
 * @return 
 */
static int imu_comm_convert_lin(imu_t *imu, int16_t *raw, uquad_mat_t *conv, imu_calib_lin_t *calib)
{
    int i,retval = ERROR_OK;
    if(!imu->calib.calib_file_ready && !imu->calib.calib_estim_ready)
    {
	err_check(ERROR_IMU_NOT_CALIB,"Cannot convert without calibration!");
    }

    for(i=0; i < 3; ++i)
	m3x1_0->m_full[i] = ((double) raw[i]);
    /// m3x1_0 has tmp answer
    /// tmp = raw - b
    retval = uquad_mat_sub(m3x1_1,m3x1_0, calib->b);
    err_propagate(retval);
    /// m3x1_1 has tmp answer
    /// conv = k*tmp
    retval = uquad_mat_prod(conv, calib->TK_inv, m3x1_1);
    err_propagate(retval);
    // conv has final answer
    return retval;
}

/** 
 *Converts raw acc data to m/s^2
 *
 *@param imu 
 *@param frame Raw data from IMU
 *@param acc_reading Acceleration, in m/s^2
 *
 *@return error code
 */
static int imu_comm_acc_convert(imu_t *imu, int16_t *raw, uquad_mat_t *acc)
{
    int retval = ERROR_OK;
    retval = imu_comm_convert_lin(imu, raw, acc, imu->calib.m_lin);
    err_propagate(retval);
    return retval;
}

/**
 *Convert raw gyro data using calibration
 *
 *
 *@param imu 
 *@param data Raw gyro data.
 *@param gyro_reading Rate in rad/s
 *
 *@return error code
 */
static int imu_comm_gyro_convert(imu_t *imu, int16_t *raw, uquad_mat_t *gyro)
{
    int retval = ERROR_OK;
    retval = imu_comm_convert_lin(imu, raw, gyro, imu->calib.m_lin + 1);
    err_propagate(retval);
    return retval;
}

/**
 * Convert raw magn data using calibration
 *
 *
 *@param imu 
 *@param data Raw magn data.
 *@param magn_reading //TODO ?
 *
 *@return error code
 */
static int imu_comm_magn_convert(imu_t *imu, int16_t *raw, uquad_mat_t *magn)
{
    int retval = ERROR_OK;
    retval = imu_comm_convert_lin(imu, raw, magn, imu->calib.m_lin + 2);
    err_propagate(retval);
    return retval;
}

/**
 * Convert raw temperature data to °C.
 * Not much fun.
 *
 *
 *@param imu 
 *@param data Raw temp data.
 *@param temp Temperature in °C
 *
 *@return error code
 */
static int imu_comm_temp_convert(imu_t *imu, uint16_t *data, double *temp)
{
    if(imu == NULL)
    {
	err_check(ERROR_NULL_POINTER,"Invalid argument.");
    }
    *temp = ((double) *data)/10;
    return ERROR_OK;
}

#define PRESS_EXP  0.190294957183635L // 1/5.255 = 0.190294957183635
/**
 * Convert raw pressure data to relative altitud.
 * The first call to this function will set a reference pressure, which
 * will correspond to altitud 0m. Succesive calls wil return altitud
 * relative to initial altitud.
 *
 *@param imu 
 *@param data Raw press data.
 *@param temp Temperature in °C
 *
 *@return error code
 */
static int imu_comm_pres_convert(imu_t *imu, uint32_t *data, double *alt)
{
    double p0;
    p0 = (imu_comm_calib_estim(imu))?
	// we have a calibration, use it.
	imu->calib.null_est.pres:
	// if not, use default
	IMU_P0_DEFAULT;

    *alt = 44330*(1- pow((((double)(*data))/p0),PRESS_EXP));
    return ERROR_OK;
}

/** 
 *Converts raw IMU data to real world data.
 *Requires calibration.
 *
 *@param data raw data
 *@param measurements converted to real world data
 *
 *@return error code
 */
int imu_comm_raw2data(imu_t *imu, imu_raw_t *raw, imu_data_t *data){
    int retval;
    if(imu == NULL || raw == NULL || data == NULL){
	err_check(ERROR_NULL_POINTER,"Non null pointers required as args...");
    }
    // Get timestamp
    data->timestamp = raw->timestamp;
    data->T_us = (double) raw->T_us;//TODO check!

    // Convert accelerometer readings    
    retval = imu_comm_acc_convert(imu, raw->acc, data->acc);
    err_propagate(retval);

    // Convert gyroscope readings
    retval = imu_comm_gyro_convert(imu, raw->gyro, data->gyro);
    err_propagate(retval);

    // Convert magnetometer readings
    retval = imu_comm_magn_convert(imu, raw->magn, data->magn);
    err_propagate(retval);

    retval = convert_2_euler(data);
    err_propagate(retval);

    // Convert temperature readings
    retval = imu_comm_temp_convert(imu, &(raw->temp), &(data->temp));
    err_propagate(retval);

    // Convert altitud readings
    retval = imu_comm_pres_convert(imu, &(raw->pres), &(data->alt));
    err_propagate(retval);

    return ERROR_OK;
}

/** 
 *Gets latest unread raw values, can give repeated data.
 *Mem must be previously allocated for answer.
 *
 *@param imu Current imu status
 *@param data Answer is returned here
 *
 *@return error code
 */
int imu_comm_get_raw_latest(imu_t *imu, imu_raw_t *raw){
    int retval;
    imu_raw_t *frame_latest = imu->frame_buff + imu->frame_buff_latest;
    retval = imu_comm_copy_frame(frame_latest,raw);
    err_propagate(retval);
    return retval;
}

/**
 * Checks if unread data (1 or more samples)
 * exists.
 *
 * @param imu
 *
 * @return answer is returned here.
 */
uquad_bool_t imu_comm_unread(imu_t *imu)
{
    return imu->unread_data > 0;
}


/** 
 *Gets latest unread raw values.
 *Mem must be previously allocated for answer.
 *
 *@param imu
 *@param data Answer is returned here
 *
 *@return error code
 */
int imu_comm_get_raw_latest_unread(imu_t *imu, imu_raw_t *raw){
    int retval;
    if(!imu_comm_unread(imu)){
	err_check(ERROR_FAIL,"No unread data available.");
    }
    imu_raw_t *frame_latest = imu->frame_buff + imu->frame_buff_latest;
    retval = imu_comm_copy_frame(frame_latest,raw);
    err_propagate(retval);
    return retval;
}

/** 
 * Calculates value of the sensor readings from the RAW data, using current imu calibration.
 * This requires a reasonable calibration.
 * Mem must be previously allocated for answer.
 *
 *@param imu Current imu status
 *@param data Answer is returned here
 *
 *@return error code
 */
int imu_comm_get_data_latest(imu_t *imu, imu_data_t *data){
    int retval = ERROR_OK;

    imu_raw_t *frame = imu->frame_buff + imu->frame_buff_latest;
    retval = imu_comm_raw2data(imu, frame, data);
    err_propagate(retval);

    return retval;
}

/** 
 *If unread data exists, then calculates the latest value of the sensor readings
 *from the raw data, using current imu calibration.
 *This requires a reasonable calibration.
 *Mem must be previously allocated for answer.
 *
 *Decrements the unread count.
 *
 *@param imu 
 *@param data Answer is returned here
 *
 *@return error code
 */
int imu_comm_get_data_latest_unread(imu_t *imu, imu_data_t *data){
    int retval = ERROR_OK;
    if(!imu_comm_unread(imu)){
	err_check(ERROR_FAIL,"No unread data available.");
    }

    imu_raw_t *frame = imu->frame_buff + imu->frame_buff_latest;
    retval = imu_comm_raw2data(imu, frame, data);
    err_propagate(retval);

    imu->unread_data -= 1;
    return retval;
}

/** 
 * Copies the data in src to dest.
 * Must previously allocate mem for dest.
 * 
 * @param src 
 * @param dest 
 * 
 * @return 
 */
int imu_comm_copy_data(imu_data_t *src, imu_data_t *dest)
{
    int i;
    if(src == NULL || dest == NULL)
	err_check(ERROR_NULL_POINTER,"NULL pointer arg is not valid.");
    dest->T_us = src->T_us;
    for (i=0; i<3; ++i)
	dest->acc[i] = src->acc[i];
    for (i=0; i<3; ++i)
	dest->gyro[i] = src->gyro[i];
    for (i=0; i<3; ++i)
	dest->magn[i] = src->magn[i];
    dest->temp = src->temp;
    dest->alt = src->alt;
    dest->timestamp = src->timestamp;
    return ERROR_OK;
}

/** 
 *If unread data exists, gets the latest value of the sensor readings. (raw data).
 *
 *Decrements the unread count.
 *
 *@param imu 
 *@param data Answer is returned here
 *
 *@return error code
 */
int imu_comm_get_data_raw_latest_unread(imu_t *imu, imu_raw_t *data){
    int retval = ERROR_OK;
    if(!imu_comm_unread(imu)){
	err_check(ERROR_FAIL,"No unread data available.");
    }

    imu_raw_t *frame = imu->frame_buff + imu->frame_buff_latest;
    retval = imu_comm_copy_frame(frame,data);
    err_propagate(retval);
    imu->unread_data -= 1;
    return retval;
}

/** 
 *Return file descriptor corresponding to the IMU.
 *This should be used when polling devices from the main control loop.
 *
 *@param imu 
 *@param fds file descriptor is returned here
 *
 *@return error code
 */
int imu_comm_get_fds(imu_t *imu,int *fds)
{
#if IMU_COMM_FAKE
    if(imu->device == NULL){
	err_check(ERROR_NULL_POINTER,"Cannot get fds, device set to NULL");
    }
    if(fds == NULL){
	err_check(ERROR_NULL_POINTER,"Cannot return fds in NULL pointer");
    }
    *fds = fileno(imu->device);
#else
    *fds = imu->device;
#endif
    return ERROR_OK;
}

// -- -- -- -- -- -- -- -- -- -- -- --
// Calibration
// -- -- -- -- -- -- -- -- -- -- -- --
/** 
 * Returns true iif calibration data has been loaded from file
 * 
 * @param imu 
 * 
 * @return 
 */
uquad_bool_t imu_comm_calib_file(imu_t *imu)
{
    return imu->calib.calib_file_ready;
}	

/** 
 * Returns true iif calibration data has been estimated
 * by calling imu_comm_calibration_start()
 * 
 * @param imu 
 * 
 * @return 
 */
uquad_bool_t imu_comm_calib_estim(imu_t *imu)
{
    return imu->calib.calib_estim_ready;
}

int imu_comm_calib_save(imu_t *imu, const char *filename)
{
    int i;
    FILE *calib_file = fopen(filename,"w");
    if(calib_file == NULL)
    {
	err_check(ERROR_OPEN,"Could not open file to save calib!");
    }

    for (i = 0; i < 3; ++i)
    {
	// TK_inv
	uquad_mat_dump(imu->calib.m_lin[i].TK_inv, calib_file);

	// b
	uquad_mat_dump(imu->calib.m_lin[i].b, calib_file);
    }
    fclose(calib_file);
    return ERROR_OK;
}

/** 
 *Get IMU calibration.
 *Currently only calibration is null estimation.
 * //TODO:
 *  - gain
 *  - non linearity
 *
 *@param imu 
 *@param calibration return data here (check return error code before using)
 *
 *@return error code
 */
int imu_comm_calibration_get(imu_t *imu, imu_calib_t **calib){

    if(!imu_comm_calib_file(imu) && !imu_comm_calib_estim(imu)){
	err_check(ERROR_IMU_STATUS,"IMU is not calibrated");
    }
    if(calib == NULL){
	err_check(ERROR_NULL_POINTER,"Cannot return data in null pointer!");
    }

    *calib = &imu->calib;
    return ERROR_OK;
}

/**
 * Checks if enough samples are available to get average.
 *
 * @param imu
 *
 * @return if true, the can perform average
 */
uquad_bool_t imu_comm_avg_ready(imu_t *imu)
{
    return imu->frame_count >= IMU_AVG_COUNT;
}

/**
 * Adds two data, destroying one.
 * After execution, A == (A+B)
 *
 * @param A
 * @param B
 *
 * @return
 */
int imu_comm_add_data(imu_data_t *A, imu_data_t *B)
{
    int retval = ERROR_OK;
    retval = uquad_mat_add(A->acc,A->acc,B->acc);
    err_propagate(retval);
    retval = uquad_mat_add(A->gyro,A->gyro,B->gyro);
    err_propagate(retval);
    retval = uquad_mat_add(A->magn,A->magn,B->magn);
    err_propagate(retval);
    A->temp += B->temp;
    A->alt += B->alt;
    return retval;
}

/**
 * Normalizes data, usefull after calculating average.
 * This is faster than dividing after adding each element
 * to the average, but assumes overflow will not happen.
 * This assumption is reasonable with the reading we get from
 * the IMU.
 *
 * @param data
 * @param k
 *
 * @return
 */
int imu_data_normalize(imu_data_t *data, int k)
{
    double dk;
    int retval = ERROR_OK;
    if(k <= 0)
    {
	err_check(ERROR_INVALID_ARG,"Will not normalize to non positive value!");
    }
    dk = (double)k;
    retval = uquad_mat_scalar_div(data->acc, NULL, dk);
    err_propagate(retval);
    retval = uquad_mat_scalar_div(data->gyro, NULL, dk);
    err_propagate(retval);
    retval = uquad_mat_scalar_div(data->magn, NULL, dk);
    err_propagate(retval);
    data->temp /= dk;
    data->alt /= dk;
    return retval;
}

/**
 * Get previous index of circ buffer
 *
 * @param curr_index
 *
 * @return answer
 */
int circ_buff_prev_index(int curr_index, int buff_len)
{
    curr_index -= 1;
    if(curr_index < 0)
	curr_index += buff_len;
    return curr_index;
}

/**
 * Calculates average based on IMU_AVG_COUNT samples.
 * Will sum up, and then normalize.
 *
 * @param imu
 * @param data answer is returned here.
 *
 * @return error code.
 */
int imu_comm_get_avg(imu_t *imu, imu_data_t *data)
{
    int retval, i, j;
    if(imu->frame_count < IMU_AVG_COUNT)
    {
	err_check(ERROR_IMU_AVG_NOT_ENOUGH,"Not enough samples to average!");
    }
    j = imu->frame_buff_latest;
    for(i = 0; i < IMU_AVG_COUNT; ++i)
    {
	if(i == 0)
	{
	    /// initialize sum
	    retval = imu_comm_raw2data(imu, imu->frame_buff + j,data);
	    err_propagate(retval);
	}
	else
	{
	    retval = imu_comm_raw2data(imu, imu->frame_buff + j,&imu->tmp_avg);
	    err_propagate(retval);
	    /// add to sum
	    retval = imu_comm_add_data(data, &imu->tmp_avg);
	    err_propagate(retval);
	}
	j = circ_buff_prev_index(j,IMU_FRAME_BUFF_SIZE);
    }
    retval = imu_data_normalize(data, IMU_AVG_COUNT);
    err_propagate(retval);
    return retval;
}


/**
 * Get average using the latest data, with at least 1
 * unread sample.
 *
 *
 * @param imu
 * @param data answer
 *
 * @return
 */
int imu_comm_get_avg_unread(imu_t *imu, imu_data_t *data)
{
    int retval;
    if(!imu_comm_unread(imu))
    {
	err_check(ERROR_IMU_NO_UPDATES,"No unread data!");
    }
    retval = imu_comm_get_avg(imu,data);
    err_propagate(retval);
    --imu->unread_data;
    return ERROR_OK;
}

/** 
 * Will output data to stream.
 * 
 * @param data 
 * @param stream 
 * 
 * @return 
 */
int imu_comm_print_data(imu_data_t *data, FILE *stream){
    if(stream == NULL){
	stream = stdout;
    }
    fprintf(stream,IMU_COMM_PRINT_DATA_FORMAT,
	    (int)data->timestamp.tv_sec,
	    (int)data->timestamp.tv_usec,
	    data->T_us,
	    data->acc->m_full[0],
	    data->acc->m_full[1],
	    data->acc->m_full[2],
	    data->gyro->m_full[0],
	    data->gyro->m_full[1],
	    data->gyro->m_full[2],
	    data->magn->m_full[0],
	    data->magn->m_full[1],
	    data->magn->m_full[2],
	    data->temp,
	    data->alt);
    return ERROR_OK;
}

/** 
 * Will output raw data to stream.
 * 
 * @param frame 
 * @param stream 
 * 
 * @return 
 */
int imu_comm_print_raw(imu_raw_t *frame, FILE *stream){
    if(stream == NULL){
	stream = stdout;
    }

    fprintf(stream,IMU_COMM_PRINT_RAW_FORMAT,
	    (int)frame->timestamp.tv_sec,
	    (int)frame->timestamp.tv_usec,
	    frame->T_us,
	    frame->acc[0],
	    frame->acc[1],
	    frame->acc[2],
	    frame->gyro[0],
	    frame->gyro[1],
	    frame->gyro[2],
	    frame->magn[0],
	    frame->magn[1],
	    frame->magn[2],
	    frame->temp,
	    frame->pres);

    return ERROR_OK;
}

int imu_comm_print_calib(imu_calib_t *calib, FILE *stream){
    //TODO
    if(calib == NULL)
    {
	err_check(ERROR_NULL_POINTER,"Invalid argument.");
    }
    if(stream == NULL)
	stream = stdout;
    err_check(ERROR_FAIL,"Not implemented.");
}















#if 0
/// unused code

/** 
 *Checks if samples used for avg fall withing a certain interval.
 *It makes no sense to average samples from completely diff moments.
 *
 *@param imu 
 *
 *@return true if ok to avg, otherwise false
 */
static uquad_bool_t imu_comm_avg_validate_time_interval(imu_t *imu){
    int retval;
    long double max_interval;
    struct timeval diff;
    retval = uquad_timeval_substract(&diff,				\
				     imu->frame_buff[frame_circ_index(imu)].timestamp, \
				     imu->frame_buff[imu->frame_buff_next].timestamp);
    if(retval<=0)
	return false;
    max_interval = (long double)imu->settings.T*IMU_COMM_AVG_MAX_INTERVAL;
    max_interval -= diff.tv_sec;
    if(max_interval<=0)
	return false;	
    max_interval -= diff.tv_usec/1000000;
    return (max_interval > 0)? true:false;
}   

static void imu_comm_restart_sampling(imu_t *imu){
    imu->unread_data = 0;
    imu->frames_sampled = 0;
    imu->frame_buff_next = 0;
    imu->avg_ready = false;
}

int imu_comm_get_acc_sens(imu_t *imu, int *acc_index){
    if(acc_index == NULL){
	err_check(ERROR_NULL_POINTER,"Cannot return value, null pointer as argument.");
    }
    *acc_index = imu->settings.acc_sens;
    return ERROR_OK;
}

// sampling freq values
uint8_t imu_fs_values[IMU_FS_OPT_COUNT] = {50 /* Hz */};
/** 
 *Set sampling frequency (fs)
 *IMU must be idle, and will be left idle.
 *
 *@param imu 
 *@param new_value index for imu_fs_values[]
 *
 *@return error code
 */
int imu_comm_set_fs(imu_t *imu, int new_value){
    int retval;
    if(new_value > IMU_FS_OPT_COUNT){
	err_check(ERROR_INVALID_ARG,"Invalid value for sampling frequency");
    }
    // IMU should be idle
    if(imu->status != IMU_COMM_STATE_STOPPED){
	err_check(ERROR_IMU_STATUS,"IMU must be idle to set fs");
    }

    // Set new fs
    retval = imu_comm_send_cmd(imu,imu_fs_values[new_value]);
    err_propagate(retval);
    // Update struct value
    imu->settings.fs = new_value;
    imu->settings.T = (double)1/imu_fs_values[new_value];
    return retval;
}

int imu_comm_get_fs(imu_t *imu, int *fs_index){
    if(fs_index == NULL){
	err_check(ERROR_NULL_POINTER,"Cannot return value, null pointer as argument.");
    }
    *fs_index = imu->settings.fs;
    return ERROR_OK;
}

static uint16_t swap_LSB_MSB_16(uint16_t a){
    return (((a&0xFF)<<8)|(a>>8));
}

static uint32_t swap_LSB_MSB_32(uint32_t a){
    uint16_t *b = (uint16_t*)&a;
    return
	swap_LSB_MSB_16(b[0]) |
	swap_LSB_MSB_16(b[1]);
    //TODO check!
}

#endif
