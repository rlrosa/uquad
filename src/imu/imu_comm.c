#include "imu_comm.h"
#include <uquad_aux_io.h>
#include <uquad_aux_time.h>
#include <macros_misc.h>
#include <math.h> // for pow()
#include <fcntl.h> // for open()
/// Aux mem
static uquad_mat_t *m3x3;
static uquad_mat_t *m3x1_0;
static uquad_mat_t *m3x1_1;

imu_status_t imu_comm_get_status(imu_t *imu){
    return imu->status;
}

int imu_data_alloc(imu_data_t *imu_data)
{
    int retval = ERROR_OK;
    imu_data->acc = uquad_mat_alloc(3,1);
    imu_data->gyro = uquad_mat_alloc(3,1);
    imu_data->magn = uquad_mat_alloc(3,1);
    if(imu_data->acc == NULL ||
       imu_data->acc == NULL ||
       imu_data->acc == NULL)
    {
	uquad_mat_free(imu_data->acc);
	uquad_mat_free(imu_data->gyro);
	uquad_mat_free(imu_data->magn);
	err_propagate(ERROR_MALLOC);
    }
    // initialize data to zeros
    retval = uquad_mat_zeros(imu_data->acc);
    err_propagate(retval);
    retval = uquad_mat_zeros(imu_data->gyro);
    err_propagate(retval);
    retval = uquad_mat_zeros(imu_data->magn);
    err_propagate(retval);

    return ERROR_OK;
}

void imu_data_free(imu_data_t *imu_data)
{
    uquad_mat_free(imu_data->acc);
    uquad_mat_free(imu_data->gyro);
    uquad_mat_free(imu_data->magn);
}

int imu_data_zero(imu_data_t *imu_data)
{
    int retval;
    if(imu_data == NULL)
    {
	err_check(ERROR_INVALID_ARG, "NULL pointer is invalid arg!");
    }
    retval = uquad_mat_zeros(imu_data->acc);
    err_propagate(retval);
    retval = uquad_mat_zeros(imu_data->gyro);
    err_propagate(retval);
    retval = uquad_mat_zeros(imu_data->magn);
    err_propagate(retval);
    imu_data->T_us = 0.0;
    imu_data->temp = 0.0;
    imu_data->alt  = 0.0;
    return retval;
}

int imu_comm_copy_data(imu_data_t *dest, imu_data_t *src)
{
    int retval = ERROR_OK;
    if(src == NULL || dest == NULL)
	err_check(ERROR_NULL_POINTER,"NULL pointer arg is not valid.");
    dest->T_us = src->T_us;
    retval = uquad_mat_copy(dest->acc,src->acc);
    err_propagate(retval);
    uquad_mat_copy(dest->gyro,src->gyro);
    err_propagate(retval);
    uquad_mat_copy(dest->magn,src->magn);
    err_propagate(retval);
    dest->temp      = src->temp;
    dest->alt       = src->alt;
    dest->timestamp = src->timestamp;
    return retval;
}

int imu_comm_copy_frame(imu_raw_t *dest, imu_raw_t *src)
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

#if !IMU_COMM_FAKE     // If reading from a log file, we have nothing to send to it
/**
 * Sends command to the IMU over serial line.
 *
 *@param imu 
 *@param cmd to send to the imu
 *
 *@return error code
 */
static int imu_comm_send_cmd(imu_t *imu, unsigned char cmd){
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
    return ERROR_OK;
}

/**
 * Sends serial command to the IMU to make it stop.
 *
 * @param imu
 *
 * @return error code.
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
 * @return error code.
 */
int imu_comm_resume(imu_t *imu){
    int retval;
    if(imu->status == IMU_COMM_STATE_RUNNING)
    {
	err_log("IMU already running.\n");
	return ERROR_OK;
    }
    // Run IMU
    retval = imu_comm_send_cmd(imu,IMU_COMMAND_RUN);
    err_propagate(retval);
    imu->status = IMU_COMM_STATE_RUNNING;
    return ERROR_OK;
}
#endif //!IMU_COMM_FAKE

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
 * Configures IMU to use default params and start sampling.
 * Depends on set_work_mode() in Output.pde (firmware)
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
 * Configure IMU.
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
    if( (strlen(device) >= 5) && (strncmp(device,"/dev/",5) == 0))
    {
	err_check(ERROR_INVALID_ARG,
		  "Expected an ascii log file!\n"			\
		  "To read from UART, IMU_COMM_FAKE must be disabled!");
    }
    // we don't want to write to the log file, just read.
    imu->device = fopen(device,"rb+");
    if(imu->device == NULL)
    {
	err_log_stderr("fopen()");
	err_propagate(ERROR_OPEN);
    }
#else
    char str[256];
    int retval;
    if( (strlen(device) < 3) || (strncmp(device,"/dev/",5) != 0))
    {
	err_check(ERROR_INVALID_ARG,
		  "Expected '/dev/*'!\n"				\
		  "If reading from a log, IMU_COMM_FAKE must be enabled!");
    }
    imu->device = open(device,O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(imu->device < 0)
    {
	err_log_stderr("open()");
	err_propagate(ERROR_OPEN);
    }
    retval = sprintf(str,"stty -F %s 115200 -echo raw",device);
    if(retval < 0)
    {
	err_log_stderr("sprintf()");
	return ERROR_FAIL;
    }
    retval = system(str);
    if(retval != 0)
    {
	err_log_stderr("system()");
	return ERROR_IO;
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
 * @return error code.
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
 * @return error code.
 */
int imu_comm_free_calib(imu_calib_t calib)
{
    int i;
    for (i = 0; i < 3; ++i)
    {
	// TK_inv
	uquad_mat_free(calib.m_lin[i].TK_inv);
	// b
	uquad_mat_free(calib.m_lin[i].b);
    }
    imu_data_free(&calib.null_est_data);
    uquad_mat_free(calib.acc_t_off);
    uquad_mat_free(calib.gyro_t_off);
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
 * @return error code.
 */
int imu_comm_load_calib(imu_t *imu, const char *path)
{
    int i,retval = ERROR_OK;
    double dtmp;
    FILE *calib_file = fopen(path,"r+");
    if(calib_file == NULL)
    {
	err_check(ERROR_OPEN,"Failed to open calib file!");
    }

    /// -- -- -- -- -- -- -- -- -- -- -- --
    /// Load linear model parameters
    /// -- -- -- -- -- -- -- -- -- -- -- --

    for (i = 0; i < 3; ++i)
    {
	// TK_inv
	retval = uquad_mat_load(imu->calib.m_lin[i].TK_inv, calib_file);
	err_propagate(retval);

	// b
	retval = uquad_mat_load(imu->calib.m_lin[i].b, calib_file);
	err_propagate(retval);
    }

    /// -- -- -- -- -- -- -- -- -- -- -- --
    /// load temperature compensation parameters
    /// -- -- -- -- -- -- -- -- -- -- -- --

    /// -- -- -- -- --
    /// accelerometer
    /// -- -- -- -- --
    // load z temp
    retval = fscanf(calib_file,"%lf",&dtmp);
    if(retval <= 0)
    {
	err_check(ERROR_READ, "Failed to read z_temp");
    }

    m3x1_0->m_full[0] = 0.0;
    m3x1_0->m_full[1] = 0.0;
    m3x1_0->m_full[2] = dtmp;
    retval = uquad_mat_prod(imu->calib.acc_t_off,
			    imu->calib.m_lin[0].TK_inv,
			    m3x1_0);
    err_propagate(retval);
    // load acc calibration temp
    retval = fscanf(calib_file,"%lf",&dtmp);
    if(retval <= 0)
    {
	err_check(ERROR_READ, "Failed to read acc_to");
    }
    imu->calib.acc_to = dtmp;

    /// -- -- -- -- --
    /// gyro
    /// -- -- -- -- --
    // load temp dep coefficients
    retval = uquad_mat_load(m3x1_0, calib_file);
    err_propagate(retval);
    retval = uquad_mat_prod(imu->calib.gyro_t_off,
			    imu->calib.m_lin[1].TK_inv,
			    m3x1_0);
    err_propagate(retval);
    // load temp indep offset
    retval = uquad_mat_load(m3x1_0, calib_file);
    err_propagate(retval);
    retval = uquad_mat_add(imu->calib.m_lin[1].b,
			   imu->calib.m_lin[1].b,
			   m3x1_0);
    err_propagate(retval);
    // load gyro calibration temp
    retval = fscanf(calib_file,"%lf",&dtmp);
    if(retval <= 0)
    {
	if(retval < 0)
	{
	    err_log_stderr("fscanf() - gyro_to");
	    return ERROR_IO;
	}
	else
	{
	    err_check(ERROR_READ, "Failed to read gyro_to");
	}
    }
    imu->calib.gyro_to = dtmp;
    retval = ERROR_OK; // clear error

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
 * @return error code.
 */
int imu_comm_init_calibration(imu_t *imu)
{
    int retval = ERROR_OK;
    retval = imu_comm_alloc_calib_lin(imu);
    err_propagate(retval);
    retval = imu_data_alloc(&imu->calib.null_est_data);
    err_propagate(retval);
    imu->calib.acc_t_off = uquad_mat_alloc(3,1);
    imu->calib.gyro_t_off = uquad_mat_alloc(3,1);
    if((imu->calib.acc_t_off == NULL) || (imu->calib.gyro_t_off == NULL))
    {
	err_check(ERROR_MALLOC, "Failed to allocate temp offset!");
    }
    /// load calibration parameters from file
    retval = imu_comm_load_calib(imu, IMU_DEFAULT_CALIB_PATH);
    err_propagate(retval);
    // Use default p0 until we get a calibration
    imu->calib.null_est.pres = IMU_P0_DEFAULT;
    return retval;
}

imu_t *imu_comm_init(const char *device){
    imu_t *imu;
    int
	retval = ERROR_OK;
    imu = (imu_t *)malloc(sizeof(imu_t));
    mem_alloc_check(imu);
    memset(imu,0,sizeof(imu_t));
    imu->status = IMU_COMM_STATE_UNKNOWN;

    m3x3 = uquad_mat_alloc(3,3);
    m3x1_0 = uquad_mat_alloc(3,1);
    m3x1_1 = uquad_mat_alloc(3,1);
    if(m3x3 == NULL || m3x1_0 == NULL || m3x1_1 == NULL)
	goto cleanup;

    // Set up filter, must have IMU_FILTER_LEN coefs
    imu->h[0] = 0.2;
    imu->h[1] = 0.2;
    imu->h[2] = 0.2;
    imu->h[3] = 0.2;
    imu->h[4] = 0.1;
    imu->h[5] = 0.1;

    // now connect to the imu
    retval = imu_comm_connect(imu,device);
    cleanup_if(retval);

    // Get aux memory
    retval = imu_data_alloc(&imu->tmp_filt);
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

    // Mark initial altitud as unknown
    imu->calib.p_z0 = -1;
    imu->calib.z0 = -1;

    // Wait 300ms + a bit more for IMU to reset (in case pgm cable is connected)
    sleep_ms(IMU_COMM_STARTUP_T_MS);
    return imu;

    cleanup:
    imu_comm_deinit(imu);
    return NULL;
}

int imu_comm_deinit(imu_t *imu){
    int
	retval = ERROR_OK;
    if(imu == NULL)
    {
	err_log("WARN: Nothing to free.");
	return ERROR_OK;
    }
    retval = imu_comm_disconnect(imu);
    // ignore answer and keep dying, leftovers are not reliable
    imu_data_free(&imu->tmp_filt);
    //TODO chec if more to free
    imu_comm_free_calib(imu->calib);
    uquad_mat_free(m3x3);
    uquad_mat_free(m3x1_0);
    uquad_mat_free(m3x1_1);
    free(imu);
    return retval;
}

#if !IMU_COMM_FAKE
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
	retval = read(imu->device, &tmp,IMU_INIT_END_SIZE);
	if(retval < 0)
	{
	    err_log_stderr("Read error: failed to get sync char...");
	    err_propagate(ERROR_CLOSE);
	}
	if(retval > 0)
	{
	    if(tmp == IMU_FRAME_END_CHAR)
	    {
		return ERROR_OK;
	    }
	    else
	    {
		err_check(ERROR_READ_SYNC,"Unexpected end of frame char: Discarding frame...");
	    }
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
static int imu_comm_get_sync_init(imu_t *imu, uquad_bool_t *sync_ok){
    int retval;//,i;
    unsigned char tmp = 'X';// Anything diff from IMU_FRAME_INIT_CHAR
    *sync_ok = false;
    static int fail_count = 0;
    retval = read(imu->device,&tmp,IMU_INIT_END_SIZE);
    if(retval < 0)
    {
	err_log_stderr("Read error: failed to get sync char...");
	err_propagate(ERROR_CLOSE);
    }
    else
    {
	if(retval > 0)
	{
	    // Match of either the init chars
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
		*sync_ok = true;
		return ERROR_OK;
	    }
	    else
	    {
		/**
		 * When starting up, its ok to miss a couple of char
		 * until sync is achieved, no need to log a bunch of
		 * scary errors.
		 *
		 */
		// wrong init char
		if(fail_count > IMU_SYNC_FAIL_MAX)
		{
		    err_check(ERROR_READ_SKIP,"Wrong sync init char!");
		}
		fail_count++;
		return ERROR_OK;
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
#endif // !IMU_COMM_FAKE

static imu_raw_null_t calib_accum;
int imu_comm_calibration_start(imu_t *imu){
    if(imu->status != IMU_COMM_STATE_RUNNING){
	err_check(ERROR_IMU_STATUS,"IMU must be running to calibrate!");
    }
    imu->status = IMU_COMM_STATE_CALIBRATING;
    /// clear calibration data/
    memset(&calib_accum,0,sizeof(imu_raw_null_t));
    imu->calib.calibration_counter = IMU_CALIB_SIZE;
    return ERROR_OK;
}

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
 * Integrate calibration data into IMU.
 * Replaces previous calibration, if any existed.
 *
 *@param imu
 *@param timestamp on the last sample used for the calibration
 *
 *@return error code.
 */
int imu_comm_calibration_finish(imu_t *imu){
    int retval = ERROR_OK, i;
    imu_data_t imu_data_tmp;
    if(imu->status != IMU_COMM_STATE_CALIBRATING){
	err_check(ERROR_IMU_STATUS,"Cannot finish calibration, IMU is not calibrating!");
    }
    if(imu->calib.calibration_counter != 0){
	err_check(ERROR_IMU_STATUS,"Not enough samples gathered!");
    }
    retval = imu_data_alloc(&imu_data_tmp);
    err_propagate(retval);

    struct timeval tv_end, tv_diff;
    gettimeofday(&tv_end,NULL);
    retval = uquad_timeval_substract(&tv_diff,tv_end,calibration_start_time);
    if(retval < 0){
	imu_data_free(&imu_data_tmp);
	err_check(ERROR_FAIL,"Calibration duration should not be negative!");
    }
    //TODO check if calib took too long, and refuse it if so
    //requires defintion of "too long"

    for(i = 0; i < 3; ++i)
    {
	imu->calib.null_est.acc[i]  =
	    (uint16_t)(calib_accum.acc[i]/IMU_CALIB_SIZE);
	imu->calib.null_est.gyro[i] =
	    (uint16_t)(calib_accum.gyro[i]/IMU_CALIB_SIZE);
	imu->calib.null_est.magn[i] =
	    (uint16_t)(calib_accum.magn[i]/IMU_CALIB_SIZE);
    }
    imu->calib.null_est.temp =
	(uint16_t)(calib_accum.temp/IMU_CALIB_SIZE);
    imu->calib.null_est.pres =
	(uint32_t)floor((((double)calib_accum.pres)/IMU_CALIB_SIZE));

    imu->calib.calibration_counter = -1;
    imu->calib.timestamp_estim = tv_end;

    // update offset estimation
    retval = imu_comm_raw2data(imu,
			       &imu->calib.null_est,
			       NULL,
			       &imu_data_tmp);
    if(retval != ERROR_OK)
	imu_data_free(&imu_data_tmp);
    err_propagate(retval);

    retval = imu_comm_copy_data(&imu->calib.null_est_data, &imu_data_tmp);
    err_propagate(retval);
    imu_data_free(&imu_data_tmp);

    /**
     * The converted altitud will be different from 0, since the calibration
     * is not yet ready. Starting position (z) is assumed as 0, so force it
     * here.
     */
    imu->calib.null_est_data.alt = 0.0;

    // If external altitud available, use it to determine p0
    if(imu->calib.z0 >= 0)
    {
	imu->calib.p_z0 = ((double)imu->calib.null_est.pres)/
	    pow(1 - imu->calib.z0/PRESS_K,PRESS_EXP_INV);
    }

    imu->calib.calib_estim_ready = true;
    imu->status = IMU_COMM_STATE_RUNNING;

    return ERROR_OK;
}

/**
 * Add frame info to build up calibration.
 * Will divide by frame count after done gathering, to avoid loosing info.
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
	calib_accum.acc[i]  += (int32_t) new_frame.acc[i];
	calib_accum.gyro[i] += (int32_t) new_frame.gyro[i];
	calib_accum.magn[i] += (int32_t) new_frame.magn[i];
    }
    calib_accum.temp += (uint32_t) new_frame.temp;
    calib_accum.pres += (uint64_t) new_frame.pres;

    --imu->unread_data;

    if(--imu->calib.calibration_counter == 0){
	retval = imu_comm_calibration_finish(imu);
	err_propagate(retval);
    }

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

    retval = imu_comm_copy_frame(imu->frame_buff + imu->frame_buff_next, new_frame);
    err_propagate(retval);
    imu->frame_buff_latest = imu->frame_buff_next;
    imu->frame_buff_next = (imu->frame_buff_next + 1)%IMU_FRAME_BUFF_SIZE;
    ++imu->unread_data;
    imu->frame_count = uquad_min(imu->frame_count + 1,IMU_FILTER_LEN);

    return ERROR_OK;
}

#if !IMU_COMM_FAKE
/**
 * Takes an array of bytes and parses them according to the format
 * of the frames sent by the IMU, generating an imu_raw_t structure.
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
	retval = read(imu->device,
		      buff_tmp_8 + buff_index,
		      (IMU_COMM_READ_1_BYTE)?
		      1:IMU_DEFAULT_FRAME_SIZE_BYTES - 2 - buff_index);
	if(retval < 0)
	{
		    err_log_stderr("Read error: no data! Restarting...");
		    buff_index = 0;
		    err_propagate(ERROR_IO);
	}

	buff_index += retval;
	if(buff_index == IMU_DEFAULT_FRAME_SIZE_BYTES-2)
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
 *Will read from logs generated by main.
 *Format is:
 *   - timestamp main
 *   - timestamp main
 *   - timestamp imu
 *   - accx
 *   - accy
 *   - accz
 *   - gyrox
 *   - gyroy
 *   - gyroz
 *   - magnx
 *   - magny
 *   - magnz
 *   - temp
 *   - pressure
 *
 *NOTES:
 *  - Will block if data is not available.
 *  - Will not take care of sync.
 *
 *@param imu 
 *@param new_frame New frame is returned here.
 *
 *@return error code
 */
int imu_comm_read_frame_ascii(imu_t *imu, imu_raw_t *new_frame, uquad_bool_t *ok)
{
    int retval = ERROR_OK,i, itmp;
    unsigned int uitmp;
    double dtmp;
    *ok = false;

    retval = fscanf(imu->device,"%lf",&dtmp);
    if(retval < 0)
    {
	if(retval == EOF)
	{
	    err_check(ERROR_IO,"End of file!");
	}
	else
	{
	    err_log_stderr("fscanf()");
	    err_propagate(ERROR_IO);
	}
    }
    double2tv(new_frame->timestamp,dtmp);

    // Discard the other timestamp
    retval = fscanf(imu->device,"%lf",&dtmp);
    if(retval < 0)
    {
	if(retval == EOF)
	{
	    err_check(ERROR_IO,"End of file!");
	}
	else
	{
	    err_log_stderr("fscanf()");
	    err_propagate(ERROR_IO);
	}
    }

    // Get sampling time
    retval = fscanf(imu->device,"%ud",&uitmp);
    new_frame->T_us = (uint16_t)uitmp;
    if(retval < 0)
    {
	err_check(ERROR_IO,"Read error: Failed to read T_us...");
    }

    // Use timestamp from log file, do not generate a new one
    // Generate timestamp
    //    gettimeofday(& new_frame->timestamp,NULL);

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
    *ok = true;
    return ERROR_OK;
}
#endif

/// IMU read state machine support
typedef enum read_status{
    IDLE = 0,
    INIT_SYNC_DONE,
    READ_FRAME_DONE,
    END_SYNC_DONE,
    READ_STATE_COUNT
} read_status_t;
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
#if IMU_COMM_FAKE
	retval = imu_comm_read_frame_ascii(imu, &new_frame, &ok);
	err_propagate(retval);
	if(ok)
	    status = END_SYNC_DONE;
#else // IMU_COMM_FAKE
	/// -- -- -- -- -- -- -- -- -- -- -- --
	/// 1 - sync init
	/// -- -- -- -- -- -- -- -- -- -- -- --
	retval = imu_comm_get_sync_init(imu,&ok);
	err_propagate(retval);
	if(ok)
	    status++;
	break;
    case INIT_SYNC_DONE:
	/// -- -- -- -- -- -- -- -- -- -- -- --
	/// 2 - frame
	/// -- -- -- -- -- -- -- -- -- -- -- --
	retval = imu_comm_read_frame_binary(imu, &new_frame, &ok);
	err_propagate(retval);
	if(ok)
	{
	    // frame completed!
	    status++;
	}
	break;
    case READ_FRAME_DONE:
	/// -- -- -- -- -- -- -- -- -- -- -- --
	/// 3 - sync end
	/// -- -- -- -- -- -- -- -- -- -- -- --
	retval = imu_comm_get_sync_end(imu);
	if(retval == ERROR_READ_SYNC)
	{
	    // sync failed, restart
	    status = IDLE;
	}
	err_propagate(retval);
	status++;

	//break; // Do not break here, keep going!
#endif // IMU_COMM_FAKE
    case END_SYNC_DONE:
	/// -- -- -- -- -- -- -- -- -- -- -- --
	/// 4 - add
	/// -- -- -- -- -- -- -- -- -- -- -- --
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
	break;
    default:
	status = IDLE;
	err_check(ERROR_FAIL,"Invalid status! Restarting...");
	break;
    }
    return ERROR_OK;
}

int convert_2_euler(imu_data_t *data)
{
    int retval;
    double psi;              // rad
    double phi;              // rad
    double theta;            // rad
    double acc_norm = uquad_mat_norm(data->acc);
    if(uquad_abs(data->acc->m_full[0]) < IMU_TH_DEADLOCK_ACC_NORM*acc_norm)
    {
	phi = -asin(data->acc->m_full[0]/acc_norm);
	psi = atan2(data->acc->m_full[1],data->acc->m_full[2]);
    }else if(data->acc->m_full[0]>0.0){
	phi=-PI/2.0;
	psi=0.0;
    }else{
	phi=PI/2.0;
	psi=0.0;
    }

    m3x3->m[0][0] = cos(phi)/(uquad_square(cos(phi)) + uquad_square(sin(phi)));
    m3x3->m[0][1] = (sin(phi)*sin(psi))/((uquad_square(cos(phi)) + uquad_square(sin(phi)))*(uquad_square(cos(psi)) + uquad_square(sin(psi))));
    m3x3->m[0][2] = (cos(psi)*sin(phi))/((uquad_square(cos(phi)) + uquad_square(sin(phi)))*(uquad_square(cos(psi)) + uquad_square(sin(psi))));
    m3x3->m[1][0] = 0.0;
    m3x3->m[1][1] = cos(psi)/(uquad_square(cos(psi)) + uquad_square(sin(psi)));
    m3x3->m[1][2] = -sin(psi)/(uquad_square(cos(psi)) + uquad_square(sin(psi)));
    m3x3->m[2][0] = -sin(phi)/(uquad_square(cos(phi)) + uquad_square(sin(phi)));
    m3x3->m[2][1] = (cos(phi)*sin(psi))/((uquad_square(cos(phi)) + uquad_square(sin(phi)))*(uquad_square(cos(psi)) + uquad_square(sin(psi))));
    m3x3->m[2][2] = (cos(phi)*cos(psi))/((uquad_square(cos(phi)) + uquad_square(sin(phi)))*(uquad_square(cos(psi)) + uquad_square(sin(psi))));

    retval = uquad_mat_prod(m3x1_0,m3x3,data->magn);
    err_propagate(retval);

    theta = -atan2(m3x1_0->m_full[1],m3x1_0->m_full[0]) + IMU_TH_DEADLOCK_ANG;//9.78;

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
 * NOTE: Either raw or raw_db must be NULL.
 *
 * @param imu 
 * @param raw input.
 * @param raw_db input, casted to doubles.
 * @param conv Answer is returned here.
 * @param calib Calibration to use for conversion.
 *
 * @return error code
 */
static int imu_comm_convert_lin(imu_t *imu, int16_t *raw, uquad_mat_t *raw_db, uquad_mat_t *conv, imu_calib_lin_t *calib)
{
    int i,retval = ERROR_OK;
    if(!imu->calib.calib_file_ready && !imu->calib.calib_estim_ready)
    {
	err_check(ERROR_IMU_NOT_CALIB,"Cannot convert without calibration!");
    }

    if((raw == NULL) == (raw_db == NULL))
    {
	err_check(ERROR_INVALID_ARG, "Either raw or raw_db must be NULL!");
    }
    if(raw != NULL)
    {
	for(i=0; i < 3; ++i)
	    m3x1_0->m_full[i] = ((double) raw[i]);
    }
    else
    {
	retval = uquad_mat_copy(m3x1_0, raw_db);
	err_propagate(retval);
    }
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
 * Converts raw acc data to m/s^2
 * Model:
 *   T*inv(K)*(raw - offset + b_t*(temp - temp_0))
 *
 * NOTE: Either raw or raw_db must be NULL.
 *
 *@param imu 
 *@param raw Raw data from IMU
 *@param raw_db Raw data from IMU, casted to doubles.
 *@param acc_reading Acceleration, in m/s^2
 *
 *@return error code
 */
static int imu_comm_acc_convert(imu_t *imu, int16_t *raw, uquad_mat_t *raw_db, uquad_mat_t *acc, double temp)
{
    int retval = ERROR_OK;
    retval = imu_comm_convert_lin(imu, raw, raw_db, acc, imu->calib.m_lin);
    err_propagate(retval);
    // temperature correction
    retval = uquad_mat_scalar_mul(m3x1_0,
				  imu->calib.acc_t_off,
				  temp - imu->calib.acc_to);
    err_propagate(retval);
    retval = uquad_mat_sub(acc, acc, m3x1_0);
    err_propagate(retval);
    return retval;  
}

/**
 * Convert raw gyro data using calibration and current temperature.
 *
 * NOTE: Either raw or raw_db must be NULL.
 *
 *@param imu 
 *@param raw Raw gyro data.
 *@param raw_db Raw gyro data, casted to doubles.
 *@param gyro Rate in rad/s
 *@param temp current temperature in °C
 *
 *@return error code
 */
static int imu_comm_gyro_convert(imu_t *imu, int16_t *raw, uquad_mat_t *raw_db, uquad_mat_t *gyro, double temp)
{
    int retval = ERROR_OK;
    retval = imu_comm_convert_lin(imu, raw, raw_db, gyro, imu->calib.m_lin + 1);
    err_propagate(retval);
    // temperature compensation
    retval = uquad_mat_scalar_mul(m3x1_0,
				  imu->calib.gyro_t_off,
				  temp - imu->calib.gyro_to);
    err_propagate(retval);
    retval = uquad_mat_sub(gyro, gyro, m3x1_0);

    // compensate startup-offset
    retval = uquad_mat_sub(gyro,gyro,imu->calib.null_est_data.gyro);
    err_propagate(retval);
    return retval;
}

/**
 * Convert raw magn data using current calibration.
 *
 * NOTE: Either raw or raw_db must be NULL.
 *
 *@param imu 
 *@param raw Raw magnetometer data.
 *@param raw_db Raw magnetometer data, casted to doubles.
 *@param magn Converted magnetometer data.
 *
 *@return error code
 */
static int imu_comm_magn_convert(imu_t *imu, int16_t *raw, uquad_mat_t *raw_db, uquad_mat_t *magn)
{
    int retval = ERROR_OK;
    retval = imu_comm_convert_lin(imu, raw, raw_db, magn, imu->calib.m_lin + 2);
    err_propagate(retval);
    return retval;
}

/**
 * Convert raw temperature data to °C.
 * Not much fun.
 *
 * NOTE: Either data or data_db must be NULL.
 *
 *@param imu 
 *@param data Raw temp data.
 *@param data_db Raw temp data, casted to double.
 *@param temp Temperature in °C
 *
 *@return error code
 */
static int imu_comm_temp_convert(imu_t *imu, uint16_t *data, double *data_db, double *temp)
{
    if(imu == NULL)
    {
	err_check(ERROR_NULL_POINTER,"Invalid argument.");
    }
    if((data == NULL) == (data_db == NULL))
    {
	err_check(ERROR_INVALID_ARG, "Either data or data_db must be NULL!");
    }
    if(data != NULL)
    {
	*temp = ((double) *data)/10;
    }
    else
    {
	*temp = (*data_db)/10;
    }
    return ERROR_OK;
}

/**
 * Convert raw pressure data to relative altitud.
 * The first call to this function will set a reference pressure, which
 * will correspond to altitud 0m. Succesive calls wil return altitud
 * relative to initial altitud.
 * NOTE: Either data or data_db must be NULL.
 *
 *@param imu 
 *@param data Raw press data.
 *@param data_db Raw press data, casted to double
 *@param temp Temperature in °C
 *
 *@return error code
 */
static int imu_comm_pres_convert(imu_t *imu, uint32_t *data, double *data_db, double *alt)
{
    double p0;
    if((data == NULL) == (data_db == NULL))
    {
	err_check(ERROR_INVALID_ARG, "Either data or data_db must be NULL!");
    }
    if(imu->calib.p_z0 >= 0)
	p0 = imu->calib.p_z0;
    else
    {
	p0 = (imu_comm_calib_estim(imu))?
	    // we have a calibration, use it.
	    imu->calib.null_est.pres:
	    // if not, use default
	    IMU_P0_DEFAULT;
    }

    if(data != NULL)
	*alt = PRESS_K*(1.0 - pow((((double)(*data))/p0),PRESS_EXP));
    else
	*alt = PRESS_K*(1.0 - pow(((*data_db)/p0),PRESS_EXP));
    return ERROR_OK;
}

int imu_comm_set_z0(imu_t *imu, double z0)
{
    if(imu == NULL)
    {
	err_check(ERROR_NULL_POINTER,"Non null pointer required as arg...");
    }
    imu->calib.z0 = z0;
    return ERROR_OK;
}

int imu_comm_raw2data(imu_t *imu, imu_raw_t *raw, imu_data_t *raw_db, imu_data_t *data){
    int retval;
    if(imu == NULL || data == NULL){
	err_check(ERROR_NULL_POINTER,"Non null pointers required as args...");
    }
    if((raw == NULL) == (raw_db == NULL))
    {
	err_check(ERROR_INVALID_ARG, "Either raw or raw_db must be NULL!");
    }

    // Get timestamp
    if(raw != NULL)
    {
	data->timestamp = raw->timestamp;
	data->T_us = (double) raw->T_us;//TODO check!

	// Convert temperature readings
	retval = imu_comm_temp_convert(imu, &(raw->temp), NULL, &(data->temp));
	err_propagate(retval);

	// Convert accelerometer readings
	retval = imu_comm_acc_convert(imu, raw->acc, NULL, data->acc, data->temp);
	err_propagate(retval);

	// Convert gyroscope readings
	retval = imu_comm_gyro_convert(imu, raw->gyro, NULL, data->gyro, data->temp);
	err_propagate(retval);

	// Convert magnetometer readings
	retval = imu_comm_magn_convert(imu, raw->magn, NULL, data->magn);
	err_propagate(retval);

	// Convert altitud readings
	retval = imu_comm_pres_convert(imu, &(raw->pres), NULL, &(data->alt));
	err_propagate(retval);
    }
    else
    {
	data->timestamp = raw_db->timestamp;
	data->T_us = raw_db->T_us;//TODO check!

	// Convert temperature readings
	retval = imu_comm_temp_convert(imu, NULL, &raw_db->temp, &(data->temp));
	err_propagate(retval);

	// Convert accelerometer readings
	retval = imu_comm_acc_convert(imu, NULL, raw_db->acc, data->acc, data->temp);
	err_propagate(retval);

	// Convert gyroscope readings
	retval = imu_comm_gyro_convert(imu, NULL, raw_db->gyro, data->gyro, data->temp);
	err_propagate(retval);

	// Convert magnetometer readings
	retval = imu_comm_magn_convert(imu, NULL, raw_db->magn, data->magn);
	err_propagate(retval);

	// Convert altitud readings
	retval = imu_comm_pres_convert(imu, NULL, &(raw_db->alt), &(data->alt));
	err_propagate(retval);
    }

    retval = convert_2_euler(data);
    err_propagate(retval);  

    return ERROR_OK;
}

int imu_comm_get_raw_latest(imu_t *imu, imu_raw_t *raw){
    int retval;
    imu_raw_t *frame_latest = imu->frame_buff + imu->frame_buff_latest;
    retval = imu_comm_copy_frame(raw, frame_latest);
    err_propagate(retval);
    return retval;
}

uquad_bool_t imu_comm_unread(imu_t *imu)
{
    return imu->unread_data > 0;
}


int imu_comm_get_raw_latest_unread(imu_t *imu, imu_raw_t *raw){
    int retval;
    if(!imu_comm_unread(imu)){
	err_check(ERROR_FAIL,"No unread data available.");
    }
    imu_raw_t *frame_latest = imu->frame_buff + imu->frame_buff_latest;
    retval = imu_comm_copy_frame(raw, frame_latest);
    err_propagate(retval);
    return retval;
}

int imu_comm_get_data_latest(imu_t *imu, imu_data_t *data){
    int retval = ERROR_OK;

    retval = imu_comm_raw2data(imu,
			       imu->frame_buff + imu->frame_buff_latest,
			       NULL,
			       data);
    err_propagate(retval);

    return retval;
}

int imu_comm_get_data_latest_unread(imu_t *imu, imu_data_t *data){
    int retval = ERROR_OK;
    if(!imu_comm_unread(imu)){
	err_check(ERROR_FAIL,"No unread data available.");
    }

    retval = imu_comm_get_data_latest(imu, data);
    err_propagate(retval);

    imu->unread_data -= 1;
    return retval;
}

int imu_comm_get_data_raw_latest_unread(imu_t *imu, imu_raw_t *data){
    int retval = ERROR_OK;
    if(!imu_comm_unread(imu)){
	err_check(ERROR_FAIL,"No unread data available.");
    }

    imu_raw_t *frame = imu->frame_buff + imu->frame_buff_latest;
    retval = imu_comm_copy_frame(data, frame);
    err_propagate(retval);
    imu->unread_data -= 1;
    return retval;
}

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
uquad_bool_t imu_comm_calib_file(imu_t *imu)
{
    return imu->calib.calib_file_ready;
}	

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

uquad_bool_t imu_comm_filter_ready(imu_t *imu)
{
    return imu->frame_count >= IMU_FILTER_LEN;
}

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

int imu_comm_scalmul_data(imu_data_t *A, double k)
{
    int retval = ERROR_OK;
    retval = uquad_mat_scalar_mul(A->acc,NULL,k);
    err_propagate(retval);
    retval = uquad_mat_scalar_mul(A->gyro,NULL,k);
    err_propagate(retval);
    retval = uquad_mat_scalar_mul(A->magn,NULL,k);
    err_propagate(retval);
    A->temp *= k;
    A->alt  *= k;
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

static void imu_comm_cast_raw2data(imu_raw_db_t *out, imu_raw_t *in)
{
    int i;
    for(i=0; i<3; ++i)
    {
	out->acc->m_full[i]  = (double) in->acc[i];
	out->gyro->m_full[i] = (double) in->gyro[i];
	out->magn->m_full[i] = (double) in->magn[i];
    }
    out->T_us      = (double) in->T_us;
    out->temp      = (double) in->temp;
    out->alt       = (double) in->pres; // THIS IS NOT ALT, IT'S PRES
    out->timestamp = in->timestamp;
}

int imu_comm_get_filtered(imu_t *imu, imu_data_t *data)
{
    int retval, i, j;
    imu_raw_t *raw_curr;
    if(imu->frame_count < IMU_FILTER_LEN)
    {
	err_check(ERROR_IMU_FILTER_LEN_NOT_ENOUGH,"Not enough samples to average!");
    }
    j = imu->frame_buff_latest;
    for(i = 0; i < IMU_FILTER_LEN; ++i)
    {
	raw_curr = imu->frame_buff + j;
	if(i == 0)
	{
	    /// initialize sum
	    imu_comm_cast_raw2data((imu_raw_db_t *)&imu->tmp_filt, raw_curr);
	    retval = imu_comm_scalmul_data(&imu->tmp_filt,imu->h[i]);
	    err_propagate(retval);
	}
	else
	{
	    imu_comm_cast_raw2data((imu_raw_db_t *)data, raw_curr);
	    retval = imu_comm_scalmul_data(data, imu->h[i]);
	    err_propagate(retval);
	    /// add to sum
	    retval = imu_comm_add_data(&imu->tmp_filt, data);
	    err_propagate(retval);
	}
	j = circ_buff_prev_index(j,IMU_FRAME_BUFF_SIZE);
    }
    retval = imu_comm_raw2data(imu,NULL,&imu->tmp_filt, data);
    err_propagate(retval);
    return retval;
}

int imu_comm_get_filtered_unread(imu_t *imu, imu_data_t *data)
{
    int retval;
    if(!imu_comm_unread(imu))
    {
	err_check(ERROR_IMU_NO_UPDATES,"No unread data!");
    }
    retval = imu_comm_get_filtered(imu,data);
    err_propagate(retval);
    --imu->unread_data;
    return ERROR_OK;
}

int imu_comm_print_data(imu_data_t *data, FILE *stream){
    if(stream == NULL){
	stream = stdout;
    }
    log_tv_only(stream,data->timestamp);
    fprintf(stream,IMU_COMM_PRINT_DATA_FORMAT,
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

    log_tv_only(stream,frame->timestamp);
    fprintf(stream,IMU_COMM_PRINT_RAW_FORMAT,
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
 * If IMU setting were to be modified from imu_comm, the
 * current setting should be stored here.
 * //TODO use or remove
 *
 */
typedef struct imu_settings{
    // sampling frequency
    //    int fs;
    // sampling period
    //    double T;
    // sens index
    //    int acc_sens;
    //    int frame_width_bytes;
}imu_settings_t;

static void imu_comm_cast_data2raw(imu_raw_t *out, imu_raw_db_t *in)
{
    int i;
    for(i=0; i<3; ++i)
    {
	out->acc[i]  = (int16_t) in->acc->m_full[i];
	out->gyro[i] = (int16_t) in->gyro->m_full[i];
	out->magn[i] = (int16_t) in->magn->m_full[i];
    }
    if(in->T_us < 0.0)
    {
	err_log_double("WARN: Cannot cast negative T_us, using 0.0:", in->T_us);
	out->T_us = 0;
    }
    else
    {
	out->T_us = (uint16_t) in->T_us;
    }
    if(in->temp < 0.0)
    {
	err_log_double("WARN: Cannot cast negative temp, using 0.0:", in->temp);
	out->temp = 0;
    }
    else
    {
	out->temp = (uint16_t) in->temp;
    }
    if(in->alt < 0)
    {
	err_log_double("WARN: Cannot cast negative pressure, using 0.0:", in->alt);
	out->pres = 0;
    }
    else
    {
	out->pres = (uint32_t) in->alt;
    }
    out->timestamp = in->timestamp;
}

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
