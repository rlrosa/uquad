/**
 * main: uquad autopilot software
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
 * @file   main.c
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 11:08:44 2012
 *
 * @brief  uquad autopilot software
 *
 * See src/main/README for information regarding how to run, configure, etc.
 *
 */
#include <uquad_config.h> // debug mode is emabled/disabled here
#if DEBUG
/**
 * Show timing information on console.
 *
 * NOTE: Should not be enabled when running real time, since
 *       ssh traffic will severely deteriorate performance.
 */
#define TIMING             0
#define TIMING_KALMAN      (1 && TIMING)
#define TIMING_IMU         (0 && TIMING)

/**
 * The following defines configures what data should be save to
 * log files.
 * Each item will reserve LOG_RAM_MB megabytes of ram. A trade-off
 * must be made between how many logs are desired and the time each
 * one should last.
 *
 * LOG_ERR          : Everything printed to stderr.
 * LOG_W            : Output of control() successfully sent to mot_control
 * LOG_W_CTRL       : Output of control()
 * LOG_IMU_RAW      : Raw data from IMU.
 * LOG_IMU_DATA     : Converted data from IMU.
 * LOG_IMU_AVG      : Converted and filtered data from IMU.
 * LOG_X_HAT        : State estimation.
 * LOG_GPS          : GPS data.
 * LOG_KALMAN_INPUT : Converted IMU data, with continuity applied to theta.
 * LOG_TV           : Each time user hits RET, the timestamp will be recorded.
 * LOG_T_ERR        : Timing errors (too slow).
 * LOG_INT          : Error accumulated by integral term in control()
 * LOG_BUKAKE       : Print a bunch of data on console. Usefull if out of RAM.
 *
 */
#define LOG_ERR          1
#define LOG_W            1
#define LOG_W_CTRL       0
#define LOG_IMU_RAW      0
#define LOG_IMU_DATA     0
#define LOG_IMU_AVG      0
#define LOG_X_HAT        1
#define LOG_GPS          1
#define LOG_KALMAN_INPUT 1
#define LOG_TV           0
#define LOG_T_ERR        0
#define LOG_INT          (0 && CTRL_INTEGRAL)
#define LOG_BUKAKE       0
/**
 * Access to SD card on beagleboard has proven to be VERY slow,
 * using an external flash drive is much faster.
 */
#define LOG_DIR_DEFAULT    "/media/sda1/"

#define LOG_W_NAME         "w"
#define LOG_W_CTRL_NAME    "w_ctrl"

#define LOG_ERR_NAME       "err"
#define LOG_IMU_RAW_NAME   "imu_raw"
#define LOG_IMU_DATA_NAME  "imu_data"
#define LOG_IMU_AVG_NAME   "imu_avg"
#define LOG_X_HAT_NAME     "x_hat"
#define LOG_KALMAN_IN_NAME "kalman_in"
#define LOG_GPS_NAME       "gps"
#define LOG_TV_NAME        "tv"
#define LOG_T_ERR_NAME     "t_err"
#define LOG_INT_NAME       "int"
#define LOG_BUKAKE_NAME    "buk"

/**
 * Realtime logging to SD card has proven to destroy performance, so
 * running main.c with realtime logs to SD card is not acceptable.
 *
 * Logging to external flash drive is ok reasonable, but it is recomended to
 * save logs in ram, and dump after program execution is completed. To achieve
 * this, set LOG_RAM_MB to the number of megabytes that logger are allowed to
 * allocate.
 *
 * If LOG_RAM_MB == 0, then realtime logging will be used.
 *
 * NOTE: Logs grow at different rates. If available RAM becomes a limitation,
 *       then the bigger logs can be given more RAM, and the smaller ones less
 *       to get complete data for a longer period of time.
 */
#define LOG_RAM_MB         10 // If 0, will log to disk (SD, flash drive, etc).
#endif

#if LOG_BUKAKE
#define LOG_BUKAKE_STDOUT  1
#endif

#include <manual_mode.h>
//#include <ncurses.h> // Timing unnacceptable when using (linking) ncurses!
#include <stdio.h>
#include <uquad_error_codes.h>
#include <uquad_check_net.h>
#include <uquad_types.h>
#include <macros_misc.h>
#include <uquad_aux_io.h>
#include <imu_comm.h>
#include <mot_control.h>
#include <uquad_kalman.h>
#include <control.h>
#include <path_planner.h>
#include <uquad_logger.h>
#if USE_GPS
#include <uquad_gps_comm.h>
#endif

#include <sys/signal.h>   // for SIGINT and SIGQUIT
#include <sys/wait.h>     // for waitpid()
#include <unistd.h>       // for STDIN_FILENO

#define QUIT           27
#define RAMP_DOWN      'q' // This is unstable, currently disabled.

#define UQUAD_HOW_TO     "./main <imu_device> /path/to/log/"
#define MAX_ERRORS       10 // Abort if more than MAX_ERRORS errors.
#define MAX_NO_UPDATES_S 1  // Abort if more than MAX_NO_UPDATES_S sec without data.
#define FIXED            10 // Consider system OK after FIXED loops without errors.

/**
 * Before running, we'll check if we have a stable sampling period.
 * If something is wrong, for example some program is hogging the CPU,
 * then we cannot run, a stable sampling period is required.
 *
 * OL_TS_STABIL: If != 0, then will wait OL_TS_STABIL+DISCARD_RUNS samples
 *               before using IMU.
 *               NOTE: This does not imply stable sampling period was achieved.
 * DISCARD_RUNS: Wait for this number of samples at a steady Ts before running.
 *               If !OL_TS_STABIL, then description above applies. Else, will
 *               wait for DISCARD_RUNS samples at a stable sampling period, and
 *               will NOT run until this condition is achieved (or timeout).
 * STARTUP_TO_S: Timeout waiting for stable IMU data [s]
 *
 * NOTE: If IMU_COMM_FAKE, then timing was already taken care of, and log file
 *       will start from the first sample used to calibrate, so no data should
 *       be discarded and timing should not be a concern.
 */
#define STARTUP_TO_S   10
#define IMU_TS_OK      -1
#if !IMU_COMM_FAKE
#define OL_TS_STABIL   0
#define DISCARD_RUNS   (50+OL_TS_STABIL)
#else // !IMU_COMM_FAKE
#define OL_TS_STABIL   1
#define DISCARD_RUNS   1
#endif // !IMU_COMM_FAKE

/**
 * After IMU calibration, STARTUP_SAMPLES are used to
 * ramp motors from mot->w_min to mot->w_hover.
 * It will take STARTUP_SAMPLES*TS_DEFAULT_US/1e6 seconds.
 * This is implemented by running everything as if we were
 * hovering, but use a lower speed setting for the motors.
 *
 * Why?
 * If we tell Kalman that we have a speed lower than mot->w_hover,
 * the the prediction will be that be will fall, and this will result
 * in an attempt to increase power.
 * Evenly lowering the motor power will not affect the differences between
 * the motors, and these differences are what the controller has determined
 * necessary to mantain a stable attitud.
 * So we get a stable ramp :)
 *
 * If RAMP_LINEAR, then linear ramp will be used, otherwise a x^2 ramp will
 * be used.
 */
#define STARTUP_SAMPLES 100
#define RAMP_LINEAR     0

/**
 * To avoid a violent stop, pull motor speed down until
 * speed is:
 *   ((mot->w_hover - mot->w_min) >> 1)
 * This will take RAMP_DOWN_SAMPLES*TS_DEFAULT_US/1e6 seconds,
 * and when completed, the motors will be shut off.
 *
 * This, in theory, could be replaced by SV_Z=0 and letting the
 * control algorithm do its thing, but the estimation of the
 * elevation has a 0.5-1m error, so this approach will allow us
 * to force to to land.
 *
 * NOTES: This should not be used if the quad is far from the ground,
 *        since it will eventually shut off the motors, and if the
 *        altitud changes a lot, then the controller will attempt to
 *        fix it, going against the purpose of a ramp down.
 */
#define RAMP_DOWN_SAMPLES 300

/**
 * Display current state estimation on console every X_HAT_STDOUT samples.
 * If set to 0, then nothing will be displayed.
 *
 * If X_HAT_STDOUT is 1, then if SVNAME the name of each element of the state
 * vector will be printed in a line before the values, this takes up space
 * but makes it easier to visualize things.
 */
#define X_HAT_STDOUT       0
#define SV_NAME            0

/**
 * Frequency at which motor controller is updated
 * Must be at least MOT_UPDATE_MAX_US
 *
 */
#define MOT_UPDATE_T MOT_UPDATE_MAX_US

enum state{
    ST_RAMPING_UP,
    ST_RUNNING,
    ST_RAMPING_DOWN,
    ST_STATE_COUNT};
typedef enum state state_t;

/// Global structs
static imu_t *imu          = NULL;
static kalman_io_t *kalman = NULL;
static uquad_mot_t *mot    = NULL;
static io_t *io            = NULL;
static ctrl_t *ctrl        = NULL;
static path_planner_t *pp  = NULL;

#if USE_GPS && !GPS_FAKE
static gps_t *gps          = NULL;
#endif
/// Global var
uquad_mat_t *w = NULL, *wt = NULL, *w_forced = NULL;
uquad_mat_t *x = NULL;
imu_data_t imu_data;
struct timeval tv_start = {0,0};
state_t uquad_state = ST_RAMPING_UP;
pid_t check_net_chld = -1;
uquad_bool_t
/**
 * Flag to allow state estimation to keep running
 * after abort, without controlling motors.
 */
interrupted = false,
/**
 * Flag to indicate that main read/estimate/control loop
 * is running.
 */
running     = false;

#if USE_GPS
gps_comm_data_t *gps_dat;
#else
#define gps_dat NULL
#endif
/// Logs
#if DEBUG
#if LOG_ERR
FILE *log_err = NULL;
#endif // LOG_ERR
#if LOG_IMU_RAW
FILE *log_imu_raw;
#endif //LOG_IMU_RAW
#if LOG_IMU_DATA
FILE *log_imu_data;
#endif //LOG_IMU_DATA
#if LOG_IMU_AVG
FILE *log_imu_avg;
#endif //LOG_IMU_AVG
#if LOG_W
FILE *log_w = NULL;
#endif //LOG_W
#if LOG_W
FILE *log_w_ctrl = NULL;
#endif //LOG_W
#if LOG_X_HAT
FILE *log_x_hat = NULL;
uquad_mat_t *x_hat_T = NULL;
#endif //LOG_X_HAT
#if LOG_KALMAN_INPUT
FILE *log_kalman_in = NULL;
#endif //LOG_KALMAN_INPUT
#if LOG_GPS && USE_GPS
FILE *log_gps = NULL;
#endif // LOG_GPS && USE_GPS
#if LOG_BUKAKE && !LOG_BUKAKE_STDOUT
FILE *log_bukake = NULL;
#endif //LOG_BUKAKE
#if LOG_TV
FILE *log_tv = NULL;
#endif // LOG_TV
#if LOG_T_ERR
FILE *log_t_err = NULL;
#endif // LOG_T_ERR
#if LOG_INT
FILE *log_int = NULL;
#endif // LOG_INT
#endif //DEBUG

/**
 * Clean up and close
 *
 */
void quit()
{
    int retval;
    struct timeval
	tv_tmp,
	tv_diff;
    if(mot != NULL)
    {
	retval = mot_deinit(mot);
	if(retval != ERROR_OK)
	{
	    err_log("Could not close motor driver correctly!");
	}
	mot = NULL;
	gettimeofday(&tv_tmp, NULL);
	retval = uquad_timeval_substract(&tv_diff,tv_tmp,tv_start);
	if(retval > 0)
	{
	    err_log_tv("Motors killed!",tv_diff);
	}
    }
    if(interrupted && running)
    {
	/**
	 * Keep IMU+kalman running to log data
	 *
	 */
	running = false;
	return;
    }

    /* clear(); */
    /* endwin(); */
    gettimeofday(&tv_tmp, NULL);
    uquad_timeval_substract(&tv_diff,tv_tmp,tv_start);
    err_log_tv("main deinit started...",tv_diff);

    /// IO manager
    retval = io_deinit(io);
    if(retval != ERROR_OK)
    {
	err_log("Could not close IO correctly!");
    }

    /// IMU
    retval = imu_comm_deinit(imu);
    if(retval != ERROR_OK)
    {
	err_log("Could not close IMU correctly!");
    }

    /// Kalman
    kalman_deinit(kalman);

    /// Control module
    control_deinit(ctrl);

    /// Path planner module
    pp_deinit(pp);

    /// Global vars
    uquad_mat_free(w);
    uquad_mat_free(w_forced);
    uquad_mat_free(wt);
    uquad_mat_free(x);
    uquad_mat_free(imu_data.acc);
    uquad_mat_free(imu_data.gyro);
    uquad_mat_free(imu_data.magn);
#if USE_GPS
    gps_comm_data_free(gps_dat);
#if !GPS_FAKE
    gps_comm_deinit(gps);
#endif // !GPS_FAKE
#endif

    // Logs
#if DEBUG
#if LOG_ERR
    uquad_logger_remove(log_err);
#endif // LOG_ERR
#if LOG_IMU_RAW
    uquad_logger_remove(log_imu_raw);
#endif //LOG_IMU_RAW
#if LOG_IMU_DATA
    uquad_logger_remove(log_imu_data);
#endif //LOG_IMU_DATA
#if LOG_IMU_AVG
    uquad_logger_remove(log_imu_avg);
#endif //LOG_IMU_AVG
#if LOG_W
    uquad_logger_remove(log_w);
#endif //LOG_W
#if LOG_W_CTRL
    uquad_logger_remove(log_w_ctrl);
#endif //LOG_W_CTRL
#if LOG_X_HAT
    uquad_logger_remove(log_x_hat);
    uquad_mat_free(x_hat_T);
#endif //LOG_X_HAT
#if LOG_KALMAN_INPUT
    uquad_logger_remove(log_kalman_in);
#endif
#if LOG_GPS && USE_GPS
    uquad_logger_remove(log_gps);
#endif //LOG_GPS && USE_GPS
#if LOG_BUKAKE
#if !LOG_BUKAKE_STDOUT
    uquad_logger_remove(log_bukake);
#else
#define log_bukake stdout
#endif // !LOG_BUKAKE_STDOUT
#endif // LOG_BUKAKE
#if LOG_TV
    uquad_logger_remove(log_tv);
#endif // LOG_TV
#if LOG_T_ERR
    uquad_logger_remove(log_t_err);
#endif // LOG_T_ERR
#if LOG_INT
    uquad_logger_remove(log_int);
#endif // LOG_INT
#endif //DEBUG

    //TODO deinit everything?
    exit(retval);
}

/**
 * Save configuration to log file.
 *
 */
void log_configuration(void)
{
    err_log_eol();
    err_log("-- -- -- -- -- -- -- --");
    err_log("main.c configuration:");
    err_log("-- -- -- -- -- -- -- --");
    err_log_num("DEBUG",DEBUG);
    err_log_num("KALMAN_BIAS",KALMAN_BIAS);
    err_log_num("CTRL_INTEGRAL",CTRL_INTEGRAL);
    err_log_num("CTRL_INTEGRAL_ANG",CTRL_INTEGRAL_ANG);
    err_log_num("FULL_CONTROL",FULL_CONTROL);
    err_log_num("USE_GPS",USE_GPS);
    err_log_num("GPS_FAKE",GPS_FAKE);
    err_log_num("IMU_COMM_FAKE",IMU_COMM_FAKE);
    err_log_num("OL_TS_STABIL",OL_TS_STABIL);
    err_log_num("CTRL_TS",CTRL_TS);
    err_log_num("DYNAMIC_COV",DYNAMIC_COV);
    err_log_double("MASA_DEFAULT",MASA_DEFAULT);
    err_log("-- -- -- -- -- -- -- --");
    err_log_eol();
}

void uquad_sig_handler(int signal_num)
{
    err_log_num("Caught signal:",signal_num);
    interrupted = true;
    quit();
}

void uquad_conn_lost_handler(int signal_num)
{
    pid_t p;
    int status;
    p = waitpid(-1, &status, WNOHANG);
    if(p == check_net_chld && running)
    {
	err_log_num("WARN: check_net client died, will kill motors! sig num:", signal_num);
	quit();
    }
}

void sanity_check(imu_data_t *imu_data, uquad_mat_t *x_hat, uquad_bool_t *insane)
{
    *insane = false;
    if((imu_data != NULL) && imu_data->temp > SANITY_MAX_TEMP)
    {
	*insane = true;
	err_log_double("WARN! Sanity check alert\t-\ttemp!\t", imu_data->temp);
	return;
    }
    if((x_hat != NULL) && (uquad_abs(x_hat->m_full[SV_PSI]) > SANITY_MAX_PSI))
    {
	err_log_double("WARN! Sanity check alert\t-\tpsi!\t", x_hat->m_full[SV_PSI]);
	*insane = true;
	return;
    }
    if((x_hat != NULL) && (uquad_abs(x_hat->m_full[SV_PHI]) > SANITY_MAX_PHI))
    {
	err_log_double("WARN! Sanity check alert\t-\tpsi!\t", x_hat->m_full[SV_PHI]);
	*insane = true;
	return;
    }
}

int main(int argc, char *argv[]){
    int
	retval = ERROR_OK,
	i,
	imu_ts_ok   = 0,
	runs_imu    = 0,
	runs_kalman = 0,
	runs_down   = 0,
	time_ret    = 0,
	insane      = 0,
	skipped     = 0,
	ctrl_samples= 0,
#if X_HAT_STDOUT
	x_hat_cnt   = 0,
#endif // X_HAT_STDOUT
	ts_error    = 0,
	err_imu     = ERROR_OK,
	err_gps     = ERROR_OK;
    char
	*device_imu,
	*device_gps,
	*log_path;
    double
	dtmp;
    unsigned long
	kalman_loops   = 0,
	ts_error_wait  = 0;
    unsigned char
	tmp_buff[2] = {0,0};

    uquad_bool_t
	read_ok       = false,
	write_ok      = false,
	imu_update    = false,
	reg_stdin     = true,
        aux_bool      = false,
	ctrl_outdated = false,
	manual_mode   = false;
    struct timeval
	tv_tmp, tv_diff,
	tv_last_m_cmd,
	tv_last_ramp,
	tv_last_kalman,
	tv_last_imu,
	tv_last_frame,
	tv_imu_stab_init,
	tv_gps_last;
#if IMU_COMM_FAKE
    struct timeval tv_imu_fake;
#endif // IMU_COMM_FAKE
#if LOG_T_ERR
    struct timeval tv_timing_off;
#endif // LOG_T_ERR

    // Catch signals
    signal(SIGINT, uquad_sig_handler);
    signal(SIGQUIT, uquad_sig_handler);

    retval = gettimeofday(&tv_start,NULL);
    err_log_std(retval);
    tv_last_ramp  = tv_start;
    tv_last_m_cmd = tv_start;
    uquad_bool_t gps_update = false;
#if USE_GPS
    tv_gps_last   = tv_start;
#if !GPS_FAKE
    uquad_bool_t reg_gps = true;
#else // !GPS_FAKE
#if GPS_RAND
    unsigned int iseed = (unsigned int)tv_start.tv_usec*tv_start.tv_sec;
    srand (iseed);
#endif // GPS_RAND
#endif // !GPS_FAKE
#endif // USE_GPS
#if TIMING
    struct timeval
	tv_pgm,
	tv_last_io_ok;
    gettimeofday(&tv_last_io_ok,NULL);
    gettimeofday(&tv_pgm,NULL);
#endif
#if TIMING_IMU
    struct timeval tv_last_imu_read, tv_imu_start;
    gettimeofday(&tv_last_imu_read,NULL);
#endif
    int count_err = 0, count_ok = FIXED;

#if LOG_IMU_RAW || LOG_IMU_DATA
    imu_raw_t imu_frame;
    struct timeval tv_raw_sample;
#endif // LOG_IMU_RAW || LOG_IMU_DATA
#if LOG_IMU_AVG
    struct timeval tv_imu_avg;
#endif // LOG_IMU_AVG

    /**
     * Init curses library, used for user input
     */
    /* initscr();  // init curses lib */
    /* cbreak();   // get user input without waiting for RET */
    /* noecho();   // do no echo user input on screen */
    /* timeout(0); // non-blocking reading of user input */
    /* refresh();  // show output on screen */

    if(argc<4)
    {
	err_log(UQUAD_HOW_TO);
	exit(1);
    }
    else
    {
	device_imu = argv[1];
	log_path   = argv[2];
	device_gps = argv[3];
    }

    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    /// Init
    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

    /// Logs
#if DEBUG
#if LOG_ERR
    log_err = uquad_logger_add(LOG_ERR_NAME, log_path, LOG_RAM_MB);
    if(log_err == NULL)
    {
	err_log("Failed to open log_imu_raw!");
	quit();
    }
    /**
     * Re-route stderr to log file.
     * This is required, since errors in uquad_error_codes.h use macros
     * that log to stderr. We want errors in a log file, so in order to
     * get every message from every module, to the log we need to re-define stderr.
     */
    stderr = log_err;
#endif // LOG_ERR
#if LOG_IMU_RAW
    log_imu_raw = uquad_logger_add(LOG_IMU_RAW_NAME, log_path, LOG_RAM_MB);
    if(log_imu_raw == NULL)
    {
	err_log("Failed to open log_imu_raw!");
	quit();
    }
#endif //LOG_IMU_RAW
#if LOG_IMU_DATA
    log_imu_data = uquad_logger_add(LOG_IMU_DATA_NAME, log_path, LOG_RAM_MB);
    if(log_imu_data == NULL)
    {
	err_log("Failed to open log_imu_data!");
	quit();
    }
#endif //LOG_IMU_DATA
#if LOG_IMU_AVG
    log_imu_avg = uquad_logger_add(LOG_IMU_AVG_NAME, log_path, LOG_RAM_MB);
    if(log_imu_avg == NULL)
    {
	err_log("Failed to open log_imu_avg!");
	quit();
    }
#endif //LOG_IMU_AVG
#if LOG_W
    log_w = uquad_logger_add(LOG_W_NAME, log_path, LOG_RAM_MB);
    if(log_w == NULL)
    {
	err_log("Failed to open log_w!");
	quit();
    }
#endif //LOG_W
#if LOG_W_CTRL
    log_w_ctrl = uquad_logger_add(LOG_W_CTRL_NAME, log_path, LOG_RAM_MB);
    if(log_w_ctrl == NULL)
    {
	err_log("Failed to open log_w_ctrl!");
	quit();
    }
#endif //LOG_W_CTRL
#if LOG_X_HAT
    log_x_hat = uquad_logger_add(LOG_X_HAT_NAME, log_path, LOG_RAM_MB);
    if(log_x_hat == NULL)
    {
	err_log("Failed to open x_hat!");
	quit();
    }
#endif //LOG_X_HAT
#if LOG_KALMAN_INPUT
    log_kalman_in = uquad_logger_add(LOG_KALMAN_IN_NAME, log_path, LOG_RAM_MB);
    if(log_kalman_in == NULL)
    {
	err_log("Failed open kalman_in log!");
	quit();
    }
#endif
#if LOG_GPS && USE_GPS
    log_gps = uquad_logger_add(LOG_GPS_NAME, log_path, LOG_RAM_MB);
    if(log_gps == NULL)
    {
	err_log("Failed open kalman_in log!");
	quit();
    }
#endif //LOG_GPS && USE_GPS
#if LOG_BUKAKE && !LOG_BUKAKE_STDOUT
    log_bukake = uquad_logger_add(LOG_BUKAKE_NAME, log_path, LOG_RAM_MB);
    if(log_bukake == NULL)
    {
	err_log("Failed open kalman_in log!");
	quit();
    }
#endif //LOG_BUKAKE && !LOG_BUKAKE_STDOUT
#if LOG_TV
    log_tv = uquad_logger_add(LOG_TV_NAME, log_path, LOG_RAM_MB);
    if(log_tv == NULL)
    {
	err_log("Failed to open tv_log!");
	quit();
    }
#endif // LOG_TV
#if LOG_T_ERR
    log_t_err = uquad_logger_add(LOG_T_ERR_NAME, log_path, LOG_RAM_MB);
    if(log_t_err == NULL)
    {
	err_log("Failed to open t_err_log!");
	quit();
    }
#endif // LOG_T_ERROR
#if LOG_INT
    log_int = uquad_logger_add(LOG_INT_NAME, log_path, LOG_RAM_MB);
    if(log_int == NULL)
    {
	err_log("Failed to open t_err_log!");
	quit();
    }
#endif // LOG_INT
#endif //DEBUG

#if CHECK_NET_BYPASS
    check_net_chld = -1;
    err_log("WARN! Not using check_net...");
#else
    /**
     * Start a child process that will ping use TCP packages
     * to verify connectivity with server (laptop)/
     * If connection is lost, then motors should be shutoff.
     */
    err_log("Will connect to check_net server...");
    check_net_chld = uquad_check_net_client(CHECK_NET_SERVER_IP,
					    CHECK_NET_PORT,
					    false);
    if(check_net_chld < 0)
    {
	quit_log_if(ERROR_FAIL,"Failed to connect to check_net server!");
    }
    else
    {
	signal(SIGCHLD, uquad_conn_lost_handler);
    }
#endif // CHECK_NET_BYPASS

    /// IO manager
    io = io_init();
    if(io==NULL)
    {
	quit_log_if(ERROR_FAIL,"io init failed!");
    }

    /// Motors
    mot = mot_init(false);
    if(mot == NULL)
    {
	quit_log_if(ERROR_FAIL,"mot init failed!");
    }

    /// IMU
    imu = imu_comm_init(device_imu);
    if(imu == NULL)
    {
	quit_log_if(ERROR_FAIL,"imu init failed!");
    }

#if USE_GPS
    gps_dat = gps_comm_data_alloc();
    if(gps_dat == NULL)
    {
	err_log("Failed to allocate GPS!...");
	quit();
    }
#if !GPS_FAKE
    /// GPS
    gps = gps_comm_init(device_gps);
    if(gps == NULL)
    {
	quit_log_if(ERROR_FAIL,"gps init failed!");
    }
    else
    {
	uquad_bool_t got_fix;
	struct timeval tv_gps_init_t_out;
	tv_gps_init_t_out.tv_sec = GPS_INIT_TOUT_S;
	tv_gps_init_t_out.tv_usec = GPS_INIT_TOUT_US;
	/* if(device_gps != NULL) */
	/* { */
	/*     //	    retval = gps_comm_read(gps, &got_fix, &tv_start); */
	/*     retval = gps_comm_read(gps, &got_fix); */
	/*     quit_if(retval); */
	/*     if(!got_fix) */
	/* 	quit_log_if(ERROR_READ, "Failed to read from log file!"); */
	/* } */
	/* else */
	{
	    err_log("Waiting for GPS fix...");
	    retval = gps_comm_wait_fix(gps,&got_fix,&tv_gps_init_t_out);
	    quit_if(retval);
	    if(!got_fix)
	    {
		quit_log_if(ERROR_GPS,"Failed to get GPS fix!");
	    }
	    err_log("GPS fix ok.");
	}

	/**
	 * Now get initial position from GPS.
	 * This information will be used as startpoint for the kalman
	 * estimator if no other GPS updates are received during IMU
	 * warmup.
	 */
	retval = gps_comm_get_data(gps, gps_dat);
	quit_log_if(retval,"Failed to get initial position from GPS!");
	retval = gps_comm_set_0(gps,gps_dat);
	quit_if(retval);
	retval = gps_comm_get_0(gps, gps_dat);
	quit_if(retval);

	/**
	 * Inform IMU about starting altitude, in order to allow barometer
	 * data to match GPS altitud estimation
	 *
	 */
	/* GPS ALTITUDE INFORMATION IS IGNORED */
	/* if(imu == NULL) */
	/* { */
	/*     quit_log_if(ERROR_FAIL,"IMU must be initialized before gps!"); */
	/* } */
	/* retval = imu_comm_set_z0(imu,gps_dat->pos->m_full[2]); */
	/* quit_if(retval); */
    }
#endif // !GPS_FAKE
#endif // USE_GPS

    /// Kalman
    kalman = kalman_init();
    if(kalman == NULL)
    {
	quit_log_if(ERROR_FAIL,"kalman init failed!");
    }

    /// Path planner module
    pp = pp_init();
    if(pp == NULL)
    {
	quit_log_if(ERROR_FAIL,"path planner init failed!");
    }

    /// Control module
    ctrl = control_init();
    if(ctrl == NULL)
    {
	quit_log_if(ERROR_FAIL,"control init failed!");
    }

    /// Global vars
    w = uquad_mat_alloc(4,1);        // Current angular speed [rad/s]
    wt = uquad_mat_alloc(1,4);        // tranpose(w)
    w_forced = uquad_mat_alloc(4,1);
    x = uquad_mat_alloc(1,STATE_COUNT);   // State vector
    retval = imu_data_alloc(&imu_data);
    quit_if(retval);

    if( x == NULL || w == NULL || wt == NULL || w_forced == NULL)
    {
	err_log("Cannot run without x or w, aborting...");
	quit();
    }

#if LOG_X_HAT
    x_hat_T = uquad_mat_alloc(1,STATE_COUNT+STATE_BIAS);
    if(x_hat_T == NULL)
    {
	err_log("Failed alloc x_hat_T!");
	quit();
    }
#endif // LOG_X_HAT

    /**
     * Save configuration to log file
     *
     */
    retval = kalman_dump(kalman, log_err);
    quit_log_if(retval,"Failed to save Kalman configuration!");
    retval = control_dump(ctrl, log_err);
    quit_log_if(retval,"Failed to save Kalman configuration!");
    log_configuration();

    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    /// Register IO devices
    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    // imu
    int imu_fds;
    retval = imu_comm_get_fds(imu,& imu_fds);
    quit_log_if(retval,"Failed to get imu fds!!");
    retval = io_add_dev(io,imu_fds);
    quit_log_if(retval,"Failed to add imu to dev list");
#if USE_GPS && !GPS_FAKE
    // gps
    int gps_fds;
    if(gps != NULL)
    {
	gps_fds = gps_comm_get_fd(gps);
	retval = io_add_dev(io,gps_fds);
	quit_log_if(retval,"Failed to add gps to dev list");
    }
#endif
    // stdin
    retval = io_add_dev(io,STDIN_FILENO);
    quit_log_if(retval, "Failed to add stdin to io list");

    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    /// Startup your engines...
    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    retval = mot_set_idle(mot);
    quit_log_if(retval, "Failed to startup motors!");

    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    /// Poll n read loop
    /// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    retval = ERROR_OK;
    //    poll_n_read:
    gettimeofday(&tv_tmp,NULL);
    uquad_timeval_substract(&tv_diff,tv_tmp,tv_start);
    err_log_tv("Entering while:",tv_diff);
#if !IMU_COMM_FAKE
    // clear imu input buffer
    err_log("Clearing IMU input buffer...");
    while(read(imu->device,tmp_buff,1) > 0);
#endif // !IMU_COMM_FAKE
#if !GPS_FAKE
    // clear gps input buffer
    err_log("Clearing GPS input buffer...");
    while(read(gps->fd,tmp_buff,1) > 0);
#endif // !GPS_FAKE
    running = true;
    while(1)
    {
	fflush(stdout); // flushes output, but does not display on screen
	//	refresh();      // displays flushed output on screen
	if((runs_imu == IMU_TS_OK) &&
	   (retval != ERROR_OK  ||
	    err_imu != ERROR_OK ||
	    err_gps != ERROR_OK))
	{
	    count_ok = 0;
	    if(count_err++ > MAX_ERRORS)
	    {
		gettimeofday(&tv_tmp,NULL);
		time_ret = uquad_timeval_substract(&tv_diff, tv_tmp, tv_start);
		err_log_tv("Too many errors! Aborting...",tv_diff);
		quit();
		/// program ends here
	    }
	}
	else
	{
	    if(count_ok < FIXED)
	    {
		count_ok++;
	    }
	    else if(count_err > 0)
	    {
		// forget abour error
		err_log_num("Recovered! Errors:",count_err);
		count_err = 0;
	    }
	}
	// reset error/update indicators
	err_imu = ERROR_OK;
	//gps_update = false; // This is cleared within the loop
	err_gps = ERROR_OK;
	retval = ERROR_OK;

	retval = io_poll(io);
	quit_log_if(retval,"io_poll() error");

	/// -- -- -- -- -- -- -- --
	/// Check stdin
	/// -- -- -- -- -- -- -- --
	if(reg_stdin)
	{
	    retval = io_dev_ready(io,STDIN_FILENO,&read_ok,NULL);
	    log_n_continue(retval, "Failed to check stdin for input!");
	    if(!read_ok)
		goto end_stdin;
	    retval = fread(tmp_buff,sizeof(unsigned char),1,stdin);
	    if(retval <= 0)
	    {
		log_n_jump(ERROR_READ, end_stdin,"No user input detected!");
	    }
	    if(tmp_buff[0] == RAMP_DOWN)
	    {
		if(uquad_state != ST_RUNNING)
		{
		    if(uquad_state == ST_RAMPING_DOWN)
		    {
			err_log("WARN: Aborting ramp down!");
			uquad_state = ST_RUNNING;
		    }
		    else
		    {
			err_log("Will not apply landing if not in ST_RUNNING!");
		    }
		}
		else
		{
		    err_log("WARN: Ramping down motors does not work correctly, ignoring...");
		    //		    err_log("WARN: Ramping down motors...");
		    //		    uquad_state = ST_RAMPING_DOWN;
		}
	    }
#if LOG_TV
	    // save to log file
	    if(tmp_buff[0] == '\n')
	    {
		gettimeofday(&tv_tmp,NULL);
		time_ret = uquad_timeval_substract(&tv_diff,tv_tmp,tv_start);
		if(time_ret <= 0)
		{
		    err_log("Absurd timing!");
		}
		log_tv(log_tv, "RET:", tv_diff);
		fflush(log_tv);
		err_log_tv("Saving timestamp...",tv_diff);
	    }
#endif
	    if(running && (tmp_buff[0] != '\n'))
	    {
		if(time_ret <= 0)
		{
		    err_log("Absurd timing!");
		}
		dtmp = 0.0;
		if(tmp_buff[0] == QUIT)
		{
		    err_log("Terminating program based on user input...");
		    quit();
		    // never gets here
		}
		if(!manual_mode && tmp_buff[0] != MANUAL_MODE)
		{
		    err_log("Manuel mode DISABLED, enable with 'm'. Ignoring input...");
		}
		else
		{
		    switch(tmp_buff[0])
		    {
		    case MANUAL_MODE:
			// switch manual mode on/off
			if(pp == NULL)
			{
			    err_log("Cannot enable manual mode, path planner not setup!");
			}
			else
			{
			    manual_mode = !manual_mode;
			    if(manual_mode)
			    {
				err_log_tv("Manuel mode ENABLED!",tv_diff);
			    }
			    else
			    {
				err_log_tv("Manuel mode DISABLED!",tv_diff);
			    }
			}
			break;
		    case MANUAL_PSI_INC:
			if(pp == NULL)
			{
			    err_log("Path planner not setup!");
			}
			pp->sp->x->m_full[SV_PSI] += MANUAL_EULER_STEP;
			break;
		    case MANUAL_PSI_DEC:
			if(pp == NULL)
			{
			    err_log("Path planner not setup!");
			}
			pp->sp->x->m_full[SV_PSI] -= MANUAL_EULER_STEP;
			break;
		    case MANUAL_PHI_INC:
			if(pp == NULL)
			{
			    err_log("Path planner not setup!");
			}
			pp->sp->x->m_full[SV_PHI] += MANUAL_EULER_STEP;
			break;
		    case MANUAL_PHI_DEC:
			if(pp == NULL)
			{
			    err_log("Path planner not setup!");
			}
			pp->sp->x->m_full[SV_PHI] -= MANUAL_EULER_STEP;
			break;
		    case MANUAL_THETA_INC:
			if(pp == NULL)
			{
			    err_log("Path planner not setup!");
			}
			pp->sp->x->m_full[SV_THETA] += MANUAL_EULER_STEP;
			break;
		    case MANUAL_THETA_DEC:
			if(pp == NULL)
			{
			    err_log("Path planner not setup!");
			}
			pp->sp->x->m_full[SV_THETA] -= MANUAL_EULER_STEP;
			break;
		    case MANUAL_WEIGHT:
			retval = mot_update_w_hover(mot, MASA_DEFAULT);
			quit_log_if(retval, "Failed to update weight!");
			break;
		    case MANUAL_WEIGHT_INC:
			if(pp == NULL)
			{
			    err_log("Path planner not setup!");
			}
			dtmp = MANUAL_WEIGHT_STEP;
			break;
		    case MANUAL_WEIGHT_DEC:
			if(pp == NULL)
			{
			    err_log("Path planner not setup!");
			}
			dtmp = -MANUAL_WEIGHT_STEP;
			break;
		    case MANUAL_Z_INC:
			if(pp == NULL)
			{
			    err_log("Path planner not setup!");
			}
			pp->sp->x->m_full[SV_Z] += MANUAL_Z_STEP;
			break;
		    case MANUAL_Z_DEC:
			if(pp == NULL)
			{
			    err_log("Path planner not setup!");
			}
			pp->sp->x->m_full[SV_Z] -= MANUAL_Z_STEP;
			break;
		    default:
			err_log("Invalid input!");
			tmp_buff[0] = ERROR_INVALID_ARG;
			break;
		    }
		    if(dtmp != 0.0)
		    {
			retval = mot_update_w_hover(mot, mot->weight + dtmp);
			quit_log_if(retval, "Failed to update weight!");
			// display on screen
			log_tv_only(stdout,tv_diff);
			log_double(stdout,"Current w hover:",mot->w_hover);
			fflush(stdout);
		    }
		    else
		    {
			if(manual_mode && tmp_buff[0] != ERROR_INVALID_ARG)
			{
			    err_log("New setpoint:");
			    print_sv_name(stdout);
			    uquad_mat_dump_vec(pp->sp->x,stderr, true);
			}
		    }
		}
	    }
	}
	end_stdin:

	/// -- -- -- -- -- -- -- --
	/// Check IMU updates
	/// -- -- -- -- -- -- -- --
	retval = io_dev_ready(io,imu_fds,&read_ok,&write_ok);
	quit_log_if(retval,"io_dev_ready() error");
	if(read_ok)
	{
            if(imu_update)
            {
		skipped++;
		imu_update = false;
            }
#if TIMING_IMU
	    err_imu = gettimeofday(&tv_imu_start,NULL);
	    err_log_std(err_imu);
#endif // TIMING_IMU

	    err_imu = imu_comm_read(imu, &aux_bool);
	    jump_if(err_imu, end_imu);
	    if(!aux_bool)
	    {
		goto end_imu;
	    }
            // data may not be of direct use, may be calib
#if IMU_COMM_FAKE
	    // simulate delay (no delay when reading from txt)
	    retval = imu_comm_get_raw_latest(imu,&imu_frame);
	    log_n_jump(retval,end_log_imu,"could not get new frame...");
	    if(runs_imu == 0)
	    {
		tv_diff = imu_frame.timestamp;
	    }
	    else
	    {
		err_imu = uquad_timeval_substract(&tv_diff, imu_frame.timestamp, tv_imu_fake);
		if(err_imu < 0)
		{
		    err_log("Absurd fake IMU timing!");
		}
	    }
	    tv_imu_fake = imu_frame.timestamp;
	    if(tv_diff.tv_sec > 0)
		sleep(tv_diff.tv_sec);
	    usleep(tv_diff.tv_usec);
#endif // IMU_COMM_FAKE

	    err_imu = gettimeofday(&tv_tmp,NULL);
	    err_log_std(err_imu);
#if LOG_IMU_RAW || LOG_IMU_DATA
	    err_imu = uquad_timeval_substract(&tv_raw_sample,tv_tmp,tv_start);
	    if(err_imu < 0)
	    {
		err_log("Timing error!");
	    }
	    err_imu = ERROR_OK;
#endif // LOG_IMU_RAW || LOG_IMU_DATA

#if TIMING_IMU
            if(runs_kalman > 0)
            {
                err_imu = gettimeofday(&tv_tmp,NULL);
                err_log_std(err_imu);
                err_imu = uquad_timeval_substract(&tv_diff,tv_tmp,tv_last_imu_read);
                if(err_imu < 0)
                {
                    err_log("IMU Timing error!");
                }
                err_imu = ERROR_OK; // clear error
                tv_last_imu_read = tv_tmp;
                tv_pgm = tv_tmp;
                err_log_tv("IMU delay:", tv_diff);
            }
#endif

	    /// discard first samples
	    if(runs_imu != IMU_TS_OK)
	    {
		if(runs_imu == 0)
		{
		    err_log("Waiting for stable IMU sampling time...");
		    tv_last_frame = tv_start;
		    gettimeofday(&tv_imu_stab_init,NULL);
		}
		runs_imu++;
		err_imu = gettimeofday(&tv_tmp, NULL);
		err_log_std(err_imu);
		err_imu = uquad_timeval_substract(&tv_diff, tv_tmp, tv_last_frame);
		log_n_jump((err_imu < 0)?ERROR_TIMING:ERROR_OK,end_imu,"Absurd timing!");
		err_imu = in_range_us(tv_diff, TS_MIN, TS_MAX);
		if(err_imu == 0 || OL_TS_STABIL)
		{
		    if(imu_ts_ok++ >= DISCARD_RUNS)
		    {
			if(runs_imu > DISCARD_RUNS + 10)
			{
			    quit_log_if(ERROR_IO, "WARN: IMU stabilization took too long, something is not working correctly...");
			}
			err_log_num("IMU: Frames read out during stabilization:",runs_imu);
			runs_imu = IMU_TS_OK; // so re-entry doesn't happen
			tv_last_imu = tv_tmp;
			tv_gps_last    = tv_last_imu;
			tv_last_kalman = tv_last_imu;
			err_imu = uquad_timeval_substract(&tv_diff,tv_last_imu,tv_start);
			if(err_imu < 0)
			{
			    log_n_jump(err_imu,end_imu,"Absurd IMU startup time!");
			}
			err_log_tv("IMU startup completed, starting calibration...", tv_diff);
		    }
		}
		else
		{
		    // We want consecutive stable samples
		    imu_ts_ok = 0;
		}
		tv_last_frame = tv_tmp;
		// check timeout
		err_imu =  uquad_timeval_substract(&tv_diff, tv_tmp, tv_imu_stab_init);
		if(err_imu < 0)
		{
		    err_log("Timing error!");
		}
		err_imu = ERROR_OK; // clear error, doesn't matter here.

		if(tv_diff.tv_sec > STARTUP_TO_S)
		{
		    quit_log_if(ERROR_IO, "Timed out waiting for stable IMU samples...");
		}
		goto end_imu;
	    }

	    /// check calibration status
	    if(imu_comm_get_status(imu) == IMU_COMM_STATE_CALIBRATING)
	    {
		// if calibrating, then data should not be used.
#if LOG_IMU_RAW
		retval = imu_comm_get_raw_latest(imu,&imu_frame);
		log_n_jump(retval,end_log_imu,"could not get new frame...");
		log_tv_only(log_imu_raw,tv_raw_sample);
		retval= imu_comm_print_raw(&imu_frame, log_imu_raw);
		log_n_jump(retval,end_log_imu,"could not print new raw frame...");
		fflush(log_imu_raw);
#endif // LOG_IMU_RAW
		goto end_imu;
	    }
	    else if(!imu_comm_calib_estim(imu))
	    {
		// if no calibration estim exists, build one.
		err_imu = imu_comm_calibration_start(imu);
		log_n_jump(err_imu,end_imu,"Failed to start calibration!");
		goto end_imu;
	    }

	    /// Get new unread data
	    if(!imu_comm_unread(imu) || !imu_comm_filter_ready(imu))
	    {
		// we only used averaged data
		goto end_imu;
	    }

	    gettimeofday(&tv_tmp,NULL);

	    err_imu = imu_comm_get_filtered_unread(imu,&imu_data);
	    log_n_jump(err_imu,end_imu,"IMU did not have new avg!");
	    //	    err_imu = imu_comm_get_lpf_unread(imu,&imu_data);
	    //	    log_n_jump(err_imu,end_imu,"LPF failed");

	    err_imu = uquad_timeval_substract(&tv_diff,tv_tmp,tv_last_imu);
	    if(err_imu < 0)
	    {
		log_n_jump(ERROR_TIMING,end_imu,"Timing error!");
	    }
#if LOG_IMU_AVG
	    err_imu = uquad_timeval_substract(&tv_imu_avg,tv_tmp,tv_start);
	    if(err_imu < 0)
	    {
		log_n_jump(ERROR_TIMING,end_imu,"Timing error!");
	    }
	    err_imu = ERROR_OK; // clear error, doesn't matter here.
#endif // LOG_IMU_AVG

	    // store time since last IMU sample
	    imu_data.timestamp = tv_diff;

	    /// new data will be useful!
	    imu_update = true;
	    err_imu = gettimeofday(&tv_last_imu,NULL);
	    err_log_std(err_imu);
#if IMU_COMM_FAKE
	    goto end_imu;
#endif // IMU_COMM_FAKE
            /**
             * Now that we have data, we'll go back to the beginning of the
             * loop to check if more data is available.
             * Small delays over time may lead to us falling behind IMU. If this
             * happens, we'll sacrifice a control loop to catch up.
             */
            continue;

	    end_imu:;
	    // will jump here if something went wrong during IMU reading
	}//if(read)

#if USE_GPS
#if !GPS_FAKE
	/// -- -- -- -- -- -- -- --
	/// Check GPS updates
	/// -- -- -- -- -- -- -- --
	if(reg_gps && !gps_update)
	{
	    err_gps = io_dev_ready(io,gps_fds,&read_ok,&write_ok);
	    if(read_ok)
	    {
		gettimeofday(&tv_tmp,NULL); // Will be used in log_gps
		if(device_gps != NULL)
		    //		    err_gps = gps_comm_read(gps,&gps_update,&tv_tmp);
		    err_gps = gps_comm_read(gps,&gps_update);
		else
		    //		    err_gps = gps_comm_read(gps,&gps_update,NULL);
		    err_gps = gps_comm_read(gps,&gps_update);
		log_n_jump(err_gps,end_gps,"GPS had no data!");
		if((!gps_update && (device_gps != NULL)) ||
		   (runs_kalman < 1) || !gps_comm_fix(gps))
		    // ignore startup data
		    goto end_gps;

		// Use latest IMU update to estimate speed from GPS data
		err_gps = gps_comm_get_data_unread(gps, gps_dat,NULL);
		log_n_jump(err_gps,end_gps,"Failed to get GPS data!");

		err_gps = uquad_timeval_substract(&tv_diff,tv_tmp,tv_start);
		if(err_gps < 0)
		{
		    log_n_jump(err_gps,end_gps,"Absurd GPS timing!");
		}
		err_gps = ERROR_OK; // clear error
#if LOG_GPS
		log_tv_only(log_gps, tv_diff);
		gps_comm_dump(gps, gps_dat, log_gps);
#endif // LOG_GPS
		tv_gps_last = tv_tmp;
	    }
	    end_gps:;
	    // will jump here if something went wrong during GPS reading
	}
#else // GPS_FAKE
	if(!gps_update)
	{
	    gettimeofday(&tv_tmp,NULL);
	    time_ret = uquad_timeval_substract(&tv_diff, tv_tmp, tv_gps_last);
	    if( (runs_kalman > 0) && (tv_diff.tv_sec > 0) )
	    {
#if GPS_RAND
		gps_dat->pos->m_full[0] = GPS_RAND_PP *(rand() / ( RAND_MAX + 1.0 ) - 0.5);
		gps_dat->pos->m_full[1] = GPS_RAND_PP *(rand() / ( RAND_MAX + 1.0 ) - 0.5);
		gps_dat->pos->m_full[2] = 2.0*GPS_RAND_PP *(rand() / ( RAND_MAX + 1.0 ) - 0.5);
		uquad_mat_dump_vec(gps_dat->pos, stdout, true);
#endif // GPS_RAND
		// gps_dat is set to 0 when allocated, so just use it.
		gps_update = true;
		tv_gps_last = tv_tmp;
		if(pp->pt != HOVER)
		{
		    quit_log_if(ERROR_GPS, "Fake GPS does not make sense if not hovering!");
		}
#if LOG_GPS
		time_ret = uquad_timeval_substract(&tv_diff, tv_tmp, tv_start);
		log_tv_only(log_gps, tv_diff);
		log_eol(log_gps);
#endif
	    }
	    else
	    {
		gps_update = false;
	    }
	}
#endif // GPS_FAKE
#endif // USE_GPS

	/// -- -- -- -- -- -- -- --
	/// check if new data
	/// -- -- -- -- -- -- -- --
	if(!imu_update || !running)
	    /**
	     * We don't check gps_update here.
	     * If gps_update && !imu_update, then
	     * wait until imu_update to avoid unstable T_s.
	     *   T_gps = 1s
	     *   T_imu = 10ms
	     * so use approx. that T_gps+T_imu ~ T_gps.
	     *
	     * running: If main was interrupted, then we want to
	     * log data but the motors should not be controlled any more.
	     */
	{
	    if(uquad_state == ST_RUNNING)
	    {
		/**
		 * If IMU has bad communication, it may be able to get the sync char
		 * frequently enough to keep main running, but it may not get a full
		 * frame. Here we verify that we haven't spent too long without a new
		 * IMU sample.
		 */
		gettimeofday(&tv_tmp,NULL);
		time_ret = uquad_timeval_substract(&tv_diff, tv_tmp, tv_last_kalman);
		if(time_ret < 0)
		{
		    err_log("Absurd timing!");
		}
		else
		{
		    if(tv_diff.tv_sec >= MAX_NO_UPDATES_S)
		    {
			quit_log_if(ERROR_TIMING, "ERR: Too long without new IMU samples!");
		    }
		}
	    }
	    continue;
	}
        else
	{
            imu_update = false; // mark data as used
	    if(skipped > 0)
	    {
		err_log_num("WARN: Skipped IMU!", skipped);
		fflush(stderr);
		skipped = 0;
	    }
	}
	/// -- -- -- -- -- -- -- --
	/// Startup Kalman estimator
	/// -- -- -- -- -- -- -- --
	if(runs_kalman == 0)
	{
	    /**
	     * Use current IMU calibration to set
	     * yaw == 0, this will keep us looking forward
	     * when hovering.
	     */
	    if(pp->pt != HOVER)
	    {
		quit_log_if(retval,"ERR: theta (Yaw) set to IMU calibration"\
			    "is only valid when in HOVER mode");
	    }
	    gettimeofday(&tv_tmp,NULL); // Will be used later
	    time_ret = uquad_timeval_substract(&tv_diff,tv_tmp,tv_start);
	    err_log_tv((time_ret < 0)?"Absurd IMU calibration time!":
		       "IMU calibration completed, running kalman+control+ramp",
		       tv_diff);
	    retval = imu_comm_raw2data(imu, &imu->calib.null_est, NULL, &imu_data);
	    quit_log_if(retval,"Failed to correct setpoint!");

#if USE_GPS && !GPS_FAKE
	    /* if(device_gps != NULL) */
	    /* { */
	    /* 	/\** */
	    /* 	 * If reading from gps log file, then change timestamp to avoid */
	    /* 	 * all draining the log because of the time spent in startup+calibration */
	    /* 	 * */
	    /* 	 *\/ */
	    /* 	retval = gps_comm_set_tv_start(gps,tv_tmp); */
	    /* 	quit_log_if(retval, "Failed to set gps startup time!"); */
	    /* } */
#endif // USE_GPS && !GPS_FAKE

	    /**
	     * Startup setpoint:
	     *  - Kalman estimator from calibration & GPS, if available.
	     */
	    if(pp->pt == HOVER)
	    {
#if USE_GPS
		// Position
		retval = uquad_mat_set_subm(pp->sp->x,SV_X,0,gps_dat->pos);
		quit_log_if(retval, "Failed to initiate kalman pos estimator from GPS data!");
		// Euler angles
		pp->sp->x->m_full[SV_THETA] = imu_data.magn->m_full[2]; // [rad]
#else // USE_GPS
		// Hover matrix is designed to work aiming north
		pp->sp->x->m_full[SV_THETA] = 0.0; // [rad]
#endif // USE_GPS
		pp->sp->x->m_full[SV_PSI]   = 0.0; // [rad]
		pp->sp->x->m_full[SV_PHI]   = 0.0; // [rad]
		pp->sp->x->m_full[SV_Z]     = 1.0; // [m]
		// Motor speed
		for(i=0; i<MOT_C; ++i)
		{
		    w_forced->m_full[i] = mot->w_min;
		    w->m_full[i]      = mot->w_hover;
		}
		//TODO Update control matrix - not implemented yet
		/* retval = control_update_K(ctrl, pp, mot->weight); */
		/* quit_log_if(retval, "Failed to update control matrix! Aborting..."); */
		/* retval = control_dump(ctrl, log_err); */
		/* quit_log_if(retval, "Failed to dump new control matrix! Aborting..."); */
	    }
	    err_log("Initial setpoint:");
	    uquad_mat_dump_vec(pp->sp->x,stderr,false);

	    /**
	     * Startup Kalman estimator
	     *  - If hovering, set initial position as setpoint. This will avoid
	     *    rough movements on startup (setpoint will match current state)	     *
	     */
#if USE_GPS
	    // Position
	    retval = uquad_mat_set_subm(kalman->x_hat,SV_X,0,gps_dat->pos);
	    quit_log_if(retval, "Failed to initiate kalman pos estimator from GPS data!");
	    /* GPS ALTITUDE INFORMATION IS IGNORED */
	    kalman->x_hat->m_full[2] = 0.0;
	    // Velocity
	    //	    retval = uquad_mat_set_subm(kalman->x_hat,SV_VQX,0,gps_dat->pos);
	    //	    quit_log_if(retval, "Failed to initiate kalman vel estimator from GPS data!");
#endif // USE_GPS
	    // Euler angles
	    kalman->x_hat->m_full[SV_PSI]   = imu_data.magn->m_full[0];
	    kalman->x_hat->m_full[SV_PHI]   = imu_data.magn->m_full[1];
	    kalman->x_hat->m_full[SV_THETA] = imu_data.magn->m_full[2];
#if KALMAN_BIAS
	    kalman->x_hat->m_full[SV_BAX] = imu_data.acc->m_full[0];
	    kalman->x_hat->m_full[SV_BAY] = imu_data.acc->m_full[1];
	    kalman->x_hat->m_full[SV_BAZ] = imu_data.acc->m_full[2] - GRAVITY;
#endif
	    retval = imu_comm_print_data(&imu_data, stderr);
	    if(retval != ERROR_OK)
	    {
		err_log("Failed to print IMU calibration!");
	    }
	    retval = ERROR_OK;// ignore error
	    fflush(stderr);

	}

	/// -- -- -- -- -- -- -- --
	/// Update state estimation
	/// -- -- -- -- -- -- -- --
	gettimeofday(&tv_tmp,NULL); // will be used to set tv_last_kalman
	if(runs_kalman == 0)
	{
	    // First time here, use fake timestamp
	    tv_diff.tv_usec = TS_DEFAULT_US;
	}
	else
	{
	    time_ret = uquad_timeval_substract(&tv_diff,tv_tmp,tv_last_kalman);
	    if(time_ret < 0)
	    {
		log_n_continue(ERROR_TIMING,"Absurd timing!");
	    }
#if TIMING_KALMAN
	    gettimeofday(&tv_pgm,NULL);
	    err_log_tv_num("Kalman:", tv_diff,(int)tv_diff.tv_usec);
#endif // TIMING_KALMAN
	    /// Check sampling period jitter
	    retval = in_range_us(tv_diff, TS_MIN, TS_MAX);
	    kalman_loops = (kalman_loops+1)%32768;// avoid overflow
	    if(retval != 0)
	    {
#if LOG_T_ERR
		uquad_timeval_substract(&tv_timing_off, tv_tmp, tv_start);
		log_eol(log_t_err);
		log_tv_only(log_t_err,tv_timing_off);
		log_tv_only(log_t_err,tv_diff);
		fflush(log_t_err);
#endif // LOG_T_ERR
		if(ts_error_wait == 0 || (ts_error_wait == TS_ERROR_WAIT))
		{
		    // Avoid saturating log
		    if(ts_error < MAX_ERRORS)
		    {
			/// Time supplied to kalman out of range!
			err_log_tv_num("TS K!", tv_diff, (int)ts_error_wait);
			fflush(stderr);
		    }
		    else
		    {
			quit_log_if(ERROR_TIMING,"ERR: Timing unacceptable! Aborting!");
		    }
		    if(ts_error_wait == TS_ERROR_WAIT)
		    {
			ts_error_wait = 0;
			ts_error++;
		    }
		}
		ts_error_wait++;
		if(tv_diff.tv_sec > 0)
		{
		    /// Be carefull with really bad timing
		    ts_error += (MAX_ERRORS >> 1);
		    ts_error_wait = TS_ERROR_WAIT;
		}
		/// Lie to kalman, avoid large drifts
		tv_diff.tv_usec = (retval > 0) ? TS_MAX:TS_MIN;
	    }
	    else
	    {
		// Print next T_s error immediately
		if(ts_error_wait > 1)
		{
		    err_log_tv_num("TS K!", tv_diff, (int)ts_error_wait);
		    fflush(stderr);
		}   
		ts_error_wait = 0;
		ts_error = uquad_max(0,ts_error - 1);
	    }
	}
	retval = uquad_kalman(kalman,
			      (uquad_state == ST_RUNNING)?
			      mot->w_curr:w,
			      &imu_data,
			      tv_diff.tv_usec,
			      mot->weight,
			      gps_update?gps_dat:NULL);
	log_n_continue(retval,"Inertial Kalman update failed");

	/// Mark time when we run Kalman
	tv_last_kalman = tv_tmp;
#if USE_GPS
	if(gps_update)
	{
	    gps_update = false; // Clear gps status
	}
#endif // USE_GPS


#if DEBUG
#if LOG_KALMAN_INPUT
	uquad_timeval_substract(&tv_diff,tv_last_kalman,tv_start);
	log_tv_only(log_kalman_in,tv_diff);
	retval = imu_comm_print_data(&imu_data, log_kalman_in);
	fflush(log_kalman_in);
#endif //LOG_KALMAN_INPUT
#if LOG_X_HAT
	retval = uquad_mat_transpose(x_hat_T, kalman->x_hat);
	quit_if(retval);
	uquad_mat_dump(x_hat_T,log_x_hat);
	fflush(log_x_hat);
#endif //LOG_X_HAT
#endif //DEBUG
	if(uquad_state == ST_RAMPING_UP)
	{
	    /**
	     * Startup:
	     *   - Ramp motors.
	     */
	    ++runs_kalman;
	    if(runs_kalman == STARTUP_SAMPLES)
	    {
		gettimeofday(&tv_tmp,NULL);
		time_ret = uquad_timeval_substract(&tv_diff,tv_tmp,tv_start);
		if(time_ret < 0)
		{
		    err_log("Absurd Kalman startup time!");
		    retval = ERROR_TIMING;
		    continue;
		}
		// save to error log
		err_log("-- --");
		err_log("-- -- -- -- -- -- -- --");
		err_log_tv("Ramp completed, running free control...", tv_diff);
		err_log("-- -- -- -- -- -- -- --");
		err_log("-- --");
		fflush(stderr);

		uquad_state = ST_RUNNING;
	    }
	}

	/// -- -- -- -- -- -- -- --
	/// Sanity check
	/// -- -- -- -- -- -- -- --
	sanity_check(&imu_data, kalman->x_hat, &aux_bool);
	if(aux_bool)
	{
	    if(insane++ > SANITY_MAX)
	    {
		err_log("-- -- -- --");
		err_log("-- -- -- -- -- -- -- --");
		err_log("ERROR! SANITY CHECK!");
		err_log("-- -- -- -- -- -- -- --");
		err_log("-- -- -- --");
		quit();
	    }
	}
	else
	{
	    insane = uquad_max(insane - 1,0);
	}


	/// -- -- -- -- -- -- -- --
	/// Sample/action ratio
	/// -- -- -- -- -- -- -- --
	if(++ctrl_samples == CTRL_TS)
	    ctrl_samples = 0;
	else
	    // Don't run ctrl loop, nor set motor speed
	    continue;

	/// -- -- -- -- -- -- -- --
	/// Update setpoint
	/// -- -- -- -- -- -- -- --
	retval = pp_update_setpoint(pp, kalman->x_hat, mot->w_hover, &ctrl_outdated);
	log_n_continue(retval,"Kalman update failed");

	/// -- -- -- -- -- -- -- --
	/// Update control matrices
	/// -- -- -- -- -- -- -- --
	if(ctrl_outdated)
	{
	    retval = control_update_K(ctrl, pp, mot->weight);
	    quit_log_if(retval, "Failed to update control matrix! Aborting...");
	    retval = control_dump(ctrl, log_err);
	    quit_log_if(retval, "Failed to dump new control matrix! Aborting...");
	}

	/// -- -- -- -- -- -- -- --
	/// Run control
	/// -- -- -- -- -- -- -- --
	gettimeofday(&tv_tmp,NULL);
	uquad_timeval_substract(&tv_diff,tv_tmp,tv_last_m_cmd);
	retval = control(ctrl, w, kalman->x_hat, pp->sp, (double)tv_diff.tv_usec);
	log_n_continue(retval,"Control failed!");
#if DEBUG
	uquad_timeval_substract(&tv_diff,tv_tmp,tv_start);
#endif // DEBUG
#if DEBUG && LOG_W_CTRL
	retval = uquad_mat_transpose(wt,w);
	log_tv_only(log_w_ctrl,tv_diff);
	uquad_mat_dump(wt,log_w_ctrl);
	fflush(log_w_ctrl);
#endif
#if LOG_INT
	log_tv_only(log_int, tv_diff);
	uquad_mat_dump_vec(ctrl->x_int, log_int, false);
	fflush(log_int);
#endif // LOG_INT

	/// -- -- -- -- -- -- -- --
	/// Set motor speed
	/// -- -- -- -- -- -- -- --
	gettimeofday(&tv_tmp,NULL);
	uquad_timeval_substract(&tv_diff,tv_tmp,tv_last_m_cmd);
	if (tv_diff.tv_usec > MOT_UPDATE_T || tv_diff.tv_sec > 1)
	{
	    /// Update motor controller
	    if(uquad_state != ST_RUNNING)
	    {
		switch(uquad_state)
		{
		case ST_RAMPING_UP:
		    /**
		     * Motors would start from hover speed
		     * Ramp them up, but keep controlling to maintain
		     * balance.
		     */
#if RAMP_LINEAR
		    dtmp = -(mot->w_hover - mot->w_min)*(1.0 - 1.0/(STARTUP_SAMPLES*STARTUP_SAMPLES)*(runs_kalman*runs_kalman));
#else // RAMP_LINEAR
		    dtmp = - (STARTUP_SAMPLES - runs_kalman)
			*((mot->w_hover - mot->w_min)/STARTUP_SAMPLES);
#endif // RAMP_LINEAR
		    break;
		case ST_RAMPING_DOWN:
		    /**
		     * Slowly pull down motor speed y approach ground,
		     * but keep the controller running for stability.
		     */
		    dtmp = -runs_down*(((mot->w_hover - mot->w_min)/2.0)/RAMP_DOWN_SAMPLES);
		    if(runs_down++ > RAMP_DOWN_SAMPLES)
		    {
			err_log("Landing completed, terminating...");
			quit();
		    }
		    break;
		default:
		    quit_log_if(ERROR_FAIL,"Invalid state!");
		    break;
		}
		for(i = 0; i < MOT_C; ++i)
		{
		    w_forced->m_full[i] = uquad_max(mot->w_min,
						    w->m_full[i] + dtmp);
		}
		retval = mot_set_vel_rads(mot, w_forced, false);
		log_n_continue(retval,"Failed to set motor speed during ramp!");
	    }
	    else
	    {
		retval = mot_set_vel_rads(mot, w, false);
		log_n_continue(retval,"Failed to set motor speed!");
	    }
#if DEBUG && LOG_W
	    uquad_timeval_substract(&tv_diff,tv_tmp,tv_start);
	    log_tv_only(log_w,tv_diff);
	    retval = uquad_mat_transpose(wt,mot->w_curr);
	    log_n_continue(retval, "Failed to prepare w transpose...");
	    uquad_mat_dump(wt,log_w);
	    fflush(log_w);
#endif

	    /**
	     * Log only data that was actually used.
	     * To check missed data, look at error log.
	     */
#if LOG_IMU_AVG
	    log_tv_only(log_imu_avg, tv_imu_avg);
	    err_imu = imu_comm_print_data(&imu_data, log_imu_avg);
	    log_n_jump(err_imu,end_imu,"Failed to log imu avg!");
#endif // LOG_IMU_AVG
#if LOG_IMU_RAW || LOG_IMU_DATA
	    retval = imu_comm_get_raw_latest(imu,&imu_frame);
	    log_n_jump(retval,end_log_imu,"could not get new frame...");
#if LOG_IMU_RAW
	    log_tv_only(log_imu_raw,tv_raw_sample);
	    retval= imu_comm_print_raw(&imu_frame, log_imu_raw);
	    log_n_jump(retval,end_log_imu,"could not print new raw frame...");
	    fflush(log_imu_raw);
#endif // LOG_IMU_RAW
#if LOG_IMU_DATA
	    retval = imu_comm_raw2data(imu, &imu_frame, NULL, &imu_data);
	    log_n_jump(retval,end_log_imu,"could not convert new raw...");
	    log_tv_only(log_imu_data,tv_raw_sample);
	    retval = imu_comm_print_data(&imu_data, log_imu_data);
	    log_n_jump(retval,end_log_imu,"could not print new data...");
	    fflush(log_imu_data);
#endif // LOG_IMU_DATA
	    end_log_imu:;
#endif // LOG_IMU_RAW || LOG_IMU_DATA

#if X_HAT_STDOUT
	    if(x_hat_cnt++ > X_HAT_STDOUT)
	    {
#if SV_NAME
		print_sv_name(stdout);
#endif // SV_NAME
		uquad_mat_dump_vec(kalman->x_hat,stdout,true);
		x_hat_cnt = 0;
		fflush(stdout);
	    }
#endif // X_HAT_STDOUT

#if LOG_BUKAKE
	    uquad_timeval_substract(&tv_diff,tv_tmp,tv_start);
	    log_tv_only(log_bukake,tv_diff);
	    int ind; uquad_mat_t *xm = kalman->x_hat; int len = xm->r*xm->c;
	    for(ind = 0; ind < len; ++ind)
		log_double_only(log_bukake,xm->m_full[ind]);
	    fdatasync(fileno(log_bukake));
	    retval = uquad_mat_transpose(wt,mot->w_curr);
	    uquad_mat_dump(wt,log_bukake);
	    fdatasync(fileno(log_bukake));
#endif
	    tv_last_m_cmd = tv_tmp;
	}
    }
    // never gets here
    return 0;
}
    
