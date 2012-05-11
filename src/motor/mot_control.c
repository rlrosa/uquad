#include "mot_control.h"
#include <stdlib.h> // for malloc()
#include <string.h> // for memset()
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>


/**
 * Allocates mem for motor controller, and launches driver module.
 *
 * NOTE: Takes MOT_WAIT_STARTUP_S secs + MOT_UPDATE_MAX_US usecs to init
 *
 * @param start_motors If initialization is successful and start_motors, then
 *        motors will be running at idle speed after init.
 *
 * @return
 */
uquad_mot_t *mot_init(uquad_bool_t start_motors)
{
    int retval = ERROR_OK;
    retval = system(KILL_MOTOR_CMD);
    // ignore return value, it's ok if it wasn't running
    retval = ERROR_OK;
    sleep_ms(150);

    uquad_mot_t *m = (uquad_mot_t *)malloc(sizeof(uquad_mot_t));
    if(m == NULL)
    {
	cleanup_log_if(ERROR_MALLOC, "Failed to allocate mem for uquad_mot_t!");
    }
    memset((void *)m,0,sizeof(uquad_mot_t));

    m->w_curr = uquad_mat_alloc(MOT_C,1);
    if(m->w_curr == NULL)
    {
	err_log("Failed to allocate mem for w_curr!");
	goto cleanup;
    }

    m->tx_log = fopen(MOT_LOG_TX,"w");
    if(m->tx_log == NULL)
    {
	err_log("Failed to open tx log!");
	goto cleanup;
    }

    retval = mot_update_w_hover(m, MASA_DEFAULT);
    cleanup_log_if(retval, "Failed to update W_HOVER!");

    retval = system(START_MOTOR_CMD);
    if(retval < 0)
    {
    	err_log("Failed to run cmd!");
	goto cleanup;
    }
    sleep(MOT_WAIT_STARTUP_S); // Wait for cmd to startup
    m->kmsgq = uquad_kmsgq_init(MOT_SERVER_KEY, MOT_DRIVER_KEY);
    if(m->kmsgq == NULL)
    {
	err_log("Failed to start message queue!");
	goto cleanup;
    }

    // initialize timer
    retval = gettimeofday(&m->last_set,NULL);
    cleanup_log_if(retval,"Failed to initialize timer");
    usleep(MOT_UPDATE_MAX_US); // Wait before allowing cmds to be sent

    if(start_motors)
    {
	retval = mot_set_idle(m);
	cleanup_log_if(retval, "Failed to startup motors!");
    }

    return m;

    cleanup:
    // if here, then something went wrong. Clean and return error
    mot_deinit(m);
    m = NULL;
    return m;
};

int mot_send(uquad_mot_t *mot, double *w)
{
    struct timeval tmp_tv, diff_tv;
    int retval = ERROR_OK;
    static uint8_t buff_out[MOT_C];
    // swap order of 0x6b and 0x6a
    buff_out[0] = mot->i2c_target[0];
    buff_out[1] = mot->i2c_target[2];
    buff_out[2] = mot->i2c_target[1];
    buff_out[3] = mot->i2c_target[3];

    retval = gettimeofday(&tmp_tv,NULL);
    err_check_std(retval);
    retval = uquad_timeval_substract(&diff_tv,tmp_tv,mot->last_set);
    if(diff_tv.tv_usec < MOT_UPDATE_MAX_US && diff_tv.tv_sec < 1)
    {
	log_tv_only(stderr,diff_tv);
	err_check(ERROR_MOT_SATURATE,"Cannot change speed so often!");//TODO restore!
    }
    retval = uquad_kmsgq_send(mot->kmsgq, buff_out, MOT_C);
    err_propagate(retval);
    retval = gettimeofday(&mot->last_set,NULL);
    err_check_std(retval);
    // update current i2c
    memcpy(mot->i2c_curr,mot->i2c_target,MOT_C*sizeof(uint8_t));
    // update current speed
    memcpy(mot->w_curr->m_full,w,MOT_C*sizeof(double));
    return ERROR_OK;
}

int mot_rad2i2c(double w, int *i2c)
{
    double w_2, w_3;
    w_2 = w*w;
    w_3 = w_2*w;
    *i2c = uquad_round_double2int(3.25038742351062e-06*w_3
				  - 0.00107345094348345*w_2
				  + 0.552199572510559*w);
    return ERROR_OK;
}

int mot_set_vel_rads(uquad_mot_t *mot, uquad_mat_t *w, uquad_bool_t force)
{
    int i,retval;
    int itmp;
    
    for(i=0; i < MOT_C; ++i)
    {
	if(!force)
	{
	    if(w->m_full[i] < mot->w_min)
	    {
		/**
		 * Setting speed to less than MOT_W_IDLE could
		 * cause motors to stop.
		 *
		 */
		err_log_num("WARN:w out of range, setting min for motor:",i);
		w->m_full[i] =  mot->w_min;
	    }
	    else if (w->m_full[i] > MOT_W_MAX)
	    {
		/**
		 * Setting speed to more than MOT_MAX_W could
		 * damage battery.
		 *
		 */
		err_log_num("WARN:w out of range, setting max for motor:",i);
		w->m_full[i] =  MOT_W_MAX;
	    }
	}
	retval = mot_rad2i2c(w->m_full[i],&itmp);
	err_propagate(retval);
	mot->i2c_target[i] = (uint8_t)itmp;
    }
    retval = mot_send(mot, w->m_full);
    err_propagate(retval);
    return retval;
};

int mot_set_idle(uquad_mot_t *mot)
{
    static double w_idle[MOT_C] = {MOT_I2C_IDLE,
			    MOT_I2C_IDLE,
			    MOT_I2C_IDLE,
			    MOT_I2C_IDLE};
    int retval = ERROR_OK,i ;
    for (i=0; i < MOT_C; ++i)
	mot->i2c_target[i] = MOT_I2C_IDLE;
    retval = mot_send(mot, w_idle);
    err_propagate(retval);
    sleep_ms(MOT_WAIT_STARTUP_MS);
    return retval;
};

int mot_stop(uquad_mot_t *mot)
{
    int retval = ERROR_OK;
    static double w_stop[MOT_C] = {0.0, 0.0, 0.0, 0.0};
    memset(mot->i2c_target, 0, MOT_C*sizeof(uint8_t));
    retval = mot_send(mot, w_stop);
    err_propagate(retval);
    return retval;    
};

int mot_deinit(uquad_mot_t *mot)
{
    int retval;
    retval = system(KILL_MOTOR_CMD);
    if(mot != NULL)
    {
	uquad_kmsgq_deinit(mot->kmsgq);
	if(mot->tx_log != NULL)
	    fclose(mot->tx_log);
	uquad_mat_free(mot->w_curr);
	free(mot);
    };
    return retval;;
};

int mot_calc_w_hover(double *w_hover, double weight)
{
    int retval;
    retval = uquad_solve_pol2(w_hover, NULL, F_B1, F_B2, -GRAVITY*weight/4.0);
    err_propagate(retval);
    return retval;
}

int mot_update_w_hover(uquad_mot_t *mot, double weight)
{
    int retval;
    if(mot == NULL)
    {
	err_check(ERROR_NULL_POINTER, "NULL pointer is invalid arg!");
    }
    retval = mot_calc_w_hover(&mot->w_hover, weight);
    err_propagate(retval);
    if(mot->w_hover > MOT_W_MAX)
    {
	err_log("WARN: Attempted to set w_hover over MOT_W_MAX, will use max...");
	mot->w_hover = MOT_W_MAX;
    }
    mot->w_min  = mot->w_hover - (MOT_W_MAX - mot->w_hover);
    mot->weight = weight;
    return ERROR_OK;
}
