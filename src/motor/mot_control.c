#include "mot_control.h"
#include <uquad_aux_time.h>
#include <stdlib.h> // for malloc()
#include <string.h> // for memset()
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>

uquad_mot_t *mot_init(void)
{
    int retval = ERROR_OK;
    retval = system(KILL_MOTOR_CMD);
    // ignore return value, it's ok if it wasn't running
    retval = ERROR_OK;
    sleep_ms(50);

    uquad_mot_t *m = (uquad_mot_t *)malloc(sizeof(uquad_mot_t));
    mem_alloc_check(m);
    memset((void *)m,0,sizeof(uquad_mot_t));
    m->tx_log = fopen(MOT_LOG_TX,"w");
    if(m->tx_log == NULL)
    {
	err_log("Failed to open tx log!");
	retval = ERROR_FAIL;
    }
    retval = system(START_MOTOR_CMD);
    if(retval < 0)
    {
    	err_log("Failed to run cmd!");
    	retval = ERROR_FAIL;
    }
    sleep_ms(10);
    m->kmsgq = uquad_kmsgq_init(MOT_SERVER_KEY, MOT_DRIVER_KEY);
    if(m->kmsgq == NULL)
    {
	err_log("Failed to start message queue!");
	retval = ERROR_FAIL;
    }
	
    if(retval != ERROR_OK)
    {
	mot_deinit(m);
	m = NULL;
    }
    return m;
};

int mot_send(uquad_mot_t *mot)
{
    struct timeval tmp_tv, diff_tv;
    int retval = ERROR_OK, i;

    gettimeofday(&tmp_tv,NULL);
    retval = uquad_timeval_substract(&diff_tv,tmp_tv,mot->last_set);
    if(diff_tv.tv_usec < MOT_UPDATE_MAX_US && diff_tv.tv_sec < 1)
    {
	err_check(ERROR_MOT_SATURATE,"Cannot change speed so often!");
    }
    retval = uquad_kmsgq_send(mot->kmsgq, mot->i2c_target, MOT_C);
    err_propagate(retval);
    gettimeofday(&mot->last_set,NULL);
    memcpy(mot->i2c_curr,mot->i2c_target,MOT_C*sizeof(uint8_t));
    return ERROR_OK;
}

int mot_rad2i2c(double w, int *i2c)
{
    double w_2, w_3;
    if(w > MAX_W)
    {
	err_check(ERROR_MOTOR_W,"Speed out of range!");
    }
    w_2 = w*w;
    w_3 = w_2*w;
    *i2c = uquad_round_double2int(0.0000022118*w_3 - 0.00071258*w_2 + 0.5106*w);
    return ERROR_OK;
}

int mot_set_vel_rads(uquad_mot_t *mot, double *w)
{
    int i,retval;
    int itmp;
    
    for(i=0; i < MOT_C; ++i)
    {
	if(w[i] < 1.0)
	{
	    err_check(ERROR_MOTOR_USAGE,"Use mot_stop() to stop motors!");
	}
	retval = mot_rad2i2c(w[i],&itmp);
	err_propagate(retval);
	mot->i2c_target[i] = (uint8_t)itmp;
    }
    retval = mot_send(mot);
    err_propagate(retval);
    return retval;
};

int mot_set_idle(uquad_mot_t *mot)
{
    int retval = ERROR_OK,i ;
    for (i=0; i < MOT_C; ++i)
	mot->i2c_target[i] = MOT_I2C_IDLE;
    retval = mot_send(mot);
    err_propagate(retval);
    sleep_ms(MOT_WAIT_STARTUP_MS);
    return retval;
};

int mot_stop(uquad_mot_t *mot)
{
    int retval = ERROR_OK;
    memset(mot->i2c_target, 0, MOT_C*sizeof(uint8_t));
    retval = mot_send(mot);
    err_propagate(retval);
    return retval;    
};

int mot_deinit(uquad_mot_t *mot)
{
    int retval;
    retval = system(KILL_MOTOR_CMD);
    if(mot != NULL)
    {
	if(mot->tx_log != NULL)
	    fclose(mot->tx_log);
	free(mot);
    };
    return retval;;
};