#include "mot_control.h"
#include <stdlib.h> // for malloc()
#include <string.h> // for memset()

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
    m->ctrl_file = fopen(MOT_COMM_FILE,"w");
    if(m->ctrl_file == NULL)
    {
	err_log("Failed to open motor control file!");
	retval = ERROR_FAIL;
    }
    m->tx_log = fopen(MOT_LOG_TX,"w");
    if(m->tx_log == NULL)
    {
	err_log("Failed to open tx log!");
	retval = ERROR_FAIL;
    }
    retval = system(START_MOTOR_CMD);
    if(retval < 0)
    {
	err_log("Failed to run cmd_motores!");
	retval = ERROR_FAIL;
    }
    if(retval != ERROR_OK)
    {
	mot_deinit(m);
	m = NULL;
    }
    return m;
};

static unsigned long tx_counter = 0;
int mot_send(uquad_mot_t *mot)
{
    int retval, i;
    retval = fprintf(mot->ctrl_file,"%d %d %d %d",
		     mot->i2c_target[0],
		     mot->i2c_target[1],
		     mot->i2c_target[2],
		     mot->i2c_target[3]);
    fflush(mot->ctrl_file);
    if(retval < MOT_C)
    {
	err_check(ERROR_MOTOR_SET,"Failed to write commands for all motors!");
    }
    gettimeofday(&mot->last_set,NULL);

    /// Log sent data, timestamp, and counter
    retval = fprintf(mot->tx_log,"%d %lu %d %d %d %d\n",
		     (int)mot->last_set.tv_usec,
		     tx_counter++,
		     mot->i2c_target[0],
		     mot->i2c_target[1],
		     mot->i2c_target[2],
		     mot->i2c_target[3]);
    memcpy(mot->i2c_curr,mot->i2c_target,MOT_C*sizeof(int));
    return ERROR_OK;
}

int mot_rad2i2c(double w, int *i2c)
{
    double w_2, w_3;
    if(w < 0 || w > MAX_W)
    {
	err_check(ERROR_MOTOR_W,"Speed out of range!");
    }

    w_2 = w*w;
    w_3 = w_2*w;
    *i2c = uquad_round_double2int(0.0000022118*w_3 - 0.00071258*w_2 + 0.5106*w);
    return ERROR_OK;
}

int mot_set_vel_rads(uquad_mot_t *mot, int *v)
{
    int i,retval;
    int itmp;
    
    for(i=0; i < MOT_C; ++i)
    {
	retval = mot_rad2i2c(v[i],&itmp);
	if(retval != ERROR_OK)
	{
	    mot->i2c_target[i] = itmp;
	}
    }
    retval = mot_send(mot);
    err_propagate(retval);
    return retval;
};

int mot_set_idle(uquad_mot_t *mot)
{
    int retval = ERROR_OK;
    memset(mot->i2c_target, MOT_I2C_IDLE, MOT_C*sizeof(int));
    retval = mot_send(mot);
    err_propagate(retval);
    return retval;
};

int mot_stop(uquad_mot_t *mot)
{
    int retval = ERROR_OK;
    memset(mot->i2c_target, 0, MOT_C*sizeof(int));
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
	if(mot->ctrl_file != NULL)
	    fclose(mot->ctrl_file);
	if(mot->tx_log != NULL)
	    fclose(mot->tx_log);
	free(mot);
    };
    return retval;;
};

