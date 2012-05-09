#include <errno.h> 
#include <math.h>
#include <string.h> 
#include <stdio.h> 
#include <signal.h> // for SIGINT, SIGQUIT
#include <stdlib.h> 
#include <stdint.h>
#include <unistd.h>
#ifndef PC_TEST
#include <linux/i2c-dev-user.h>
#else
#include <linux/i2c-dev.h>
#endif
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/time.h>

// kernel queues
#include <sys/ipc.h> // for IPC_NOWAIT
#include <sys/msg.h>

#include <time.h>

#define USAGE_STR "Incorrect arguments! Use no arguments (all enabled),"\
    "or 4, enabling each motor.\n\n\t./cmd ena9 enaA enaB ena8\nwhere"\
    "enaX enables motor X, valid is 0 or 1\n\n"

#define MAX_SPEED                 220 // i2c
#define MIN_SPEED                 45  // i2c

#define DEBUG                     0
#if DEBUG
//#define LOG_VELS
//#define LOG_TIMING
#define LOG_TIMING_KERNEL_CALLS
#endif

#define TV_TH_US                  500UL

#define PRINT_COUNT               500
#define MOT_COUNT                 4
#define UQUAD_MOT_I2C_REG_ENABLE  0xA0
#define UQUAD_MOT_ENABLE_DAT      0x00
#define UQUAD_MOT_I2C_REG_DISABLE 0xA1
#define UQUAD_MOT_DISABLE_DAT     0x00
#define UQUAD_MOT_I2C_REG_VEL     0xA2
#define MOT_SELECTED              1
#define MOT_NOT_SELECTED          0

/// Startup parameters
#define RAMP_START                30
#define RAMP_END                  50
#define START_JITTER              25
#define STARTUP_SWAP              0

#define OK                        0
#define NOT_OK                    -1

#define UQUAD_STARTUP_RETRIES     100
#define UQUAD_STOP_RETRIES        1000
#define UQUAD_USE_DIFF            0
#define UQUAD_USE_SIN             (1 && UQUAD_USE_DIFF)

#define LOOP_T_US                 2000UL

#define LOG_ERR                   stdout

#define backtrace()     fprintf(LOG_ERR,"%s:%d\n",__FUNCTION__,__LINE__)
#define log_to_err(msg) fprintf(LOG_ERR,"%s: %s:%d\n",msg,__FUNCTION__,__LINE__)

#define sleep_ms(ms)    usleep(1000*ms)

#define log_tv_only(log,tv)					\
    {								\
	fprintf(log,"%ld.%06ld\t",tv.tv_sec, tv.tv_usec);	\
    }

#define tv2double(db,tv)					\
    {								\
	db = ((double) tv.tv_sec) + ((double) tv.tv_usec)/1e6;	\
    }								\

int uquad_timeval_substract (struct timeval * result, struct timeval x, struct timeval y){
    /* Perform the carry for the later subtraction by updating y. */
    if (x.tv_usec < y.tv_usec) {
	int nsec = (y.tv_usec - x.tv_usec) / 1000000 + 1;
	y.tv_usec -= 1000000 * nsec;
	y.tv_sec += nsec;
    }
    if (x.tv_usec - y.tv_usec > 1000000) {
	int nsec = (y.tv_usec - x.tv_usec) / 1000000;
	y.tv_usec += 1000000 * nsec;
	y.tv_sec -= nsec;
    }
    
    /* Compute the time remaining to wait.
       tv_usec is certainly positive. */
    result->tv_sec = x.tv_sec - y.tv_sec;
    result->tv_usec = x.tv_usec - y.tv_usec;
    
    if(x.tv_sec < y.tv_sec)
	// -1 if diff is negative
	return -1;
    if(x.tv_sec > y.tv_sec)
	// 1 if diff is positive
	return 1;
    // second match, check usec
    if(x.tv_usec < y.tv_usec)
	// -1 if diff is negative
	return -1;
    if(x.tv_usec > y.tv_usec)
	// 1 if diff is positive
	return 1;

    // 0 if equal
    return 0;
}

/// Forwards defs
int uquad_mot_i2c_addr_open(int i2c_dev, int addr);
int uquad_mot_i2c_send_byte(int i2c_dev, __u8 reg, __u8 value);
int uquad_mot_i2c_send(int i2c_dev, int i2c_addr, __u8 reg, __u8 val);

int uquad_mot_enable(int i2c_dev, int mot_i2c_addr);
int uquad_mot_enable_all(int i2c_dev);
int uquad_mot_disable(int i2c_dev, int mot_i2c_addr);
int uquad_mot_disable_all(int i2c_dev);
int uquad_mot_set_speed(int i2c_dev, int mot_i2c_addr, __u8 val);
int uquad_mot_set_speed_all(int i2c_dev, __u8 *v, int swap_order);
/// Aux
void uquad_sig_handler(int signal_num);
/// Intercom via kernel msgq
typedef struct msgbuf {
    long    mtype;
    uint8_t mtext[MOT_COUNT];
} message_buf_t;

typedef enum {RUNNING,STOPPED}mot_state_t;

// Global vars
#ifndef CHECK_STDIN
static int msqid; // Set by mot_control.h
static message_buf_t rbuf;
#endif // CHECK_STDIN
const static key_t key_s = 169; // must match MOT_SERVER_KEY (in mot_control.h)
const static key_t key_c = 170; // must match MOT_DRIVER_KEY (in mot_control.h)
#ifdef LOG_VELS
static struct timeval timestamp;
#endif // LOG_VELS
static int i2c_file = -1;
static int mot_i2c_addr[MOT_COUNT] = {0x69,
				      0x6a,
				      0x6b,
				      0x68};
static unsigned short mot_selected[MOT_COUNT] = {MOT_NOT_SELECTED,
						 MOT_NOT_SELECTED,
						 MOT_NOT_SELECTED,
						 MOT_NOT_SELECTED};

static __u8 vels[MOT_COUNT] = {0,0,0,0};

#ifdef LOG_VELS
static FILE *log_rx;
#endif
#ifdef PC_TEST
#if DEBUG
static FILE *i2c_fake;
#define FAKE_I2C_PATH "i2c.dev"
#endif
#endif // LOG_VELS

int uquad_mot_i2c_addr_open(int i2c_dev, int addr){
#ifndef PC_TEST
#ifdef LOG_TIMING_KERNEL_CALLS
    struct timeval tv_before, tv_after, tv_diff;
    gettimeofday(&tv_before,NULL);
#endif // LOG_TIMING_KERNEL_CALLS
    if (ioctl(i2c_dev,I2C_SLAVE,addr) < 0)
    {
	/* ERROR HANDLING; you can check errno to see what went wrong */
	fprintf(LOG_ERR,"ERROR! %s failed to write to 0x%02X...\n"	\
		"errno info:\t %s\n",__FUNCTION__,addr,strerror(errno));
#ifdef CHECK_STDIN
	fflush(LOG_ERR);
#endif // CHECK_STDIN
	return NOT_OK;
    }
#ifdef LOG_TIMING_KERNEL_CALLS
    else
    {
	gettimeofday(&tv_after,NULL);
	uquad_timeval_substract(&tv_diff, tv_after, tv_before);
	if(tv_diff.tv_usec > TV_TH_US)
	{
	    log_tv_only(stdout, tv_diff);
	    fprintf(stdout,"ioctl()\n");
	}
    }
#endif // LOG_TIMING_KERNEL_CALLS
#else
#if DEBUG
    fprintf(i2c_fake,"%d",addr);
#endif
#endif
    return OK;
}

int uquad_mot_i2c_send_byte(int i2c_dev, __u8 reg, __u8 value){
#ifndef PC_TEST
#ifdef LOG_TIMING_KERNEL_CALLS
    struct timeval tv_before, tv_after, tv_diff;
    gettimeofday(&tv_before,NULL);
#endif // LOG_TIMING_KERNEL_CALLS
    if(i2c_smbus_write_byte_data(i2c_dev,reg,value) < 0)
    {
	/* ERROR HANDLING: i2c transaction failed */
	fprintf(LOG_ERR,"Failed to send value %d\tto 0x%02X\n."\
		"errno info:\t %s\n",(int)value,(int)reg,strerror(errno));
#ifdef CHECK_STDIN
	fflush(LOG_ERR);
#endif // CHECK_STDIN
    }
#ifdef LOG_TIMING_KERNEL_CALLS
    else
    {
	gettimeofday(&tv_after,NULL);
	uquad_timeval_substract(&tv_diff, tv_after, tv_before);
	if(tv_diff.tv_usec > TV_TH_US)
	{
	    log_tv_only(stdout, tv_diff);
	    fprintf(stdout,"i2c_smbus_write_byte_data()\n");
	}
    }
#endif // LOG_TIMING_KERNEL_CALLS
#else
#if DEBUG
    struct timeval t;
    gettimeofday(&t,NULL);
    fprintf(i2c_fake,"\t%d\t%d\t%ld\n",(int)reg, (int)value,(long int)t.tv_usec);
#endif
#endif
    return OK;
}

int uquad_mot_i2c_send(int i2c_dev, int i2c_addr, __u8 reg, __u8 val){
    if(uquad_mot_i2c_addr_open(i2c_dev, i2c_addr) < 0)
    {
	backtrace();
	return NOT_OK;
    }
    if(uquad_mot_i2c_send_byte(i2c_dev,reg,val) < 0)
    {
	backtrace();
	return NOT_OK;
    }
    return OK;
}

int uquad_mot_enable(int i2c_dev, int mot_i2c_addr){
    return uquad_mot_i2c_send(
				 i2c_dev,
				 mot_i2c_addr,
				 UQUAD_MOT_I2C_REG_ENABLE,
				 UQUAD_MOT_ENABLE_DAT);
}

int uquad_mot_enable_all(int i2c_dev){
    int i;
    for(i=0;i<MOT_COUNT;++i)
    {
	if(!mot_selected[i])
	   continue;
	if(uquad_mot_enable(i2c_dev,mot_i2c_addr[i]))
	{
	    backtrace();
	    break;
	}
    }
    if(i<MOT_COUNT)
    {
	backtrace();
	return NOT_OK;
    }
    return OK;
}

int uquad_mot_disable(int i2c_dev, int mot_i2c_addr){
    return uquad_mot_i2c_send(
				 i2c_dev,
				 mot_i2c_addr,
				 UQUAD_MOT_I2C_REG_DISABLE,
				 UQUAD_MOT_DISABLE_DAT);
}

int uquad_mot_disable_all(int i2c_dev){
    int i;
    for(i=0;i<MOT_COUNT;++i)
    {
	if(!mot_selected[i])
	    continue;
	if(uquad_mot_disable(i2c_dev,mot_i2c_addr[i]))
	{
	    backtrace();
	    break;
	}
    }
    if(i<MOT_COUNT)
    {
	backtrace();
	return NOT_OK;
    }
    return OK;
}

int uquad_mot_set_speed(int i2c_dev, int mot_i2c_addr, __u8 val){
    return uquad_mot_i2c_send(
				 i2c_dev,
				 mot_i2c_addr,
				 UQUAD_MOT_I2C_REG_VEL,
				 val);
}

int uquad_mot_set_speed_all(int i2c_dev, __u8 *v, int swap_order){
    int i, index;
    __u8 u8tmp = 0;
    for(i=0;i<MOT_COUNT;++i)
    {
	if(!mot_selected[i])
	    continue;
	index = swap_order?(3+i)%4:i;
	u8tmp = (v==NULL)?0:v[index];
	if(uquad_mot_set_speed(i2c_dev,
			       mot_i2c_addr[index],
			       u8tmp))
	{
	    backtrace();
	    break;
	}
    }
    if(i<MOT_COUNT)
    {
	backtrace();
	return NOT_OK;
    }
    return OK;
}

#define STARTUPS 1 // send startup command repeatedly
int uquad_mot_startup_all(int i2c_file){
    int i, j, watchdog = 0, enable_counts = 0;
    __u8 tmp_vel[MOT_COUNT]; __u8 ramp = 0;
#if STARTUP_SWAP
    if(uquad_mot_set_speed_all(i2c_file, NULL,1) < 0)
    {
	backtrace();
	return NOT_OK;
    }
#endif
    while(uquad_mot_enable_all(i2c_file) < 0 || ++enable_counts < STARTUPS)
    {
	usleep(400);
	// wait for start
	if(++watchdog == UQUAD_STARTUP_RETRIES)
	{
	    backtrace();
	    return NOT_OK;
	}
    }	    
    sleep_ms(420);
    for(ramp = RAMP_START; ramp < RAMP_END; ramp++)
    {
	for(j = 0; j < 2*ramp; ++j)
	{
	    if(j!=0)
		usleep(1235);
	    for(i = 0; i < MOT_COUNT; ++i)
		tmp_vel[i] = (!(j%7))?
		    ramp-START_JITTER:
		    ramp+START_JITTER;
	    uquad_mot_set_speed_all(i2c_file, tmp_vel,0);
	}
    }
    return OK;
}

int uquad_mot_stop_all(int i2c_file, __u8 *v){
    int watchdog = 0;
    if(uquad_mot_set_speed_all(i2c_file, v,1) < 0)
    {
	backtrace();
	return NOT_OK;
    }	
    while(uquad_mot_disable_all(i2c_file) < 0)
    {
	// wait for stop
	if(++watchdog == UQUAD_STOP_RETRIES)
	{
	    backtrace();
	    return NOT_OK;
	}
    }
    return OK;
}
	
void uquad_sig_handler(int signal_num){
    int ret = OK;
    fprintf(LOG_ERR,"Caught signal %d.\n",signal_num);
    if( i2c_file>= 0 )
    {
	fprintf(LOG_ERR,"Shutting down motors...\n");
	while(uquad_mot_disable_all(i2c_file) != OK)
	    fprintf(LOG_ERR,"Failed to shutdown motors!... Retrying...\n");
	fprintf(LOG_ERR,"Motors successfully stopped!\n");
    }
#ifdef LOG_VELS
    fclose(log_rx);
#endif // LOG_VELS
    fclose(LOG_ERR);
    exit(ret);
}

#ifdef LOG_VELS
unsigned long rx_counter = 0;
static inline void log_vels(void)
{
    gettimeofday(&timestamp,NULL);
    fprintf(log_rx,"%d\t%d\t%d\t%d\t%d\t%lu\n",
	    (int)vels[0],
	    (int)vels[1],
	    (int)vels[2],
	    (int)vels[3],
	    (int)timestamp.tv_usec,
	    rx_counter++);
}
#endif // LOG_VELS

static char ack_counter = 0;
int uquad_send_ack()
{
    int msqid, retries;
    message_buf_t ack_msg;
    ack_msg.mtype = 1;
    if ((msqid = msgget(key_c, IPC_CREAT | 0666 )) < 0)
    {
	fprintf(LOG_ERR,"msgget failed!:%s",strerror(errno));
	return NOT_OK;
    }
    ack_msg.mtext[0] = 'A';
    ack_msg.mtext[1] = 'C';
    ack_msg.mtext[2] = 'K';
    ack_msg.mtext[3] = ack_counter++;
    /// send msg
    retries = 0;
    if (msgsnd(msqid, &ack_msg, MOT_COUNT, IPC_NOWAIT) < 0)
    {
	fprintf(LOG_ERR,"msgsnd failed!:%s",strerror(errno));
	fflush(LOG_ERR);
	return NOT_OK;
    }
    return OK;
}

#ifdef CHECK_STDIN
static int itmp[MOT_COUNT] = {0,0,0,0};
#endif
int uquad_read(void){
    int retval = OK, i;
    
#ifdef CHECK_STDIN
    fd_set rfds;
    struct timeval tv;
    // Watch stdin and ctrl_file, check for speed updates
    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);

    // Don't wait
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    /// get speed data from stdin
    retval = select(STDIN_FILENO+1, &rfds, NULL, NULL, &tv);
    // Don't rely on the value of tv now!

    if (retval < 0)
    {
	log_to_err("select()");
	return NOT_OK;
    }
    else
    {
	if (retval == 0)
	    return NOT_OK;
	else
	{
	    if(!FD_ISSET(STDIN_FILENO, &rfds))
		return NOT_OK;
	    retval = fscanf(stdin,"%d %d %d %d",
			 itmp + 0,
			 itmp + 1,
			 itmp + 2,
			 itmp + 3
			 );
	    if (retval < MOT_COUNT)
		return NOT_OK;
	    else
		retval = OK;
	    for(i=0; i < MOT_COUNT; ++i)
	    	if( itmp[i] >= 0 && itmp[i] < MAX_SPEED)
	    	{
	    	    vels[i] = (__u8)itmp[i];
	    	}
	    	else
	    	{
	    	    log_to_err("Refused to set m speed, invalid argument.");
	    	}
#ifdef LOG_VELS
	    log_vels();
#endif // LOG_VELS
	}
    }
#else // CHECK_STDIN
    __u8 u8tmp;
    /// get speed data from kernel msgq
    if ((msqid = msgget(key_s, 0666)) < 0)
	return NOT_OK;
    
    /*
     * Receive an answer of message type 1.
     */
    if (msgrcv(msqid, &rbuf, 4, 1, IPC_NOWAIT) < 0)
	return NOT_OK;

    /*
     * Print the answer.
     */
    if(retval == OK)
    {
	for(i=0; i < MOT_COUNT; ++i)
	{
	    u8tmp = 0xff & rbuf.mtext[i];
	    if(u8tmp < MAX_SPEED)
		vels[i] = u8tmp;
	    else
	    {
		log_to_err("Refused to set m speed, invalid argument.");
	    }
	}
#ifdef LOG_VELS
	log_vels();
#endif // LOG_VELS
    }
#endif
    return retval;
}

int main(int argc, char *argv[])
{
    /* Open i2c bus */
    int adapter_nr = 2;
    int tmp, i;
    char filename[20];
    double dtmp;
    struct timeval
	tv_in,
	tv_diff,
	tv_end;

#ifdef LOG_VELS
    // Open log files
    log_rx = fopen("cmd_rx.log","w");
    if (log_rx == NULL) {
	fprintf(LOG_ERR,"ERROR! Failed to open log...\n");
	return -1;
    }
#endif // LOG_VELS

    if(argc != 5) // vel de los 4 motores + el nombre del programa (argv[0]).
    {
	if(argc == 1)
	{
	    fprintf(LOG_ERR,"Input speed and press RET to set.\n");
	    for(i=0;i<MOT_COUNT;++i)
		mot_selected[i] = 1;
	}
	else
	{
	    printf(USAGE_STR);
	    return -1;
	}
    }else{
	for(i=0;i<4;++i)
	{
	    tmp = atoi(argv[i+1]);
	    if((tmp != MOT_NOT_SELECTED) && (tmp != MOT_SELECTED))
	    {
		fprintf(LOG_ERR,"Valid arguments are:\n"\
			"\tSelected:\t%d\n"\
			"\tNot selected:\t%d\n",
		       MOT_SELECTED, MOT_NOT_SELECTED);
		return -1;
	    }
	    mot_selected[i] = (unsigned short) tmp;
	    fprintf(LOG_ERR,"Mot %d:\t%s\n",i,mot_selected[i]?"Enabled.":"Not enabled");
	}
    }

    /// Override input (for testing)
    /* mot_selected[0] = MOT_NOT_SELECTED; */
    /* mot_selected[1] = MOT_NOT_SELECTED; */
    /* mot_selected[2] = MOT_NOT_SELECTED; */
    /* mot_selected[3] = MOT_NOT_SELECTED; */

    sprintf(filename,"/dev/i2c-%d",adapter_nr);
    fprintf(LOG_ERR,"Opening %s...\n",filename);

    // Open ctrl interface

    // Open i2c
#ifndef PC_TEST
    if ((i2c_file = open(filename,O_RDWR | O_NONBLOCK)) < 0) {
	fprintf(LOG_ERR,"ERROR! Failed to open %s!\nAborting...\n",filename);
	fprintf(LOG_ERR,"errno info:\t %s\n",strerror(errno));
	/* ERROR HANDLING; you can check errno to see what went wrong */
	return -1;
    }
#else
#if DEBUG
    i2c_fake = fopen(FAKE_I2C_PATH,"w");
    if (i2c_fake == NULL) {
	fprintf(LOG_ERR,"ERROR! Failed to open %s!\nAborting...\n",filename);
	fprintf(LOG_ERR,"errno info:\t %s\n",strerror(errno));
	/* ERROR HANDLING; you can check errno to see what went wrong */
	return -1;
    }
#endif
#endif

    // Catch signals
    signal(SIGINT, uquad_sig_handler);
    signal(SIGQUIT, uquad_sig_handler);
    
    fprintf(LOG_ERR,"%s successfully opened!\n\nEntering loop, Ctrl+C to quit...\n\n",filename);
    fflush(LOG_ERR);

    // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    // Loop
    // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    int ret;
    mot_state_t m_status;
    m_status = STOPPED;
    int do_sleep = 0;
    ret = uquad_mot_enable_all(i2c_file);
    // motors are disabled when Ctrl+C
    if(ret<0)
    {
	backtrace();
	return NOT_OK;
    }
    for(;;)
    {
	gettimeofday(&tv_in,NULL);
	if(do_sleep)
	{
	    // avoid saturating i2c driver
	    log_to_err("Will sleep to avoid saturating.");
	    fflush(LOG_ERR);
	    sleep_ms(10);
	    do_sleep = 0;
	}
	// all
#if UQUAD_USE_DIFF
	if(m_status == RUNNING)
	{
#if UQUAD_USE_SIN
	    tv2double(dtmp,tv_in);
	    dtmp = 5.0*sin(2.0*3.14*3.0*dtmp); // 17Hz sin()
#else // UQUAD_USE_SIN
#endif // UQUAD_USE_SIN
	}
	else
#endif // UQUAD_USE_DIFF
	    dtmp = 0.0;
	for(i = 0; i < MOT_COUNT; ++i)
	{
	    ret = uquad_mot_set_speed(i2c_file,
				      mot_i2c_addr[i],
				      mot_selected[i]?
				      (__u8) (((double)vels[i])+dtmp):0);
	    if(ret != OK)
	    {
		backtrace();
		do_sleep = 1;
		break;
	    }
	}
	if (ret != OK)
	{
	    do_sleep = 1;
	    continue;
	}
	ret = uquad_read();
	if(ret == OK)
	{
	    /// we got new a new speed setting, apply it and ack.

	    /// 1. apply new speed

	    // startup
	    if( (m_status == STOPPED) && (
			     (vels[0] > MIN_SPEED)&&
			     (vels[1] > MIN_SPEED)&&
			     (vels[2] > MIN_SPEED)&&
			     (vels[3] > MIN_SPEED)))
	    {
		log_to_err("Will startup motors...\n");
		ret = uquad_mot_startup_all(i2c_file);
		if(ret != OK)
		{
		    fprintf(LOG_ERR,"FAILED to startup motors...\n");
		    do_sleep = 1;
		}
		else
		{
		    fprintf(LOG_ERR,"Startup was successfull!...\n");
		    m_status = RUNNING;
		    gettimeofday(&tv_in,NULL); // Restart timer
		}
	    }
	    // stop
	    if ( (ret == OK) &&
		 (m_status == RUNNING) && (
					   (vels[0] == 0) ||
					   (vels[1] == 0) ||
					   (vels[2] == 0) ||
					   (vels[3] == 0)
					   )
		 )
	    {
		log_to_err("Will stop motors...\n");
		ret = uquad_mot_stop_all(i2c_file, vels);
		if(ret != OK)
		{
		    log_to_err("FAILED to stop motors...");
		    do_sleep = 1;
		}
		else
		{
		    log_to_err("Stopping was successfull!...");
		    m_status = STOPPED;
		    gettimeofday(&tv_in,NULL); // Restart timer
		}
	    }
	    /// send ack
	    ret = uquad_send_ack();
	    if(ret != OK)
	    {
		log_to_err("Failed to send ack!");
	    }
	    // continue
	}
	gettimeofday(&tv_end,NULL);
	/// Check if we have to wait a while
	ret = uquad_timeval_substract(&tv_diff, tv_end, tv_in);
	if(ret > 0)
	{
	    if(tv_diff.tv_usec < LOOP_T_US)
	    {
		usleep(LOOP_T_US - tv_diff.tv_usec);
	    }
#ifdef LOG_TIMING
	    else
	    {
		if(tv_diff.tv_sec > 0)
		{
		    tv2double(dtmp,tv_diff);
		    fprintf(LOG_ERR,"ERR: Loop took too long! (%0.6f)\nAborting...", dtmp);
		}
		else
		{
		    fprintf(LOG_ERR, "WARN: Loop should be %lu but was %lu\n", LOOP_T_US, tv_diff.tv_usec);
		}
	    }
#endif
	}
	else
	{
	    log_to_err("WARN: Absurd timing!");
	}
    }
 
    return 0;
}
