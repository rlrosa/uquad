#include <errno.h> 
#include <string.h> 
#include <stdio.h> 
#include <signal.h> // for SIGINT, SIGQUIT
#include <stdlib.h> 
#include <unistd.h> //#include <linux/i2c-dev.h>
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

#include <time.h>

#define USAGE_STR "Incorrect arguments! Use no arguments (all enabled), or 4, enabling each motor.\n\n\t./cmd ena9 enaA enaB ena8\nwhere enaX enables motor X, valid is 0 or 1\n\n"

#define MOT_COMM_FILE "mot_control.file" // must match mot_control.h

#define MAX_SPEED 220
#define MIN_SPEED 0
#define DEBUG 1
#define LOG_FAILS 2000 // evita saturar la UART o el SSH
#define MAX_FAILS 20000
#define PRINT_COUNT 500
#define RUN_FOREVER 0
#define SLEEP_INC_US 0 // esto es lo que incrementa el sleep dsp de los 4 motores
#define LOOPS 10000
#define ZERO_LOOPS 200
#define FIN_LOOPS 1
#define MOT_COUNT 4
#define UQUAD_MOT_I2C_REG_ENABLE 0xA0
#define UQUAD_MOT_ENABLE_DAT 0x00
#define UQUAD_MOT_I2C_REG_DISABLE 0xA1
#define UQUAD_MOT_DISABLE_DAT 0x00
#define UQUAD_MOT_I2C_REG_VEL 0xA2

#define OK 0
#define NOT_OK -1

#define UQUAD_STARTUP_RETRIES 100
#define UQUAD_STOP_RETRIES 1000

#define backtrace() fprintf(log_err,"%s:%d\n",__FUNCTION__,__LINE__)
#define log_to_err(msg) fprintf(log_err,"%s: %s:%d\n",msg,__FUNCTION__,__LINE__)
#define log_vels() fprintf(log_rx,"%d\t%d\t%d\t%d\n",(int)vels[0],(int)vels[1],(int)vels[2],(int)vels[3]);

#define sleep_ms(ms) usleep(1000*ms)

typedef enum {RUNNING,STOPPED}mot_state_t;

// Global vars
static int i2c_file = -1;
static int ctrl_file_fd = -1;
static int mot_i2c_addr[MOT_COUNT] = {0x69,
				       0x6a,
				       0x6b,
				       0x68};
#define MOT_SELECTED 1
#define MOT_NOT_SELECTED 0
static unsigned short mot_selected[MOT_COUNT] = {MOT_NOT_SELECTED,
						 MOT_NOT_SELECTED,
						 MOT_NOT_SELECTED,
						 MOT_NOT_SELECTED};

static __u8 vels[MOT_COUNT] = {0,0,0,0};

static FILE *ctrl_file, *log_rx, *log_err;

// Forwards defs
int uquad_mot_i2c_addr_open(int i2c_dev, int addr);
int uquad_mot_i2c_send_byte(int i2c_dev, __u8 reg, __u8 value);
int uquad_mot_i2c_send(int i2c_dev, int i2c_addr, __u8 reg, __u8 val);

int uquad_mot_enable(int i2c_dev, int mot_i2c_addr);
int uquad_mot_enable_all(int i2c_dev);
int uquad_mot_disable(int i2c_dev, int mot_i2c_addr);
int uquad_mot_disable_all(int i2c_dev);
int uquad_mot_set_speed(int i2c_dev, int mot_i2c_addr, __u8 val);
int uquad_mot_set_speed_all(int i2c_dev, __u8 *v, int swap_order);
// Aux
void uquad_sigint_handler(int signal_num);

int uquad_mot_i2c_addr_open(int i2c_dev, int addr){
    if (ioctl(i2c_dev,I2C_SLAVE,addr) < 0)
    {
	/* ERROR HANDLING; you can check errno to see what went wrong */
	printf("ERROR! %s failed to write to 0x%02X...\n",__FUNCTION__,addr);
	printf("errno info:\t %s\n",strerror(errno));
	return NOT_OK;
    }
    return OK;
}

int uquad_mot_i2c_send_byte(int i2c_dev, __u8 reg, __u8 value){
    int ret;
    if(i2c_smbus_write_byte_data(i2c_dev,reg,value) < 0)
    {
	/* ERROR HANDLING: i2c transaction failed */
	fprintf(stderr,"Failed to send value %d\tto 0x%02X\n",(int)value,(int)reg);
	printf("errno info:\t %s\n",strerror(errno));
    }
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
    int i, watchdog = 0, enable_counts = 0;
    if(uquad_mot_set_speed_all(i2c_file, NULL,1) < 0)
    {
	backtrace();
	return NOT_OK;
    }
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
	

void uquad_sigint_handler(int signal_num){
    int ret = OK;
    printf("Caught signal %d.\n",signal_num);
    if( i2c_file>= 0 )
    {
	printf("Shutting down motors...\n");
	while(uquad_mot_disable_all(i2c_file) != OK)
	    fprintf(stderr,"Failed to shutdown motors!... Retrying...\n");
	printf("Motors successfully stoped!\n");
    }
    fclose(log_rx);
    fclose(log_err);
    exit(ret);
}

static int itmp[MOT_COUNT] = {0,0,0,0};
static int max_fd_plus_one = -1;
int uquad_read(void){
    fd_set rfds;
    FILE *src;
    struct timeval tv;
    int retval = OK, i;
    
    // Watch stdin and ctrl_file, check for speed updates
    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);
    FD_SET(ctrl_file_fd, &rfds);

    // Don't wait
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    retval = select(max_fd_plus_one, &rfds, NULL, NULL, &tv);
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
	    //	    src = FD_ISSET(STDIN_FILENO, &rfds) ? stdin:ctrl_file;
	    if(FD_ISSET(STDIN_FILENO, &rfds))
		src = stdin;
	    else
		if(FD_ISSET(ctrl_file_fd, &rfds))
		    src = ctrl_file;
		else
		    return NOT_OK;
	    retval = fscanf(src,"%d %d %d %d",
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
	    log_vels();
	}
    }
    return OK;
}

int main(int argc, char *argv[])
{
    /* Open i2c bus */
    int adapter_nr = 2;
    int tmp, i;
    int watchdog = 0, success_count = 0;
    char filename[20];

    // Open log files
    log_rx = fopen("cmd_rx.log","w");
    if (log_rx == NULL) {
	printf("ERROR! Failed to open log...\n");
	return -1;
    }

    log_err = fopen("cmd_err.log","w");
    if (log_err == NULL) {
	printf("ERROR! Failed to open log...\n");
	return -1;
    }

    if(argc != 5) // vel de los 4 motores + el nombre del programa (argv[0]).
    {
	if(argc == 1)
	{
	    printf("Input speed and press RET to set.\n");
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
		printf("Valid arguments are:\n\tSelected:\t%d\n\tNot selected:\t%d\n",
		       MOT_SELECTED, MOT_NOT_SELECTED);
		return -1;
	    }
	    mot_selected[i] = (unsigned short) tmp;
	    fprintf(log_err,"Mot %d:\t%s\n",i,mot_selected[i]?"Enabled.":"Not enabled");
	}
    }
    
    sprintf(filename,"/dev/i2c-%d",adapter_nr);
    fprintf(log_err,"Opening %s...\n",filename);

    // Open ctrl interface
    if ((ctrl_file = fopen(MOT_COMM_FILE,"w+")) < 0) {
	fprintf(log_err,"ERROR! Failed to open %s!\nAborting...\n",MOT_COMM_FILE);
	fprintf(log_err,"errno info:\t %s\n",strerror(errno));
	/* ERROR HANDLING; you can check errno to see what went wrong */
	return -1;
    }
    ctrl_file_fd = fileno(ctrl_file);
    max_fd_plus_one = (STDIN_FILENO>ctrl_file_fd)?
	STDIN_FILENO:ctrl_file_fd;
    max_fd_plus_one++;

    // Open i2c
    if ((i2c_file = open(filename,O_RDWR)) < 0) {
	fprintf(log_err,"ERROR! Failed to open %s!\nAborting...\n",filename);
	fprintf(log_err,"errno info:\t %s\n",strerror(errno));
	/* ERROR HANDLING; you can check errno to see what went wrong */
	return -1;
    }

    // Catch signals
    signal(SIGINT, uquad_sigint_handler);
    signal(SIGQUIT, uquad_sigint_handler);
    
    fprintf(log_err,"%s successfully opened!\n\nEntering loop, Ctrl+C to quit...\n\n",filename);
    fflush(log_err);

    // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    // Loop
    // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    int ret, new_vel;
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
	if(do_sleep)
	{
	    // avoid saturating i2c driver
	    log_to_err("Will sleep to avoid saturating.");
	    sleep_ms(10);
	    do_sleep = 0;
	}
	// all
	if(m_status == RUNNING)
	{
	    for(i = 0; i < MOT_COUNT; ++i)
	    {
		ret = uquad_mot_set_speed(i2c_file,
					  mot_i2c_addr[i],
					  (__u8)vels[i]);	
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
	}
	ret = uquad_read();
	if(ret == OK)
	{
	    // startup
	    if( (m_status == STOPPED) && (
			     (vels[0] > 0)||
			     (vels[1] > 0)||
			     (vels[2] > 0)||
			     (vels[3] > 0)))
	    {
		fprintf(log_err,"Will startup motors...\n");
		ret = uquad_mot_startup_all(i2c_file);
		if(ret != OK)
		{
		    fprintf(log_err,"FAILED to startup motors...\n");
		    do_sleep = 1;
		    continue;
		}
		else
		{
		    fprintf(log_err,"Startup was successfull!...\n");
		    m_status = RUNNING;
		}
	    }
	    // stop
	    if( (m_status == RUNNING) && (
			   (vels[0] == 0) ||
			   (vels[1] == 0) ||
			   (vels[2] == 0) ||
			   (vels[3] == 0)))
	       {
		fprintf(log_err,"Will stop motors...\n");
		ret = uquad_mot_stop_all(i2c_file, vels);
		if(ret != OK)
		{
		    fprintf(log_err,"FAILED to stop motors...\n");
		    do_sleep = 1;
		    continue;
		}
		else
		{
		    fprintf(log_err,"Stopping was successfull!...\n");
		    m_status = STOPPED;
		}
	    }
	    // continue
	}
	usleep(1500);
    }
 
    return 0;
}
