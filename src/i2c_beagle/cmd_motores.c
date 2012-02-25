#include <errno.h> 
#include <string.h> 
#include <stdio.h> 
#include <signal.h> // for SIGINT, SIGQUIT
#include <stdlib.h> 
#include <unistd.h> //#include <linux/i2c-dev.h>
#include <linux/i2c-dev-user.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <fcntl.h>

#include <time.h>

#define USAGE_STR "Incorrect arguments! Use no arguments (all enabled), or 4, enabling each motor.\n\n\t./cmd ena9 enaA enaB ena8\nwhere enaX enables motor X, valid is 0 or 1\n\n"

#define MAX_SPEED 255
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

#define backtrace() fprintf(stderr,"%s:%d\n",__FUNCTION__,__LINE__)

#define sleep_ms(ms) usleep(1000*ms)

// Global vars
static int i2c_file = -1;
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

// Forwards defs
int uquad_mot_i2c_addr_open(int i2c_dev, int addr);
int uquad_mot_i2c_send_byte(int i2c_dev, __u8 reg, __u8 value);
int uquad_mot_i2c_send(int i2c_dev, int i2c_addr, __u8 reg, __u8 val);

int uquad_mot_enable(int i2c_dev, int mot_i2c_addr);
int uquad_mot_enable_all(int i2c_dev);
int uquad_mot_disable(int i2c_dev, int mot_i2c_addr);
int uquad_mot_disable_all(int i2c_dev);
int uquad_mot_set_speed(int i2c_dev, int mot_i2c_addr, __u8 val);
int uquad_mot_set_speed_all(int i2c_dev, __u8 val, int swap_order);
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

int uquad_mot_set_speed_all(int i2c_dev, __u8 val, int swap_order){
    int i;
    for(i=0;i<MOT_COUNT;++i)
    {
	if(!mot_selected[i])
	    continue;
	if(uquad_mot_set_speed(i2c_dev,
			       mot_i2c_addr[swap_order?(3+i)%4:i],
			       val))
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
    if(uquad_mot_set_speed_all(i2c_file, 0,1) < 0)
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

int uquad_mot_stop_all(int i2c_file, __u8 curr_vel){
    int watchdog = 0;
    if(uquad_mot_set_speed_all(i2c_file, curr_vel,1) < 0)
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
    exit(ret);
}

int uquad_read_stdin(void){
    fd_set rfds;
    struct timeval tv;
    int retval;
    int vel = -1;
    
    // Watch stdin to see when it has input.
    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);

    // Don't wait
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    retval = select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv);
    // Don't rely on the value of tv now!

    if (retval < 0)
    {
	perror("select()");
	exit(retval);
    }
    else if (retval && FD_ISSET(STDIN_FILENO, &rfds))
    {
	scanf("%d", &vel);
    }
    return vel;
}

int main(int argc, char *argv[])
{
    /* Open i2c bus */
    int adapter_nr = 2;
    int tmp, i;
    int watchdog = 0, success_count = 0;
    FILE * log_file;
    char filename[20];
    //    __u8 vel[4];
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
		       MOT_NOT_SELECTED);
		return -1;
	    }
	    mot_selected[i] = (unsigned short) tmp;
#if DEBUG
	    printf("Mot %d:\t%s\n",i,mot_selected[i]?"Enabled.":"Not enabled");
#endif
	}
    }
    
    sprintf(filename,"/dev/i2c-%d",adapter_nr);
    printf("Opening %s...\n",filename);
    
    // Abrir i2c
    if ((i2c_file = open(filename,O_RDWR)) < 0) {
	printf("ERROR! Failed to open %s!\nAborting...\n",filename);
	printf("errno info:\t %s\n",strerror(errno));
	/* ERROR HANDLING; you can check errno to see what went wrong */
	return -1;
    }
    
    // Abrir log
    log_file = fopen("cmd_out.log","w");
    if (log_file == NULL) {
	printf("ERROR! Failed to open log...\n");
	return -1;
    }

    // Catch signals
    signal(SIGINT, uquad_sigint_handler);
    signal(SIGQUIT, uquad_sigint_handler);
    
    printf("%s successfully opened!\n\nEntering loop, Ctrl+C to quit...\n\n",filename);
    fflush(stdout);

    // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    // Loop
    // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    int ret, new_vel;
    __u8 curr_vel = 0;
    ret = uquad_mot_enable_all(i2c_file);
    // motors are disabled when Ctrl+C
    printf("Starting at %c\n", (char)curr_vel);
    if(ret<0)
    {
	backtrace();
	return NOT_OK;
    }
    for(;;)
    { 
	// all
      	ret = uquad_mot_set_speed_all(i2c_file, curr_vel,0);
	if(ret<0)
	{
	    backtrace();
	    return NOT_OK;
	}
	new_vel = uquad_read_stdin();
	if((new_vel >= 0) && (new_vel < 0xff))
	{
	    // startup
	    if((curr_vel == 0) && (new_vel > 0))
	    {
		printf("Will startup motors...\n");
		ret = uquad_mot_startup_all(i2c_file);
		if(ret != OK)
		    printf("FAILED to startup motors...\n");
		else
		    printf("Startup was successfull!...\n");
	    }
	    // stop
	    if((curr_vel > 0) && (new_vel == 0))
	    {
		printf("Will stop motors...\n");
		ret = uquad_mot_stop_all(i2c_file, curr_vel);
		if(ret != OK)
		    printf("FAILED to stop motors...\n");
		else
		    printf("Stopping was successfull!...\n");
	    }
	    // continue
	    curr_vel = (__u8) new_vel;
	    printf("Speed changed to %c\n",curr_vel);
	}
	usleep(1500);
    }
 
    fclose(log_file);
    return 0;
}
