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

#define USAGE_STR "Necesito mas argumentos!\nPara ejecutar el programa correr:\n\t./cmd v1 v2 v3 v4\ndonde vX es la velocidad (en decimal) que se le manda al motor numero X\n\n"

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

#define backtrace() fprintf(stderr,"%s:%d\n",__FUNCTION__,__LINE__)

#define sleep_ms(ms) usleep(1000*ms)

// Global vars
static int i2c_file = -1;
static int mot_i2c_addr[MOT_COUNT] = {0x69,
				       0x6a,
				       0x6b,
				       0x68};

// Forwards defs
int uquad_mot_i2c_addr_open(int i2c_dev, int addr);
int uquad_mot_i2c_send_byte(int i2c_dev, __u8 reg, __u8 value);
int uquad_mot_i2c_send(int i2c_dev, int i2c_addr, __u8 reg, __u8 val);

int uquad_mot_enable(int i2c_dev, int mot_i2c_addr);
int uquad_mot_enable_all(int i2c_dev);
int uquad_mot_disable(int i2c_dev, int mot_i2c_addr);
int uquad_mot_disable_all(int i2c_dev);
int uquad_mot_set_speed(int i2c_dev, int mot_i2c_addr, __u8 val);
int uquad_mot_set_speed_all(int i2c_dev, __u8 val);
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
	backtrace();
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

int uquad_mot_set_speed_all(int i2c_dev, __u8 val){
    int i;
    for(i=0;i<MOT_COUNT;++i)
    {
	if(uquad_mot_set_speed(i2c_dev,mot_i2c_addr[i],val))
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

void uquad_sigint_handler(int signal_num){
    int ret = OK;
    printf("Caught signal %d.\n",signal_num);
    if( i2c_file>= 0 )
    {
	printf("Shutting down motors...\n");
	ret = uquad_mot_disable_all(i2c_file);
	if(ret != OK)
	    fprintf(stderr,"Failed to shutdown motors...\n");
	else
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

    if (retval == -1)
	perror("select()");
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
    __u8 vel[4];
    if(argc != 5) // vel de los 4 motores + el nombre del programa (argv[0]).
    {
	if(argc == 1)
	    printf("Input speed and press RET to set.\n");
	else
	{
	    printf(USAGE_STR);
	    return -1;
	}
    }else{
	for(i=0;i<4;++i)
	{
	    tmp = atoi(argv[i+1]);
	    if((tmp > MAX_SPEED) || (tmp < MIN_SPEED))
	    {
		printf("Velocidades deben ser 0 < Velocidad < 255\n");
		return -1;
	    }
	    vel[i] = (__u8)tmp;
#if DEBUG
	    printf("V%d:\t%d\n",i,vel[i]);
#endif
	}
    }
    
    sprintf(filename,"/dev/i2c-%d",adapter_nr);
    printf("Intentare abrir %s...\n",filename);
    
    // Abrir i2c
    if ((i2c_file = open(filename,O_RDWR)) < 0) {
	printf("ERROR! No pude abrir %s!\nAbortando...\n",filename);
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
    
    printf("%s abierto con exito!\n\nEntrando al loop, salir con Ctrl+C...\n\n",filename);
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
      	ret = uquad_mot_set_speed_all(i2c_file, curr_vel);
	// only 1
	//	ret = uquad_mot_set_speed(i2c_file, 0x68, curr_vel);
	if(ret<0)
	{
	    backtrace();
	    return NOT_OK;
	}
	new_vel = uquad_read_stdin();
	if((new_vel >= 0) && (new_vel < 0xff))
	{
	    curr_vel = (__u8) new_vel;
	    printf("Speed changed to %c\n",curr_vel);
	}	    
    }
 
    fclose(log_file);
    return 0;
}
