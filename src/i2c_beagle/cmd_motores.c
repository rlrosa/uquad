#include <errno.h> 
#include <string.h> 
#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h> //#include <linux/i2c-dev.h>
#include <linux/i2c-dev-user.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <fcntl.h>

#define USAGE_STR "Necesito mas argumentos!\nPara ejecutar el programa correr:\n\t./cmd v1 v2 v3 v4\ndonde vX es la velocidad (en decimal) que se le manda al motor numero X\n\n"

#define MAX_SPEED 255
#define MIN_SPEED 0
#define DEBUG 1
#define LOG_FAILS 2000 // evita saturar la UART o el SSH
#define MAX_FAILS 20000
#define PRINT_COUNT 500
#define RUN_FOREVER 0
#define SLEEP_INC_US 0 // esto es lo que incrementa el sleep dsp de los 4 motores
#define LOOPS 300
#define LOOP_ZEROS_LEN 300
#define SLEEP_DSP_DE_4_MOTORES_INIT_US 700

#define sleep_ms(ms) usleep(1000*ms)

int main(int argc, char *argv[])
{
	/* Open i2c bus */
	int file;
	int adapter_nr = 2;
	int tmp, i;
	int watchdog = 0, success_count = 0;
	FILE * log_file;
	char filename[20];
	__u8 vel[4];
	if(argc != 5) // vel de los 4 motores + el nombre del programa (argv[0]).
	{
		printf(USAGE_STR);
		return -1;
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
	if ((file = open(filename,O_RDWR)) < 0) {
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

	printf("%s abierto con exito!\n\nEntrando al loop, salir con Ctrl+C...\n\n",filename);
	fflush(stdout);

	int addr[4] = {0x69, 0x6a, 0x6b, 0x68}; /* The I2C address */
	int us_sleep = SLEEP_DSP_DE_4_MOTORES_INIT_US; // Esto es lo que espera entre 
	int j,state,index;

	while(1)
	{
	    for (state = 0; state < 2; state++)
	    {
		for (j = 0; j < LOOPS; ++j)
		{
		
		    for (i = 0; i < 4; i++)
		    {
			// Cambiar el orden al en la ultima tirada
			index = (j == LOOPS - 1) ? (3 + i) % 4 : i;
			/* Open device with address addr*/	
			if (ioctl(file,I2C_SLAVE,addr[index]) < 0)
			{
			    /* ERROR HANDLING; you can check errno to see what went wrong */
			    printf("ERROR! No pude enviar 0x%02X al i2c...\nAbortando..\n",addr[index]);
			    printf("errno info:\t %s\n",strerror(errno));
			    return -1;
			}

			/* Write value to the device */
			__u8 registeraaaa = 0xA2; /* Device register to access */
			__s32 res;

			/* Using SMBus commands */
			res = i2c_smbus_write_byte_data(file,registeraaaa,
							state?vel[index]:0);
			if (res < 0)
			{
			    /* ERROR HANDLING: i2c transaction failed */
			    if (! (watchdog % LOG_FAILS) )
			    {
				printf("ERROR! transaccion i2c fracaso... Errores:\t%d\n",watchdog);
				printf("errno info:\t %s\n",strerror(errno));
			    }
#if !RUN_FOREVER
			    if(++watchdog > MAX_FAILS)
			    {
				printf("ERROR! demasiados (%d) errores i2c, abortando...\n\n",watchdog);
				return -1;
			    }
#endif
			}
			else
			{
			    if(success_count > PRINT_COUNT)
			    {
				success_count = 0;
				printf("Motor:\t%d\tVel:%d\n",index,vel[index]);
			    }
			    fprintf(log_file, "Motor:\t%d\tVel:%d\n",index,vel[index]);
			    /* write successful */
			    //sleep_ms(1.95);
			    watchdog = 0;
			}
		    }
		     // loop motores
		    success_count++;
		    usleep(us_sleep);
		} // loop de LOOPS
		if(!state)
		    sleep_ms(420);
		else
		    state = 0;// para que siga en loop xa siempre
	    } // estados (ceros, o velocidades)
	    us_sleep += SLEEP_INC_US;
	    printf("\n\n\nSleep set to %dus\n\n\n",us_sleep);
	}
	// igual no llega aca
	fclose(log_file);
	return 0;
}
