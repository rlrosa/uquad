#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
//#include <linux/i2c-dev.h>
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
#define RUN_FOREVER 0

#define sleep_ms(ms) usleep(1000*ms)

int main(int argc, char *argv[])
{
	/* Open i2c bus */
	int file;
	int adapter_nr = 2;
	int i;
	int tmp;
	int watchdog = 0;
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

	if ((file = open(filename,O_RDWR)) < 0) {
		printf("ERROR! No pude abrir %s!\nAbortando...\n",filename);
		printf("errno info:\t %s\n",strerror(errno));
		/* ERROR HANDLING; you can check errno to see what went wrong */
		return -1;
	}
	printf("%s abierto con exito!\n\nEntrando al loop, salir con Ctrl+C...\n\n",filename);
	fflush(stdout);







//
// Este while es como el otro pero tira siempre 0
//


int addr;
int addresses[]={0x69, 0x6a, 0x6b 0x68};

int mati_cont;
for(mati_cont=1;mati_cont<9;mati_cont++)
{
	for (i = 0; i < 4; i++)
	{	
		/* Open device with address addr*/	
		addr = addresses[i]; /* The I2C address */
		if (ioctl(file,I2C_SLAVE,addr) < 0)
		{
			/* ERROR HANDLING; you can check errno to see what went wrong */
			printf("ERROR! No pude enviar 0x%02X al i2c...\nAbortando..\n",addr);
			printf("errno info:\t %s\n",strerror(errno));
			return -1;
		}

		/* Write value to the device */
		__u8 registeraaaa = 0xA2; /* Device register to access */
		__s32 res;
		char buf[10];

		/* Using SMBus commands */
		res = i2c_smbus_write_byte_data(file,registeraaaa,(__u8)0);
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
			printf("Motor:\t%d\tVel:%d\n",i,(__u8)0);
			/* write successful */
//			sleep_ms(120);
			watchdog = 0;
		}
	} // loop motores
} // EL loop





//
//
// Ahora mando 1 sola vez con el orden cambiado
//
//





for (i = 0; i < 4; i++)
	{	
		/* Open device with address addr*/	
		addr = 0x68 + i; /* The I2C address */
		if (ioctl(file,I2C_SLAVE,addr) < 0)
		{
			/* ERROR HANDLING; you can check errno to see what went wrong */
			printf("ERROR! No pude enviar 0x%02X al i2c...\nAbortando..\n",addr);
			printf("errno info:\t %s\n",strerror(errno));
			return -1;
		}

		/* Write value to the device */
		__u8 registeraaaa = 0xA2; /* Device register to access */
		__s32 res;
		char buf[10];

		/* Using SMBus commands */
		res = i2c_smbus_write_byte_data(file,registeraaaa,(__u8)0);
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
			printf("Motor:\t%d\tVel:%d\n",i,(__u8)0);
			/* write successful */
//			sleep_ms(120);
			watchdog = 0;
		}
	} // loop motores






















	sleep_ms(420);


	while(1)
{
	for (i = 0; i < 4; i++)
	{	
		/* Open device with address addr*/	
		addr = addresses[i]; /* The I2C address */
		if (ioctl(file,I2C_SLAVE,addr) < 0)
		{
			/* ERROR HANDLING; you can check errno to see what went wrong */
			printf("ERROR! No pude enviar 0x%02X al i2c...\nAbortando..\n",addr);
			printf("errno info:\t %s\n",strerror(errno));
			return -1;
		}

		/* Write value to the device */
		__u8 registeraaaa = 0xA2; /* Device register to access */
		__s32 res;
		char buf[10];

		/* Using SMBus commands */
		res = i2c_smbus_write_byte_data(file,registeraaaa,vel[i]);
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
			printf("Motor:\t%d\tVel:%d\n",i,vel[i]);
			/* write successful */
//			sleep_ms(120);
			watchdog = 0;
		}
	} // loop motores
} // EL loop
return 0;

}
