#include <avr/io.h>
#include "i2cmaster.h"

#define reg  0xA2
#define val  0x32

int main(void)
{
    unsigned char ret;
    
    DDRB  = 0xff;                              // use all pins on port B for output 
    PORTB = 0xff;                              // (active low LED's )

    i2c_init();                                // init I2C interface
	
	int i;
	int add [] = {0x68,0x69,0x6a,0x6b};

	while(1){
		for(i=0;i<4;i++)
			/* Escribir al motor */
			ret = i2c_start(add[i]+I2C_WRITE);       // set device address and write mode
			if ( ret ) {
				/* failed to issue start condition, possibly no device found */
				i2c_stop();
				PORTB=0x00;                            // activate all 8 LED to show error */
			}else {
				/* issuing start condition ok, device accessible */
				i2c_write(reg);                       // write address = reg
				i2c_write(val);                       // ret=0 -> Ok, ret=1 -> no ACK 
				i2c_stop();                            // set stop conditon = release bus
			}
		}
		
	}
    

}
