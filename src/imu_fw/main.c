/*
    6DOF Atomic
	
	2/25/09, Pete DOkter
	
	Copyright Sparkfun Electronics
	
	This code is "beerware". If you use it for personal gain, you must buy me beer.
	
	
*/


#include <avr/io.h>
#include "rprintf.h"
#include <math.h>
#include <avr/interrupt.h>

#define FOSC 10000000// Clock Speed
#define BAUD 115200
#define MYUBRR FOSC/8/BAUD-1

#define STAT 5	//PB5

#define x_active	5
#define y_active	4
#define z_active	3
#define pitch_active	2
#define roll_active	1
#define yaw_active	0

#define FREQ_LOW 0
#define FREQ_HIGH	1
#define SENSE_AR_MODE	2
#define ACT_CHAN	3

#define GS1 0
#define GS2	1

//Define functions
//======================
void EEPROM_write(unsigned int uiAddress, unsigned char ucData);
unsigned char EEPROM_read(unsigned int uiAddress);
void ioinit(void);      //Initializes IO
void USART_Init( unsigned int ubrr);
void put_char(char byte);
int get_adc(void);
void config_menu(void);
char get_char(void);

void delay_ms(uint16_t x); //General purpose delay
void delay_us(uint8_t x);

//Global Variables
//======================
int x_accel;
int y_accel;
int z_accel;
int pitch;
int roll;
int yaw;

char active_channels = 0b00111111;
char auto_run = 0;
char asc = 1;
float freq = 0;
int tmr_cnt;



int main (void)
{
	char temp;
	short b;

	
    ioinit(); //Setup IO pins and defaults
	USART_Init(10);//set up for 115200
	rprintf_devopen(put_char); /* init rrprintf */
	
	for (b = 0; b < 5; b++)
	{
		PORTB &= (~(1<<STAT));//stat on
		delay_ms(50);
		PORTB |= (1<<STAT);//stat off
		delay_ms(50);
	}
	
	asc = 0;
	auto_run = 0;
	
	//check for existing preset values==============================================================
	temp = EEPROM_read((unsigned int)FREQ_HIGH);
	
	if (temp == 255)//unwritten
	{
		cli();//Disable Interrupts
		
		EEPROM_write((unsigned int) FREQ_LOW, 100);//100Hz
		EEPROM_write((unsigned int) FREQ_HIGH, 0);
		EEPROM_write((unsigned int) SENSE_AR_MODE, 0);//1.5g, auto-run off, binary output
		EEPROM_write((unsigned int) ACT_CHAN, 0x3F);//all channels active
		
		sei();//Enable Interrupts
		
		freq = 100;
		asc = 0;//binary
		auto_run = 0;//auto run off
		//set for 1.5g sensitivity
		PORTB &= (~((1<<GS1) | (1<<GS2)));//GS1 low, GS2 low
	}
	
	//get presets
	else
	{
		b = EEPROM_read((unsigned int)FREQ_HIGH);
		b <<= 8;
		b |= EEPROM_read((unsigned int)FREQ_LOW);
		
		freq = (float)b;
		
		active_channels = EEPROM_read((unsigned int)ACT_CHAN);
		
		temp = EEPROM_read((unsigned int)SENSE_AR_MODE);
		
		if (temp & 0x08) PORTB |= (1<<GS2);//GS2 High
		if (temp & 0x04) PORTB |= (1<<GS1);//GS2 High
		
		if (temp & 0x02) auto_run = 1;
		if (temp & 0x01) asc = 1;
	}
	
	//main loop==================================================================
	while(1)
	{

		if (auto_run == 1)
		{
			while(1)
			{
				//This is the sampling loop. It runs in get_adc() until somebody stops it.
				get_adc();
				
				//If it dumps out of the sampling loop, go to the config menu.
				config_menu();
				
				//Bail out if auto run is off.
				if (auto_run == 0) break;
			}
		}

	
		if (UCSR0A & (1<<RXC0))//if something comes in...
		{
			  
			temp = UDR0;

			if (temp == 35)	//# to run
			{
				asc = 0;
				get_adc();
			
			}
		  
			else if (temp == 37)	//% to set range to 1.5g
			{
				//set for 1.5g sensitivity
				PORTB &= (~((1<<GS1) | (1<<GS2)));//GS1 low, GS2 low
			
			}
			
			else if (temp == 38)	//& to set range to 2g
			{
				PORTB |= (1<<GS1);//GS1 High
				PORTB &= (~(1<<GS2));//GS2 low   
			  
			}
			
			else if (temp == 39)	//' to set range to 4g
			{
				PORTB &= (~(1<<GS1));//GS1 low
				PORTB |= (1<<GS2);//GS2 High
			}
			
			else if (temp == 40)	//( to set range to 6g
			{
				PORTB |= ((1<<GS1) | (1<<GS2));//GS1, GS2 high
				
			}

		  else if (temp == 41) freq = 50;	//) to run at 50Hz
		  
		  else if (temp == 42)	freq = 100;// to run at 100Hz

		  else if (temp == 43)	freq = 150;//+ to run at 150Hz
		  
		  else if (temp == 44)	freq = 200;//, to run at 200Hz
		 
		  else if (temp == 45)	freq = 250;//- to run at 250Hz
		  
		  else if (temp == 32)	//
		  {
			  while(1)
			  {
				  config_menu();

				  if (auto_run == 0) break;

				  get_adc();

				 
			  }
		  }

		  temp = 0;

			  
		}
	}
	
	while(1);
	

}

void ioinit (void)
{
	
    //1 = output, 0 = input

	PORTB |= (1<<STAT);//stat off
	DDRB |= ((1<<STAT) | (1<<GS1) | (1<<GS2));//set PB5 as output
	
	TCCR1B = (1<<CS12);
	TCCR2B = (1<<CS21);
	
}


void USART_Init( unsigned int ubrr)
{
	// Set baud rate 
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	
	// Enable receiver and transmitter 
	UCSR0A = (1<<U2X0);
	UCSR0B = ((1<<RXEN0)|(1<<TXEN0));
	
	// Set frame format: 8data, 2stop bit 
	UCSR0C = ((1<<UCSZ00)|(1<<UCSZ01));

}


//General short delays
void delay_ms(uint16_t x)
{
	for (; x > 0 ; x--)
    {
        delay_us(250);
        delay_us(250);
        delay_us(250);
        delay_us(250);
    }
	
}

//Not really a uS, but not critical for system timing anyway.
void delay_us(uint8_t x)
{
	char temp;
	
	if (x == 0) temp = 1;
	else temp = x;
	
	TIFR2 |= 0x01;//Clear any interrupt flags on Timer2
    
    TCNT2 = 256 - temp;

	while(!(TIFR2 & 0x01));	
	
	
}



void put_char(char byte)
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	/* Put data into buffer, sends the data */
	UDR0 = byte;
}


char get_char(void)
{
	while(!(UCSR0A & (1<<RXC0)));
	return UDR0;
}


int get_adc(void)
{
	int l2;//temp low
	int h2;//temp high
	int time = 0;
	short a;
	char q;
	
	x_accel = 0;
	y_accel = 0;
	z_accel = 0;
	pitch = 0;
	roll = 0;
	yaw = 0;
	
	h2 = 0; l2 = 0;
	
	tmr_cnt = (int)((1/freq)/.0000256);
	
	while(1)
	{
		TIFR1 |= 0x01;
		TCNT1 = 65536 - tmr_cnt;
		
		//PORTB &= (~(1<<STAT));//stat on
		
		x_accel = 0, y_accel = 0, z_accel = 0;
		pitch = 0, roll = 0, yaw = 0;
		
		put_char('A');  //start character
		
		//Send count first================================================================
        if (asc == 0)
        {
            a = ((short)time & 0x0000FF00) / 0x100;
            q = (char)a;
            put_char(q);
            q = (char)time & 0x000000FF;
            put_char(q);
        }

        else if (asc == 1)
        {
            put_char(9);
            rprintf("%d",time);
            put_char(9);
        }
		
		if (active_channels & (1<<x_active))
		{	
			//x_accel conversion====================================================================
			ADMUX = 0x40;
			ADCSRA = (1 << ADEN)|(1 << ADSC)|(1<<ADPS2)|(1<<ADPS1);
			
			while(ADCSRA & (1 << ADSC));
			l2 = (ADCL & 0xFF);
			h2 = (ADCH & 0x03);
			x_accel = ((h2<<8) | l2);
			
			if (asc == 0)
            {
                a = (((short)x_accel & 0xFF00) >> 8);
                q = (char)a;
                put_char(q);
                q = (char)x_accel & 0xFF;
                put_char(q);
            }
    
            else if (asc == 1)
            {
                rprintf("%d",x_accel);
                put_char(9);
            }
			
			h2 = 0; l2 = 0;
		}
		
		
		if (active_channels & (1<<y_active))
		{
			//y_accel conversion===================================================================
			ADMUX = 0x41;
			ADCSRA = (1 << ADEN)|(1 << ADSC)|(1<<ADPS2)|(1<<ADPS1);
			
			while(ADCSRA & (1 << ADSC));
			l2 = (ADCL & 0xFF);
			h2 = (ADCH & 0x03);
			y_accel = ((h2<<8) | l2);
			
			if (asc == 0)
            {
                a = (((short)y_accel & 0xFF00) >> 8);
                q = (char)a;
                put_char(q);
                q = (char)y_accel & 0xFF;
                put_char(q);
            }
    
            else if (asc == 1)
            {
                rprintf("%d",y_accel);
                put_char(9);
            }
			
			h2 = 0; l2 = 0;
		}
		
		
		if (active_channels & (1<<z_active))
		{
			//z_accel conversion=================================================================
			ADMUX = 0x42;
			ADCSRA = (1 << ADEN)|(1 << ADSC)|(1<<ADPS2)|(1<<ADPS1);
			
			while(ADCSRA & (1 << ADSC));
			l2 = (ADCL & 0xFF);
			h2 = (ADCH & 0x03);
			z_accel = ((h2<<8) | l2);
			
			if (asc == 0)
            {
                a = (((short)z_accel & 0xFF00) >> 8);
                q = (char)a;
                put_char(q);
                q = (char)z_accel & 0xFF;
                put_char(q);
            }
    
            else if (asc == 1)
            {
                rprintf("%d",z_accel);
                put_char(9);
            }
			
			h2 = 0; l2 = 0;
		}
		
		//PORTB |= (1<<STAT);//stat off
		
		if (active_channels & (1<<pitch_active))
		{
			//pitch conversion=================================================================
			ADMUX = 0x43;
			ADCSRA = (1 << ADEN)|(1 << ADSC)|(1<<ADPS2)|(1<<ADPS1);
			
			while(ADCSRA & (1 << ADSC));
			l2 = (ADCL & 0xFF);
			h2 = (ADCH & 0x03);
			pitch = ((h2<<8) | l2);
			
			if (asc == 0)
            {
                a = (((short)pitch & 0xFF00) >> 8);
                q = (char)a;
                put_char(q);
                q = (char)pitch & 0xFF;
                put_char(q);
            }
    
            else if (asc == 1)
            {
                rprintf("%d",pitch);
                put_char(9);
            }
			
			h2 = 0; l2 = 0;
		}
		
		
		if (active_channels & (1<<roll_active))
		{
			//roll conversion===================================================================
			ADMUX = 0x44;
			ADCSRA = (1 << ADEN)|(1 << ADSC)|(1<<ADPS2)|(1<<ADPS1);
			
			while(ADCSRA & (1 << ADSC));
			l2 = (ADCL & 0xFF);
			h2 = (ADCH & 0x03);
			roll = ((h2<<8) | l2);
			
			if (asc == 0)
            {
                a = (((short)roll & 0xFF00) >> 8);
                q = (char)a;
                put_char(q);
                q = (char)roll & 0xFF;
                put_char(q);
            }
    
            else if (asc == 1)
            {
                rprintf("%d",roll);
                put_char(9);
            }
			
			h2 = 0; l2 = 0;
		}
		
		
		if (active_channels & (1<<yaw_active))
		{
			//yaw conversion======================================================================
			ADMUX = 0x45;
			ADCSRA = (1 << ADEN)|(1 << ADSC)|(1<<ADPS2)|(1<<ADPS1);
			
			while(ADCSRA & (1 << ADSC));
			l2 = (ADCL & 0xFF);
			h2 = (ADCH & 0x03);
			yaw = ((h2<<8) | l2);
			
			if (asc == 0)
            {
                a = (((short)yaw & 0xFF00) >> 8);
                q = (char)a;
                put_char(q);
                q = (char)yaw & 0xFF;
                put_char(q);
            }
    
            else if (asc == 1)
            {
                rprintf("%d",yaw);
                put_char(9);
            }
			
			h2 = 0; l2 = 0;
		}
		
		//Stop Character Z
        put_char('Z');

        if (asc == 1)
        {
            put_char(10);
            put_char(13);
        }
		
		time++;
        if (time >= 0x8000) time = 0;
		
		if (time % 64 == 0) PORTB ^= (1<<STAT);
		
		if (TIFR1 & 0x01)
		{
			rprintf("\r\n\nToo much data.  Please lower your sample frequency\r\n\n",0);
			break;
		}
		
		
		while(!(TIFR1 & 0x01));//wait for timer

		
		if (UCSR0A & (1<<RXC0))//if something comes in...
		{
			q = UDR0;
			if (q == 32) break;
		}
		
		
	}

	return;

}

void EEPROM_write(unsigned int uiAddress, unsigned char ucData)
{
/* Wait for completion of previous write */
while(EECR & (1<<EEPE))
;
/* Set up address and Data Registers */
EEAR = uiAddress;
EEDR = ucData;
/* Write logical one to EEMPE */
EECR |= (1<<EEMPE);
/* Start eeprom write by setting EEPE */
EECR |= (1<<EEPE);
}

unsigned char EEPROM_read(unsigned int uiAddress)
{
/* Wait for completion of previous write */
while(EECR & (1<<EEPE))
;
/* Set up address register */
EEAR = uiAddress;
/* Start eeprom read by writing EERE */
EECR |= (1<<EERE);
/* Return data from Data Register */
return EEDR;
}


//All the configuration happens here...
void config_menu(void)
{
	char a, temp_sense;
	short temp_freq;
	
	temp_freq = (short)freq;
	
	temp_sense = EEPROM_read((unsigned int)SENSE_AR_MODE);
	temp_sense &= 0x0C;
	
	PORTB &= (~(1<<STAT));//stat on
	
	while (1)
	{
		rprintf("\r\n6DOF Atomic setup, version 1.0\r\n",0);
		rprintf("========================================\r\n",0);
		
		rprintf("1) View/edit active channel list\r\n",0);
		rprintf("2) Change output mode, currently ",0);
		if (asc == 0) rprintf("binary\r\n",0);
		else rprintf("ASCII\r\n",0);
		rprintf("3) Set Auto run mode, currently ",0);
		if (auto_run == 0) rprintf("off\r\n",0);
		else rprintf("on\r\n",0);
		rprintf("4) Set accelerometer sensitivity, currently ",0);
		if ((temp_sense & 0x0C) == 0) rprintf("1.5g\r\n",0);
		else if ((temp_sense & 0x0C) == 0x04) rprintf("2g\r\n",0);
		else if ((temp_sense & 0x0C) == 0x08) rprintf("4g\r\n",0);
		else if ((temp_sense & 0x0C) == 0x0C) rprintf("6g\r\n",0);		
		rprintf("5) Set output frequency, currently %d\r\n",temp_freq);
       
		rprintf("9) Save settings and run unit\r\n",0);
		
		a = get_char();
		
		//change active channel list===================================================
		if (a == '1')
		{
			while(1)
			{
				rprintf("1) Accel X = ",0);
				if (active_channels & 0x20) rprintf("on\r\n",0);
				else rprintf("off\r\n",0);

				rprintf("2) Accel Y = ",0);
				if (active_channels & 0x10) rprintf("on\r\n",0);
				else rprintf("off\r\n",0);

				rprintf("3) Accel Z = ",0);
				if (active_channels & 0x08) rprintf("on\r\n",0);
				else rprintf("off\r\n",0);

				rprintf("4) Pitch = ",0);
				if (active_channels & 0x04) rprintf("on\r\n",0);
				else rprintf("off\r\n",0);

				rprintf("5) Roll = ",0);
				if (active_channels & 0x02) rprintf("on\r\n",0);
				else rprintf("off\r\n",0);

				rprintf("6) Yaw = ",0);
				if (active_channels & 0x01) rprintf("on\r\n",0);
				else rprintf("off\r\n",0);

				rprintf("Press the number of the channel you wish to change,\r\n",0);
				rprintf("or press x to exit\r\n",0);

				a = get_char();

				if (a == '1')
				{
					active_channels ^= 0x20;
				}

				else if (a == '2')
				{
					active_channels ^= 0x10;
				}
				
				else if (a == '3')
				{
					active_channels ^= 0x08;
				}

				else if (a == '4')
				{
					active_channels ^= 0x04;
				}

				else if (a == '5')
				{
					active_channels ^= 0x02;
				}

				else if (a == '6')
				{
					active_channels ^= 0x01;
				}

				else if (a == 'x')
				{
					EEPROM_write((unsigned int) ACT_CHAN, active_channels);
					a = 0;
					break;
				}

			}

		}

        //change ascii/binary==================================================================    
		else if (a == '2')
		{
			asc ^= 1;
			if (asc == 0) temp_sense &= 0xFE;
			else if (asc == 1) temp_sense |= 0x01;
		}
		
		//change auto_run======================================================================
		else if (a == '3')
		{
			auto_run ^= 1;
			if (auto_run == 0) temp_sense &= 0xFD;
			else if (auto_run == 1) temp_sense |= 0x02;
		}
		
		//change sensitivity======================================================================
		else if (a == '4')
		{
			rprintf("\r\nSet to:\r\n",0);
			rprintf("1) 1.5g\r\n",0);
			rprintf("2) 2g\r\n",0);
			rprintf("3) 4g\r\n",0);
			rprintf("4) 6g\r\n",0);

			
			
			while(1)
			{
				a = get_char();

				if (a == '1')
				{
					PORTB &= (~((1<<GS1) | (1<<GS2)));//GS1 low, GS2 low
					temp_sense &= 0xF3;
					break;
				}
				else if (a == '2')
				{
					PORTB |= (1<<GS1);//GS1 High
					PORTB &= (~(1<<GS2));//GS2 low   
					temp_sense &= 0xF7;//GS2 low  
					temp_sense |= 0x04;//GS1 High
					break;
				}
				else if (a == '3')
				{
					PORTB &= (~(1<<GS1));//GS1 low
					PORTB |= (1<<GS2);//GS2 High
					temp_sense &= 0xFB;//GS1 low
					temp_sense |= 0x08;//GS2 high
					break;
				}
				else if (a == '4')
				{
					PORTB |= ((1<<GS1) | (1<<GS2));//GS1, GS2 high
					temp_sense |= 0x0C;
					break;
				}
			}

		}
		
		//change frequency===================================================================
		else if(a == '5')
		{
			rprintf("\r\nPress [i] to increase, [d] to decrease, [x] to exit\r\n",0);

			while(1)
			{
				rprintf("%d",temp_freq);

				a = get_char();

				if (a == 'i') temp_freq++;
				else if (a == 'd') temp_freq--;
				else if (a == 'x') break;

				if (temp_freq < 10) temp_freq = 10;
				rprintf("     \r",0);
			}
			
			freq = (float)temp_freq;
			
			a = ((temp_freq&0x03)>>8);
			
			EEPROM_write((unsigned int) FREQ_HIGH, a);
			a = (char)(temp_freq & 0xFF);
			EEPROM_write((unsigned int) FREQ_LOW, a);

		}

        //save and exit===================================================================
		else if (a == '9')
		{
                    
                    EEPROM_write((unsigned int) SENSE_AR_MODE, temp_sense);
                    rprintf("Exiting...\r\n",0);
                    break;
			
		}
                
				
		
	}
        
    PORTB |= (1<<STAT);//stat off    
}


