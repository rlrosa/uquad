#include "i2c.h"
#include "io430x54x.h"
//#include "io430.h"
#include <intrinsics.h>

#define TAM 100

int datos[TAM];
int contador=0;

void i2c_init_slave(){
  UCB0CTL1 |= UCSWRST; // eUSCI_B in reset state
  UCB0CTL0 |= UCMODE_3 || UCSYNC; // I2C slave mode
  UCB0I2COA = 0x0068; // own address is 68hex
  P3SEL |= 0x06; // configure I2C pins (device specific) 
  UCB0CTL1 &= UCSWRST; // eUSCI_B in operational state
  UCB0IE |= UCTXIE + UCRXIE; // enable TX&RX-interrupt
  GIE; // general interrupt enable
}



void init_i2c_master(){
  
  UCB0CTL1 |= UCSWRST || UCSSEL_3; // eUSCI_B in reset state
  UCB0CTL0 |= UCMST || UCMODE_3 ||UCSYNC; //Set master mode
  UCB0I2COA = 0x0067; // own address is 67hex
  P3SEL |= 0x06; // configure I2C pins (device specific)
  UCB0CTL1 &= !UCSWRST; // eUSCI_B in operational state
  UCB0IE |= UCTXIE + UCRXIE; // enable TX&RX-interrupt
  GIE;// general interrupt enable 
  
  
  
}

#pragma vector=0x70
__interrupt void guardar_dato(void){
  
    if ((UCB0IV == 0x000A)&&(contador<=10)){
        *(datos+contador)=UCB0RXBUF;
        UCA2IFG_bit.UCRXIFG = 0;
        contador++;
    }
    
}
 
