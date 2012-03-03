#include "i2c.h"
#include "io430x54x.h"
//#include "io430.h"
#include <intrinsics.h>
#include <assert.h>

#define TAM 4096
static unsigned char datos[TAM];
static int contador;

void i2c_init_slave(void){
  UCB0CTL1 |= UCSWRST; // eUSCI_B in reset state
  UCB0CTL0 |= UCMODE_3 | UCSYNC; // I2C slave mode
  UCB0I2COA = 0x0400|0x0069; // own address is 68hex and enable
  P3SEL |= 0x06; // configure I2C pins (device specific) 
  UCB0CTL1 &= !UCSWRST; // eUSCI_B in operational state
  UCB0IE = UCRXIE; // enable RX-interrupt // quiere ver de habilitar el UCRXIE & start bit
  contador = 0;
  __enable_interrupt();
}
 
#pragma vector = USCI_B0_VECTOR
__interrupt void guardar_dato(void){
    
  __disable_interrupt();
    if (UCB0IV == 0x00A){
//      assert(contador < TAM);
      if(contador < TAM)
        {
          datos[contador] = UCB0RXBUF;
          contador = contador + 1;
        }
       UCA2IFG_bit.UCRXIFG = 0;
      
    }
    __enable_interrupt();
}



void init_i2c_master(){
  
  UCB0CTL1 |= UCSWRST | UCSSEL_3; // eUSCI_B in reset state
  UCB0CTL0 |= UCMST | UCMODE_3 |UCSYNC; //Set master mode
  UCB0I2COA = 0x0067; // own address is 67hex
  P3SEL |= 0x06; // configure I2C pins (device specific)
  UCB0CTL1 &= !UCSWRST; // eUSCI_B in operational state
  UCB0IE |= UCTXIE + UCRXIE; // enable TX&RX-interrupt
  GIE;// general interrupt enable 
  
  
  
}