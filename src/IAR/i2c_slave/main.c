#include "i2c.h"
#include "io430x54x.h"
//#include "io430.h"
#include <intrinsics.h>

main(){
 
  WDTCTL=WDTPW + WDTHOLD;
  i2c_init_slave();

  while (1){
  }
  
}

