/*
HMC58X3.cpp - Interface a Honeywell HMC58X3 or HMC5883L magnetometer to an Arduino via i2c
Copyright (C) 2011 Fabio Varesano <fvaresano@yahoo.it>

Based on:
http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1274748346
 Modification/extension of the following by E.J.Muller
http://eclecti.cc/hardware/hmc5843-magnetometer-library-for-arduino
 Copyright (c) 2009 Nirav Patel, 

The above were based on:
http://www.sparkfun.com/datasheets/Sensors/Magneto/HMC58X3-v11.c
http://www.atmel.com/dyn/resources/prod_documents/doc2545.pdf


This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/


#include "WProgram.h"
#include <HMC58X3.h>


/* PUBLIC METHODS */

HMC58X3::HMC58X3() { 
  
  x_scale=1;
  y_scale=1;
  z_scale=1;
}


void HMC58X3::init(bool setmode) {
  // note that we don't initialize Wire here. 
  // You'll have to do that in setup() in your Arduino program
  delay(5); // you need to wait at least 5ms after power on to initialize
  if (setmode) {
    setMode(0);
  }
  
  writeReg(HMC58X3_R_CONFA, 0x70);
  writeReg(HMC58X3_R_CONFB, 0xA0);
  writeReg(HMC58X3_R_MODE, 0x00);
}


void HMC58X3::setMode(unsigned char mode) { 
  if (mode > 2) {
    return;
  }
  
  writeReg(HMC58X3_R_MODE, mode);
  delay(100);
}


void HMC58X3::calibrate(unsigned char gain) {
  x_scale=1; // get actual values
  y_scale=1;
  z_scale=1;
  writeReg(HMC58X3_R_CONFA, 0x010 + HMC_POS_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to pos bias
  setGain(gain);
  float x, y, z, mx=0, my=0, mz=0, t=10;
  
  for (int i=0; i<(int)t; i++) { 
    setMode(1);
    getValues(&x,&y,&z);
    if (x>mx) mx=x;
    if (y>my) my=y;
    if (z>mz) mz=z;
  }
  
  float max=0;
  if (mx>max) max=mx;
  if (my>max) max=my;
  if (mz>max) max=mz;
  x_max=mx;
  y_max=my;
  z_max=mz;
  x_scale=max/mx; // calc scales
  y_scale=max/my;
  z_scale=max/mz;
  //writeReg(HMC58X3_R_CONFA, 0x010); // set RegA/DOR back to default
  writeReg(HMC58X3_R_CONFA, 0x70); // set RegA/DOR back to default
}


// set data output rate
// 0-6, 4 default, normal operation assumed
void HMC58X3::setDOR(unsigned char DOR) {
  if (DOR>6) return;
  writeReg(HMC58X3_R_CONFA,DOR<<2);
}


void HMC58X3::setGain(unsigned char gain) { 
  // 0-7, 1 default
  if (gain > 7) return;
  writeReg(HMC58X3_R_CONFB, gain << 5);
}


void HMC58X3::writeReg(unsigned char reg, unsigned char val) {
  Wire.beginTransmission(HMC58X3_ADDR);
  Wire.send(reg);        // send register address
  Wire.send(val);        // send value to write
  Wire.endTransmission(); //end transmission
}


void HMC58X3::getValues(int *x,int *y,int *z) {
  float fx,fy,fz;
  getValues(&fx,&fy,&fz);
  *x= (int) (fx + 0.5);
  *y= (int) (fy + 0.5);
  *z= (int) (fz + 0.5);
}


void HMC58X3::getValues(float *x,float *y,float *z) {
  int xr,yr,zr;
  
  getRaw(&xr, &yr, &zr);
  *x= ((float) xr) / x_scale;
  *y = ((float) yr) / y_scale;
  *z = ((float) zr) / z_scale;
}


void HMC58X3::getRaw(int *x,int *y,int *z) {
  Wire.beginTransmission(HMC58X3_ADDR);
  Wire.send(HMC58X3_R_XM); // will start from DATA X MSB and fetch all the others
  Wire.endTransmission();
  
  Wire.beginTransmission(HMC58X3_ADDR);
  Wire.requestFrom(HMC58X3_ADDR, 6);
  if(6 == Wire.available()) {
    // read out the 3 values, 2 bytes each.
    *x = (Wire.receive() << 8) | Wire.receive();
    #ifdef ISHMC5843
      *y = (Wire.receive() << 8) | Wire.receive();
      *z = (Wire.receive() << 8) | Wire.receive();
    #else // the Z registers comes before the Y registers in the HMC5883L
      *z = (Wire.receive() << 8) | Wire.receive();
      *y = (Wire.receive() << 8) | Wire.receive();
    #endif
    // the HMC58X3 will automatically wrap around on the next request
  }
  Wire.endTransmission();
}


void HMC58X3::getValues(float *xyz) {
  getValues(&xyz[0], &xyz[1], &xyz[2]);
}

