/*
HMC58X3.cpp - Interface a Honeywell HMC58X3 magnetometer to an Arduino via i2c
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

//uncomment the following line if you are using this library with the HMC5843
//#define ISHMC5843

#include "WProgram.h"
#include <Wire.h>

#ifndef HMC58X3_h
#define HMC58X3_h

#define HMC58X3_ADDR 0x1E // 7 bit address of the HMC58X3 used with the Wire library
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2

// HMC58X3 register map. For details see HMC58X3 datasheet
#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_R_XM 3
#define HMC58X3_R_XL 4

#ifdef ISHMC5843
  #define HMC58X3_R_YM 5
  #define HMC58X3_R_YL 6
  #define HMC58X3_R_ZM 7
  #define HMC58X3_R_ZL 8
#else // HMC5883L
  #define HMC58X3_R_YM 7
  #define HMC58X3_R_YL 8
  #define HMC58X3_R_ZM 5
  #define HMC58X3_R_ZL 6
#endif

#define HMC58X3_R_STATUS 9
#define HMC58X3_R_IDA 10
#define HMC58X3_R_IDB 11
#define HMC58X3_R_IDC 12


class HMC58X3
{
  public:
    HMC58X3();
    void init(bool setmode);
    void init(int address, bool setmode);
    void getValues(int *x,int *y,int *z);
    void getValues(float *x,float *y,float *z);
    void getValues(float *xyz);
    void getRaw(int *x,int *y,int *z);
    void calibrate(unsigned char gain);
    void setMode(unsigned char mode);
    void setDOR(unsigned char DOR);
    void setGain(unsigned char gain);
  private:
    void writeReg(unsigned char reg, unsigned char val);
    float x_scale,y_scale,z_scale,x_max,y_max,z_max;
};

#endif // HMC58X3_h
