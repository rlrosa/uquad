AVR IMU Interface
=================

Application Notes - 03/01/07
============================

This is an AVR Studio project for the AVR ATmega168 MCU that processes
raw analog voltage values from the SparkFun IMU 5 Degrees of Freedom
through a Kalman filter to output pitch values in degrees along the X
and Y axis of the IMU.  More information on the SparkFun IMU can be
found at the following URL:

http://www.sparkfun.com/commerce/product_info.php?products_id=741

Hardware Configuration
======================

Currently, this project assumes the AVR ATmega168 MCU is hosted on the
AVR STK500 development board with a 20MHz external crystal and the
VTarget voltage set to 3.3V.  The spare serial port on the AVR STK500 is
utilized to output the pitch values for each axis at 57.6K baud.

The SparkFun IMU 5 Degrees of Freedom is attached to the AVR ATmega168
MCU on the STK500 as described below:

  STK500 PINS      IMU 5 DF JP1 PINS

  VTG              3.3V    (Pin 1)
  GND              GND     (Pin 2)
  PC0 (ADC0)       X Accel (Pin 9)
  PC1 (ADC1)       Y Accel (Pin 8)
  PC2 (ADC2)       Z Accel (Pin 7)
  PC3 (ADC3)       X Rate  (Pin 3)
  PC4 (ADC4)       Y Rate  (Pin 4)
  PC5 (ADC5)       VRef    (Pin 5)

Project Usage
=============

To use this project configure the hardware as described above and 
program the project into the AVR ATmega160 Flash.  The application will 
automatically begin to output pitch values for the X and Y axis of the 
IMU. Output on the spare serial port at 57.6K baud should look something 
like the following with the pitch of the IMU indicated in degrees:

    x= +0.3    y= +2.6
    x= +0.2    y= +2.7

Software License
================

Copyright (c) 2007 Michael P. Thompson <mpthompson@gmail.com>

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use, copy,
modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
