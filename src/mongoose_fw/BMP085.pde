// uQuad!
//
// Based on:
//    BMP085 Extended Example Code
//    by: Jim Lindblom
//    SparkFun Electronics
//    date: 1/18/11
//    license: CC BY-SA v3.0 - http://creativecommons.org/licenses/by-sa/3.0/
//    
//    Get pressure and temperature from the BMP085 and calculate altitude.
//    Serial.print it out at 9600 baud to serial monitor.
//
//
#include <Wire.h>
#include <assert.h>

#define BMP085_ADDRESS 0x77  // I2C address of BMP085

#define BMP085_TEMP_WAIT_US 4500UL // time between temp request and data ready

static unsigned char OSS = 0;  // Oversampling Setting

// Calibration values for Barometric Pressure sensor
int ac1;
int ac2; 
int ac3; 
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1; 
int b2;
int mb;
int mc;
int md;

// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5; 

/// State machine support structs.
enum barom_state {
    BAROM_IDLE = 0,
    BAROM_WAITING,
    BAROM_DONE
};
barom_state barom_press = BAROM_IDLE, barom_temp = BAROM_IDLE;

static bool calibration_loaded = false;
/** 
 * Prints calibration parameters loaded from EEPROM to
 * serial.
 * These parameters are used to convert compensate temp/press
 * data. Raw data is uncompensated.
 * 
 */
void bmp085Display_Calibration(){
    if(calibration_loaded)
    {
	Serial.println("BMP085 Calibration:");
	Serial.print("\tac1 =\t");
	Serial.println(ac1);
	Serial.print("\tac2 =\t");
	Serial.println(ac2);
	Serial.print("\tac3 =\t");
	Serial.println(ac3);
	Serial.print("\tac4 =\t");
	Serial.println(ac4);
	Serial.print("\tac5 =\t");
	Serial.println(ac5);
	Serial.print("\tac6 =\t");
	Serial.println(ac6);

	Serial.print("\tb1 =\t");
	Serial.println(b1);
	Serial.print("\tb2 =\t");
	Serial.println(b2);

	Serial.print("\tmb =\t");
	Serial.println(mb);
	Serial.print("\tmc =\t");
	Serial.println(mc);
	Serial.print("\tmd =\t");
	Serial.println(md);
    }
    else
    {
	Serial.println("Calibration not loaded from EEPROM...");
    }
}

// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
void Init_Baro()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
  calibration_loaded = true;
}

static unsigned long barom_req_time_us;
/** 
 * Guides through a state machine that will perform the following tasks:
 *   - Request a new temp reading.
 *   - Read new temp data, update struct data.
 *   - Request a new pressure reading.
 *   - Read new pressure data, update struct data.
 *
 * NOTE: If the state machine is idle, then calling this function will
 *       only return the current state, idle. New data should only be
 *       be requested if the state machine is in idle mode.
 * 
 * @return Current state of the machine.
 */
int barom_update_state_machine()
{
    if((barom_press == BAROM_WAITING) &&
       (micros() - barom_req_time_us > bmp085GetUPWaitUS()))
    {
	Read_Pressure(true);
	// at this poing press has been updated
    }
    else if((barom_temp == BAROM_WAITING) &&
	    (micros() - barom_req_time_us > BMP085_TEMP_WAIT_US))
    {
	Read_Temperature(true);
	if(sensors.pressure)
	    Read_Pressure(false);
    }
    if((barom_press == BAROM_IDLE) &&
       (barom_temp == BAROM_IDLE))
	return BAROM_IDLE;
    return BAROM_WAITING;
}

/** 
 * Starts state machine, will result in new data after
 * a complete loop (temp+press reading).
 * This function will request new data, which will be ready in:
 *   - Temperature: 4.5ms
 *   - Pressure: 4.5-25.5ms + Temp conversion time.
 *
 * NOTE: Should only be called if state machine is idle. This
 *       can be checked with barom_update_state_machine()
 * 
 */
void Baro_req_update()
{
#if DEBUG
    // check for unread data
    if((barom_press != BAROM_IDLE) ||
	   (barom_temp != BAROM_IDLE))
    {
	/* Serial.println("\n\n-- -- -- -- --"); */
	/* Serial.println("Missed barom data!!"); */
	/* Serial.println("-- -- -- -- --\n\n"); */
	Serial.println("B");
	return;
    }
#else
    assert((barom_press == BAROM_IDLE) &&
	   (barom_temp == BAROM_IDLE));
#endif
    // start state machine
    if(sensors.temp)
	Read_Temperature(false);
    else
	Read_Pressure(false);
}

int Read_Temperature(bool req_done)
{
    if(req_done)
    {
	// Get temp reading
	sen_data.baro_temp = bmp085GetTemperature(bmp085ReadUT(true));
    }
    else
    {
	// request temp reading
	bmp085ReadUT(false);
	barom_req_time_us = micros();
	barom_temp = BAROM_WAITING;
    }
    return barom_temp;
}

int Read_Pressure(bool req_done)
{
    if(req_done)
    {
	// Get pressure reading
	sen_data.baro_pres = bmp085GetPressure(bmp085ReadUP(true));
    }
    else
    {
	// request pressure reading
	bmp085ReadUP(false);
	barom_req_time_us = micros();
	barom_press = BAROM_WAITING;
    }
    return barom_press;
} 

// Calculate temperature given ut.
// Value returned will be in units of 0.1 deg C
short bmp085GetTemperature(unsigned int ut)
{
    // verify updated data
#if DEBUG
    if(barom_temp != BAROM_DONE)
	Serial.println("WARN:Old temp!");
#else
    assert(barom_temp == BAROM_DONE);
#endif
    barom_temp = BAROM_IDLE; // mark as read

    long x1, x2;
  
    x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
    x2 = ((long)mc << 11)/(x1 + md);
    b5 = x1 + x2;

    return ((b5 + 8)>>4);  
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up)
{
    // verify updated data
#if DEBUG
    if(barom_press != BAROM_DONE)
	Serial.println("WARN:Old pres!");
#else
    assert(barom_press == BAROM_DONE);
#endif
    barom_press = BAROM_IDLE; // mark as read

    long x1, x2, x3, b3, b6, p;
    unsigned long b4, b7;
  
    b6 = b5 - 4000;
    // Calculate B3
    x1 = (b2 * (b6 * b6)>>12)>>11;
    x2 = (ac2 * b6)>>11;
    x3 = x1 + x2;
    b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;
  
    // Calculate B4
    x1 = (ac3 * b6)>>13;
    x2 = (b1 * ((b6 * b6)>>12))>>16;
    x3 = ((x1 + x2) + 2)>>2;
    b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
  
    b7 = ((unsigned long)(up - b3) * (50000>>OSS));
    if (b7 < 0x80000000)
	p = (b7<<1)/b4;
    else
	p = (b7/b4)<<1;
    
    x1 = (p>>8) * (p>>8);
    x1 = (x1 * 3038)>>16;
    x2 = (-7357 * p)>>16;
    p += (x1 + x2 + 3791)>>4;
  
    return p;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.send(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;
    
  return Wire.receive();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.send(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.receive();
  lsb = Wire.receive();
  
  return (int) msb<<8 | lsb;
}

/** 
 * Read/request the uncompensated temperature value.
 * The BMP085 takes up to 4.5ms between temp request and data
 * available.
 *
 * @param req_done If(false):
 *                   requests a new temperature reading.
 *                 else
 *                   reads temperature. Asumes new temperature
 *                   had been previously requested.
 * 
 * @return If new data was read, returnes the value, else 0.
 */
unsigned int bmp085ReadUT(bool req_done)
{
  unsigned int ut;


  if(req_done)
  {
      // Read temperature
      // Read two bytes from registers 0xF6 and 0xF7
      ut = bmp085ReadInt(0xF6);
      sen_data.baro_temp_raw = ut;
      barom_temp = BAROM_DONE;
  }
  else
  {
      // Request a temperature reading
      // Write 0x2E into Register 0xF4
      // This requests a temperature reading
      Wire.beginTransmission(BMP085_ADDRESS);
      Wire.send(0xF4);
      Wire.send(0x2E);
      Wire.endTransmission();
      ut = 0;
      // Now wait BMP085_TEMP_WAIT_US us for conversion.
      // State machine will take care of this.
  }
  return ut;
}

/** 
 * Give the max time that the BMP085 will take to have a 
 * new pressure sample ready.
 * Pressure conversion time is not fixed, it's OSS dependant.
 * 
 * @return Max pressure conversion time in us.
 */
unsigned long bmp085GetUPWaitUS()
{
    return 1500UL + 1000UL*(3<<OSS);
}


/** 
 * Read/Request the uncompensated pressure value
 * The BMP085 takes [4.5 7.5 13.5 25.5]ms between new pressure
 * is requested and data is ready. Time depends on OSS setting.
 *
 * @param req_done If(false):
 *                   requests a new uncompensated pressure reading.
 *                 else
 *                   reads pressure. Asumes new pressure
 *                   had been previously requested.
 * 
 * @return If new data was read, returnes the value, else 0.
 *         Data needs to be compensated using the algorithm
 *         described in the datasheet, with the constants
 *         flashed into each specific unit. The algorithm is
 *         implemented in bmp085GetPressure().
 */
unsigned long bmp085ReadUP(bool req_done)
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;

  if(req_done)
  {  
      // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
      Wire.beginTransmission(BMP085_ADDRESS);
      Wire.send(0xF6);
      Wire.endTransmission();
      Wire.requestFrom(BMP085_ADDRESS, 3);
  
      // Wait for data to become available
      while(Wire.available() < 3)
	  ;
      msb = Wire.receive();
      lsb = Wire.receive();
      xlsb = Wire.receive();
  
      up = (((unsigned long) msb << 16)  |
	    ((unsigned long) lsb << 8)   |
	    (unsigned long) xlsb) >> (8-OSS);
  
      sen_data.baro_pres_raw = up;
      barom_press = BAROM_DONE;
  }
  else
  {
      // Write 0x34+(OSS<<6) into register 0xF4
      // Request a pressure reading w/ oversampling setting
      Wire.beginTransmission(BMP085_ADDRESS);
      Wire.send(0xF4);
      Wire.send(0x34 + (OSS<<6));
      Wire.endTransmission();
      up = 0;
      // Now wait for conversion, delay time dependent on OSS.
      // State machine takes care of this.
  }
  return up;
}

#if DEBUG
/** 
 * Set oversampling settings (OSS).
 * 
 * @param OSS_new New value for OSS.
 * 
 * @return false if argument was invalid, else true.
 */
bool bmp085SetOSS(unsigned char OSS_new)
{
    if(OSS_new < 4)
	OSS = OSS_new;
    else
	// invalid input
	return false;
    return true;
}

/** 
 * Get current value of OSS.
 * 
 * 
 * @return 
 */
unsigned char bmp085GetOSS(void)
{
    return OSS;
}

#endif
