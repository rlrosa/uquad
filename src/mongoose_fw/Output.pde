#if ATOMIC_IMU_FORMAT
#define ATOMIC_IMU_SEPARATOR_ASCII '\t'
#define ATOMIC_IMU_INIT 'A'
#define ATOMIC_IMU_END 'Z'
#endif

static char atomic_imu_separator = ATOMIC_IMU_SEPARATOR_ASCII;
static char atomic_imu_init = ATOMIC_IMU_INIT;
static char atomic_imu_end = ATOMIC_IMU_END;

static unsigned long sampling_T_us;// = SAMP_TX_T; // Time at which last sample was sent (us)
static unsigned long tx_Dt_us; // Time since last sample was sent (us)
#if DEBUG_TX_TIMING
#define LOOPS 100
int cnt_loops = 0;
int cnt_fails = 0;
unsigned long fail_val = 0;
#endif // DEBUG_TX_TIMING
void printdata(void)
{
    unsigned long t_curr_us = micros();
    void (*print_method)(void*,int);
    if(print_binary)
    {
	print_method = queue_raw_bin;
	atomic_imu_init ^= 2; // switch A C por si pierdo una tirada.
    }
    else
    {
	print_method = send_raw_ascii;
	atomic_imu_init = ATOMIC_IMU_INIT;
    }

    tx_Dt_us = t_curr_us - sampling_T_us;
    sampling_T_us = t_curr_us;

    // warn if sampling rate is off
    if((tx_Dt_us > timing.T_extr_max) ||
       (tx_Dt_us < timing.T_extr_min))
    {
#if WARNINGS
	Serial.print("%%!!");
	Serial.println(tx_Dt_us);
#endif
    }

#if DEBUG_TX_TIMING
    if((tx_Dt_us > timing.T_extr_max) ||
       (tx_Dt_us < timing.T_extr_min))
    {
	cnt_fails++;
	if(tx_Dt_us > fail_val)
	    fail_val = tx_Dt_us;
    }
	
    if(cnt_loops++ > LOOPS)
    {
	if(cnt_fails > 0)
	{
	    Serial.print("!");
	    Serial.print(cnt_fails);
	    Serial.print(',');
	    Serial.print(fail_val);
	    Serial.println();
	}
	cnt_loops = 0;
	cnt_fails = 0;
    }
#endif // DEBUG_TX_TIMING

#if PRINT_DATA
#if ATOMIC_IMU_FORMAT
    while(print_binary && tx_busy)
	delayMicroseconds(100); // wait
    (*print_method)((void*)&atomic_imu_init,sizeof(char));
    if(!print_binary)
	(*print_method)((void*)&atomic_imu_separator,sizeof(char));
    (*print_method)((void*)&tx_Dt_us,sizeof(unsigned long));
    if(!print_binary)
	(*print_method)((void*)&atomic_imu_separator,sizeof(char));
    (*print_method)((void*)&sen_data.accel_x_raw,sizeof(int));
    if(!print_binary)
	(*print_method)((void*)&atomic_imu_separator,sizeof(char));
    (*print_method)((void*)&sen_data.accel_y_raw,sizeof(int));
    if(!print_binary)
	(*print_method)((void*)&atomic_imu_separator,sizeof(char));
    (*print_method)((void*)&sen_data.accel_z_raw,sizeof(int));
    if(!print_binary)
	(*print_method)((void*)&atomic_imu_separator,sizeof(char));
    (*print_method)((void*)&sen_data.gyro_x_raw,sizeof(int));
    if(!print_binary)
	(*print_method)((void*)&atomic_imu_separator,sizeof(char));
    (*print_method)((void*)&sen_data.gyro_y_raw,sizeof(int));
    if(!print_binary)
	(*print_method)((void*)&atomic_imu_separator,sizeof(char));
    (*print_method)((void*)&sen_data.gyro_z_raw,sizeof(int));
    if(!print_binary)
	(*print_method)((void*)&atomic_imu_separator,sizeof(char));
    (*print_method)((void*)&sen_data.magnetom_x_raw,sizeof(int));
    if(!print_binary)
	(*print_method)((void*)&atomic_imu_separator,sizeof(char));
    (*print_method)((void*)&sen_data.magnetom_y_raw,sizeof(int));
    if(!print_binary)
	(*print_method)((void*)&atomic_imu_separator,sizeof(char));
    (*print_method)((void*)&sen_data.magnetom_z_raw,sizeof(int));
    if(!print_binary)
	(*print_method)((void*)&atomic_imu_separator,sizeof(char));
    (*print_method)((void*)&sen_data.baro_temp,sizeof(short));
    if(!print_binary)
	(*print_method)((void*)&atomic_imu_separator,sizeof(char));
    (*print_method)((void*)&sen_data.baro_pres,sizeof(unsigned long));
    if(!print_binary)
	(*print_method)((void*)&atomic_imu_separator,sizeof(char));
#if DEBUG
    if(print_raw_bmp085)
    {
	(*print_method)((void*)&sen_data.baro_temp_raw,sizeof(int));
	if(!print_binary)
	    (*print_method)((void*)&atomic_imu_separator,sizeof(char));
	(*print_method)((void*)&sen_data.baro_pres_raw,sizeof(unsigned long));
	if(!print_binary)
	    (*print_method)((void*)&atomic_imu_separator,sizeof(char));
    }
#endif // DEBUG
    (*print_method)((void*)&atomic_imu_end,sizeof(char));
    if(print_binary)
	UartSendData();
    else
	Serial.println();

#else // ATOMIC_IMU_FORMAT
    // use ckdevices format
      Serial.print("!");

      
      #if PRINT_EULER == 1
      Serial.print("ANG:,");
      Serial.print(ToDeg(roll));
      Serial.print(",");
      Serial.print(ToDeg(pitch));
      Serial.print(",");
      Serial.print(ToDeg(yaw));
      #endif   
      
      
      #if PRINT_SENSOR_DATA==1
      Serial.print(",SEN:,");
      Serial.print(sen_data.accel_x);
      Serial.print (",");
      Serial.print(sen_data.accel_y);
      Serial.print (",");
      Serial.print(sen_data.accel_z);
      Serial.print(",");
      Serial.print(sen_data.gyro_x);
      Serial.print(",");
      Serial.print(sen_data.gyro_y);
      Serial.print(",");
      Serial.print(sen_data.gyro_z);  
      Serial.print(",");
      Serial.print((int)sen_data.magnetom_x);
      Serial.print (",");
      Serial.print((int)sen_data.magnetom_y);
      Serial.print (",");
      Serial.print((int)sen_data.magnetom_z);    
      Serial.print (",");
      Serial.print(sen_data.magnetom_heading );     
      Serial.print (",");
      Serial.print(sen_data.baro_temp ); 
      Serial.print (",");
      Serial.print(sen_data.baro_pres ); 
      #endif
      
      
      #if PRINT_SENSOR_DATA_RAW==1
      Serial.print(",RAW:,");
      Serial.print(sen_data.accel_x_raw);
      Serial.print (",");
      Serial.print(sen_data.accel_y_raw);
      Serial.print (",");
      Serial.print(sen_data.accel_z_raw);
      Serial.print(",");
      Serial.print(sen_data.gyro_x_raw);
      Serial.print(",");
      Serial.print(sen_data.gyro_y_raw);
      Serial.print(",");
      Serial.print(sen_data.gyro_z_raw);  
      Serial.print(",");
      Serial.print((int)sen_data.magnetom_x_raw);
      Serial.print (",");
      Serial.print((int)sen_data.magnetom_y_raw);
      Serial.print (",");
      Serial.print((int)sen_data.magnetom_z_raw);    
      #endif
      
      
      #if PRINT_DCM == 1
      Serial.print (",DCM:");
      Serial.print(convert_to_dec(DCM_Matrix[0][0]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[0][1]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[0][2]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[1][0]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[1][1]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[1][2]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[2][0]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[2][1]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[2][2]));
      #endif
      
      
      Serial.println();    
#endif
#endif // PRINT_DATA
}

long convert_to_dec(float x)
{
  return x*10000000;
}

#if DEBUG
void PrintCompassCalibration(void)
{
    float xyz_scale[3];
    float xyz_max[3];

    GetMagnCalibration(xyz_scale, xyz_max);

    Serial.println();
    Serial.println("HMC58X3 scale (x,y,z):");
    Serial.print("\t");
    Serial.print(xyz_scale[0]);
    Serial.print("\t");
    Serial.print(xyz_scale[1]);
    Serial.print("\t");
    Serial.println(xyz_scale[2]);

    Serial.println("HMC58X3 max (x,y,z):");
    Serial.print("\t");
    Serial.print(xyz_max[0]);
    Serial.print("\t");
    Serial.print(xyz_max[1]);
    Serial.print("\t");
    Serial.println(xyz_max[2]);

    Serial.println();
}
#endif

// ***********************************
// ASCII data TX
// ***********************************
void send_raw_ascii(void *data, int size_bytes)
{
    switch(size_bytes)
    {
    case sizeof(char):
	Serial.print(*(char*)data);
	break;
    case sizeof(int):
	Serial.print(*(int*)data);
	break;
    case sizeof(unsigned long):
	Serial.print(*(unsigned long*)data);
	break;
    default:
	Serial.println("Invalid data!");
	break;
    }
}


// ***********************************
// User interface
// ***********************************
void set_work_mode()
{
    sensors_enabled_set_defaults();
    print_binary = true;
    running = true;
}

void print_menu(void){
    Serial.println();
    Serial.println("uQuad!");
    Serial.println();
    Serial.println("Commands will enable/disable sensor reading:");

    Serial.println("\t#:\t Exit and run.");

    Serial.print("\t2:\t Acc.:\t");
    Serial.print(sensors.acc);
    Serial.println();

    Serial.print("\t3:\t Gyro.:\t");
    Serial.print(sensors.gyro);
    Serial.println();

    Serial.print("\t4:\t Compass.:\t");
    Serial.print(sensors.compass);
    Serial.println();

    Serial.print("\t5:\t Temp.:\t");
    Serial.print(sensors.temp);
    Serial.println();

    Serial.print("\t6:\t Press.:\t");
    Serial.print(sensors.pressure);
    Serial.println();

    Serial.print("\t7:\t Show barometer calibration.");
    Serial.println();

#if DEBUG
    Serial.print("\tq:\t Print BMP085 raw.");
    Serial.print(print_raw_bmp085);
    Serial.println();

    Serial.print("\tw:\t Print Compass calibration.");
    Serial.println();

    Serial.print("\te:\t Set BMP085 OSS:\t");
    Serial.print(bmp085GetOSS());
    Serial.println();

    Serial.print("\tb:\t Print binary data:\t");
    Serial.print(print_binary);
    Serial.println();
#endif

    Serial.print("\tCommand:");
}

int menu_execute(int command){
    if(command == '!')
    {
	// set to normal mode
	set_work_mode();
	return 0;
    }
    if(running)
    {
	if(command == '$')
	{
	    // stop!
	    running = false;
	}
    }
    else // not running
    {
	switch (command)
	{
	case '#':
	    running = true;
	    break;
	case '2':
	    sensors.acc = !sensors.acc;
	    if(!sensors.acc)
	    {
		sen_data.accel_x_raw = 0;
		sen_data.accel_y_raw = 0;
		sen_data.accel_z_raw = 0;
	    }
	    break;
	case '3':
	    sensors.gyro = !sensors.gyro;
	    if(!sensors.gyro)
	    {
		sen_data.gyro_x_raw = 0;
		sen_data.gyro_y_raw = 0;
		sen_data.gyro_z_raw = 0;
	    }
	    break;
	case '4':
	    sensors.compass = !sensors.compass;
	    if(!sensors.compass)
	    {
		sen_data.magnetom_x_raw = 0;
		sen_data.magnetom_y_raw = 0;
		sen_data.magnetom_z_raw = 0;
	    }
	    break;
	case '5':
	    sensors.temp = !sensors.temp;
	    if(!sensors.temp)
	    {
		sen_data.baro_temp_raw = 0;
	    }
	    break;
	case '6':
	    sensors.pressure = !sensors.pressure;
	    if(!sensors.pressure)
	    {
		sen_data.baro_pres_raw = 0;
	    }
	    break;
	case '7':
	    bmp085Display_Calibration();
	    break;
#if DEBUG
	case 'q':
	    print_raw_bmp085 = !print_raw_bmp085;
	    break;
	case 'w':
	    PrintCompassCalibration();
	    break;
	case 'e':
	    Serial.println("\nIncrementing OSS.");
	    if(bmp085SetOSS((bmp085GetOSS()+1)%4))
		Serial.println("\nSuccess!");
	    else
		Serial.println("\nFAILED!");
	    break;
	case 'b':
	    print_binary = !print_binary;
	    break;
#endif
	default:
	    // invalid command
	    return -1;
	}
    }
    if(!running)
	print_menu();
    return 0;
}
