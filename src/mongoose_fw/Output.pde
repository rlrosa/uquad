#if 0 && ATOMIC_IMU_FORMAT
#define ATOMIC_IMU_SEPARATOR ','
#define ATOMIC_IMU_INIT 'A'
#define ATOMIC_IMU_END 'Z'
#endif

static char ATOMIC_IMU_SEPARATOR = ',';
static char ATOMIC_IMU_INIT = 'A';
static char ATOMIC_IMU_END = 'Z';

static unsigned long sampling_T_us;// = SAMP_TX_T; // Time at which last sample was sent (us)
static unsigned long tx_Dt_us; // Time since last sample was sent (us)
#if DEBUG_TX_TIMING
#define LOOPS 100
int cnt_loops = 0;
int cnt_fails = 0;
#endif // DEBUG_TX_TIMING
unsigned long fail_val = 0;
void printdata(void)
{
    unsigned long t_curr_us = micros();

    void (*print_method)(void*,int) = (print_binary)?
	send_raw_bin:send_raw_ascii;

    tx_Dt_us = t_curr_us - sampling_T_us;
    sampling_T_us = t_curr_us;

#if DEBUG_TX_TIMING
    if((tx_Dt_us > (timing.T_extr + timing.jitter_extr)) ||
       (tx_Dt_us < (timing.T_extr - timing.jitter_extr)))
    /* if((tx_Dt_us > (SAMP_T_EXTR_MAX)) || */
    /*    (tx_Dt_us < (SAMP_T_EXTR_MIN))) */
    {
	cnt_fails++;
	Serial.print(timing.T_extr);
	Serial.print('|');
	Serial.print(timing.jitter_extr);
	Serial.print('|');
	Serial.print('|');
	Serial.print(timing.T_extr - timing.jitter_extr);
	Serial.print('|');
	Serial.print(tx_Dt_us);
	Serial.print('|');
	Serial.println(timing.T_extr + timing.jitter_extr);
    }
	
    if(cnt_loops++ > LOOPS)
    {
	if(cnt_fails > 0 || 1)
	{
	    Serial.print("!");
	    Serial.print(cnt_fails);
	    Serial.print(',');
	    Serial.print(fail_val);
	    Serial.print(',');
	    Serial.print(21250);
	    Serial.print(',');
	    Serial.print(SAMP_T_EXTR);
	    Serial.print("Dt:");
	    Serial.print(tx_Dt_us);
	    for(cnt_fails=0;cnt_fails < SAMP_INTRS_EXTR;cnt_fails++)
	    {
		Serial.print("\t");
		Serial.print(dt_tmp[cnt_fails]);
	    }
	    Serial.println();
	}
	cnt_loops = 0;
	cnt_fails = 0;
    }
#endif // DEBUG_TX_TIMING

#if PRINT_DATA
#if ATOMIC_IMU_FORMAT
    if(print_binary)
	send_raw_bin_clear();
    (*print_method)((void*)&ATOMIC_IMU_INIT,sizeof(char));
    (*print_method)((void*)&ATOMIC_IMU_SEPARATOR,sizeof(char));
    (*print_method)((void*)&tx_Dt_us,sizeof(unsigned long));
    (*print_method)((void*)&ATOMIC_IMU_SEPARATOR,sizeof(char));
    (*print_method)((void*)&sen_data.accel_x_raw,sizeof(int));
    (*print_method)((void*)&ATOMIC_IMU_SEPARATOR,sizeof(char));
    (*print_method)((void*)&sen_data.accel_y_raw,sizeof(int));
    (*print_method)((void*)&ATOMIC_IMU_SEPARATOR,sizeof(char));
    (*print_method)((void*)&sen_data.accel_z_raw,sizeof(int));
    (*print_method)((void*)&ATOMIC_IMU_SEPARATOR,sizeof(char));
    (*print_method)((void*)&sen_data.gyro_x_raw,sizeof(int));
    (*print_method)((void*)&ATOMIC_IMU_SEPARATOR,sizeof(char));
    (*print_method)((void*)&sen_data.gyro_y_raw,sizeof(int));
    (*print_method)((void*)&ATOMIC_IMU_SEPARATOR,sizeof(char));
    (*print_method)((void*)&sen_data.gyro_z_raw,sizeof(int));
    (*print_method)((void*)&ATOMIC_IMU_SEPARATOR,sizeof(char));
    (*print_method)((void*)&sen_data.magnetom_x_raw,sizeof(int));
    (*print_method)((void*)&ATOMIC_IMU_SEPARATOR,sizeof(char));
    (*print_method)((void*)&sen_data.magnetom_y_raw,sizeof(int));
    (*print_method)((void*)&ATOMIC_IMU_SEPARATOR,sizeof(char));
    (*print_method)((void*)&sen_data.magnetom_z_raw,sizeof(int));
    (*print_method)((void*)&ATOMIC_IMU_SEPARATOR,sizeof(char));
    (*print_method)((void*)&sen_data.baro_temp,sizeof(short));
    (*print_method)((void*)&ATOMIC_IMU_SEPARATOR,sizeof(char));
    (*print_method)((void*)&sen_data.baro_pres,sizeof(unsigned long));
    (*print_method)((void*)&ATOMIC_IMU_SEPARATOR,sizeof(char));
#if DEBUG
    if(print_raw_bmp085)
    {
	(*print_method)((void*)&sen_data.baro_temp_raw,sizeof(int));
	(*print_method)((void*)&ATOMIC_IMU_SEPARATOR,sizeof(char));
	(*print_method)((void*)&sen_data.baro_pres_raw,sizeof(unsigned long));
	(*print_method)((void*)&ATOMIC_IMU_SEPARATOR,sizeof(char));
    }
#endif // DEBUG
    (*print_method)((void*)&ATOMIC_IMU_END,sizeof(char));
    if(print_binary)
	send_raw_bin_flush();
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

static char buff[256];
static int buff_index = 0;
static int flushed_index = 0;

void send_raw_bin_background()
{
    int i = 0;
    while(flushed_index < buff_index && i++ < 32)
	Serial.write(buff[flushed_index++]);
}   

void send_raw_bin_flush()
{
    flushed_index = 0;
    Serial.println();
    Serial.print(buff_index,DEC);
    Serial.println();
    //    Serial.write((uint8_t*)buff,buff_index);
    //    buff_index = 0;
}

void send_raw_bin_clear()
{
    buff_index = 0;
}

void send_raw_bin(void *data, int size_bytes)
{
  int i;
  for (i = 0 ; i < size_bytes ; i++)
      //      Serial.write(*((char*)data + i));
      buff[buff_index++] = *((char*)data + i);
}

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
