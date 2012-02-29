#if ATOMIC_IMU_FORMAT
#define ATOMIC_IMU_SEPARATOR "\t"
#define ATOMIC_IMU_INIT "A"
#define ATOMIC_IMU_END "Z"
#endif

static unsigned long sampling_T_us = SAMP_TX_T; // Time at which last sample was sent (us)
static unsigned long tx_Dt_us = 0; // Time since last sample was sent (us)
#if DEBUG_TX_TIMING
#define LOOPS 100
int cnt_loops = 0;
int cnt_fails = 0;
#endif // DEBUG_TX_TIMING

void printdata(void)
{
    unsigned long t_curr_us = micros();
    tx_Dt_us = t_curr_us - sampling_T_us;
    sampling_T_us = t_curr_us;

#if DEBUG_TX_TIMING
    if((tx_Dt_us > SAMP_TX_T_MAX) ||
       (tx_Dt_us < SAMP_TX_T_MIN))
	cnt_fails++;
    if(cnt_loops++ > LOOPS)
    {
	if(cnt_fails > 0)
	{
	    Serial.print("!");
	    Serial.print(cnt_fails);
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
    if(!print_binary)
    {
	Serial.print(ATOMIC_IMU_INIT);
	Serial.print(ATOMIC_IMU_SEPARATOR);
	Serial.print(tx_Dt_us);
	Serial.print(ATOMIC_IMU_SEPARATOR);
	Serial.print(sen_data.accel_x_raw);
	Serial.print(ATOMIC_IMU_SEPARATOR);
	Serial.print(sen_data.accel_y_raw);
	Serial.print(ATOMIC_IMU_SEPARATOR);
	Serial.print(sen_data.accel_z_raw);
	Serial.print(ATOMIC_IMU_SEPARATOR);
	Serial.print(sen_data.gyro_x_raw);
	Serial.print(ATOMIC_IMU_SEPARATOR);
	Serial.print(sen_data.gyro_y_raw);
	Serial.print(ATOMIC_IMU_SEPARATOR);
	Serial.print(sen_data.gyro_z_raw);
	Serial.print(ATOMIC_IMU_SEPARATOR);
	Serial.print(sen_data.magnetom_x_raw);
	Serial.print(ATOMIC_IMU_SEPARATOR);
	Serial.print(sen_data.magnetom_y_raw);
	Serial.print(ATOMIC_IMU_SEPARATOR);
	Serial.print(sen_data.magnetom_z_raw);
	Serial.print(ATOMIC_IMU_SEPARATOR);
	Serial.print(sen_data.baro_temp);
	Serial.print(ATOMIC_IMU_SEPARATOR);
	Serial.print(sen_data.baro_pres);
	Serial.print(ATOMIC_IMU_SEPARATOR);
#if DEBUG
	if(print_raw_bmp085)
	{
	    Serial.print(sen_data.baro_temp_raw);
	    Serial.print(ATOMIC_IMU_SEPARATOR);
	    Serial.print(sen_data.baro_pres_raw);
	    Serial.print(ATOMIC_IMU_SEPARATOR);
	}
#endif // DEBUG
	Serial.print(ATOMIC_IMU_END);
	Serial.println();
    }
    else // !print_binary
    {
	Serial.println("Not implemented.");
    }

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
