
void printdata(void)
{    
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
}

long convert_to_dec(float x)
{
  return x*10000000;
}

