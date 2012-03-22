/* ******************************************************* */
/* I2C code for HMC5583 magnetometer                       */
/*                                                         */
/* ******************************************************* */


//I2C addresses 
int CompassAddress = 0x1E;   //Write:0x3C  Read:0x3D

HMC58X3 magn;

//============================================
// Magnetormeter
//============================================
void Init_Compass()
{
  // no delay needed as we have already a delay(5) in HMC5843::init()
  magn.init(false); // Dont set mode yet, we'll do that later on.
  // Calibrate HMC using self test, not recommended to change the gain after calibration.
  // Will apply a ~1.1Ga field to the sensor, so gain>1 to avoid clipping.
  magn.calibrate(1); // valid: 0-7, 7 not recommended.
  // Single mode conversion was used in calibration, now set continuous mode
  magn.setMode(0);
}



void Read_Compass()
{
          
    
          
          
          magn.getRaw(&sen_data.magnetom_x_raw,&sen_data.magnetom_y_raw,&sen_data.magnetom_z_raw);
          
	  // NOT USED
          /* float magRot_x; */
          /* float magRot_y; */
          /* float magRot_z;           */
          /* magn.getValues(&sen_data.magnetom_x,&sen_data.magnetom_y,&sen_data.magnetom_z); */

          /* //=============================================== */
          /* // Apply Hard iron compensation  */
          /* // This removes effects of permanetly magnetic material */
          /* // that is on the sensor platform */
          /* // (offset X,Y,and Z of raw data) */
          
          /* sen_data.magnetom_x -= sen_offset.magnetom_offset[0]; */
          /* sen_data.magnetom_y -= sen_offset.magnetom_offset[1]; */
          /* sen_data.magnetom_z -= sen_offset.magnetom_offset[2]; */
          
          
          
          /* //=============================================== */
          /* // Apply Soft iron compensation */
          /* // This removes the distortion caused by ferrous  */
          /* // material that is on the sensor platform */
          /* // (turn the elipse into a circle) */
   
          /* //=============================================== */
          /* //Apply correction to the X axis */
          
          /* //rotate the XY vector */
          /* magRot_x = sen_data.magnetom_x * cos(sen_offset.magnetom_XY_Theta) + sen_data.magnetom_y *sin(sen_offset.magnetom_XY_Theta); */
          /* magRot_y = sen_data.magnetom_y * cos(sen_offset.magnetom_XY_Theta) - sen_data.magnetom_x *sin(sen_offset.magnetom_XY_Theta); */
          
          /* //Scale the X */
          /* sen_data.magnetom_x = magRot_x/sen_offset.magnetom_XY_Scale; */
          
          /* //rotate the XY vector back */
          /* sen_data.magnetom_x = sen_data.magnetom_x * cos(-sen_offset.magnetom_XY_Theta) + magRot_y * sin(-sen_offset.magnetom_XY_Theta); */
          /* sen_data.magnetom_y = magRot_y * cos(-sen_offset.magnetom_XY_Theta) + sen_data.magnetom_x * sin(-sen_offset.magnetom_XY_Theta); */
          
          
          
          /* //=============================================== */
          /* //Apply correction to the Z axis */
          
          /* //rotate the YZ vector */
          /* magRot_z = sen_data.magnetom_z * cos(sen_offset.magnetom_YZ_Theta) + sen_data.magnetom_y *sin(sen_offset.magnetom_YZ_Theta); */
          /* magRot_y = sen_data.magnetom_y * cos(sen_offset.magnetom_YZ_Theta) - sen_data.magnetom_z *sin(sen_offset.magnetom_YZ_Theta); */
          
          /* //Scale the Z */
          /* sen_data.magnetom_z = magRot_z/sen_offset.magnetom_YZ_Scale; */
          
          /* //rotate the YZ vector back */
          /* sen_data.magnetom_z = sen_data.magnetom_z * cos(-sen_offset.magnetom_YZ_Theta) + magRot_y * sin(-sen_offset.magnetom_YZ_Theta); */
          /* sen_data.magnetom_y = magRot_y * cos(-sen_offset.magnetom_YZ_Theta) + sen_data.magnetom_z * sin(-sen_offset.magnetom_YZ_Theta); */
          
          
          /* //=============================================== */
          /* // Now that the magnetometer data is corrected, */
          /* // we can calculate our magnetic heading */
          /* Compass_Heading(); // Calculate magnetic heading  */
}



//==============================================================================================================

void Compass_Heading()
{
  float MAG_X;
  float MAG_Y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  
  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);
  
  // Tilt compensated Magnetic field X:
  MAG_X = sen_data.magnetom_x*cos_pitch + sen_data.magnetom_y*sin_roll*sin_pitch + sen_data.magnetom_z*cos_roll*sin_pitch;
  
  // Tilt compensated Magnetic field Y:
  MAG_Y = sen_data.magnetom_y*cos_roll - sen_data.magnetom_z*sin_roll;
  
  // Magnetic Heading
  sen_data.magnetom_heading = atan2(-1*MAG_Y,MAG_X);
  
  if(sen_data.magnetom_heading < 0) 
    sen_data.magnetom_heading += 2 * M_PI;
 
}

#if DEBUG
void GetMagnCalibration(float *xyz_scale, float *xyz_max) {
    magn.getCalibration(xyz_scale, xyz_max);
}
#endif
