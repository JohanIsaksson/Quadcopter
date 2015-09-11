
#include "imu.h"

/* Initializes the imu and sets parameters */
bool init_imu(imu* g){
	// join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

 	// initialize MPU6050
  g->gyro.initialize();

  g->gyro.setXAccelOffset(-2601);
  g->gyro.setYAccelOffset(-1688);
  g->gyro.setZAccelOffset(1344);

  g->gyro.setXGyroOffset(98);
  g->gyro.setYGyroOffset(32);
  g->gyro.setZGyroOffset(18);

  // initialize HMC5883L
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();

  bool ret = g->gyro.testConnection(); 


  // lp filter
  for (int i = 0; i < LP_BUFFER_SIZE; i++){
    g->ax_buf[i] = 0;
    g->ay_buf[i] = 0;
    g->az_buf[i] = 0;
  }
  g->ax_sum = 0;
  g->ay_sum = 0;
  g->az_sum = 0;
  g->lp_pos = 0;


  // angles
  for (int i = 0; i < 3; i++){
    g->ypr[i] =  0.0;
  }

  g->height = 0.0;
  g->vertical_speed = 0.0;
  g->vertical_acc = 0.0;


  return ret;
}

void read_magnetometer(imu* g){
  uint8_t buffer[6];

  //read from magnetometer
  I2Cdev::readBytes(MAG_ADDR, 0x03, 6, buffer);
  g->mx = (((int16_t)buffer[0]) << 8) | buffer[1];
  g->mz = (((int16_t)buffer[2]) << 8) | buffer[3];
  g->my = (((int16_t)buffer[4]) << 8) | buffer[5];
}

/* special offset removal for magnetometer */  
void remove_offsets(imu* g) {
  /* remove bias */
  g->mx = g->mx - MAG_OFF_X;
  g->my = g->my - MAG_OFF_Y;
  g->mz = g->mz - MAG_OFF_Z;

  /* remove hard and soft iron offset */
  g->x_mag = M11*((double)g->mx) + M12*((double)g->my) + M13*((double)g->mz);
  g->y_mag = M21*((double)g->mx) + M22*((double)g->my) + M23*((double)g->mz);
  g->z_mag = M31*((double)g->mx) + M32*((double)g->my) + M33*((double)g->mz);
}


void complementary_filter(imu* g, double tim){

  /* low pass filter */
  g->ax_sum -= g->ax_buf[g->lp_pos];
  g->ax_buf[g->lp_pos] = g->ax;
  g->ax_sum += g->ax;
  g->ax = g->ax_sum/LP_BUFFER_SIZE;

  g->ay_sum -= g->ay_buf[g->lp_pos];
  g->ay_buf[g->lp_pos] = g->ay;
  g->ay_sum += g->ay;
  g->ay = g->ay_sum/LP_BUFFER_SIZE;

  g->az_sum -= g->az_buf[g->lp_pos];
  g->az_buf[g->lp_pos] = g->az;
  g->az_sum += g->az;
  g->az = g->az_sum/LP_BUFFER_SIZE;

  if (g->lp_pos < LP_BUFFER_SIZE-1){
    g->lp_pos++;
  }else{
    g->lp_pos = 0;
  }
  

  /* calculate accelerometer angle and angular velocity */
  double x = (double)g->ax * ACC_SCALE_X;
  double y = (double)g->ay * ACC_SCALE_Y;
  double z = (double)g->az * ACC_SCALE_Z;

  g->x_acc = atan(x/sqrt(y*y + z*z));

  g->y_gyr = ((double)(g->gy)) * GYRO_SCALE_Y;

  //g->y_acc = ((double)(g->ay)) * g->scale_ay;
  g->y_acc = atan(y/sqrt(x*x + z*z));

  g->x_gyr = ((double)(g->gx)) * GYRO_SCALE_X;


  //g->ypr[PITCH] = g->x_acc;
  //g->ypr[ROLL] = g->y_acc;

  g->ypr_rad[PITCH] = -(P1*(-g->ypr_rad[PITCH] - g->y_gyr*tim*GYRO_GAIN_PITCH) + (1.0-P1)*g->x_acc);
  g->ypr_rad[ROLL] = (P2*(g->ypr_rad[ROLL] + g->x_gyr*tim*GYRO_GAIN_ROLL) + (1.0-P2)*g->y_acc);


  g->ypr[PITCH] = g->ypr_rad[PITCH] * RAD_TO_DEG;
  g->ypr[ROLL] = g->ypr_rad[ROLL] * RAD_TO_DEG;
  
}


void tilt_compensation(imu* g){
  /* calculate magnetometer angles */

  //g->x_mag = (double)(g->mx) * MAG_SCALE_X;
  //g->y_mag = (double)(g->my) * MAG_SCALE_Y;
  //g->z_mag = (double)(g->mz) * MAG_SCALE_Z;

  /* perform tilt compensation */
  g->xh = g->x_mag*cos(g->ypr_rad[PITCH]) 
          + g->y_mag*sin(g->ypr_rad[PITCH])*sin(g->ypr_rad[ROLL]) 
          + g->z_mag*sin(g->ypr_rad[PITCH])*cos(g->ypr_rad[PITCH]);

  g->yh = g->y_mag*cos(g->ypr_rad[ROLL]) 
          - g->z_mag*sin(g->ypr_rad[ROLL]);

  g->ypr[YAW] = atan2(-g->yh, g->xh)*(180.0/M_PI);

  /* get yaw rate from gyro */
  g->z_gyr = (double)g->gz * GYRO_SCALE_Z;

}


void height_estimation(imu* g, double tim){
  double x = (double)g->ax * ACC_SCALE_X * sin(g->ypr_rad[ROLL]);
  double y = (double)g->ay * ACC_SCALE_Y * sin(g->ypr_rad[PITCH]);
  double z = (double)g->az * ACC_SCALE_Z;

  g->vertical_acc = z - sqrt(1.0 - (x*x + y*y)) + 0.002;
  g->vertical_speed = g->vertical_acc * 9.82 * tim;
  g->height += g->vertical_speed * tim;
}

/* Reads raw data from gyro and calculates yaw, pitch and roll */
void read_imu(imu* g, uint32_t tim){

	// read raw accel/gyro measurements from device
  g->gyro.getMotion6(&(g->ax), &(g->ay), &(g->az), 
                      &(g->gx), &(g->gy), &(g->gz));

  // read raw data from magnetometer
  read_magnetometer(g);

  //offsets
  remove_offsets(g);
  
  //get pitch and roll
  double t = ((double)tim)/1000.0;
  complementary_filter(g, t);

  //get yaw
  tilt_compensation(g);

  //get height
  height_estimation(g, t);
}