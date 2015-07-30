
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

  // intialize HMC5883L
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


  // magnetometer offsets - needs to be retested at mount
  g->off_mx = 0; //61;
  g->off_my = 0; //16;
  g->off_mz = 0; //-154;

  // magnetometer scalers
  g->scale_mx = 1.0/519.0;
  g->scale_my = 1.0/449.0;
  g->scale_mz = 1.0/526.0;


  // angles
  for (int i = 0; i < 3; i++){
    g->ypr[i] =  0.0;
  }


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

void remove_offsets(imu* g) {
  /* remove offset for each imu axis */
  g->ax = g->ax - ACC_OFF_X;
  g->ay = g->ay - ACC_OFF_Y;
  g->az = g->az - ACC_OFF_Z;

  g->gx = g->gx - GYRO_OFF_X;
  g->gy = g->gy - GYRO_OFF_Y;
  g->gz = g->gz - GYRO_OFF_Z;

  /* special offset removal for magnetometer */
  g->m[0] = (double)(g->mx);
  g->m[1] = (double)(g->my);
  g->m[2] = (double)(g->mz);

  double len_m = sqrt(g->m[0]*g->m[0]
                    + g->m[1]*g->m[1]
                    + g->m[2]*g->m[2]);

  double len_last_m = sqrt(g->last_m[0]*g->last_m[0]
                         + g->last_m[1]*g->last_m[1]
                         + g->last_m[2]*g->last_m[2]);

  double len_diff = sqrt((g->m[0]-g->last_m[0])*(g->m[0]-g->last_m[0])
                       + (g->m[1]-g->last_m[1])*(g->m[1]-g->last_m[1])
                       + (g->m[2]-g->last_m[2])*(g->m[2]-g->last_m[2]));
  
  if (abs(len_diff) > 0.00001){
    g->off_m[0] = g->off_m[0] + MAG_OFF_GAIN*((g->m[0]-g->last_m[0])/len_diff)*(len_m-len_last_m);
    g->off_m[1] = g->off_m[1] + MAG_OFF_GAIN*((g->m[1]-g->last_m[1])/len_diff)*(len_m-len_last_m);
    g->off_m[2] = g->off_m[2] + MAG_OFF_GAIN*((g->m[2]-g->last_m[2])/len_diff)*(len_m-len_last_m);
  
  }
  
  g->last_m[0] = (double)(g->mx);
  g->last_m[1] = (double)(g->my);
  g->last_m[2] = (double)(g->mz);

  g->mx = g->mx - (int)g->off_m[0];
  g->my = g->my - (int)g->off_m[1];
  g->mz = g->mz - (int)g->off_m[2];
}


void complementary_filter(imu* g){

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
  //g->x_acc = ((double)(g->ax)) * g->scale_ax;
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

  g->ypr_rad[PITCH] = -(P1*(-g->ypr[PITCH] + g->y_gyr*0.001) + (1.0-P1)*g->x_acc);
  g->ypr_rad[ROLL] = (P2*(g->ypr[ROLL] + g->x_gyr*0.001) + (1.0-P2)*g->y_acc);

  g->ypr[PITCH] = g->ypr_rad[PITCH]*RAD_TO_DEG;
  g->ypr[ROLL] = g->ypr_rad[ROLL]*RAD_TO_DEG;
  
}


void tilt_compensation(imu* g){
  /* calculate magnetometer angles */
  /*double scale = sqrt((double)g->mx*(double)g->mx + (double)g->my*(double)g->my + (double)g->mz*(double)g->mz);
  g->x_mag = (double)(g->mx) / scale;
  g->y_mag = (double)(g->my) / scale;
  g->z_mag = (double)(g->mz) / scale;*/

  g->x_mag = (double)(g->mx) * g->scale_mx;
  g->y_mag = (double)(g->my) * g->scale_my;
  g->z_mag = (double)(g->mz) * g->scale_mz;

  /* perform tilt compensation */
  g->xh = g->x_mag*cos(g->ypr[PITCH]*(M_PI/180.0)) 
          + g->y_mag*sin(g->ypr[PITCH]*(M_PI/180.0))*sin(g->ypr[ROLL]*(M_PI/180.0)) 
          + g->z_mag*sin(g->ypr[PITCH]*(M_PI/180.0))*cos(g->ypr[PITCH]*(M_PI/180.0));

  g->yh = g->y_mag*cos(g->ypr[ROLL]*(M_PI/180.0)) 
          - g->z_mag*sin(g->ypr[ROLL]*(M_PI/180.0));

  g->ypr[YAW] = atan2(-g->yh, g->xh)*(180.0/M_PI);
}

/* Reads raw data from gyro and calculates yaw, pitch and roll */
void read_imu(imu* g){

	// read raw accel/gyro measurements from device
  g->gyro.getMotion6(&(g->ax), &(g->ay), &(g->az), 
                      &(g->gx), &(g->gy), &(g->gz));

  // read raw data from magnetometer
  read_magnetometer(g);

  //offsets
  remove_offsets(g);
  
  //get pitch and roll
  complementary_filter(g);

  //get yaw
  tilt_compensation(g);
}