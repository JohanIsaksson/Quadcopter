
#include "imu.h"



void MPU6050_init(){
  I2Cdev::writeBits(MPU6050_ADDR, 0x6B, 2, 3, 0x01); //set internal clock to XGYRO - should be best

  I2Cdev::writeBits(MPU6050_ADDR, 0x1B, 4, 2, 0x00); //set full scale gyro range +- 250 deg/s

  I2Cdev::writeBits(MPU6050_ADDR, 0x1C, 4, 2, 0x00); //set full scale accelerometer range +- 2g

  I2Cdev::writeBit(MPU6050_ADDR, 0x6B, 6, false); //set sleep to false

  //set offsets
  I2Cdev::writeWord(MPU6050_ADDR, 0x06, 251);// -2601); //x acc

  I2Cdev::writeWord(MPU6050_ADDR, 0x08, -3198);// -1688); //y acc

  I2Cdev::writeWord(MPU6050_ADDR, 0x0A, 2482);// 1344); //z acc


  I2Cdev::writeWord(MPU6050_ADDR, 0x13, 39);// 98); //x gyro

  I2Cdev::writeWord(MPU6050_ADDR, 0x15, -13);// 32); //y gyro

  I2Cdev::writeWord(MPU6050_ADDR, 0x17, 75);// 18); //z gyro

}

void MPU6050_read(imu* g){
  I2Cdev::readBytes(MPU6050_ADDR, MPU6050_DATAREG, 14, g->I2C_buffer);
  g->ax = (((int16_t)g->I2C_buffer[0]) << 8) | g->I2C_buffer[1];
  g->ay = (((int16_t)g->I2C_buffer[2]) << 8) | g->I2C_buffer[3];
  g->az = (((int16_t)g->I2C_buffer[4]) << 8) | g->I2C_buffer[5];
  g->gx = (((int16_t)g->I2C_buffer[8]) << 8) | g->I2C_buffer[9];
  g->gy = (((int16_t)g->I2C_buffer[10]) << 8) | g->I2C_buffer[11];
  g->gz = (((int16_t)g->I2C_buffer[12]) << 8) | g->I2C_buffer[13];
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


void magnetometer_calibration(imu* g){
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
  g->axs = (double)g->ax * ACC_SCALE_X;
  g->ays = (double)g->ay * ACC_SCALE_Y;
  g->azs = (double)g->az * ACC_SCALE_Z;


  g->x_acc = atan(g->axs/sqrt(g->ays*g->ays + g->azs*g->azs));
  g->y_gyr = ((double)(g->gy)) * GYRO_SCALE_Y;

  g->y_acc = atan(g->ays/sqrt(g->axs*g->axs + g->azs*g->azs));
  g->x_gyr = ((double)(g->gx)) * GYRO_SCALE_X;


  g->ypr_rad[PITCH] = -(P1*(-g->ypr_rad[PITCH] - g->y_gyr*tim*GYRO_GAIN_PITCH) + (1.0-P1)*g->x_acc);
  g->ypr_rad[ROLL] = (P2*(g->ypr_rad[ROLL] + g->x_gyr*tim*GYRO_GAIN_ROLL) + (1.0-P2)*g->y_acc);


  g->ypr[PITCH] = g->ypr_rad[PITCH] * RAD_TO_DEG;
  g->ypr[ROLL] = g->ypr_rad[ROLL] * RAD_TO_DEG;

  g->cosr = cos(g->ypr_rad[ROLL]);
  g->sinr = sin(g->ypr_rad[ROLL]);
  g->sinp = sin(g->ypr_rad[PITCH]);
  g->cosp = cos(g->ypr_rad[PITCH]);
  
}


void tilt_compensation(imu* g){

  /* perform tilt compensation */
  g->xh = g->x_mag*g->cosp 
          + g->y_mag*g->sinp*g->sinr 
          + g->z_mag*g->sinp*g->cosr;

  g->yh = g->y_mag*g->cosr 
          - g->z_mag*g->sinr;

  g->ypr_rad[YAW] = atan2(-g->yh, g->xh);
  g->ypr[YAW] = g->ypr_rad[YAW]*RAD_TO_DEG;

  /* get yaw rate from gyro */
  g->z_gyr = (double)g->gz * GYRO_SCALE_Z;

}


void height_estimation(imu* g, double tim){
  double x = g->axs * g->sinr;
  double y = g->ays * g->sinp;

  g->vertical_acc = g->azs - sqrt(1.0 - (x*x + y*y)) + 0.002;
  g->vertical_speed = g->vertical_acc * 9.82 * tim;
  g->height += g->vertical_speed * tim;
}



/* Initializes the imu and sets parameters */
void imu_init(imu* g){
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  // initialize MPU6050
  MPU6050_init();

  // initialize HMC5883L
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();

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
}

/* Reads raw data from gyro and calculates yaw, pitch and roll */
void imu_update_horizon(imu* g, uint32_t tim){

	// read raw accel/gyro measurements from device
  MPU6050_read(g);

  // read raw data from magnetometer
  read_magnetometer(g);

  //offsets
  //remove_offsets(g);
  
  //get pitch and roll
  double t = ((double)tim)/1000000.0;
  complementary_filter(g, t);

  //get yaw
  tilt_compensation(g);

  //get height
  height_estimation(g, t);
}

//only reads mpu6050 for gyro
void imu_update_acro(imu* g, uint32_t tim){

  // read raw accel/gyro measurements from device
  MPU6050_read(g);

  ///* scale angular velocity */
  g->axs = (double)g->ax * ACC_SCALE_X;
  g->ays = (double)g->ay * ACC_SCALE_Y;
  g->azs = (double)g->az * ACC_SCALE_Z;
}