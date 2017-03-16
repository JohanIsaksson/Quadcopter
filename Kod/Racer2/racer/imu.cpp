

#include "imu.h"



void IMU::MPU6050_init(){
  I2Cdev::writeBits(MPU6050_ADDR, 0x6B, 2, 3, 0x01); //set internal clock to XGYRO - should be best

  I2Cdev::writeBits(MPU6050_ADDR, 0x1B, 4, 2, 0x00); //set full scale gyro range +- 250 deg/s

  I2Cdev::writeBits(MPU6050_ADDR, 0x1C, 4, 2, 0x00); //set full scale accelerometer range +- 2g

  I2Cdev::writeBit(MPU6050_ADDR, 0x6B, 6, false); //set sleep to false


  //set offsets (on chip 1)
  I2Cdev::writeWord(MPU6050_ADDR, 0x06, 1085); //x acc
  I2Cdev::writeWord(MPU6050_ADDR, 0x08, -1008); //y acc
  I2Cdev::writeWord(MPU6050_ADDR, 0x0A, 1659); //z acc

  I2Cdev::writeWord(MPU6050_ADDR, 0x13, 86);// 98); //x gyro
  I2Cdev::writeWord(MPU6050_ADDR, 0x15, 39);// 32); //y gyro
  I2Cdev::writeWord(MPU6050_ADDR, 0x17, -32);// 18); //z gyro
}

void IMU::MPU6050_read(){
  I2Cdev::readBytes(MPU6050_ADDR, MPU6050_DATAREG, 14, I2C_buffer);
  ax = (((int16_t)I2C_buffer[0]) << 8) | I2C_buffer[1];
  ay = (((int16_t)I2C_buffer[2]) << 8) | I2C_buffer[3];
  az = (((int16_t)I2C_buffer[4]) << 8) | I2C_buffer[5];
  gx = (((int16_t)I2C_buffer[8]) << 8) | I2C_buffer[9];
  gy = (((int16_t)I2C_buffer[10]) << 8) | I2C_buffer[11];
  gz = (((int16_t)I2C_buffer[12]) << 8) | I2C_buffer[13];
}

void IMU::read_magnetometer(){
  uint8_t buffer[6];

  //read from magnetometer
  I2Cdev::readBytes(MAG_ADDR, 0x03, 6, buffer);
  mx = (((int16_t)buffer[0]) << 8) | buffer[1];
  mz = (((int16_t)buffer[2]) << 8) | buffer[3];
  my = (((int16_t)buffer[4]) << 8) | buffer[5];
}
/* special offset removal for magnetometer */  
void IMU::remove_offsets() {
  /* remove bias */
  mx = mx - MAG_OFF_X;
  my = my - MAG_OFF_Y;
  mz = mz - MAG_OFF_Z;

  /* remove hard and soft iron offset */
  x_mag = M11*((double)mx) + M12*((double)my) + M13*((double)mz);
  y_mag = M21*((double)mx) + M22*((double)my) + M23*((double)mz);
  z_mag = M31*((double)mx) + M32*((double)my) + M33*((double)mz);
}


void IMU::complementary_filter(double tim){

  /* low pass filter through moving average */
  ax_sum -= ax_buf[lp_pos];
  ax_buf[lp_pos] = ax;
  ax_sum += ax;
  ax = ax_sum >> LP_SHIFT;

  ay_sum -= ay_buf[lp_pos];
  ay_buf[lp_pos] = ay;
  ay_sum += ay;
  ay = ay_sum >> LP_SHIFT;

  az_sum -= az_buf[lp_pos];
  az_buf[lp_pos] = az;
  az_sum += az;
  az = az_sum >> LP_SHIFT;

  if (lp_pos < LP_BUFFER_SIZE-1){
    lp_pos++;
  }else{
    lp_pos = 0;
  }
  

  /* calculate accelerometer angle and angular velocity */
  axs = (double)ax * ACC_SCALE_X;
  ays = (double)ay * ACC_SCALE_Y;
  azs = (double)az * ACC_SCALE_Z;

  x_acc = atan(axs/sqrt(ays*ays + azs*azs)); //can be optimized to
  //x_acc = atan(ax/sqrt(ay*ay + az*az));


  y_acc = atan(ays/sqrt(axs*axs + azs*azs));
  //x_gyr = ((double)(gx)) * GYRO_SCALE_X;
  

  ///* scale angular velocity */
  y_gyr = y_gyr*0.8 + (((double)(gy)) * GYRO_SCALE_Y)*0.2;
  x_gyr = x_gyr*0.8 + (((double)(gx)) * GYRO_SCALE_X)*0.2;
  z_gyr = z_gyr*0.8 + (((double)(gz)) * GYRO_SCALE_Z)*0.2;


  



  //ypr_rad[PITCH] = -(P1*(-ypr_rad[PITCH] - y_gyr*tim*GYRO_GAIN_PITCH) + (1.0-P1)*x_acc);
  ypr_rad[PITCH] = P1*(ypr_rad[PITCH] + y_gyr*tim*GYRO_GAIN_PITCH) - _P1*x_acc;


  ypr_rad[ROLL] = (P2*(ypr_rad[ROLL] + x_gyr*tim*GYRO_GAIN_ROLL) + _P2*y_acc);


  ypr[PITCH] = ypr_rad[PITCH] * RAD_TO_DEG;
  ypr[ROLL] = ypr_rad[ROLL] * RAD_TO_DEG;

  //240us + 2*atan + 2*sqrt
}

void IMU::tilt_compensation(){

  /* perform tilt compensation */
  xh = x_mag*cosp 
          + y_mag*sinp*sinr 
          + z_mag*sinp*cosr;

  yh = y_mag*cosr 
          - z_mag*sinr;

  ypr_rad[YAW] = atan2(-yh, xh);
  ypr[YAW] = ypr_rad[YAW]*RAD_TO_DEG;

  /* get yaw rate from gyro */
  z_gyr = (double)gz * GYRO_SCALE_Z;
}


/* Initializes the imu and sets parameters */
void IMU::init(){
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
    ax_buf[i] = 0;
    ay_buf[i] = 0;
    az_buf[i] = 0;
  }
  ax_sum = 0;
  ay_sum = 0;
  az_sum = 0;
  lp_pos = 0;

  // angles
  for (int i = 0; i < 3; i++){
    ypr[i] =  0.0;
  }

}

/* Reads raw data from sensors and calculates yaw, pitch and roll */
void IMU::update_horizon(double tim){

	// read raw accel/gyro measurements from device
  MPU6050_read();

  // read raw data from magnetometer
  //read_magnetometer(g);

  //offsets
  //remove_offsets(g);
  
  //get pitch and roll
  complementary_filter(tim);

  //get yaw
  //tilt_compensation(g);

  //get height
  //height_estimation(g, t);
}

//only reads mpu6050 for gyro
void IMU::update_acro(double tim){

  // read raw accel/gyro measurements from device
  MPU6050_read();

  ///* scale angular velocity */
  y_gyr = y_gyr*0.8 + (((double)(gy)) * GYRO_SCALE_Y)*0.2;
  x_gyr = x_gyr*0.8 + (((double)(gx)) * GYRO_SCALE_X)*0.2;
  z_gyr = z_gyr*0.8 + (((double)(gz)) * GYRO_SCALE_Z)*0.2;

}
