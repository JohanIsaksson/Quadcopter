
#include "SpeedTrig.h"
#include "imu.h"


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
  //azs = (double)az * ACC_SCALE_Z;

  // accelerometer angles
  x_acc = SpeedTrig.atan2(axs,sqrt(ACC_SCALE_X *((double)(ay*ay + az*az))));
  y_acc = SpeedTrig.atan2(ays,sqrt(ACC_SCALE_X *((double)(ax*ax + az*az))));
  

  // scale and filter angular velocity
  y_gyr_u = (y_gyr_u >> 2) + (y_gyr_u >> 1) + (gy >> 1);
  y_gyr = ((double)y_gyr_u)*GYRO_SCALE_Y;

  x_gyr_u = (x_gyr_u >> 2) + (x_gyr_u >> 1) + (gx >> 1);
  x_gyr = ((double)x_gyr_u)*GYRO_SCALE_X;

  z_gyr_u = (z_gyr_u >> 2) + (z_gyr_u >> 1) + (gz >> 1);
  z_gyr = ((double)z_gyr_u)*GYRO_SCALE_Z;

  //z_gyr = z_gyr*0.75 + (((double)(gz)) * GYRO_SCALE_Z)*0.25;


  ypr_rad[PITCH] = -(P1*(-ypr_rad[PITCH] - y_gyr*tim*GYRO_GAIN_PITCH) + (1.0-P1)*x_acc);
  ypr_rad[ROLL] = (P2*(ypr_rad[ROLL] + x_gyr*tim*GYRO_GAIN_ROLL) + (1.0-P2)*y_acc);

  ypr[PITCH] = ypr_rad[PITCH] * RAD_TO_DEG;
  ypr[ROLL] = ypr_rad[ROLL] * RAD_TO_DEG;

}


/* Initializes the imu and sets parameters */
void IMU::init(){
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  // init mpu
  if (imu.begin() != INV_SUCCESS)
  {
    while (1); //break
  }
  
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              200); // Set DMP FIFO rate to 200 Hz
}

void IMU::calculate_gyro(){
  // scale and filter angular velocity
  y_gyr_u = (y_gyr_u >> 2) + (y_gyr_u >> 1) + (imu.gy >> 1);
  y_gyr = ((double)y_gyr_u)*GYRO_SCALE_Y;

  x_gyr_u = (x_gyr_u >> 2) + (x_gyr_u >> 1) + (imu.gx >> 1);
  x_gyr = ((double)x_gyr_u)*GYRO_SCALE_X;

  z_gyr_u = (z_gyr_u >> 2) + (z_gyr_u >> 1) + (imu.gz >> 1);
  z_gyr = ((double)z_gyr_u)*GYRO_SCALE_Z;
}

// Reads raw data from sensors and calculates yaw, pitch and roll
void IMU::update_horizon(double tim){

	// read raw accel/gyro measurements from device
  //MPU6050_read();
  if (imu.fifoAvailable()){
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if (imu.dmpUpdateFifo() == INV_SUCCESS){
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      //mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      //imu.computeEulerAngles();
      }
  }

  float q0 = imu.calcQuat(imu.qw);
  float q1 = imu.calcQuat(imu.qx);
  float q2 = imu.calcQuat(imu.qy);
  float q3 = imu.calcQuat(imu.qz);

  float sq0 = q0*q0;
  float sq1 = q1*q1;
  float sq2 = q2*q2;
  float sq3 = q3*q3;
  
  // we can now use the same terms as in the textbook.
  ypr_rad[ROLL]  = atan2f(2.0f * q2 * q3 + 2.0f * q0 * q1, sq3 - sq2 - sq1 + sq0);
  ypr_rad[PITCH] = -asin(2.0f * q1 * q3 - 2.0f * q0 * q2);
  ypr_rad[YAW] = atan2f(2.0f * q1 * q2 + 2.0f * q0 * q3, sq1 + sq0 - sq3 - sq2);

  ypr[PITCH] = ypr_rad[PITCH] * RAD_TO_DEG;
  ypr[ROLL] = ypr_rad[ROLL] * RAD_TO_DEG;
  ypr[YAW] = ypr_rad[YAW] * RAD_TO_DEG;

  //scale and filter angular velocity
  y_gyr = y_gyr*0.8 + (((double)(gy)) * GYRO_SCALE_Y)*0.2;
  x_gyr = x_gyr*0.8 + (((double)(gx)) * GYRO_SCALE_X)*0.2;
  z_gyr = z_gyr*0.8 + (((double)(gz)) * GYRO_SCALE_Z)*0.2;
  

  //get height
  //height_estimation(g, t);

  
}

//only reads mpu6050 for gyro
void IMU::update_acro(double tim){

  // read raw accel/gyro measurements from device
  //MPU6050_read();
  //imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  //scale angular velocity
  y_gyr = y_gyr*0.8 + (((double)(gy)) * GYRO_SCALE_Y)*0.2;
  x_gyr = x_gyr*0.8 + (((double)(gx)) * GYRO_SCALE_X)*0.2;
  z_gyr = z_gyr*0.8 + (((double)(gz)) * GYRO_SCALE_Z)*0.2;

}
