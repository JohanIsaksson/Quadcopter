
#include "imu.h"
#include <math.h>

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

  // accelerometer scalers
  g->scale_ax = 1.0/15800.0;
  g->scale_ay = 1.0/16600.0;
  g->scale_az = 1.0/15300.0;

  // accelerometer offsets
  g->off_ax = 525;
  g->off_ay = -128;
  g->off_az = -200;


  // gyro scalers - not tested yet
  g->scale_gx = 1.0/15800.0;
  g->scale_gy = 1.0/15800.0;
  g->scale_gz = 1.0/15800.0;

  // gyro offsets - not tested
  g->off_gx = -320;
  g->off_gy = -110;
  g->off_gz = -30;

  // magnetometer offsets - needs to be retested at mount
  g->off_mx = 61;
  g->off_my = -154;
  g->off_mz = 16;

  // magnetometer scalers
  g->scale_mx = 1.0/519.0;
  g->scale_my = 1.0/526.0;
  g->scale_mz = 1.0/449.0;


  // angles
  for (int i = 0; i < 3; i++){
    g->ypr[i] =  0.0;
  }

  //constants
  g->p1 = 0.8; g->p2 = 0.2;
  g->r1 = 0.8; g->r2 = 0.2;
  g->y1 = 1.0; g->y2 = 0.0;

  return ret;
}

void read_magnetometer(imu* g){
  uint8_t buffer[6];

  //read from magnetometer
  I2Cdev::readBytes(MAG_ADDR, 0x03, 6, buffer);
  g->mx = (((int16_t)buffer[0]) << 8) | buffer[1];
  g->my = (((int16_t)buffer[2]) << 8) | buffer[3];
  g->mz = (((int16_t)buffer[4]) << 8) | buffer[5];
}

void remove_offsets(imu* g) {
  /* remove offsets - needs to be tested better */
  g->ax = g->ax - g->off_ax;
  g->ay = g->ay - g->off_ay;
  g->az = g->az - g->off_az;

  g->gx = g->gx - g->off_gx;
  g->gy = g->gy - g->off_gy;
  g->gz = g->gz - g->off_gz;

  g->mx = g->mx - g->off_mx;
  g->my = g->my - g->off_my;
  g->mz = g->mz - g->off_mz;
}


void complementary_filter(imu* g){

  /* calculate accelerometer angle and angular velocity */
  g->x_acc = ((double)(g->ax)) * g->scale_ax;
  g->y_gyr = ((double)(g->gy)) * g->scale_gy;

  g->y_acc = ((double)(g->ay)) * g->scale_ay;
  g->x_gyr = ((double)(g->gx)) * g->scale_gx;


  /* calculate pitch and roll */
  g->ypr[PITCH] = (g->p1*(g->ypr[PITCH]/DEG_PITCH + g->y_gyr*0.001) + g->p2*g->x_acc)*DEG_PITCH;
  g->ypr[ROLL] = (g->r1*(g->ypr[ROLL]/DEG_ROLL + g->x_gyr*0.001) + g->r2*g->y_acc)*DEG_ROLL;
}

void tilt_compensation(imu* g){
  /* calculate magnetometer angles */
  g->x_mag = (double)(g->mx) * g->scale_mx;
  g->y_mag = (double)(g->my) * g->scale_my;
  g->z_mag = (double)(g->mz) * g->scale_mz;

  g->xh = g->x_mag*cos(g->ypr[PITCH]) 
          + g->y_mag*sin(g->ypr[PITCH])*sin(g->ypr[ROLL]) 
          + g->z_mag*sin(g->ypr[PITCH])*cos(g->ypr[PITCH]);

  g->yh = g->y_mag*cos(g->ypr[ROLL]) 
          + g->z_mag*sin(g->ypr[ROLL]);

  g->ypr[YAW] = atan2(-g->yh, g->xh);
  /* calculate magnetometer angles */
  g->x_mag = (double)(g->mx) * g->scale_mx;
  g->y_mag = (double)(g->my) * g->scale_my;
  g->z_mag = (double)(g->mz) * g->scale_mz;

  /* perform tilt compensation */
  g->xh = g->x_mag*cos(g->ypr[PITCH]) 
          + g->y_mag*sin(g->ypr[PITCH])*sin(g->ypr[ROLL]) 
          + g->z_mag*sin(g->ypr[PITCH])*cos(g->ypr[PITCH]);

  g->yh = g->y_mag*cos(g->ypr[ROLL]) 
          + g->z_mag*sin(g->ypr[ROLL]);

  g->ypr[YAW] = atan2(-g->yh, g->xh);
}

/* Reads raw data from gyro and perform complementary filtering */
void read_imu(imu* g){

	// read raw accel/gyro measurements from device
  g->gyro.getMotion6(&(g->ax), &(g->ay), &(g->az), 
                      &(g->gx), &(g->gy), &(g->gz));

  //offsets
  remove_offsets(g);
  
  //get pitch and roll
  complementary_filter(g);

  //get yaw
  tilt_compensation(g);
}