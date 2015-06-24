
#include "gyro.h"


bool init_gyro(gyro* g){
	// join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

 	// initialize device
  g->gyro.initialize();

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


void read_gyro(gyro* g){

	// read raw accel/gyro measurements from device
  g->gyro.getMotion6(&(g->ax), &(g->ay), &(g->az), &(g->gx), &(g->gy), &(g->gz));


  /* ================== Complementary filter ================== */

  /* remove offsets - needs to be tested better */
  g->ax = g->ax - g->off_ax;
  g->ay = g->ay - g->off_ay;
  g->az = g->az - g->off_az;

  g->gx = g->gx - g->off_gx;
  g->gy = g->gy - g->off_gy;
  g->gz = g->gz - g->off_gz;


  /* calculate accelerometer angle and angular velocity */
  g->x_acc = ((double)(g->ax)) * g->scale_ax;
  g->y_gyr = ((double)(g->gy)) * g->scale_gy;

  g->y_acc = ((double)(g->ay)) * g->scale_ay;
  g->x_gyr = ((double)(g->gx)) * g->scale_gx;


  /* calculate pitch and roll */
  g->ypr[1] = (g->p1*(g->ypr[1]/DEG_PITCH + g->y_gyr*0.001) + g->p2*g->x_acc)*DEG_PITCH;
  g->ypr[2] = (g->r1*(g->ypr[2]/DEG_ROLL + g->x_gyr*0.001) + g->r2*g->y_acc)*DEG_ROLL;

  /* ========================================================== */
}