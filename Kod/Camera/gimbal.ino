#include "I2Cdev.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <math.h>

#include <Servo.h>


#define MPU6050_ADDR 0x68
#define MPU6050_DATAREG 0x3B
#define ACC_SCALE 16384.0
#define GYRO_SCALE 131


int16_t ax, ay, az, gx, gy, gz;
double axs, ays, azs, gxs, gys, gzs;
double roll, pitch;
uint32_t t, last_t;
double dt;
uint8_t I2C_buffer[14];
int servo_roll, servo_pitch;

Servo servo_r, servo_p;


double map_d(double x, double in_min, double in_max, double out_min, double out_max){
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void MPU6050_init(){
  I2Cdev::writeBits(MPU6050_ADDR, 0x6B, 2, 3, 0x01); //set internal clock to XGYRO - should be best

  I2Cdev::writeBits(MPU6050_ADDR, 0x1B, 4, 2, 0x00); //set full scale gyro range +- 250 deg/s

  I2Cdev::writeBits(MPU6050_ADDR, 0x1C, 4, 2, 0x00); //set full scale accelerometer range +- 2g

  I2Cdev::writeBit(MPU6050_ADDR, 0x6B, 6, false); //set sleep to false


  //set offsets (on gimbal mpu6050)
  I2Cdev::writeWord(MPU6050_ADDR, 0x06, -2568); //x acc
  I2Cdev::writeWord(MPU6050_ADDR, 0x08, -1660); //y acc
  I2Cdev::writeWord(MPU6050_ADDR, 0x0A, 1358); //z acc

  I2Cdev::writeWord(MPU6050_ADDR, 0x13, 102);// 98); //x gyro
  I2Cdev::writeWord(MPU6050_ADDR, 0x15, 31);// 32); //y gyro
  I2Cdev::writeWord(MPU6050_ADDR, 0x17, 18);// 18); //z gyro
}

void MPU6050_read(){

  I2Cdev::readBytes(MPU6050_ADDR, MPU6050_DATAREG, 14, I2C_buffer);
  ax = (((int16_t)I2C_buffer[0]) << 8) | I2C_buffer[1];
  ay = (((int16_t)I2C_buffer[2]) << 8) | I2C_buffer[3];
  az = (((int16_t)I2C_buffer[4]) << 8) | I2C_buffer[5];
  gx = (((int16_t)I2C_buffer[8]) << 8) | I2C_buffer[9];
  gy = (((int16_t)I2C_buffer[10]) << 8) | I2C_buffer[11];
  gz = (((int16_t)I2C_buffer[12]) << 8) | I2C_buffer[13];
}

void setup(){
  Serial.begin(38400);
  Wire.begin();
  MPU6050_init(); 

  servo_r.attach(9);
  servo_p.attach(8);


}


void loop(){
  t = micros();
  dt = ((double)(t - last_t))/1000000.0;
  last_t = t;

  MPU6050_read();


  
  axs = (double)ax / ACC_SCALE;
  ays = (double)ay / ACC_SCALE;
  azs = (double)az / ACC_SCALE;

  gxs = (double)gx / GYRO_SCALE;
  gys = (double)gy / GYRO_SCALE;
  gzs = (double)gz / GYRO_SCALE;

  /* calculate accelerometer angle and angular velocity */
  pitch = -(0.99*(-pitch - gys*dt) + 0.01*((atan2(axs, sqrt(ays*ays + azs*azs))*180.0)/M_PI));
  roll = (0.99*(roll + gxs*dt) + 0.01*((atan2(-ays, azs)*180.0)/M_PI));
  

  Serial.println(roll, 10);

  servo_pitch = (int)map_d(pitch, -80.0, 80.0, 140.0, 0.0);

  servo_p.write(servo_pitch);

  delay(1);

}