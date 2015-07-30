#ifndef IMU_H
#define IMU_H


#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <math.h>


#define RAD_TO_DEG 180.0/M_PI

#define LP_BUFFER_SIZE 10

#define YAW 0
#define PITCH 1
#define ROLL 2

#define P1 0.9
#define P2 0.9
#define P3 0.9

#define MAG_ADDR 0x1E
#define MAG_OFF_GAIN 20.0


// gyro offsets 
#define GYRO_OFF_X -395
#define GYRO_OFF_Y -110
#define GYRO_OFF_Z -60

// gyro scalers
#define GYRO_SCALE_X M_PI/23870.0
#define GYRO_SCALE_Y M_PI/23870.0
#define GYRO_SCALE_Z M_PI/23870.0

// accelerometer offsets
#define ACC_OFF_X 525
#define ACC_OFF_Y -128
#define ACC_OFF_Z -200

// accelerometer scalers
#define ACC_SCALE_X 1.0/15800.0
#define ACC_SCALE_Y 1.0/16600.0
#define ACC_SCALE_Z 1.0/15300.0

#define MAG_OFF_X
#define MAG_OFF_Y
#define MAG_OFF_Z

#define MAG_SCALE_X
#define MAG_SCALE_Y
#define MAG_SCALE_Z






/* Gyro structure containing necessary info */
struct imu{
	
	MPU6050 gyro;

	// mpu6050 raw data
	int16_t ax, ay, az;
	int16_t gx, gy, gz;


	// accelerometer angles
	double x_acc, y_acc, z_acc;

	// accelerometer low pass filter
	int16_t ax_buf[LP_BUFFER_SIZE], ay_buf[LP_BUFFER_SIZE], az_buf[LP_BUFFER_SIZE];
	int lp_pos;
	int32_t ax_sum, ay_sum, az_sum;


	// gyro angular rates
	double x_gyr, y_gyr, z_gyr;


	// HMC5883L raw data
	int16_t mx, my, mz;

	// magnetometer offsets
	int off_mx, off_my, off_mz;
	double off_m[3];
	double m[3], last_m[3];



	//magnetometer scalers
	double scale_mx, scale_my, scale_mz;

	//magnetometer angles
	double x_mag, y_mag, z_mag;

	//tilt help parameters
	double xh, yh;


	// angles
	double ypr[3];
	double ypr_rad[3];

	//constants
	double p1, p2;
	double r1, r2;
	double y1, y2;

};
typedef struct imu imu;

/* Initializes the gyro and sets parameters */
bool init_imu(imu* g);

/* Reads raw data from gyro and perform complementary filtering */
void read_imu(imu* g);



#endif