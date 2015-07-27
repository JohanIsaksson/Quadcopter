#ifndef IMU_H
#define IMU_H


#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define DEG_PITCH 70.0
#define DEG_ROLL 70.0

#define LP_BUFFER_SIZE 10

#define YAW 0
#define PITCH 1
#define ROLL 2

#define MAG_ADDR 0x1E
#define MAG_OFF_GAIN 20.0

/* Gyro structure containing necessary info */
struct imu{
	
	MPU6050 gyro;

	// mpu6050 raw data
	int16_t ax, ay, az;
	int16_t gx, gy, gz;

	// accelerometer scalers
	double scale_ax, scale_ay, scale_az;

	// accelerometer offsets
	int off_ax, off_ay, off_az;

	// accelerometer angles
	double x_acc, y_acc, z_acc;

	// accelerometer low pass filter
	int16_t ax_buf[LP_BUFFER_SIZE], ay_buf[LP_BUFFER_SIZE], az_buf[LP_BUFFER_SIZE];
	int lp_pos;
	int32_t ax_sum, ay_sum, az_sum;

	// gyro scalers
	double scale_gx, scale_gy, scale_gz; //not tested

	// gyro offsets 
	int off_gx, off_gy, off_gz;

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