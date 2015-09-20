#ifndef IMU_H
#define IMU_H


#include "I2Cdev.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <math.h>


#define RAD_TO_DEG 180.0/M_PI
#define DEG_TO_RAD M_PI/180.0

#define LP_BUFFER_SIZE 20

#define YAW 0
#define PITCH 1
#define ROLL 2


#define MAG_ADDR 0x1E
#define MAG_OFF_GAIN 20.0

#define MPU6050_ADDR 0x68
#define MPU6050_DATAREG 0x3B



// gyro offsets 
#define GYRO_OFF_X -20
#define GYRO_OFF_Y -25
#define GYRO_OFF_Z -16

// gyro scalers
#define GYRO_SCALE_X M_PI/23870.0
#define GYRO_SCALE_Y M_PI/23870.0
#define GYRO_SCALE_Z M_PI/23870.0

// accelerometer offsets
#define ACC_OFF_X 525
#define ACC_OFF_Y -128
#define ACC_OFF_Z -200

// accelerometer scalers
#define ACC_SCALE_X 1.0/16384.0 //15800.0
#define ACC_SCALE_Y 1.0/16384.0 //16600.0
#define ACC_SCALE_Z 1.0/16384.0 //15300.0

// magnetometer offsets
#define MAG_OFF_X 96
#define MAG_OFF_Y -967
#define MAG_OFF_Z -42

#define M11 1.623
#define M21 0.024
#define M31 0.059
#define M12 -0.009
#define M22 1.66
#define M32 0.065
#define M13 0.0
#define M23 0.123
#define M33 1.87

#define MAG_SCALE_X 1.0/505.0
#define MAG_SCALE_Y 1.0/513.0
#define MAG_SCALE_Z 1.0/447.0


// complementary parameters
#define GYRO_GAIN_PITCH 1.2
#define GYRO_GAIN_ROLL 1.3
#define P1 0.99
#define P2 0.99
#define P3 0.99




/* Gyro structure containing necessary info */
struct imu{
	
	uint8_t I2C_buffer[14];

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
	double off_m[3];
	double m[3], last_m[3];


	//magnetometer angles
	double x_mag, y_mag, z_mag;

	//tilt help parameters
	double xh, yh;

	//barometer data
	double height;
	double vertical_speed;
	double vertical_acc;
	


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
void imu_init(imu* g);

/* Reads raw data from gyro and perform complementary filtering */
void imu_update(imu* g, uint32_t tim);



#endif