#ifndef IMU_H
#define IMU_H


#include "I2Cdev.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <math.h>
#include <SparkFunMPU9250-DMP.h>

#define RAD_TO_DEG 180.0/M_PI
#define DEG_TO_RAD M_PI/180.0

#define LP_BUFFER_SIZE 16
#define LP_SHIFT 4

#define YAW 0
#define PITCH 1
#define ROLL 2

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

// complementary parameters
#define GYRO_GAIN_PITCH 1.2
#define GYRO_GAIN_ROLL 1.3
#define P1 0.99
#define P2 0.99
#define P3 0.99


class IMU{

private:
	uint8_t I2C_buffer[14];

	// mpu6050 raw data
	int16_t ax, ay, az;
	int16_t gx, gy, gz;

	//mpu6050 scaled data
	double axs, ays, azs;

	// accelerometer angles
	double x_acc, y_acc, z_acc;

	// accelerometer low pass filter
	int16_t ax_buf[LP_BUFFER_SIZE], ay_buf[LP_BUFFER_SIZE], az_buf[LP_BUFFER_SIZE];
	int lp_pos;
	int32_t ax_sum, ay_sum, az_sum;

	MPU9250_DMP imu;



	void complementary_filter(double time);
	void calculate_gyro();

public:

	// angles
	double ypr[3];
	double ypr_rad[3];

	// gyro angular rates
	int16_t x_gyr_u, y_gyr_u, z_gyr_u;
	double x_gyr, y_gyr, z_gyr;

	/* Initializes the gyro and sets parameters */
	void init();

	/* Reads raw data from sensors and perform complementary filtering */
	void update(double tim);

};

#endif
