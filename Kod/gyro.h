#ifndef GYRO_H
#define GYRO_H


#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define DEG_PITCH 88.0
#define DEG_ROLL 90.0


struct gyro{
	
	MPU6050 gyro;

	// mpu6050 raw data
	int16_t ax, ay, az;
	int16_t gx, gy, gz;

	// accelerometer scalers
	double scale_ax;
	double scale_ay;
	double scale_az;

	// accelerometer offsets
	int off_ax;
	int off_ay;
	int off_az;

	// accelerometer angles
	double x_acc, y_acc, z_acc;

	// gyro scalers
	double scale_gx, scale_gy, scale_gz; //not tested

	// gyro offsets 
	int off_gx, off_gy, off_gz;

	// gyro angular rates
	double x_gyr, y_gyr, z_gyr;

	// angles
	double ypr[3];

	//constants
	double p1, p2;
	double r1, r2;
	double y1, y2;

};
typedef struct gyro gyro;


bool init_gyro(gyro* g);

void read_gyro(gyro* g);



#endif