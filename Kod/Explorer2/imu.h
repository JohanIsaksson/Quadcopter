#ifndef IMU_H
#define IMU_H


#include "I2Cdev.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <math.h>
#include "kalman.h"
//#include <SparkFunMPU9250-DMP.h>

#define MAG_ADDR 0x1E

#define MPU6050_ADDR 0x68
#define MPU6050_DATAREG 0x3B

#define MPU9250_ADDR 0x68

#define RAD_TO_DEG 180.0/M_PI
#define DEG_TO_RAD M_PI/180.0

#define LP_BUFFER_SIZE 16
#define LP_SHIFT 4

#define LP_TEMP_BUFFER_SIZE 4
#define LP_TEMP_SHIFT 2

#define LP_PRESSURE_BUFFER_SIZE 4
#define LP_PRESSURE_SHIFT 2

#define YAW 0
#define PITCH 1
#define ROLL 2

// gyro scalers
#define GYRO_SCALE_X M_PI/23870.0
#define GYRO_SCALE_Y M_PI/23870.0
#define GYRO_SCALE_Z M_PI/23870.0

// accelerometer scalers
#define ACC_SCALE 1.0/16384.0
#define ACC_SCALE_2 (ACC_SCALE*ACC_SCALE)

// complementary parameters
#define GYRO_GAIN_PITCH 1.2
#define GYRO_GAIN_ROLL 1.3

#define P 0.99
#define P_ (1.0 - P)

#define BMP180_ADDR 0x77 // 7-bit address
#define	BMP180_REG_CONTROL 0xF4
#define	BMP180_REG_RESULT 0xF6
#define	BMP180_COMMAND_TEMPERATURE 0x2E
#define	BMP180_COMMAND_PRESSURE0 0x34
#define	BMP180_COMMAND_PRESSURE1 0x74
#define	BMP180_COMMAND_PRESSURE2 0xB4
#define	BMP180_COMMAND_PRESSURE3 0xF4

#define MS5611_ADDR 0x77
#define	MS5611_COMMAND_TEMPERATURE 0x58
#define	MS5611_COMMAND_PRESSURE 0x48
#define	MS5611_ADC_READ 0x00
#define	MS5611_PROM_READ 0x48

class IMU{

private:

public:
	uint8_t I2C_buffer[22];

	//mpu6050 scaled data
	double axs, ays, azs;
 
	// accelerometer low pass filter
	int16_t ax_buf[LP_BUFFER_SIZE], ay_buf[LP_BUFFER_SIZE], az_buf[LP_BUFFER_SIZE];
	int lp_pos;
	int32_t ax_sum, ay_sum, az_sum;

	//MPU9250_DMP imu;

	// Barometer variables
	uint8_t baro_state;
	uint8_t baro_temp_count;

	uint32_t temp_buf[LP_TEMP_BUFFER_SIZE];
	int temp_pos;
	uint32_t temp_sum;

	uint32_t pressure_buf[LP_PRESSURE_BUFFER_SIZE];
	int pressure_pos;
	uint32_t pressure_sum;

	// BMP180 variables
	int16_t AC1,AC2,AC3,VB1,VB2,MB,MC,MD;
	uint16_t AC4,AC5,AC6; 
	double c5,c6,mc,md,x0,x1,x2,y0,y1,y2,p0,p1,p2;

	// MS5611 variables
	uint16_t C[7];
	uint16_t C1,C2,C3,C4,C5,C6; 
	int64_t OFF, OFF_C2, C5_8, SENS, SENS_C1, P_s;
	int64_t dT, dT_C5, TEMP;
	uint32_t raw_temp, lp_temp;
	uint32_t raw_pressure, lp_pressure;


	// Altitude kalman filter
	Kalman kalman;


	void ComplementaryFilter(double time);
	void CalculateGyro();

	void BMP180_temp_start();
	void BMP180_temp_read();
	void BMP180_pressure_start();
	void BMP180_pressure_read();
	void BMP180_init();
	void BMP180_update();

	void MS5611_temp_start();
	void MS5611_temp_read();
	void MS5611_pressure_start();
	void MS5611_pressure_read();
	void MS5611_init();
	void MS5611_update();

	void CalculateAltitude(double dt);

  void MPU6050_init();
  void MPU6050_update();

  void MPU9250_init();
  void MPU9250_update();




  // mpu6050 raw data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

	/* Angles */
	double ypr[3];
	double ypr_rad[3];

  // accelerometer angles
  double x_acc, y_acc, z_acc;

	/* Angular rates */
	int16_t x_gyr_u, y_gyr_u, z_gyr_u;
	double x_gyr, y_gyr, z_gyr;

	/* Altitude estimation */
  double temp;  
  double baro_altitude;
  double pressure, base_pressure;
	double altitude, vertical_speed, vertical_acc;

	/* Initializes the gyro and sets parameters */
	void Init();

	/* Update Everything: Gyroscope, Accelerometer, Barometer, (GPS) */
	void Update(double dt);

	/* Update gyroscope and accelerometer data */
	void UpdateHorizon(double dt);

	/* Update gyroscope data only */
	void UpdateAcro(double dt);


};

#endif
