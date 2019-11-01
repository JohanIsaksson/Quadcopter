#ifndef IMU_H
#define IMU_H

#include <math.h>
#include "kalman.h"
#include <stdint.h>
//#include <SparkFunMPU9250-DMP.h>

#define MAG_ADDR 0x1E

#define MPU6050_ADDR 0x68
#define MPU6050_DATAREG 0x3B

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

#define MS5611_ADDR 0x77
#define MS5611_COMMAND_TEMPERATURE 0x58
#define MS5611_COMMAND_PRESSURE 0x48
#define MS5611_ADC_READ 0x00
#define MS5611_PROM_READ 0x48

class IMU
{

private:
  void I2C_writeWord(uint8_t devAddress, uint16_t regAddress, int16_t data);

public:
  uint8_t I2C_buffer[22];

  //mpu6050 scaled data
  float axs, ays, azs;
 
  // accelerometer low pass filter
  int16_t ax_buf[LP_BUFFER_SIZE], ay_buf[LP_BUFFER_SIZE], az_buf[LP_BUFFER_SIZE];
  int lp_pos;
  int32_t ax_sum, ay_sum, az_sum;

  // Barometer variables
  uint8_t baro_state;
  uint8_t baro_temp_count;

  uint32_t temp_buf[LP_TEMP_BUFFER_SIZE];
  int temp_pos;
  uint32_t temp_sum;

  uint32_t pressure_buf[LP_PRESSURE_BUFFER_SIZE];
  int pressure_pos;
  uint32_t pressure_sum;

  // MS5611 variables
  uint16_t C[7];
  uint16_t C1,C2,C3,C4,C5,C6; 
  int64_t OFF, OFF_C2, C5_8, SENS, SENS_C1, P_s;
  int64_t dT, dT_C5, TEMP;
  uint32_t raw_temp, lp_temp;
  uint32_t raw_pressure, lp_pressure;


  // Altitude kalman filter
  Kalman kalman;


  void ComplementaryFilter(float time);
  void CalculateGyro();


  void MS5611_temp_start();
  void MS5611_temp_read();
  void MS5611_pressure_start();
  void MS5611_pressure_read();
  void MS5611_init();
  void MS5611_update();

  void CalculateAltitude(float dt);

  void MPU6050_init();
  void MPU6050_update();


  // mpu6050 raw data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  /* Angles */
  double ypr[3];
  double ypr_rad[3];

  // accelerometer angles
  float x_acc, y_acc, z_acc;

  /* Angular rates */
  int16_t x_gyr_u, y_gyr_u, z_gyr_u;
  float x_gyr, y_gyr, z_gyr;

  /* Altitude estimation */
  float temp;  
  float baro_altitude;
  float pressure, base_pressure;
  float altitude, altitude_last, vertical_speed, vertical_acc;

  /* Initializes the gyro and sets parameters */
  void Setup();

  /* Update Everything: Gyroscope, Accelerometer, Barometer, (GPS) */
  void Update(float dt);

  /* Update gyroscope and accelerometer data */
  void UpdateHorizon(float dt);

  /* Update gyroscope data only */
  void UpdateAcro(float dt);


};

#endif
