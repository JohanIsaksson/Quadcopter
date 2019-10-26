
#include "SpeedTrig.h"
#include "imu.h"
#include "Wire.h"
#include "I2Cdev.h"


void IMU::ComplementaryFilter(float tim){

  // Low pass filter accelerometer data through moving average */
  ax_sum -= ax_buf[lp_pos];
  ax_buf[lp_pos] = ax;
  ax_sum += ax;
  ax = ax_sum >> LP_SHIFT;

  ay_sum -= ay_buf[lp_pos];
  ay_buf[lp_pos] = ay;
  ay_sum += ay;
  ay = ay_sum >> LP_SHIFT;

  az_sum -= az_buf[lp_pos];
  az_buf[lp_pos] = az;
  az_sum += az;
  az = az_sum >> LP_SHIFT;

  if (lp_pos < LP_BUFFER_SIZE-1){
    lp_pos++;
  }else{
    lp_pos = 0;
  }  

  // Calculate accelerometer angles
  axs = (float)ax * ACC_SCALE;
  ays = (float)ay * ACC_SCALE;
  azs = (float)az * ACC_SCALE;

  //x_acc = SpeedTrig.atan2(axs,sqrt(ACC_SCALE_X *((double)(ay*ay + az*az))));
  //y_acc = SpeedTrig.atan2(ays,sqrt(ACC_SCALE_X *((double)(ax*ax + az*az))));
  x_acc = atan(axs/sqrt(ACC_SCALE_2 *((float)(ay*ay + az*az))));
  y_acc = atan(ays/sqrt(ACC_SCALE_2 *((float)(ax*ax + az*az))));

  // Scale and filter angular velocity
  CalculateGyro();

  // Calculate pitch and roll
  ypr_rad[PITCH] = -(P*(-ypr_rad[PITCH] - y_gyr*tim) + P_*x_acc);
  ypr_rad[ROLL] = (P*(ypr_rad[ROLL] + x_gyr*tim) + P_*y_acc);

  ypr[PITCH] = ypr_rad[PITCH] * RAD_TO_DEG;
  ypr[ROLL] = ypr_rad[ROLL] * RAD_TO_DEG;
}

void IMU::CalculateGyro(){
  // scale and filter angular velocity
  y_gyr_u = (y_gyr_u >> 2) + (y_gyr_u >> 1) + (gy >> 2);
  y_gyr = ((float)y_gyr_u)*GYRO_SCALE_Y;

  x_gyr_u = (x_gyr_u >> 2) + (x_gyr_u >> 1) + (gx >> 2);
  x_gyr = ((float)x_gyr_u)*GYRO_SCALE_X;

  z_gyr_u = (z_gyr_u >> 2) + (z_gyr_u >> 1) + (gz >> 2);
  z_gyr = ((float)z_gyr_u)*GYRO_SCALE_Z;
}

/* Initializes the imu and sets parameters */
void IMU::Init(){
  // join I2C bus
  Wire.end();
  Wire.begin();
  Wire.setClock(400000);

  SerialUSB.println("Initializing IMU");

  MPU6050_init();
  MS5611_init();
}

/* ------------------------------------------------------------------------- */

void IMU::CalculateAltitude(float dt){

  float cosr = cos(ypr_rad[ROLL]);
  float sinr = sin(ypr_rad[ROLL]);
  float sinp = sin(ypr_rad[PITCH]);
  float cosp = cos(ypr_rad[PITCH]);

  // Calculate vertical acceleration (needs more testing)
  vertical_acc = 0.8*vertical_acc + 0.2*(-axs*sinp*cosr + ays*cosp*sinr + azs*cosp*cosr -1.0025);

  // Run kalman 
  kalman.Update(baro_altitude, vertical_acc-1.0025, dt);
  altitude  = kalman.GetAltitude();
  //vertical_speed = kalman.GetVerticalSpeed();
  //vertical_acc = kalman.GetVerticalAcceleration();
  /*altitude = 0.98*altitude + 0.02*baro_altitude;
  if (abs(baro_altitude - altitude) > 0.2){
      altitude += 0.1*(baro_altitude - altitude);      
  }
  

  vertical_speed = 0.98*vertical_speed + 0.02*(0.8*(altitude - altitude_last)/dt + 0.2*vertical_acc*9.82*dt); //

  altitude_last = altitude;*/
}

/* ------------------------------------------------------------------------- */

void IMU::MS5611_temp_start(){
  SerialUSB.println("MS5611_temp_start");
  //I2Cdev::writeBytes(MS5611_ADDR, MS5611_COMMAND_TEMPERATURE, 0, NULL);
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(MS5611_COMMAND_TEMPERATURE);
  Wire.endTransmission();
}

void IMU::MS5611_temp_read(){
  SerialUSB.println("MS5611_temp_read");
  //I2Cdev::readBytes(MS5611_ADDR, 0, 3, I2C_buffer);
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write((uint8_t)0x00);
  Wire.endTransmission();
  Wire.requestFrom(MS5611_ADDR, 3);

  raw_temp = Wire.read() << 16 | Wire.read() << 8 | Wire.read();

  /* low pass filter through moving average */
  temp_sum -= temp_buf[temp_pos];
  temp_buf[temp_pos] = raw_temp;
  temp_sum += raw_temp;
  lp_temp = temp_sum >> LP_TEMP_SHIFT;
  temp_pos = (temp_pos + 1) % LP_TEMP_BUFFER_SIZE;
}

void IMU::MS5611_pressure_start(){
  SerialUSB.println("MS5611_pressure_start");
  //I2Cdev::writeBytes(MS5611_ADDR, MS5611_COMMAND_PRESSURE, 0, NULL);
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(MS5611_COMMAND_PRESSURE);
  Wire.endTransmission();
}

void IMU::MS5611_pressure_read(){
  SerialUSB.println("MS5611_pressure_read");
  //I2Cdev::readBytes(MS5611_ADDR, 0, 3, I2C_buffer);
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write((uint8_t)0x00);
  Wire.endTransmission();
  Wire.requestFrom(MS5611_ADDR, 3);
  
  raw_pressure = Wire.read() << 16 | Wire.read() << 8 | Wire.read();

  /* low pass filter through moving average */
  pressure_sum -= pressure_buf[pressure_pos];
  pressure_buf[pressure_pos] = raw_pressure;
  pressure_sum += raw_pressure;
  lp_pressure = pressure_sum >> LP_PRESSURE_SHIFT;
  pressure_pos = (pressure_pos + 1) % LP_PRESSURE_BUFFER_SIZE;

  // Calculate pressure as explained in the datasheet of the MS-5611.
  dT = (int64_t)lp_temp - C5_8;                                         // D2 - C5 * 2^8
  OFF = OFF_C2 + (((int64_t)dT * (int64_t)C4) / 128);           // OFF = C2 * 2^16 + (C4 * dT) / 2^7
  SENS = SENS_C1 + (((int64_t)dT * (int64_t)C3) / 256);         // SENS = C1 * 2^15 + (C3 * dT) / 2^8
  P_s = ((((int64_t)lp_pressure * SENS) / 2097152) - OFF) / 32768;    // P = (D1 * SENS / 2^21 - OFF) / 2^15
  pressure = ((float)P_s)*0.01;

  // Second order temperature compensation
  /*if (temp < 20.0){
    T2 = dT*dT

    
  }*/  

  // Calculate altitude from air pressure
  baro_altitude = 44330.0 * (1.0 - pow(pressure / base_pressure, 0.190294957)); //1/5.255));
  TEMP = 2000 + (dT * (int64_t)C6) / 8388608;
  temp = (float)TEMP * 0.01;
  //baro_altitude = 153.8*(273.15 + (double)temp_calc*0.01)*(pow(base_pressure/pressure,1/5.257));
}

void IMU::MS5611_init(){
  SerialUSB.println("MS5611_init");
  // Retrieve calibration data from device:
  for (uint8_t start = 1; start <= 6; start++) {
    SerialUSB.println("------ 1");
    Wire.beginTransmission(MS5611_ADDR);
    SerialUSB.println("------ 2");
    Wire.write(0xA0 + start * 2);
    SerialUSB.println("------ 3");
    Wire.endTransmission();

    SerialUSB.println("before request");
    Wire.requestFrom(MS5611_ADDR, 2);
    SerialUSB.println("after request");
    C[start] = Wire.read() << 8 | Wire.read();
  }
  
  //I2Cdev::readBytes(MS5611_ADDR, 0xA0, 12, I2C_buffer);
  C1 = C[1];
  C2 = C[2];
  C3 = C[3];
  C4 = C[4];
  C5 = C[5];
  C6 = C[6];

  OFF_C2 = (int64_t)C2 * 65536;
  SENS_C1 = (int64_t)C1 * 32768;
  C5_8 = (int64_t)C5 * 256;

  // Reset LP filters
  temp_sum = 0;
  temp_pos = 0;
  pressure_sum = 0;
  pressure_pos = 0;

  for (int i = 0; i < LP_TEMP_BUFFER_SIZE; i++){
    temp_buf[i] = 0;
  }
  for (int i = 0; i < LP_PRESSURE_BUFFER_SIZE; i++){
    pressure_buf[i] = 0;
  }

  // Get initial values
  for (int i = 0; i < 8; i++){
    MS5611_temp_start();
    delay(10);
    MS5611_temp_read();
    MS5611_pressure_start();
    delay(30);
    MS5611_pressure_read();
  }
  base_pressure = pressure;
  altitude = 0.0;
  baro_altitude = 0.0;
  baro_state = 0;

  // Init kalman filter
  kalman.InitPreset();
}

void IMU::MS5611_update(){

  // Alternates between reading temp and pressure
  switch(baro_state){
    case 0:
      baro_temp_count = 0;
      MS5611_temp_start();
      baro_state++;
      break;

    case 1:
      baro_state++;
      break;

    case 2:
      // Read temp if meassured
      if (baro_temp_count == 0){
        MS5611_temp_read();
      }

      MS5611_pressure_start();      
      baro_state++;
      break;

    case 3:
      baro_state++;
      break;

    case 4:
      MS5611_pressure_read();
      
      if (baro_temp_count >= 0){
        baro_state = 1;
        baro_temp_count = 0;
        MS5611_temp_start();
      }else{
        baro_state = 2;
        baro_temp_count++;
      }
      baro_state = 1;
      break;

    default:
      baro_state = 0;
      break;
  }
}

/* ------------------------------------------------------------------------- */

void IMU::MPU6050_init(){
  I2Cdev::writeBits(MPU6050_ADDR, 0x6B, 2, 3, 0x01); //set internal clock to XGYRO - should be best
  I2Cdev::writeBits(MPU6050_ADDR, 0x1B, 4, 2, 0x00); //set full scale gyro range +- 250 deg/s
  I2Cdev::writeBits(MPU6050_ADDR, 0x1C, 4, 2, 0x00); //set full scale accelerometer range +- 2g
  I2Cdev::writeBit(MPU6050_ADDR, 0x6B, 6, false); //set sleep to false

  //set offsets (on chip 1)
  I2Cdev::writeWord(MPU6050_ADDR, 0x06, -772); //x acc
  I2Cdev::writeWord(MPU6050_ADDR, 0x08, -622); //y acc
  I2Cdev::writeWord(MPU6050_ADDR, 0x0A, 1000); //z acc

  I2Cdev::writeWord(MPU6050_ADDR, 0x13, 77);// 98); //x gyro
  I2Cdev::writeWord(MPU6050_ADDR, 0x15, 28);// 32); //y gyro
  I2Cdev::writeWord(MPU6050_ADDR, 0x17, -24);// 18); //z gyro

  // Reset low pass filters
  for (int i = 0; i < LP_BUFFER_SIZE; i++){
    ax_buf[i] = 0;
    ay_buf[i] = 0;
    az_buf[i] = 0;
  }
  lp_pos = 0;

  // Get initial values
  for (int i = 0; i < LP_BUFFER_SIZE; i++){
    MPU6050_update();
    ComplementaryFilter(0.002);
    delay(2);
  }
}

void IMU::MPU6050_update(){
  I2Cdev::readBytes(MPU6050_ADDR, MPU6050_DATAREG, 14, I2C_buffer);
  ax = (((int16_t)I2C_buffer[0]) << 8) | I2C_buffer[1];
  ay = (((int16_t)I2C_buffer[2]) << 8) | I2C_buffer[3];
  az = (((int16_t)I2C_buffer[4]) << 8) | I2C_buffer[5];
  gx = (((int16_t)I2C_buffer[8]) << 8) | I2C_buffer[9];
  gy = (((int16_t)I2C_buffer[10]) << 8) | I2C_buffer[11];
  gz = (((int16_t)I2C_buffer[12]) << 8) | I2C_buffer[13];
}

/* ------------------------------------------------------------------------- */

void IMU::Update(float dt){
	// Update attitude estimates
  MPU6050_update();
  ComplementaryFilter(dt);

  // Update altitude estimate
  MS5611_update();
  CalculateAltitude(dt);

  // Update heading estimate


  // Update position estimate
}

void IMU::UpdateHorizon(float dt){
  // Update attitude estimates
  MPU6050_update();
  ComplementaryFilter(dt);
}

void IMU::UpdateAcro(float dt){
  // Scale and filter angular velocity
  MPU6050_update();
  CalculateGyro();
}
