
#include "SpeedTrig.h"
#include "imu.h"


void IMU::ComplementaryFilter(double tim){

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
  axs = (double)ax * ACC_SCALE;
  ays = (double)ay * ACC_SCALE;
  azs = (double)az * ACC_SCALE;

  //x_acc = SpeedTrig.atan2(axs,sqrt(ACC_SCALE_X *((double)(ay*ay + az*az))));
  //y_acc = SpeedTrig.atan2(ays,sqrt(ACC_SCALE_X *((double)(ax*ax + az*az))));
  x_acc = atan(axs/sqrt(ACC_SCALE_2 *((double)(ay*ay + az*az))));
  y_acc = atan(ays/sqrt(ACC_SCALE_2 *((double)(ax*ax + az*az))));

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
  y_gyr = ((double)y_gyr_u)*GYRO_SCALE_Y;

  x_gyr_u = (x_gyr_u >> 2) + (x_gyr_u >> 1) + (gx >> 2);
  x_gyr = ((double)x_gyr_u)*GYRO_SCALE_X;

  z_gyr_u = (z_gyr_u >> 2) + (z_gyr_u >> 1) + (gz >> 2);
  z_gyr = ((double)z_gyr_u)*GYRO_SCALE_Z;
}

/* Initializes the imu and sets parameters */
void IMU::Init(){
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  SerialUSB.println("Initializing IMU");

  //BMP180_init();
  MS5611_init();
  MPU6050_init();
}

/* ------------------------------------------------------------------------- */

void IMU::BMP180_temp_start(){
  I2Cdev::writeByte(BMP180_ADDR, BMP180_REG_CONTROL, BMP180_COMMAND_TEMPERATURE);
}

void IMU::BMP180_temp_read(){
  double tu, a;
  I2Cdev::readBytes(BMP180_ADDR, BMP180_REG_RESULT, 2, I2C_buffer);

  int16_t tu_ = I2C_buffer[0];
  tu_ = tu_ << 8;
  tu_ += I2C_buffer[1];

  /* low pass filter through moving average */
  temp_sum -= temp_buf[temp_pos];
  temp_buf[temp_pos] = tu_;
  temp_sum += tu_;
  tu_ = temp_sum >> LP_TEMP_SHIFT;

  temp_pos = (temp_pos + 1) % LP_TEMP_BUFFER_SIZE;

  tu = (double)tu_;

  //example from Bosch datasheet
  //tu = 27898;

  //example from http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf
  //tu = 0x69EC;
  
  a = c5 * (tu - c6);
  temp = a + (mc / (a + md));
}

void IMU::BMP180_pressure_start(){
  I2Cdev::writeByte(BMP180_ADDR, BMP180_REG_CONTROL, BMP180_COMMAND_PRESSURE0);
}

void IMU::BMP180_pressure_read(){
  double pu,s,x,y,z;

  I2Cdev::readBytes(BMP180_ADDR, BMP180_REG_RESULT, 3, I2C_buffer);

  int32_t pu_1 = I2C_buffer[0];
  pu_1 = pu_1 << 16;
  int32_t pu_2 = I2C_buffer[1];
  pu_2 = pu_2 << 8;
  int32_t pu_3 = (pu_1 + pu_2 + I2C_buffer[2]);

  /* low pass filter through moving average */
  pressure_sum -= pressure_buf[pressure_pos];
  pressure_buf[pressure_pos] = pu_3;
  pressure_sum += pu_3;
  pu_3 = pressure_sum >> LP_PRESSURE_SHIFT;

  pressure_pos = (pressure_pos + 1) % LP_PRESSURE_BUFFER_SIZE;

  pu = (double)pu_3 / 256.0;

  //pu = (I2C_buffer[0] * 256.0) + I2C_buffer[1] + (I2C_buffer[2]/256.0);

  //example from Bosch datasheet
  //pu = 23843;

  //example from http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf, pu = 0x982FC0; 
  //pu = (0x98 * 256.0) + 0x2F + (0xC0/256.0);
  
  s = temp - 25.0;
  x = (x2 * pow(s,2)) + (x1 * s) + x0;
  y = (y2 * pow(s,2)) + (y1 * s) + y0;
  z = (pu - x) / y;
  pressure = (p2 * pow(z,2)) + (p1 * z) + p0;  

  // Calculate altitude from air pressure
  baro_altitude = 44330.0*(1-pow(pressure/base_pressure,1/5.255));
}

void IMU::BMP180_init(){

  // Retrieve calibration data from device:
  I2Cdev::readBytes(BMP180_ADDR, 0xAA, 22, I2C_buffer);
  AC1 = (((int16_t)I2C_buffer[0]) << 8) | I2C_buffer[1];
  AC2 = (((int16_t)I2C_buffer[2]) << 8) | I2C_buffer[3];
  AC3 = (((int16_t)I2C_buffer[4]) << 8) | I2C_buffer[5];
  AC4 = (((uint16_t)I2C_buffer[6]) << 8) | I2C_buffer[7];
  AC5 = (((uint16_t)I2C_buffer[8]) << 8) | I2C_buffer[9];
  AC6 = (((uint16_t)I2C_buffer[10]) << 8) | I2C_buffer[11];
  VB1 = (((int16_t)I2C_buffer[12]) << 8) | I2C_buffer[13];
  VB2 = (((int16_t)I2C_buffer[14]) << 8) | I2C_buffer[15];
  MB = (((int16_t)I2C_buffer[16]) << 8) | I2C_buffer[17];
  MC = (((int16_t)I2C_buffer[18]) << 8) | I2C_buffer[19];
  MD = (((int16_t)I2C_buffer[20]) << 8) | I2C_buffer[21];


  double c3,c4,b1;

  c3 = 160.0 * pow(2,-15) * AC3;
  c4 = pow(10,-3) * pow(2,-15) * AC4;
  b1 = pow(160,2) * pow(2,-30) * VB1;
  c5 = (pow(2,-15) / 160) * AC5;
  c6 = AC6;
  mc = (pow(2,11) / pow(160,2)) * MC;
  md = MD / 160.0;
  x0 = AC1;
  x1 = 160.0 * pow(2,-13) * AC2;
  x2 = pow(160,2) * pow(2,-25) * VB2;
  y0 = c4 * pow(2,15);
  y1 = c4 * c3;
  y2 = c4 * b1;
  p0 = (3791.0 - 8.0) / 1600.0;
  p1 = 1.0 - 7357.0 * pow(2,-20);
  p2 = 3038.0 * 100.0 * pow(2,-36);

  // Reset filters
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
    BMP180_temp_start();
    delay(10);
    BMP180_temp_read();
    BMP180_pressure_start();
    delay(30);
    BMP180_pressure_read();
  }

  base_pressure = pressure;
  altitude = 0.0;
  baro_altitude = 0.0;
  baro_state = 0;

  // Init kalman filter
  kalman.InitPreset();
}

void IMU::BMP180_update(){

  // Alternates between reading temp and pressure
  switch(baro_state){
    case 0:
      baro_temp_count = 0;
      BMP180_temp_start();
      baro_state++;
      break;

    case 1:
      baro_state++;
      break;

    case 2:
      // Read temp if meassured
      if (baro_temp_count == 0){
        BMP180_temp_read();
      }

      BMP180_pressure_start();      
      baro_state++;
      break;

    case 3:
      baro_state++;
      break;

    case 4:
      BMP180_pressure_read();
      
      if (baro_temp_count >= 50){
        baro_state = 1;
        baro_temp_count = 0;
        BMP180_temp_start();
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

void IMU::CalculateAltitude(double dt){

  double cosr = cos(ypr_rad[ROLL]);
  double sinr = sin(ypr_rad[ROLL]);
  double sinp = sin(ypr_rad[PITCH]);
  double cosp = cos(ypr_rad[PITCH]);

  // Calculate vertical acceleration (needs more testing)
  vertical_acc = 0.8*vertical_acc + 0.2*(-axs*sinp*cosr + ays*cosp*sinr + azs*cosp*cosr -1.0025);

  // Run kalman 
  kalman.Update(baro_altitude, vertical_acc-1.0025, dt);
  altitude = kalman.GetAltitude();
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
  //I2Cdev::writeBytes(MS5611_ADDR, MS5611_COMMAND_TEMPERATURE, 0, NULL);
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(MS5611_COMMAND_TEMPERATURE);
  Wire.endTransmission();
}

void IMU::MS5611_temp_read(){
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
  //I2Cdev::writeBytes(MS5611_ADDR, MS5611_COMMAND_PRESSURE, 0, NULL);
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(MS5611_COMMAND_PRESSURE);
  Wire.endTransmission();
}

void IMU::MS5611_pressure_read(){
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
  pressure = ((double)P_s)*0.01;

  // Second order temperature compensation
  /*if (temp < 20.0){
    T2 = dT*dT

    
  }*/  

  // Calculate altitude from air pressure
  baro_altitude = 44330.0 * (1.0 - pow(pressure / base_pressure, 0.190294957)); //1/5.255));
  TEMP = 2000 + (dT * (int64_t)C6) / 8388608;
  temp = (double)TEMP * 0.01;
  //baro_altitude = 153.8*(273.15 + (double)temp_calc*0.01)*(pow(base_pressure/pressure,1/5.257));
}

void IMU::MS5611_init(){

  // Retrieve calibration data from device:
  for (uint8_t start = 1; start <= 6; start++) {
    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(0xA0 + start * 2);
    Wire.endTransmission();

    Wire.requestFrom(MS5611_ADDR, 2);
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

void IMU::MPU9250_init(){

  SerialUSB.println("INIT MPU9250");

  //I2Cdev::writeByte(MPU9250_ADDR, 0x23, 0x00); // Disabled fifo
  

  I2Cdev::writeBit(MPU9250_ADDR, 0x1D, 3, 1); //
  I2Cdev::writeBit(MPU9250_ADDR, 0x37, 7, 1); //

  I2Cdev::writeBit(MPU9250_ADDR, 0x67, 1, 1); //
  I2Cdev::writeBit(MPU9250_ADDR, 0x67, 0, 1); //

  //I2Cdev::writeByte(MPU9250_ADDR, 0x19, 0); //

  //I2Cdev::writeBits(MPU9250_ADDR, 0x6A, 6, 2, 0x00); // disable fifo and i2c master
  //I2Cdev::writeBits(MPU9250_ADDR, 0x1A, 2, 3, 0x00); // disable low pass filter
  I2Cdev::writeBits(MPU9250_ADDR, 0x6B, 2, 3, 0x01); //set to internal clock  
  I2Cdev::writeBits(MPU9250_ADDR, 0x1B, 4, 2, 0x00); //set full scale gyro range +- 250 deg/s
  I2Cdev::writeBits(MPU9250_ADDR, 0x1C, 4, 2, 0x00); //set full scale accelerometer range +- 2g
  I2Cdev::writeBit(MPU9250_ADDR, 0x6B, 6, false); //set sleep to false

  // Set offsets  
  I2Cdev::writeWord(MPU9250_ADDR, 0x77, 0); //x acc
  I2Cdev::writeWord(MPU9250_ADDR, 0x7A, 0); //y acc
  I2Cdev::writeWord(MPU9250_ADDR, 0x7D, 0); //z acc

  I2Cdev::writeWord(MPU9250_ADDR, 0x13, -36); //x gyro
  I2Cdev::writeWord(MPU9250_ADDR, 0x15, -169); //y gyro
  I2Cdev::writeWord(MPU9250_ADDR, 0x17, -37); //z gyro

  //I2Cdev::writeWord(MPU9250_ADDR, 0x13, 159); //x mag
  //I2Cdev::writeWord(MPU9250_ADDR, 0x15, -9); //y mag
  //I2Cdev::writeWord(MPU9250_ADDR, 0x17, 104); //z mag
}

void IMU::MPU9250_update(){
  I2Cdev::readBytes(MPU9250_ADDR, 0x3B, 14, I2C_buffer);
  
  ax = (((int16_t)I2C_buffer[0]) << 8) | I2C_buffer[1];
  ay = (((int16_t)I2C_buffer[2]) << 8) | I2C_buffer[3];
  az = (((int16_t)I2C_buffer[4]) << 8) | I2C_buffer[5];
  gx = (((int16_t)I2C_buffer[8]) << 8) | I2C_buffer[9];
  gy = (((int16_t)I2C_buffer[10]) << 8) | I2C_buffer[11];
  gz = (((int16_t)I2C_buffer[12]) << 8) | I2C_buffer[13];
}

/* ------------------------------------------------------------------------- */

void IMU::Update(double dt){
	// Update attitude estimates
  MPU6050_update();
  ComplementaryFilter(dt);

  // Update altitude estimate
  MS5611_update();
  CalculateAltitude(dt);

  // Update heading estimate


  // Update position estimate

}

void IMU::UpdateHorizon(double dt){
  // Update attitude estimates
  MPU6050_update();
  ComplementaryFilter(dt);
}

void IMU::UpdateAcro(double dt){
  // Scale and filter angular velocity
  MPU6050_update();
  CalculateGyro();
}

