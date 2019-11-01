#pragma GCC optimize("-O3")
#include "explorer.hpp"

float Explorer::map_f(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
// Initalizes ESCs by keeping throttle low for 3s. (MUST BE DONE ASAP AT POWER ON)
void Explorer::initMotors()
{
  uint32_t initializationTime = micros() + 3000000;
  for(int i = 0; i < 4; i++){
    throttle[i] = SPEED_MIN;
  }

  // Set speed
  motorControl.SetSpeedFrontLeft(throttle[0]);
  motorControl.SetSpeedFrontRight(throttle[1]);
  motorControl.SetSpeedBackLeft(throttle[2]);
  motorControl.SetSpeedFBackLeft(throttle[3]);

  while (micros() < initializationTime);
}

// Limit integer x to interval [a,b]
uint16_t Explorer::limit(uint16_t x, uint16_t a, uint16_t b)
{
  if (x < a) return a;
  else if (x > b) return b;
  return x;
}

// Sets final throttle by combining outputs from all PIDs 
void Explorer::UpdateMotorSpeeds()
{
  //front left
  throttle[0] = rad_throttle + altitude_rate_out + front + left + cw;

  //front right
  throttle[1] = rad_throttle + altitude_rate_out + front - left - cw;

  //back left
  throttle[2] = rad_throttle + altitude_rate_out - front + left - cw;

  //back right
  throttle[3] = rad_throttle + altitude_rate_out - front - left + cw;

  #ifdef AIR_MODE
  // Air mode adjustment
  int min = SPEED_MIN;
  int max = SPEED_MAX;
  for(int i=0; i<4; i++){
      if (throttle[i] < min)
        min = throttle[i];
      else if (throttle[i] > max)
        max = throttle[i];
  }
  if (((min+max)>>1) > 1500)
    air_mode_throttle = SPEED_MAX - max;
  else if (((min+max)>>1) < 1500)
    air_mode_throttle = SPEED_MIN - min;

  for(int i=0; i<4; i++){
      throttle[i] += air_mode_throttle;
  }
  #endif

  // Verify that speeds are valid
  throttle[0] = limit(throttle[0], SPEED_MIN, SPEED_MAX);
  throttle[1] = limit(throttle[1], SPEED_MIN, SPEED_MAX);
  throttle[2] = limit(throttle[2], SPEED_MIN, SPEED_MAX);
  throttle[3] = limit(throttle[3], SPEED_MIN, SPEED_MAX);

  // Set speed
  motorControl.SetSpeedFrontLeft(throttle[0]);
  motorControl.SetSpeedFrontRight(throttle[1]);
  motorControl.SetSpeedBackLeft(throttle[2]);
  motorControl.SetSpeedFBackLeft(throttle[3]);
}

// Sets minimum throttle for all motors
void Explorer::setMotorSpeedsMin()
{
  motorControl.SetSpeedFrontLeft(SPEED_MIN);
  motorControl.SetSpeedFrontRight(SPEED_MIN);
  motorControl.SetSpeedBackLeft(SPEED_MIN);
  motorControl.SetSpeedFBackLeft(SPEED_MIN);
}

void Explorer::setMotorSpeedsMax()
{
  motorControl.SetSpeedFrontLeft(SPEED_MAX);
  motorControl.SetSpeedFrontRight(SPEED_MAX);
  motorControl.SetSpeedBackLeft(SPEED_MAX);
  motorControl.SetSpeedFBackLeft(SPEED_MAX);
}

void Explorer::Print()
{

  if (print_cnt >= 10){

    SerialUSB.print((motors_on ? "On " : "Off"));
    SerialUSB.print(", ");
  
    if (flight_mode == MODE_ACRO)
      SerialUSB.print("ACRO         ");
    else if (flight_mode == MODE_HORIZON)
      SerialUSB.print("HORIZON      ");
    else if (flight_mode == MODE_ALT_HOLD)
      SerialUSB.print("ALTITUDE HOLD");
    SerialUSB.print(", ");

    SerialUSB.print(rx.GetChannelData(0));
    SerialUSB.print(", ");
    SerialUSB.print(rx.GetChannelData(1));
    SerialUSB.print(", ");
    SerialUSB.print(rx.GetChannelData(2));
    SerialUSB.print(", ");
    SerialUSB.print(rx.GetChannelData(3));
    SerialUSB.print(", ");
    SerialUSB.print(rx.GetChannelData(4));
    SerialUSB.print(", ");
    SerialUSB.print(rx.GetChannelData(5));
    SerialUSB.print(", ");
    SerialUSB.print(failSafe ? "Disconnected" : "Connected   ");
    /*SerialPort.print(receiver_input_channel_7);
    SerialPort.print(", ");
    SerialPort.println(receiver_input_channel_8);*/
    
    SerialUSB.print("ypr: ");
    SerialUSB.print(imu.ypr[0]);
    SerialUSB.print(", ");
    SerialUSB.print(imu.ypr[1]);
    SerialUSB.print(", ");
    SerialUSB.print(imu.ypr[2]);
    SerialUSB.print(", ");
  
    /*SerialPort.print("pressure: ");
    SerialPort.print(imu.pressure);
    SerialPort.print(", ");*/
  
    /*SerialPort.print("temp: ");
    SerialPort.print(imu.temp);
    SerialPort.print(", ");

    SerialPort.print("baro altitude: ");
    SerialPort.print(imu.baro_altitude);
    SerialPort.print(", ");*/
    
    //SerialPort.print("kalman: ");
    /*SerialPort.print(imu.altitude);*/
    SerialUSB.print(", ");
    SerialUSB.println(time_diff);

    print_cnt = 0;
  }
  print_cnt++;
}


Explorer::Explorer()
{

}

/*
  ####  ###### ###### ##  ## #####
 ##  ## ##       ##   ##  ## ##  ##
 ##     ##       ##   ##  ## ##  ##
  ####  ####     ##   ##  ## #####
     ## ##       ##   ##  ## ##
 ##  ## ##       ##   ##  ## ##
  ####  ######   ##    ####  ##
*/
void Explorer::Setup()
{
  // Communication with radio
  Serial1.begin(100000, SERIAL_8E2);
  radio_off_counter = 0;
  failSafe = false;

  // PID values
  P_pitch_a = 1.2;
  P_roll_a = 1.2;

  I_pitch_a = 0.8;
  I_roll_a = 0.8;

  D_pitch_a = 0.06;
  D_roll_a = 0.06;

  P_pitch_h = 4.0;
  P_roll_h = 4.0;

  P_yaw = 2.0;
  I_yaw = 0.8;

  imu.Setup();

  pitchRatePID.Init();
  pitchRatePID.SetConstants(P_pitch_a, I_pitch_a, D_pitch_a, INTEGRAL_MAX);
  rollRatePID.Init();
  rollRatePID.SetConstants(P_roll_a, I_roll_a, D_roll_a, INTEGRAL_MAX);
  yawRatePID.Init();
  yawRatePID.SetConstants(P_yaw, I_yaw, D_yaw, INTEGRAL_MAX);

  pitchStabilityPID.Init();
  pitchStabilityPID.SetConstants(P_pitch_h, I_pitch_h, D_pitch_h, INTEGRAL_MAX);
  rollStabilityPID.Init();
  rollStabilityPID.SetConstants(P_roll_h, I_roll_h, D_roll_h, INTEGRAL_MAX);
  yawStabilityPID.Init();
  yawStabilityPID.SetConstants(P_yaw, I_yaw, D_yaw, INTEGRAL_MAX);

  P_altitude_r = 1.2;
  P_altitude_h = 1.2;

  I_altitude_r = 0.8;
  I_altitude_h = 0.8;

  D_altitude_r = 0.06;
  D_altitude_h = 0.06;

  pid_altitude_rate.Init();
  pid_altitude_rate.SetConstants(P_altitude_r, I_altitude_r, D_altitude_r, INTEGRAL_MAX);
  pid_altitude_hold.Init();
  pid_altitude_hold.SetConstants(P_altitude_h, I_altitude_h, D_altitude_h, INTEGRAL_MAX);

  motorControl.Setup();

  front = 0;
  left = 0;
  cw = 0;

  #ifdef DEBUG
  SerialPort.println("Initializing ESCs...");
  #endif

  // Initialize ESCs
  initMotors();

  // Initialize arm controls
  arm_mode = UNARMED;
  motors_on = false;
  pinMode(13, OUTPUT);

  time_last = micros();

  #ifdef DEBUG
  SerialPort.println("Running...");
  #endif
}


/*
 ##  ##  ####  #####  #### ######  ####  ##  ##
 ##  ## ##  ## ##  ##  ##      ## ##  ## ### ##
 ##  ## ##  ## ##  ##  ##     ##  ##  ## ######
 ###### ##  ## #####   ##    ##   ##  ## ######
 ##  ## ##  ## ####    ##   ##    ##  ## ## ###
 ##  ## ##  ## ## ##   ##  ##     ##  ## ##  ##
 ##  ##  ####  ##  ## #### ######  ####  ##  ##
*/
void Explorer::UpdateHorizon(float t)
{
  //map inputs to angles
  rad_roll = map_f((float)rx.GetChannelData(0),1000.0, 2000.0, -REF_MAX_HORIZON, REF_MAX_HORIZON);
  rad_pitch = map_f((float)rx.GetChannelData(1),1000.0, 2000.0, -REF_MAX_HORIZON, REF_MAX_HORIZON);
  //rad_yaw = map_f((double)receiver_input_channel_4,1000.0, 2000.0, -REF_MAX_YAW, REF_MAX_YAW);

  //calculate pids
                                                                          //may need to calibrate for offsets
  pitch_stab = pitchStabilityPID.Update(rad_pitch, (imu.ypr[1]-0.4), t, 1.0);  //(imu.ypr[1]-0.4)
  roll_stab = rollStabilityPID.Update(rad_roll, (imu.ypr[2]-0.35), t, 1.0);     //(imu.ypr[2]-0.3)
  yaw_stab = yawStabilityPID.Update(rad_yaw, (imu.ypr[0]), t, 1.0);

  front = pitchRatePID.Update(pitch_stab, imu.y_gyr*RAD_TO_DEG, t, -1.0);
  left = rollRatePID.Update(roll_stab, imu.x_gyr*RAD_TO_DEG, t, 1.0);
  cw = yawRatePID.Update(yaw_stab, -imu.z_gyr*RAD_TO_DEG, t, -1.0);
}

/*
   ##    ####  #####   ####  #####    ##   ###### ####  ####
  ####  ##  ## ##  ## ##  ## ##  ##  ####    ##    ##  ##  ##
 ##  ## ##     ##  ## ##  ## ##  ## ##  ##   ##    ##  ##
 ###### ##     #####  ##  ## #####  ######   ##    ##  ##
 ##  ## ##     ####   ##  ## ##  ## ##  ##   ##    ##  ##
 ##  ## ##  ## ## ##  ##  ## ##  ## ##  ##   ##    ##  ##  ##
 ##  ##  ####  ##  ##  ####  #####  ##  ##   ##   ####  ####
*/
void Explorer::UpdateAcro(float t)
{
  rad_roll = map_f((float)rx.GetChannelData(0),1000.0, 2000.0, -REF_MAX_ACRO, REF_MAX_ACRO);
  rad_pitch = map_f((float)rx.GetChannelData(1),1000.0, 2000.0, -REF_MAX_ACRO, REF_MAX_ACRO);
  rad_yaw = map_f((float)rx.GetChannelData(3),1000.0, 2000.0, -REF_MAX_YAW, REF_MAX_YAW);

  //calculate pids
  front = pitchRatePID.Update(rad_pitch, imu.y_gyr*RAD_TO_DEG, t, -1.0);
  left = rollRatePID.Update(rad_roll, imu.x_gyr*RAD_TO_DEG, t, 1.0);
  cw = yawRatePID.Update(rad_yaw, -imu.z_gyr*RAD_TO_DEG, t, -1.0);
}

/*
 ###### ##    ###### #### ###### ##  ## ####   ######
 ##  ## ##      ##    ##    ##   ##  ## ## ##  ##
 ##  ## ##      ##    ##    ##   ##  ## ##  ## ##
 ###### ##      ##    ##    ##   ##  ## ##  ## ####
 ##  ## ##      ##    ##    ##   ##  ## ##  ## ##
 ##  ## ##      ##    ##    ##   ##  ## ## ##  ##
 ##  ## ######  ##   ####   ##    ####  ####   ######
*/
void Explorer::UpdateAltitudeHold(float t)
{
  //map inputs to angles
  rad_roll = map_f((float)rx.GetChannelData(0),1000.0, 2000.0, -REF_MAX_HORIZON, REF_MAX_HORIZON);
  rad_pitch = map_f((float)rx.GetChannelData(1),1000.0, 2000.0, -REF_MAX_HORIZON, REF_MAX_HORIZON);
  //rad_yaw = map_f((double)receiver_input_channel_4,1000.0, 2000.0, -REF_MAX_YAW, REF_MAX_YAW);

  //calculate pids
                                                                          //may need to calibrate for offsets
  pitch_stab = pitchStabilityPID.Update(rad_pitch, (imu.ypr[1]-0.4), t, 1.0);  //(imu.ypr[1]-0.4)
  roll_stab = rollStabilityPID.Update(rad_roll, (imu.ypr[2]-0.35), t, 1.0);     //(imu.ypr[2]-0.3)
  yaw_stab = yawStabilityPID.Update(rad_yaw, (imu.ypr[0]), t, 1.0);

  front = pitchRatePID.Update(pitch_stab, imu.y_gyr*RAD_TO_DEG, t, -1.0);
  left = rollRatePID.Update(roll_stab, imu.x_gyr*RAD_TO_DEG, t, 1.0);
  cw = yawRatePID.Update(yaw_stab, -imu.z_gyr*RAD_TO_DEG, t, -1.0);


  // Altitude control
  if (rx.GetChannelData(2) > 1400 && rx.GetChannelData(2) < 1600){
    // Hold
    if (previous_input_channel_3 < 1400 && previous_input_channel_3 > 1600)
      altitude_setpoint = imu.altitude;
    altitude_hold_out = pid_altitude_hold.Update(altitude_setpoint, imu.altitude, t, 1.0);

  }else if (rx.GetChannelData(2) < 1400){
    // Descend
    if (!altitude_startup)
      altitude_hold_out = -0.5;
  }else if (rx.GetChannelData(2) > 1600){
    // Ascend
    altitude_hold_out = 0.5;
  } 

  altitude_rate_out = pid_altitude_rate.Update(altitude_hold_out, imu.vertical_speed, t, 1.0);

  previous_input_channel_3 = rx.GetChannelData(2);
}

/*
 ##      ####   ####  #####
 ##     ##  ## ##  ## ##  ##
 ##     ##  ## ##  ## ##  ##
 ##     ##  ## ##  ## #####
 ##     ##  ## ##  ## ##
 ##     ##  ## ##  ## ##
 ######  ####   ####  ##
*/
void Explorer::Update()
{
  SerialUSB.println("Update");

  time_diff = micros() - time_last;
  time_last = micros();

  #ifdef DEBUG
  //Print();
  #endif
  
  rx.Update();

  // Motor arming control
  switch (arm_mode)
  {
    case UNARMED:
      if (rx.GetChannelData(5) > 1700)
      {
        if (rx.GetChannelData(2) < 1050)
        {
          arm_mode = ARMED;
        }
        else
        {
          arm_mode = UNSAFE;
        }
      }
      break;

    case ARMED:
      if (rx.GetChannelData(5) < 1300)
      {
        arm_mode = UNARMED;
      }
      break;

    case UNSAFE:
      if (rx.GetChannelData(5) < 1300 && rx.GetChannelData(2) < 1050)
      {
        arm_mode = UNARMED;
      }
  }

  if (arm_mode == ARMED)
  {
    motors_on = true;
  }
  else
  {
    motors_on = false;
  }

  // Determine flight mode
  if (rx.GetChannelData(4) < 1300){
    flight_mode = MODE_ALT_HOLD;
    if (previous_flight_mode == MODE_HORIZON){
      // Set altitude
      altitude_setpoint = imu.altitude;
    }
  }
  else if (rx.GetChannelData(4) > 1350 && rx.GetChannelData(4) < 1650)
    flight_mode = MODE_HORIZON;
  else if(rx.GetChannelData(4) > 1700)
    flight_mode = MODE_ACRO;

  previous_flight_mode = flight_mode;

  // Emergency landing and stop
  if (failSafe)
  {
    /*if (radio_off_counter >= LOST_CONNECTION_COUNT)
    {
      receiver_input_channel_1 = 1500;
      receiver_input_channel_2 = 1500;
      receiver_input_channel_3 = 1100;
      receiver_input_channel_4 = 1500;
      receiver_input_channel_5 = 1000;
  
      if (radio_off_counter >= LOST_CONNECTION_COUNT2){
        receiver_input_channel_6 = 1000;
      }
    }*/
    radio_off_counter++;
  }
  else
  {
    radio_off_counter = 0;
  }

  // Update all sensors in the imu
  timed = (float)time_diff/1000000.0;
  uint32_t watch = micros();
  imu.Update(timed);
  SerialUSB.println(micros() - watch);

  // Map throttle sent from radio
  rad_throttle = map(rx.GetChannelData(2), 1000, 2000, 1060, 1800);

  if (motors_on)
  {
    digitalWrite(13, HIGH);
    // Update according to set flight mode
    switch (flight_mode)
    {
        case MODE_HORIZON:
          UpdateHorizon(timed);
          break;

        case MODE_ACRO:
          UpdateAcro(timed);
          break;

        case MODE_ALT_HOLD:
          UpdateAltitudeHold(timed);
          break;

        default:
          // do something
          break;
    }   
    // Apply new speeds to motors
    UpdateMotorSpeeds();
  }
  else
  {
    digitalWrite(13, LOW);
    // Keep motors updated
    setMotorSpeedsMin();
  }
}


