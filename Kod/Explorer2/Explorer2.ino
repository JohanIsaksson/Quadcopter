
#pragma GCC optimize("-O3")

#include "imu.h"
#include "pid.h"

#define SPEED_MIN 1000
#define SPEED_MAX 2000

#define REF_MAX_HORIZON 25.0
#define REF_MAX_ACRO 90.0
#define REF_MAX_YAW 180.0

#define INTEGRAL_MAX 150.0

#define PARAM_ADDR 0

#define LOST_CONNECTION_COUNT 100
#define LOST_CONNECTION_COUNT2 12000
#define EMERGENCY_THROTTLE 0

// Flight modes
#define MODE_HORIZON 0
#define MODE_ACRO 1
#define MODE_ALT_HOLD 2

// Arming modes
#define UNARMED 0
#define ARMED 1
#define UNSAFE 2

// Debug mode (prints via serial)
#define DEBUG

// Inertial measurement unit
IMU imu;

// Motor signals
int throttle[4]; /* 0 = front left
                    1 = front right
                    2 = back left
                    3 = back right
                  */

// Motor pins
#define fl_pin 10 //front left
#define fr_pin 11 //front right
#define bl_pin 12 //back left
#define br_pin 13 //back right

// Faster IO for samd21
#ifdef _VARIANT_ARDUINO_ZERO_
  volatile uint32_t *set_fl = &PORT->Group[g_APinDescription[fl_pin].ulPort].OUTSET.reg;
  volatile uint32_t *clr_fl = &PORT->Group[g_APinDescription[fl_pin].ulPort].OUTCLR.reg;
  const uint32_t  fl_MASK = (1ul << g_APinDescription[fl_pin].ulPin);

  volatile uint32_t *set_fr = &PORT->Group[g_APinDescription[fr_pin].ulPort].OUTSET.reg;
  volatile uint32_t *clr_fr = &PORT->Group[g_APinDescription[fr_pin].ulPort].OUTCLR.reg;
  const uint32_t  fr_MASK = (1ul << g_APinDescription[fr_pin].ulPin);

  volatile uint32_t *set_bl = &PORT->Group[g_APinDescription[bl_pin].ulPort].OUTSET.reg;
  volatile uint32_t *clr_bl = &PORT->Group[g_APinDescription[bl_pin].ulPort].OUTCLR.reg;
  const uint32_t  bl_MASK = (1ul << g_APinDescription[bl_pin].ulPin);

  volatile uint32_t *set_br = &PORT->Group[g_APinDescription[br_pin].ulPort].OUTSET.reg;
  volatile uint32_t *clr_br = &PORT->Group[g_APinDescription[br_pin].ulPort].OUTCLR.reg;
  const uint32_t  br_MASK = (1ul << g_APinDescription[br_pin].ulPin);
#endif

// PID outputs for each axis
int front;
int left;
int cw;

// Horizon mode stabilzation outputs
double pitch_stab;
double roll_stab;
double yaw_stab;

// PID controllers for each axis
Pid pid_pitch_rate, pid_roll_rate, pid_yaw_rate;
Pid pid_pitch_stab, pid_roll_stab, pid_yaw_stab;

// PID parameters
double P_pitch_a, I_pitch_a, D_pitch_a,
        P_pitch_h, I_pitch_h, D_pitch_h,
        P_roll_a, I_roll_a, D_roll_a,
        P_roll_h, I_roll_h, D_roll_h,
        P_yaw, I_yaw, D_yaw;

// Altitude hold PID
Pid pid_altitude_hold, pid_altitude_rate;
double P_altitude_r, I_altitude_r, D_altitude_r,
        P_altitude_h, I_altitude_h, D_altitude_h;

// PID outputs for altitude hold
int altitude_rate_out;
double altitude_hold_out;

// altitude hold variables
double altitude_setpoint;
int previous_input_channel_3;
bool altitude_startup;

// Maximum angle and rotation sppeds
uint16_t max_pitch, max_roll, max_yaw;
uint16_t max_p, max_r, max_y;

// L.O.S. safety counter
int radio_off_counter;
bool failSafe; 

// TODO: Air mode throttle
int air_mode_throttle;
//#define AIR_MODE

// Time keeping
uint32_t time_diff;
uint32_t time_last;
double timed;

// Radio variables

volatile uint16_t receiver_input_channel_1;
volatile uint16_t receiver_input_channel_2; 
volatile uint16_t receiver_input_channel_3; 
volatile uint16_t receiver_input_channel_4; 
volatile uint16_t receiver_input_channel_5; 
volatile uint16_t receiver_input_channel_6;

double rad_roll, rad_pitch, rad_yaw;
volatile uint16_t rad_throttle;

// Motor control variables
bool motors_on, disable_sticks;
uint32_t esc_time, start_time, end_time;

// Arming control
uint8_t arm_mode;
uint8_t next_arm_mode;

// Flight mode control
uint8_t flight_mode;
uint8_t previous_flight_mode;

// Yaw control
bool set_yaw;

// Serial bus for SAMD21
#ifdef DEBUG
#define SerialPort SerialUSB
int print_cnt = 0;
#endif

// Mapping function for double data type
double map_d(double x, double in_min, double in_max, double out_min, double out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Initalizes ESCs by keeping throttle low for 3s. (MUST BE DONE ASAP AT POWER ON)
void init_motors(){
  uint32_t init_esc_time = micros() + 3000000;
  for(int i = 0; i < 4; i++){
    throttle[i] = SPEED_MIN;
  }

  while (micros() < init_esc_time){ //run for 3s approx

    start_time = micros();
    end_time = start_time + 2000;
    start_motor_pulses();

    esc_time = micros();

    while(end_time > esc_time){                         
      if (esc_time - start_time >= throttle[0]) *clr_fl = fl_MASK; //front left
      if (esc_time - start_time >= throttle[1]) *clr_fr = fr_MASK; //front right
      if (esc_time - start_time >= throttle[2]) *clr_bl = bl_MASK; //back left
      if (esc_time - start_time >= throttle[3]) *clr_br = br_MASK; //back right
      esc_time = micros();
    }

    stop_motor_pulses(); //for safety, set all outputs to 0*/


    /*PORTB |= B00001111; // set all inputs to 1

    esc_time = micros();

    while(end_time > esc_time){                         
      if (esc_time - start_time >= throttle[0]) PORTB &= B11111110; //front left
      if (esc_time - start_time >= throttle[1]) PORTB &= B11111101; //front right
      if (esc_time - start_time >= throttle[2]) PORTB &= B11111011; //back left
      if (esc_time - start_time >= throttle[3]) PORTB &= B11110111; //back right
      esc_time = micros();
    }

    PORTB &= B11110000; //for sfety, set all outputs to 0*/
      
    while(micros() - time_last < 4500);
  }
}

// Limit integer x to interval [a,b]
uint16_t limit(uint16_t x, uint16_t a, uint16_t b){
  if (x < a) return a;
  else if (x > b) return b;
  return x;
}

// Sets all motor pulses high
void start_motor_pulses(){
  *set_fl = fl_MASK; //front left
  *set_fr = fr_MASK; //front right
  *set_bl = bl_MASK; //back left
  *set_br = br_MASK; //back right
}

// Sets all motor pulses low
void stop_motor_pulses(){
  *clr_fl = fl_MASK; //front left
  *clr_fr = fr_MASK; //front right
  *clr_bl = bl_MASK; //back left
  *clr_br = br_MASK; //back right
}

// Sets final throttle by combining outputs from all PIDs 
void set_motor_speeds(){
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

  //check and set speeds
  throttle[0] = limit(throttle[0], SPEED_MIN, SPEED_MAX);
  throttle[1] = limit(throttle[1], SPEED_MIN, SPEED_MAX);
  throttle[2] = limit(throttle[2], SPEED_MIN, SPEED_MAX);
  throttle[3] = limit(throttle[3], SPEED_MIN, SPEED_MAX);
    
  esc_time = micros();

  while(end_time > esc_time){
    if (esc_time - start_time >= throttle[0]) *clr_fl = fl_MASK; //front left
    if (esc_time - start_time >= throttle[1]) *clr_fr = fr_MASK; //front right
    if (esc_time - start_time >= throttle[2]) *clr_bl = bl_MASK; //back left
    if (esc_time - start_time >= throttle[3]) *clr_br = br_MASK; //back right
    esc_time = micros();
  }
  //turn all pulses off for safety
  stop_motor_pulses();

  if (flight_mode == MODE_HORIZON){
    while(micros() - time_last < 4000);
  }else if (flight_mode == MODE_ACRO){
    while(micros() - time_last < 2800);
  }
}

// Sets minimum throttle for all motors
void set_motor_speeds_min(){

  while(end_time > micros());
  stop_motor_pulses();

  while(micros() - time_last < 4000);
}

void set_motor_speeds_max(){

  while(end_time > micros());
  stop_motor_pulses();

    
  while(micros() - time_last < 4000);
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
void setup(){
  delay(2000);

#ifdef DEBUG
  SerialPort.begin(115200);
  SerialPort.println("Starting up");
#endif

  // Communication with radio
  Serial1.begin(100000, SERIAL_8E2);
  radio_off_counter = 0;
  failSafe = false;
  receiver_input_channel_1 = 1500;
  receiver_input_channel_2 = 1500;
  receiver_input_channel_3 = 1000;
  receiver_input_channel_4 = 1500;
  receiver_input_channel_5 = 1000;
  receiver_input_channel_6 = 1000;

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

  imu.Init();	

  pid_pitch_rate.Init();
  pid_pitch_rate.SetConstants(P_pitch_a, I_pitch_a, D_pitch_a, INTEGRAL_MAX);
  pid_roll_rate.Init();
  pid_roll_rate.SetConstants(P_roll_a, I_roll_a, D_roll_a, INTEGRAL_MAX);
  pid_yaw_rate.Init();
  pid_yaw_rate.SetConstants(P_yaw, I_yaw, D_yaw, INTEGRAL_MAX);

  pid_pitch_stab.Init();
  pid_pitch_stab.SetConstants(P_pitch_h, I_pitch_h, D_pitch_h, INTEGRAL_MAX);
  pid_roll_stab.Init();
  pid_roll_stab.SetConstants(P_roll_h, I_roll_h, D_roll_h, INTEGRAL_MAX);
  pid_yaw_stab.Init();
  pid_yaw_stab.SetConstants(P_yaw, I_yaw, D_yaw, INTEGRAL_MAX);

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

  front = 0;
  left = 0;
  cw = 0;

  #ifdef DEBUG
  SerialPort.println("Initializing ESCs...");
  #endif

  //set pins as outputs
  pinMode(fl_pin, OUTPUT);
  pinMode(fr_pin, OUTPUT);
  pinMode(bl_pin, OUTPUT);
  pinMode(br_pin, OUTPUT);

  // Initialize ESCs
  init_motors();

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
void Update_horizon(double t)
{
  
  //compensate for mistimings with deadband
  if (receiver_input_channel_1 > 1485 && receiver_input_channel_1 < 1515) receiver_input_channel_1 = 1500; //roll
  if (receiver_input_channel_2 > 1485 && receiver_input_channel_2 < 1515) receiver_input_channel_2 = 1500; //pitch
  if (receiver_input_channel_4 > 1485 && receiver_input_channel_4 < 1515){                                  //yaw
    receiver_input_channel_4 = 1500;
    if (set_yaw){
      rad_yaw = imu.ypr[0];
    }
    set_yaw = false;
  }else{
    set_yaw = true;
  }

  //map inputs to angles
  rad_roll = map_d((double)receiver_input_channel_1,1000.0, 2000.0, -REF_MAX_HORIZON, REF_MAX_HORIZON);
  rad_pitch = map_d((double)receiver_input_channel_2,1000.0, 2000.0, -REF_MAX_HORIZON, REF_MAX_HORIZON);
  //rad_yaw = map_d((double)receiver_input_channel_4,1000.0, 2000.0, -REF_MAX_YAW, REF_MAX_YAW);

  //calculate pids
                                                                          //may need to calibrate for offsets
  pid_pitch_stab.Update(&pitch_stab, rad_pitch, (imu.ypr[1]-0.4), t, 1.0);  //(imu.ypr[1]-0.4)
  pid_roll_stab.Update(&roll_stab, rad_roll, (imu.ypr[2]-0.35), t, 1.0);     //(imu.ypr[2]-0.3)
  pid_yaw_stab.Update(&yaw_stab, rad_yaw, (imu.ypr[0]), t, 1.0);

  pid_pitch_rate.Update(&front, pitch_stab, imu.y_gyr*RAD_TO_DEG, t, -1.0);
  pid_roll_rate.Update(&left, roll_stab, imu.x_gyr*RAD_TO_DEG, t, 1.0);
  pid_yaw_rate.Update(&cw, yaw_stab, -imu.z_gyr*RAD_TO_DEG, t, -1.0);

}

/*
   ##    ####  #####   ####  #####    ## ###### #### ####
  ####  ##  ## ##  ## ##  ## ##  ##  ####  ##    ## ##  ##
 ##  ## ##     ##  ## ##  ## ##  ## ##  ## ##    ## ##
 ###### ##     #####  ##  ## #####  ###### ##    ## ##
 ##  ## ##     ####   ##  ## ##  ## ##  ## ##    ## ##
 ##  ## ##  ## ## ##  ##  ## ##  ## ##  ## ##    ## ##  ##
 ##  ##  ####  ##  ##  ####  #####  ##  ## ##   #### ####
*/
void Update_acro(double t)
{

  //compensate for mistimings with deadband
  if (receiver_input_channel_1 > 1475 && receiver_input_channel_1 < 1525) receiver_input_channel_1 = 1500;
  if (receiver_input_channel_2 > 1475 && receiver_input_channel_2 < 1525) receiver_input_channel_2 = 1500;
  if (receiver_input_channel_4 > 1475 && receiver_input_channel_4 < 1525) receiver_input_channel_4 = 1500;

  //p.K_tmp = map_d((double)receiver_input_channel_6,1000.0, 2000.0, 0.0, 0.8);

  //map inputs to anglerates
  //rad_roll = map_x3((double)(receiver_input_channel_1-1500));
  //rad_pitch = map_x3((double)(receiver_input_channel_2-1500));
  //rad_yaw = map_x3((double)(receiver_input_channel_4-1500));

  rad_roll = map_d((double)receiver_input_channel_1,1000.0, 2000.0, -REF_MAX_ACRO, REF_MAX_ACRO);
  rad_pitch = map_d((double)receiver_input_channel_2,1000.0, 2000.0, -REF_MAX_ACRO, REF_MAX_ACRO);
  rad_yaw = map_d((double)receiver_input_channel_4,1000.0, 2000.0, -REF_MAX_YAW, REF_MAX_YAW);
  
  //SerialPort.println(rad_roll); delay(50);


  //calculate pids
  pid_pitch_rate.Update(&front, rad_pitch, imu.y_gyr*RAD_TO_DEG, t, -1.0);
  pid_roll_rate.Update(&left, rad_roll, imu.x_gyr*RAD_TO_DEG, t, 1.0);
  pid_yaw_rate.Update(&cw, rad_yaw, -imu.z_gyr*RAD_TO_DEG, t, -1.0);
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
void Update_altitude_hold(double t)
{

  //compensate for mistimings with deadband
  if (receiver_input_channel_1 > 1485 && receiver_input_channel_1 < 1515) receiver_input_channel_1 = 1500; //roll
  if (receiver_input_channel_2 > 1485 && receiver_input_channel_2 < 1515) receiver_input_channel_2 = 1500; //pitch
  if (receiver_input_channel_4 > 1485 && receiver_input_channel_4 < 1515){                                  //yaw
    receiver_input_channel_4 = 1500;
    if (set_yaw){
      rad_yaw = imu.ypr[0];
    }
    set_yaw = false;
  }else{
    set_yaw = true;
  }

  //map inputs to angles
  rad_roll = map_d((double)receiver_input_channel_1,1000.0, 2000.0, -REF_MAX_HORIZON, REF_MAX_HORIZON);
  rad_pitch = map_d((double)receiver_input_channel_2,1000.0, 2000.0, -REF_MAX_HORIZON, REF_MAX_HORIZON);
  //rad_yaw = map_d((double)receiver_input_channel_4,1000.0, 2000.0, -REF_MAX_YAW, REF_MAX_YAW);

  //calculate pids
                                                                          //may need to calibrate for offsets
  pid_pitch_stab.Update(&pitch_stab, rad_pitch, (imu.ypr[1]-0.4), t, 1.0);  //(imu.ypr[1]-0.4)
  pid_roll_stab.Update(&roll_stab, rad_roll, (imu.ypr[2]-0.35), t, 1.0);     //(imu.ypr[2]-0.3)
  pid_yaw_stab.Update(&yaw_stab, rad_yaw, (imu.ypr[0]), t, 1.0);

  pid_pitch_rate.Update(&front, pitch_stab, imu.y_gyr*RAD_TO_DEG, t, -1.0);
  pid_roll_rate.Update(&left, roll_stab, imu.x_gyr*RAD_TO_DEG, t, 1.0);
  pid_yaw_rate.Update(&cw, yaw_stab, -imu.z_gyr*RAD_TO_DEG, t, -1.0);


  // Altitude control
  if (receiver_input_channel_3 > 1400 && receiver_input_channel_3 < 1600){
    // Hold
    if (previous_input_channel_3 < 1400 && previous_input_channel_3 > 1600)
      altitude_setpoint = imu.altitude;
    pid_altitude_hold.Update(&altitude_hold_out, altitude_setpoint, imu.altitude, t, 1.0);

  }else if (receiver_input_channel_3 < 1400){
    // Descend
    if (!altitude_startup)
      altitude_hold_out = -0.5;
  }else if (receiver_input_channel_3 > 1600){
    // Ascend
    altitude_hold_out = 0.5;
  } 

  pid_altitude_rate.Update(&altitude_rate_out, altitude_hold_out, imu.vertical_speed, t, 1.0);

  previous_input_channel_3 = receiver_input_channel_3;
}

/*
 ##     ####   ####  #####
 ##    ##  ## ##  ## ##  ##
 ##    ##  ## ##  ## ##  ##
 ##    ##  ## ##  ## #####
 ##    ##  ## ##  ## ##
 ##    ##  ## ##  ## ##
 ###### ####   ####  ##
*/
void loop()
{


  time_diff = micros() - time_last;
  time_last = micros();


  uint8_t sbus_packet[25];
  uint16_t channels[16];
  if (Serial1.available() >= 25)
  {
    //SerialPort.println(Serial1.available());
    if (Serial1.available() == 25)
    {
      for (int i = 0; i < 25; i++)
      {
        sbus_packet[i] = Serial1.read();
      }

      // Verify packet
      if ((sbus_packet[0] == 0x0F))// && (sbus_packet[24] == 0x00))
      {
        // Read channels
        
        // Channel 1
        receiver_input_channel_1 = (sbus_packet[2] & 0x07);
        receiver_input_channel_1 <<= 8;
        receiver_input_channel_1 += sbus_packet[1];
        receiver_input_channel_1 = map(receiver_input_channel_1, 192, 1792, 1000, 2000);

        // Channel 2
        receiver_input_channel_2 = (sbus_packet[3] & 0x3F);
        receiver_input_channel_2 <<= 5;
        receiver_input_channel_2 += ((sbus_packet[2] & 0xF7) >> 3);
        receiver_input_channel_2 = map(receiver_input_channel_2, 192, 1792, 1000, 2000);

        // Channel 3
        receiver_input_channel_3 = (sbus_packet[5] & 0x01);
        receiver_input_channel_3 <<= 8;
        receiver_input_channel_3 += sbus_packet[4];
        receiver_input_channel_3 <<= 2;
        receiver_input_channel_3 += ((sbus_packet[3] & 0xC0) >> 6);
        receiver_input_channel_3 = map(receiver_input_channel_3, 192, 1792, 1000, 2000);

        // Channel 4
        receiver_input_channel_4 = (sbus_packet[6] & 0x0F);
        receiver_input_channel_4 <<= 7;
        receiver_input_channel_4 += ((sbus_packet[5] & 0xFE) >> 1);
        receiver_input_channel_4 = map(receiver_input_channel_4, 192, 1792, 1000, 2000);

        // Channel 5
        receiver_input_channel_5 = (sbus_packet[7] & 0x7F);
        receiver_input_channel_5 <<= 4;
        receiver_input_channel_5 += ((sbus_packet[6] & 0xF0) >> 4) ;
        receiver_input_channel_5 = map(receiver_input_channel_5, 192, 1792, 1000, 2000);

        // Channel 6
        receiver_input_channel_6 = (sbus_packet[9] & 0x03);
        receiver_input_channel_6 <<= 8;
        receiver_input_channel_6 += sbus_packet[8];
        receiver_input_channel_6 <<= 1;
        receiver_input_channel_6 += ((sbus_packet[7] & 0x80) >> 7);
        receiver_input_channel_6 = map(receiver_input_channel_6, 192, 1792, 1000, 2000);

        // Failsafe
        failSafe = (sbus_packet[23] & 0x10) ? true : false;
      }
    }
    else
    {
      // Remove bad data
      while (Serial1.available() > 0)
      {
        Serial1.read();
      }
    }
  }

#ifdef DEBUG

  if (print_cnt >= 10){

    SerialPort.print((motors_on ? "On " : "Off"));
    SerialPort.print(", ");
  
    if (flight_mode == MODE_ACRO)
      SerialPort.print("ACRO         ");
    else if (flight_mode == MODE_HORIZON)
      SerialPort.print("HORIZON      ");
    else if (flight_mode == MODE_ALT_HOLD)
      SerialPort.print("ALTITUDE HOLD");
    SerialPort.print(", ");

    SerialPort.print(receiver_input_channel_1);
    SerialPort.print(", ");
    SerialPort.print(receiver_input_channel_2);
    SerialPort.print(", ");
    SerialPort.print(receiver_input_channel_3);
    SerialPort.print(", ");
    SerialPort.print(receiver_input_channel_4);
    SerialPort.print(", ");
    SerialPort.print(receiver_input_channel_5);
    SerialPort.print(", ");
    SerialPort.print(receiver_input_channel_6);
    SerialPort.print(", ");
    SerialPort.println(failSafe ? "Disconnected" : "Connected   ");
    /*SerialPort.print(receiver_input_channel_7);
    SerialPort.print(", ");
    SerialPort.println(receiver_input_channel_8);*/
    
    /*SerialPort.print("ypr: ");
    SerialPort.print(imu.ypr[0]);
    SerialPort.print(", ");
    SerialPort.print(imu.ypr[1]);
    SerialPort.print(", ");
    SerialPort.print(imu.ypr[2]);
    SerialPort.print(", ");*/
  
    /*SerialPort.print("pressure: ");
    SerialPort.print(imu.pressure);
    SerialPort.print(", ");*/
  
    /*SerialPort.print("temp: ");
    SerialPort.print(imu.temp);
    SerialPort.print(", ");

    SerialPort.print("baro altitude: ");
    SerialPort.print(imu.baro_altitude);
    SerialPort.print(", ");*/
    
    /*SerialPort.print("kalman: ");
    SerialPort.print(imu.altitude);
    SerialPort.print(", ");
    SerialPort.println(imu.vertical_speed);*/

    print_cnt = 0;
  }
  print_cnt++;
#endif

  // Motor arming control
  switch (arm_mode)
  {
    case UNARMED:
      if (receiver_input_channel_6 > 1700)
      {
        if (receiver_input_channel_3 < 1050)
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
      if (receiver_input_channel_6 < 1300)
      {
        arm_mode = UNARMED;
      }
      break;

    case UNSAFE:
      if (receiver_input_channel_6 < 1300 && receiver_input_channel_3 < 1050)
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
  if (receiver_input_channel_5 < 1300){
    flight_mode = MODE_ALT_HOLD;
    if (previous_flight_mode == MODE_HORIZON){
      // Set altitude
      altitude_setpoint = imu.altitude;
    }
  }
  else if (receiver_input_channel_5 > 1350 && receiver_input_channel_5 < 1650)
    flight_mode = MODE_HORIZON;
  else if(receiver_input_channel_5 > 1700)
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
  timed = (double)time_diff/1000000.0;
  imu.Update(timed); 

  // Start esc pulses
  start_time = micros();  
  start_motor_pulses();

  // Map throttle sent from radio
  rad_throttle = map(receiver_input_channel_3, 1000, 2000, 1060, 1800);

  if (motors_on)
  {
    digitalWrite(13, HIGH);
    // Update according to set flight mode
    switch (flight_mode)
    {
        case MODE_HORIZON:
          Update_horizon(timed);
          break;

        case MODE_ACRO:
          Update_acro(timed);
          break;

        case MODE_ALT_HOLD:
          Update_altitude_hold(timed);
          break;

        default:
          // do something
          break;
    }   
    // Apply new speeds to motors
    end_time = start_time + 2000;
    set_motor_speeds();
  }
  else
  {
    digitalWrite(13, LOW);
    // Keep motors updated
    end_time = start_time + 1015;
    set_motor_speeds_min();
  }
}


