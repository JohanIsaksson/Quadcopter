#pragma GCC optimize("-O3")

#include "imu.h"
#include "pid.h"
#include <EEPROM.h>
#include "fixed_point.h"

#define SPEED_MIN 1000
#define SPEED_MAX 2000

#define REF_MAX_HORIZON 25.0
#define REF_MAX_ACRO 90.0
#define REF_MAX_YAW 180.0

#define INTEGRAL_MAX 150.0

#define PARAM_ADDR 0

#define LOST_CONNECTION_COUNT 100
#define EMERGENCY_THROTTLE 0

#define MODE_HORIZON 0
#define MODE_ACRO 1


//#define YPR_DATA
//#define COMP_DATA
//#define PID_DATA
//#define MAG_DATA
//#define JAVA_DATA
//#define RADIO_DATA
//#define ESC_DATA


// Tunig control
#define TUNING_MAX 10.0
#define TUNING_MIN 0.0
//#define TUNING_MODE 6
double tun;
/*
0   -   P pitch/roll acro
1   -   I pitch/roll acro
2   -   D pitch/roll acro
3   -   P yaw
4   -   I yaw
5   -   D yaw
6   -   P pitch/roll horizon
7   -   I pitch/roll horizon
8   -   D pitch/roll horizon
9   -   


*/




IMU imu;

//motors
int throttle[4]; /* 0 = front left
                    1 = front right
                    2 = back left
                    3 = back right
                  */

//pid
int front;
int left;
int cw;

int pitch_stab, roll_stab;

Pid pid_pitch_rate, pid_roll_rate, pid_yaw_rate;
Pid pid_pitch_stab, pid_roll_stab;

double P_pitch_a, I_pitch_a, D_pitch_a,
        P_pitch_h, I_pitch_h, D_pitch_h,
        P_roll_a, I_roll_a, D_roll_a,
        P_roll_h, I_roll_h, D_roll_h,
        P_yaw, I_yaw, D_yaw;

uint16_t max_pitch, max_roll, max_yaw;
uint16_t max_p, max_r, max_y;
uint8_t exp_lin;

uint8_t buf[26];

//output
int count;

//safety stuff
int radio_off_counter;

//time keeping
uint32_t time_diff;
uint32_t time_last;
double timed;

//radio variables
byte last_channel_1, 
      last_channel_2, 
      last_channel_3, 
      last_channel_4,
      last_channel_5, 
      last_channel_6;
int receiver_input_channel_1, 
    receiver_input_channel_2, 
    receiver_input_channel_3, 
    receiver_input_channel_4, 
    receiver_input_channel_5, 
    receiver_input_channel_6;
uint32_t timer_1, 
          timer_2, 
          timer_3, 
          timer_4,
          timer_5,
          timer_6;
double rad_roll, rad_pitch, rad_yaw;
uint16_t rad_throttle;

//motor control variables
bool motors_on, disable_sticks;
uint32_t us, esc_time, start_time, end_time;

//flight mode control
uint8_t flight_mode;


double map_d(double x, double in_min, double in_max, double out_min, double out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* Third order polynomial */
double map_x3(double x){
  return x*(0.00000096*x*x + 0.12);
}

void init_motors(){
  uint32_t init_esc_time = micros() + 3000000;
  for(int i = 0; i < 4; i++){
    throttle[i] = SPEED_MIN;
  }

  while (micros() < init_esc_time){ //run for 3s approx

    start_time = micros();
    end_time = start_time + 2000;
    PORTB |= B00001111; // set all inputs to 1

    esc_time = micros();

    while(end_time > esc_time){                         
      if (esc_time - start_time >= throttle[0]) PORTB &= B11111110; //front left
      if (esc_time - start_time >= throttle[1]) PORTB &= B11111101; //front right
      if (esc_time - start_time >= throttle[2]) PORTB &= B11111011; //back left
      if (esc_time - start_time >= throttle[3]) PORTB &= B11110111; //back right
      esc_time = micros();
    }

    PORTB &= B11110000; //for sfety, set all outputs to 0
      
    while(micros() - time_last < 4500);
  }
}

void set_motor_speeds(){
  //front left
  throttle[0] = rad_throttle + front + left + cw; //+ CALIBRATION_COMP;

  //front right
  throttle[1] = rad_throttle + front - left - cw; //+ CALIBRATION_COMP;

  //back left
  throttle[2] = rad_throttle - front + left - cw; //- CALIBRATION_COMP;

  //back right
  throttle[3] = rad_throttle - front - left + cw; //- CALIBRATION_COMP;

  //check and set speeds
  if (radio_off_counter >= LOST_CONNECTION_COUNT){
    for (int i = 0; i < 4; i++){
      throttle[i] = SPEED_MIN;
    }
  }else{
    for (int i = 0; i < 4; i++){
      if (throttle[i] > SPEED_MAX){
        throttle[i] = SPEED_MAX;
      }else if (throttle[i] < SPEED_MIN){
        throttle[i] = SPEED_MIN;
      }        
    }
  } 

  esc_time = micros();

  while(end_time > esc_time){                         //Start the pulse after 1250 micro seconds.
    if (esc_time - start_time >= throttle[0]) PORTB &= B11111110; //front left
    if (esc_time - start_time >= throttle[1]) PORTB &= B11111101; //front left
    if (esc_time - start_time >= throttle[2]) PORTB &= B11111011; //front left
    if (esc_time - start_time >= throttle[3]) PORTB &= B11110111; //front left
    esc_time = micros();
  }
  PORTB &= B11110000; //turn all pulses off for safety

  if (flight_mode == MODE_HORIZON){
    while(micros() - time_last < 4000);
    }else{
      while(micros() - time_last < 2800);
    }
}

void set_motor_speeds_min(){
  //start esc pulses
  start_time = micros(); 
  end_time = start_time + 1000;
  PORTB |= B00001111;


  while(end_time > micros());
  PORTB &= B11110000; //turn all pulses off for safety

    
  while(micros() - time_last < 4000);
}

void set_motor_speeds_max(){
  //start esc pulses
  start_time = micros(); 
  end_time = start_time + 2000;
  PORTB |= B00001111;

  while(end_time > micros());
  PORTB &= B11110000; //turn all pulses off for safety

    
  while(micros() - time_last < 4000);
}

void EEPROM_get(){
  size_t addr = PARAM_ADDR;
  EEPROM.get(addr, P_pitch_a);
  addr += sizeof(double);
  EEPROM.get(addr, I_pitch_a);
  addr += sizeof(double);
  EEPROM.get(addr, D_pitch_a);
  addr += sizeof(double);

  EEPROM.get(addr, P_roll_a);
  addr += sizeof(double);
  EEPROM.get(addr, I_roll_a);
  addr += sizeof(double);
  EEPROM.get(addr, D_roll_a);
  addr += sizeof(double);

  EEPROM.get(addr, P_pitch_h);
  addr += sizeof(double);
  EEPROM.get(addr, I_pitch_h);
  addr += sizeof(double);
  EEPROM.get(addr, D_pitch_h);
  addr += sizeof(double);

  EEPROM.get(addr, P_roll_h);
  addr += sizeof(double);
  EEPROM.get(addr, I_roll_h);
  addr += sizeof(double);
  EEPROM.get(addr, D_roll_h);
  addr += sizeof(double);

  EEPROM.get(addr, P_yaw);
  addr += sizeof(double);
  EEPROM.get(addr, I_yaw);
  addr += sizeof(double);
  EEPROM.get(addr, D_yaw);
  addr += sizeof(double);
}

void EEPROM_put(){
  size_t addr = PARAM_ADDR;
  EEPROM.put(addr, P_pitch_a);
  addr += sizeof(double);
  EEPROM.put(addr, I_pitch_a);
  addr += sizeof(double);
  EEPROM.put(addr, D_pitch_a);
  addr += sizeof(double);

  EEPROM.put(addr, P_roll_a);
  addr += sizeof(double);
  EEPROM.put(addr, I_roll_a);
  addr += sizeof(double);
  EEPROM.put(addr, D_roll_a);
  addr += sizeof(double);

  EEPROM.put(addr, P_pitch_h);
  addr += sizeof(double);
  EEPROM.put(addr, I_pitch_h);
  addr += sizeof(double);
  EEPROM.put(addr, D_pitch_h);
  addr += sizeof(double);

  EEPROM.put(addr, P_roll_h);
  addr += sizeof(double);
  EEPROM.put(addr, I_roll_h);
  addr += sizeof(double);
  EEPROM.put(addr, D_roll_h);
  addr += sizeof(double);

  EEPROM.put(addr, P_yaw);
  addr += sizeof(double);
  EEPROM.put(addr, I_yaw);
  addr += sizeof(double);
  EEPROM.put(addr, D_yaw);
  addr += sizeof(double);

  
}


void insert_32(uint32_t data, int pos){
  buf[pos] = (data >> 24) & 0x000000FF;
  buf[pos+1] = (data >> 16) & 0x000000FF;
  buf[pos+2] = (data >> 8) & 0x000000FF;
  buf[pos+3] = data & 0x000000FF;
}

void insert_16(uint16_t data, int pos){
  buf[pos] = (data >> 8) & 0x000000FF;
  buf[pos+1] = data & 0x000000FF;
}

void send_data(){

  // Transfering order:
  //  [P_pitch_a, I_pitch_a, D_pitch_a, P_pitch_h, I_pitch_h, D_pitch_h, P_roll_a, I_roll_a, D_roll_a, P_roll_h, I_roll_h, D_roll_h, P_yaw, I_yaw, D_yaw, max_pitch, max_roll, max_yaw, exp_lin];


  //send first part
  buf[0] = 1;

  int pos = 1;

  insert_32(encode_d(P_pitch_a), pos);
  pos+=4;
  insert_32(encode_d(I_pitch_a), pos); 
  pos+=4;
  insert_32(encode_d(D_pitch_a), pos);
  pos+=4;
  insert_32(encode_d(P_pitch_h), pos); 
  pos+=4;
  insert_32(encode_d(I_pitch_h), pos); 
  pos+=4;
  insert_32(encode_d(D_pitch_h), pos); 

  Wire.beginTransmission(15);
  Wire.write(buf, 25);
  Wire.endTransmission(); 


  //send second part

  buf[0] = 2;

  pos = 1;

  insert_32(encode_d(P_roll_a), pos); 
  pos+=4;
  insert_32(encode_d(I_roll_a), pos); 
  pos+=4;
  insert_32(encode_d(D_roll_a), pos); 
  pos+=4;
  insert_32(encode_d(P_roll_h), pos); 
  pos+=4;
  insert_32(encode_d(I_roll_h), pos); 
  pos+=4;
  insert_32(encode_d(D_roll_h), pos); 

  Wire.beginTransmission(15);
  Wire.write(buf, 25);
  Wire.endTransmission(); 


  // send third part

  buf[0] = 3;

  pos = 1;

  insert_32(encode_d(P_yaw), pos); 
  pos+=4;
  insert_32(encode_d(I_yaw), pos); 
  pos+=4;
  insert_32(encode_d(D_yaw), pos);
  pos+=4;

  insert_16(max_pitch, pos);
  pos+=2;
  insert_16(max_roll, pos);
  pos+=2;
  insert_16(max_yaw, pos);
  pos+=2;

  buf[19] = exp_lin;

  Wire.beginTransmission(15);
  Wire.write(buf, 20);
  Wire.endTransmission(); 
}


void get_data(){
  // Transfering order:
  //  [P_pitch_a, I_pitch_a, D_pitch_a, P_pitch_h, I_pitch_h, D_pitch_h, P_roll_a, I_roll_a, D_roll_a, P_roll_h, I_roll_h, D_roll_h, P_yaw, I_yaw, D_yaw, max_pitch, max_roll, max_yaw, exp_lin];
  
  uint32_t buf_tmp[6];


  Wire.requestFrom(15, 25);

  int i = 0;
  while (Wire.available()) { // slave may send less than requested
    buf[i] = Wire.read(); // receive a byte
    Serial.println(buf[i]);
    i++;
  }
  Serial.println("--------");

  switch(buf[0]){
    case 1:
      for (int i = 0; i < 6; i++){
        buf_tmp[i] = ((uint32_t)buf[i*4 + 1] << 24) + ((uint32_t)buf[i*4 + 2] << 16) + ((uint32_t)buf[i*4 + 3] << 8) + buf[i*4 + 4];    
      }

      P_pitch_a = decode_d(buf_tmp[0]);
      I_pitch_a = decode_d(buf_tmp[1]);
      D_pitch_a = decode_d(buf_tmp[2]);
     
      P_pitch_h = decode_d(buf_tmp[3]);
      I_pitch_h = decode_d(buf_tmp[4]);
      D_pitch_h = decode_d(buf_tmp[5]);

    break;


    case 2:
      for (int i = 0; i < 6; i++){
        buf_tmp[i] = ((uint32_t)buf[i*4 + 1] << 24) + ((uint32_t)buf[i*4 + 2] << 16) + ((uint32_t)buf[i*4 + 3] << 8) + buf[i*4 + 4];    
      }

      P_roll_a = decode_d(buf_tmp[0]);
      I_roll_a = decode_d(buf_tmp[1]);
      D_roll_a = decode_d(buf_tmp[2]);
     
      P_roll_h = decode_d(buf_tmp[3]);
      I_roll_h = decode_d(buf_tmp[4]);
      D_roll_h = decode_d(buf_tmp[5]);

    break;


    case 3:
      for (int i = 0; i < 3; i++){
        buf_tmp[i] = ((uint32_t)buf[i*4 + 1] << 24) + ((uint32_t)buf[i*4 + 2] << 16) + ((uint32_t)buf[i*4 + 3] << 8) + buf[i*4 + 4];    
      }

      P_yaw = decode_d(buf_tmp[0]);
      I_yaw = decode_d(buf_tmp[1]);
      D_yaw = decode_d(buf_tmp[2]);

      max_pitch = ((uint16_t)buf[13] << 8) + (uint16_t)buf[14];
      max_roll = ((uint16_t)buf[15] << 8) + (uint16_t)buf[16];
      max_yaw = ((uint16_t)buf[17] << 8) + (uint16_t)buf[18];

      exp_lin = buf[19];

    break;
  }



  Wire.requestFrom(15, 25);

  i = 0;
  while (Wire.available()) { // slave may send less than requested
    buf[i] = Wire.read(); // receive a byte
    Serial.println(buf[i]);
    i++;
  }
  Serial.println("--------");

  switch(buf[0]){
    case 1:
      for (int i = 0; i < 6; i++){
        buf_tmp[i] = ((uint32_t)buf[i*4 + 1] << 24) + ((uint32_t)buf[i*4 + 2] << 16) + ((uint32_t)buf[i*4 + 3] << 8) + buf[i*4 + 4];    
      }

      P_pitch_a = decode_d(buf_tmp[0]);
      I_pitch_a = decode_d(buf_tmp[1]);
      D_pitch_a = decode_d(buf_tmp[2]);
     
      P_pitch_h = decode_d(buf_tmp[3]);
      I_pitch_h = decode_d(buf_tmp[4]);
      D_pitch_h = decode_d(buf_tmp[5]);

    break;


    case 2:
      for (int i = 0; i < 6; i++){
        buf_tmp[i] = ((uint32_t)buf[i*4 + 1] << 24) + ((uint32_t)buf[i*4 + 2] << 16) + ((uint32_t)buf[i*4 + 3] << 8) + buf[i*4 + 4];    
      }

      P_roll_a = decode_d(buf_tmp[0]);
      I_roll_a = decode_d(buf_tmp[1]);
      D_roll_a = decode_d(buf_tmp[2]);
     
      P_roll_h = decode_d(buf_tmp[3]);
      I_roll_h = decode_d(buf_tmp[4]);
      D_roll_h = decode_d(buf_tmp[5]);

    break;


    case 3:
      for (int i = 0; i < 3; i++){
        buf_tmp[i] = ((uint32_t)buf[i*4 + 1] << 24) + ((uint32_t)buf[i*4 + 2] << 16) + ((uint32_t)buf[i*4 + 3] << 8) + buf[i*4 + 4];    
      }

      P_yaw = decode_d(buf_tmp[0]);
      I_yaw = decode_d(buf_tmp[1]);
      D_yaw = decode_d(buf_tmp[2]);

      max_pitch = ((uint16_t)buf[13] << 8) + (uint16_t)buf[14];
      max_roll = ((uint16_t)buf[15] << 8) + (uint16_t)buf[16];
      max_yaw = ((uint16_t)buf[17] << 8) + (uint16_t)buf[18];

      exp_lin = buf[19];

    break;
  }


  Wire.requestFrom(15, 25);

  i = 0;
  while (Wire.available()) { // slave may send less than requested
    buf[i] = Wire.read(); // receive a byte
    Serial.println(buf[i]);
    i++;
  }
  Serial.println("--------");

  switch(buf[0]){
    case 1:
      for (int i = 0; i < 6; i++){
        buf_tmp[i] = ((uint32_t)buf[i*4 + 1] << 24) + ((uint32_t)buf[i*4 + 2] << 16) + ((uint32_t)buf[i*4 + 3] << 8) + buf[i*4 + 4];    
      }

      P_pitch_a = decode_d(buf_tmp[0]);
      I_pitch_a = decode_d(buf_tmp[1]);
      D_pitch_a = decode_d(buf_tmp[2]);
     
      P_pitch_h = decode_d(buf_tmp[3]);
      I_pitch_h = decode_d(buf_tmp[4]);
      D_pitch_h = decode_d(buf_tmp[5]);

    break;


    case 2:
      for (int i = 0; i < 6; i++){
        buf_tmp[i] = ((uint32_t)buf[i*4 + 1] << 24) + ((uint32_t)buf[i*4 + 2] << 16) + ((uint32_t)buf[i*4 + 3] << 8) + buf[i*4 + 4];    
      }

      P_roll_a = decode_d(buf_tmp[0]);
      I_roll_a = decode_d(buf_tmp[1]);
      D_roll_a = decode_d(buf_tmp[2]);
     
      P_roll_h = decode_d(buf_tmp[3]);
      I_roll_h = decode_d(buf_tmp[4]);
      D_roll_h = decode_d(buf_tmp[5]);

    break;


    case 3:
      for (int i = 0; i < 3; i++){
        buf_tmp[i] = ((uint32_t)buf[i*4 + 1] << 24) + ((uint32_t)buf[i*4 + 2] << 16) + ((uint32_t)buf[i*4 + 3] << 8) + buf[i*4 + 4];    
      }

      P_yaw = decode_d(buf_tmp[0]);
      I_yaw = decode_d(buf_tmp[1]);
      D_yaw = decode_d(buf_tmp[2]);

      max_pitch = ((uint16_t)buf[13] << 8) + (uint16_t)buf[14];
      max_roll = ((uint16_t)buf[15] << 8) + (uint16_t)buf[16];
      max_yaw = ((uint16_t)buf[17] << 8) + (uint16_t)buf[18];

      exp_lin = buf[19];

    break;
  }
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


  Serial.begin(38400);
  Serial.println("Starting up");

	//Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs
  //Mask:                       Channel:    Signal:
  PCMSK2 |= (1 << PCINT18); //  1           roll
  PCMSK2 |= (1 << PCINT19); //  2           pitch
  PCMSK2 |= (1 << PCINT20); //  3           throttle
  PCMSK2 |= (1 << PCINT21); //  4           yaw
  PCMSK2 |= (1 << PCINT22); //  5           ACRO/HORIZON
  PCMSK2 |= (1 << PCINT23); //  6           -

  radio_off_counter = 0;
  receiver_input_channel_1 = 1500;
  receiver_input_channel_2 = 1500;
  receiver_input_channel_3 = 1000;
  receiver_input_channel_4 = 1500;
  receiver_input_channel_5 = 1000;
  receiver_input_channel_6 = 1000;

  //retrieve pid parameters from eeprom
  #ifndef TUNING_MODE
    EEPROM_get();
    /*Serial.print(P_pitch_a); Serial.print("\t"); Serial.print(I_pitch_a); Serial.print("\t"); Serial.println(D_pitch_a);
    Serial.print(P_yaw); Serial.print("\t"); Serial.print(I_yaw); Serial.print("\t"); Serial.println(D_yaw);
    Serial.print(P_pitch_h); Serial.print("\t"); Serial.print(I_pitch_h); Serial.print("\t"); Serial.println(D_pitch_h);*/
    Serial.println("Retrieved PID parameters.");
  #else
    /*EEPROM_get();
    Serial.print(P_pitch_a); Serial.print("\t"); Serial.print(I_pitch_a); Serial.print("\t"); Serial.println(D_pitch_a);
    Serial.print(1.2); Serial.print("\t"); Serial.print(0.8); Serial.print("\t"); Serial.println(0.06);


    Serial.println();
    Serial.print(P_pitch_h); Serial.print("\t"); Serial.print(I_pitch_h); Serial.print("\t"); Serial.println(D_pitch_h);
    Serial.print(4.0); Serial.print("\t"); Serial.print(0.0); Serial.print("\t"); Serial.println(0.0);

    Serial.println();
    Serial.print(P_yaw); Serial.print("\t"); Serial.print(I_yaw); Serial.print("\t"); Serial.println(D_yaw);
    Serial.print(2.0); Serial.print("\t"); Serial.print(0.0); Serial.print("\t"); Serial.println(0.0);

*/


    P_pitch_a = 1.2;
    P_roll_a = 1.2;

    I_pitch_a = 0.8;
    I_roll_a = 0.8;

    D_pitch_a = 0.06;
    D_roll_a = 0.06;

    P_pitch_h = 4.0;
    P_roll_h = 4.0;

    P_yaw = 2.0;
    I_yaw = 0.01;

    //EEPROM_put();
  #endif


  imu.init();
	

  pid_pitch_rate.init();
  pid_pitch_rate.set_constants(P_pitch_a, I_pitch_a, D_pitch_a, INTEGRAL_MAX);
  pid_roll_rate.init();
  pid_roll_rate.set_constants(P_roll_a, I_roll_a, D_roll_a, INTEGRAL_MAX);
  pid_yaw_rate.init();
  pid_yaw_rate.set_constants(P_yaw, I_yaw, D_yaw, INTEGRAL_MAX);

  pid_pitch_stab.init();
  pid_pitch_stab.set_constants(P_pitch_h, I_pitch_h, D_pitch_h, INTEGRAL_MAX);
  pid_roll_stab.init();
  pid_roll_stab.set_constants(P_roll_h, I_roll_h, D_roll_h, INTEGRAL_MAX);

  front = 0;
  left = 0;
  cw = 0;


  count = 0;

  //wait for esc init;
  Serial.println("Initializing ESCs...");
  //asign motors to pins

  DDRB |= B00001111; //set pins as outputs

  //initialize escs
  init_motors();
  motors_on = false;
  //motors_on = true;


  time_last = micros();
  Serial.println("Running...");

  PCICR |= (1 << PCIE2);    // set PCIE2 to enable PCMSK0 scan
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
void update_horizon(uint32_t t){
  timed = (double)t/1000000.0;

  //update all sensor on the imu
  imu.update_horizon(timed); 

  //start esc pulses
  start_time = micros(); 
  end_time = start_time + 2000;
  PORTB |= B00001111;


  //compensate for mistimings with deadband
  if (receiver_input_channel_1 > 1485 && receiver_input_channel_1 < 1515) receiver_input_channel_1 = 1500;
  if (receiver_input_channel_2 > 1485 && receiver_input_channel_2 < 1515) receiver_input_channel_2 = 1500;
  if (receiver_input_channel_4 > 1485 && receiver_input_channel_4 < 1515) receiver_input_channel_4 = 1500;

  //map inputs to angles
  rad_roll = map_d((double)receiver_input_channel_1,1000.0, 2000.0, -REF_MAX_HORIZON, REF_MAX_HORIZON);
  rad_pitch = map_d((double)receiver_input_channel_2,1000.0, 2000.0, -REF_MAX_HORIZON, REF_MAX_HORIZON);
  rad_yaw = map_d((double)receiver_input_channel_4,1000.0, 2000.0, -REF_MAX_ACRO, REF_MAX_ACRO);

  //calculate pids
                                                                          //may need to calibrate for offsets
  pid_pitch_stab.update(&pitch_stab, rad_pitch, (imu.ypr[1]-2.5), timed, 1.0);  //(imu.ypr[1]-2.7)
  pid_roll_stab.update(&roll_stab, rad_roll, (imu.ypr[2]-0.2), timed, 1.0);     //(imu.ypr[2]-0.3)

  pid_pitch_rate.update(&front, (double)pitch_stab, imu.y_gyr*RAD_TO_DEG, timed, -1.0);
  pid_roll_rate.update(&left, (double)roll_stab, imu.x_gyr*RAD_TO_DEG, timed, 1.0);
  pid_yaw_rate.update(&cw, rad_yaw, -imu.z_gyr*RAD_TO_DEG, timed, -1.0);

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
void update_acro(uint32_t t){
  timed = (double)t/1000000.0;

  //update only the gyroscope on the imu
  imu.update_acro(timed); 

  //start esc pulses
  start_time = micros(); 
  end_time = start_time + 2000;
  PORTB |= B00001111;

  //compensate for mistimings with deadband
  if (receiver_input_channel_1 > 1475 && receiver_input_channel_1 < 1525) receiver_input_channel_1 = 1500;
  if (receiver_input_channel_2 > 1475 && receiver_input_channel_2 < 1525) receiver_input_channel_2 = 1500;
  if (receiver_input_channel_4 > 1475 && receiver_input_channel_4 < 1525) receiver_input_channel_4 = 1500;

  //p.K_tmp = map_d((double)receiver_input_channel_6,1000.0, 2000.0, 0.0, 0.8);

  //map inputs to anglerates
  rad_roll = map_x3((double)(receiver_input_channel_1-1500));
  rad_pitch = map_x3((double)(receiver_input_channel_2-1500));
  rad_yaw = map_d((double)receiver_input_channel_4,1000.0, 2000.0, -REF_MAX_ACRO, REF_MAX_ACRO);
  //rad_yaw = map_x3((double)(receiver_input_channel_4-1500));
  
  //Serial.println(rad_roll); delay(50);


  //calculate pids
  pid_pitch_rate.update(&front, rad_pitch, imu.y_gyr*RAD_TO_DEG, timed, -1.0);
  pid_roll_rate.update(&left, rad_roll, imu.x_gyr*RAD_TO_DEG, timed, 1.0);
  pid_yaw_rate.update(&cw, rad_yaw, -imu.z_gyr*RAD_TO_DEG, timed, -1.0);
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
void loop(){

  if (digitalRead(12) == LOW){
    delay(2000);
    send_data();
    set_motor_speeds_min();
    delay(2000);

    while(digitalRead(12) == LOW){
      get_data();
      set_motor_speeds_min();
      delay(2000);
    }
    EEPROM_put();
    pid_pitch_rate.set_constants(P_pitch_a, I_pitch_a, D_pitch_a, INTEGRAL_MAX);
    pid_roll_rate.set_constants(P_roll_a, I_roll_a, D_roll_a, INTEGRAL_MAX);
    pid_yaw_rate.set_constants(P_yaw, I_yaw, D_yaw, INTEGRAL_MAX);

    pid_pitch_stab.set_constants(P_pitch_h, I_pitch_h, D_pitch_h, INTEGRAL_MAX);
    pid_roll_stab.set_constants(P_roll_h, I_roll_h, D_roll_h, INTEGRAL_MAX);

  }else{

    time_diff = micros() - time_last;
    time_last = micros();

    rad_throttle = map(receiver_input_channel_3, 1000, 2000, 1060, 1800);

    //motor arming control
    if (receiver_input_channel_6 < 1300){
      motors_on = false;
    }else if (receiver_input_channel_6 > 1700){
      motors_on = true;
    }


    //horizon stabilization or acrobatic mode
    if (receiver_input_channel_5 < 1300){
      flight_mode = MODE_HORIZON;    
    }else if(receiver_input_channel_5 > 1700){
      flight_mode = MODE_ACRO;
    }

    #ifdef TUNING_MODE

      tun = map_d((double)receiver_input_channel_6, 1000.0, 2000.0, TUNING_MIN, TUNING_MAX);


      switch (TUNING_MODE) {
          case 0:            
            pid_pitch_rate.set_constants(tun, I_pitch_a, D_pitch_a, INTEGRAL_MAX);
            pid_roll_rate.set_constants(tun, I_pitch_a, D_pitch_a, INTEGRAL_MAX);
            break;
          case 1:
            pid_pitch_rate.set_constants(P_pitch_a, tun, D_pitch_a, INTEGRAL_MAX);
            pid_roll_rate.set_constants(P_pitch_a, tun, D_pitch_a, INTEGRAL_MAX);
            break;
          case 2:
            pid_pitch_rate.set_constants(P_pitch_a, I_pitch_a, tun, INTEGRAL_MAX);
            pid_roll_rate.set_constants(P_pitch_a, I_pitch_a, tun, INTEGRAL_MAX);
            break;
          case 3:
            pid_yaw_rate.set_constants(tun, I_yaw, D_yaw, INTEGRAL_MAX);
            break;
          case 4:
            pid_yaw_rate.set_constants(P_yaw, tun, D_yaw, INTEGRAL_MAX);
            break;
          case 5:
            pid_yaw_rate.set_constants(P_yaw, I_yaw, tun, INTEGRAL_MAX);
            break;
          case 6:
            pid_pitch_stab.set_constants(tun, I_pitch_h, D_pitch_h, INTEGRAL_MAX);
            pid_roll_stab.set_constants(tun, I_roll_h, D_roll_h, INTEGRAL_MAX);
            break;
          case 7:
            pid_pitch_stab.set_constants(P_pitch_h, tun, D_pitch_h, INTEGRAL_MAX);
            pid_roll_stab.set_constants(P_roll_h, tun, D_roll_h, INTEGRAL_MAX);
            break;
          case 8:
            pid_pitch_stab.set_constants(P_pitch_h, I_pitch_h, tun, INTEGRAL_MAX);
            pid_roll_stab.set_constants(P_roll_h, I_roll_h, tun, INTEGRAL_MAX);
            break;

      }

      Serial.println(tun); delay(500);

    #endif
    

    if (motors_on){
      //update according to set flight mode
      if(flight_mode == MODE_HORIZON){
        update_horizon(time_diff);
      }else if (flight_mode == MODE_ACRO){
        update_acro(time_diff);
      }    
      //apply new speed to motors
      set_motor_speeds();
    }else{
      //keep motors updated
      set_motor_speeds_min();
    }
  }
  //delay(10);
  //Serial.println(imu.ypr[1]);

  /*delay(25);
  Serial.print(receiver_input_channel_1);
  Serial.print("\t\t");

  Serial.print(receiver_input_channel_2);
  Serial.print("\t\t");

  Serial.print(receiver_input_channel_3);
  Serial.print("\t\t");

  Serial.print(receiver_input_channel_4);
  Serial.print("\t\t");

  Serial.print(receiver_input_channel_5);
  Serial.print("\t\t");

  Serial.print(receiver_input_channel_6);
  Serial.println("\t\t");*/
} 


/*
 #####    ##   ####  #### ####
 ##  ##  ####  ## ##  ## ##  ##
 ##  ## ##  ## ##  ## ## ##  ##
 #####  ###### ##  ## ## ##  ##
 ####   ##  ## ##  ## ## ##  ##
 ## ##  ##  ## ## ##  ## ##  ##
 ##  ## ##  ## ####  #### ####
*/
//This routine is called every time input 8, 9, 10 or 11 changed state
ISR(PCINT2_vect){
  //read micros
  us = micros();

  //Channel 1=========================================
  if(last_channel_1 == 0 && PIND & B00000100 ){         //Input 8 changed from 0 to 1
    last_channel_1 = 1;                                 //Remember current input state
    timer_1 = us;                                 //Set timer_1 to micros()
  }
  else if(last_channel_1 == 1 && !(PIND & B00000100)){  //Input 8 changed from 1 to 0
    last_channel_1 = 0;                                 //Remember current input state
    receiver_input_channel_1 = us - timer_1;      //Channel 1 is micros() - timer_1
  }

  //Channel 2=========================================
  if(last_channel_2 == 0 && PIND & B00001000 ){         //Input 9 changed from 0 to 1
    last_channel_2 = 1;                                 //Remember current input state
    timer_2 = us;                                 //Set timer_2 to micros()
  }
  else if(last_channel_2 == 1 && !(PIND & B00001000)){  //Input 9 changed from 1 to 0
    last_channel_2 = 0;                                 //Remember current input state
    receiver_input_channel_2 = us - timer_2;      //Channel 2 is micros() - timer_2
  }

  //Channel 3=========================================
  if(last_channel_3 == 0 && PIND & B00010000 ){         //Input 10 changed from 0 to 1
    last_channel_3 = 1;                                 //Remember current input state
    timer_3 = us;                                 //Set timer_3 to micros()
  }
  else if(last_channel_3 == 1 && !(PIND & B00010000)){  //Input 10 changed from 1 to 0
    last_channel_3 = 0;                                 //Remember current input state
    receiver_input_channel_3 = us - timer_3;      //Channel 3 is micros() - timer_3
  }

  //Channel 4=========================================
  if(last_channel_4 == 0 && PIND & B00100000 ){         //Input 11 changed from 0 to 1
    last_channel_4 = 1;                                 //Remember current input state
    timer_4 = us;                                 //Set timer_4 to micros()
  }
  else if(last_channel_4 == 1 && !(PIND & B00100000)){  //Input 11 changed from 1 to 0
    last_channel_4 = 0;                                 //Remember current input state
    receiver_input_channel_4 = us - timer_4;      //Channel 4 is micros() - timer_4
  }

  //Channel 5=========================================
  if(last_channel_5 == 0 && PIND & B01000000 ){         //Input 10 changed from 0 to 1
    last_channel_5 = 1;                                 //Remember current input state
    timer_5 = us;                                 //Set timer_3 to micros()
  }
  else if(last_channel_5 == 1 && !(PIND & B01000000)){  //Input 10 changed from 1 to 0
    last_channel_5 = 0;                                 //Remember current input state
    receiver_input_channel_5 = us - timer_5;      //Channel 3 is micros() - timer_3
  }

  //Channel 6=========================================
  if(last_channel_6 == 0 && PIND & B10000000 ){         //Input 11 changed from 0 to 1
    last_channel_6 = 1;                                 //Remember current input state
    timer_6 = us;                                 //Set timer_4 to micros()
  }
  else if(last_channel_6 == 1 && !(PIND & B10000000)){  //Input 11 changed from 1 to 0
    last_channel_6 = 0;                                 //Remember current input state
    receiver_input_channel_6 = us - timer_6;      //Channel 4 is micros() - timer_4
  }
}


