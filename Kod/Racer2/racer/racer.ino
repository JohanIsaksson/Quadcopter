#pragma GCC optimize("-O3")

#include "imu.h"
#include "pid.h"
#include <EEPROM.h>

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

IMU imu();

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
  receiver_input_channel_3 = 1108;
  receiver_input_channel_4 = 1500;
  receiver_input_channel_5 = 1000;
  receiver_input_channel_6 = 1000;

  //retrieve pid parameters from eeprom
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

  //update all sensor on the imu
  imu_update_horizon(&im, t); 

  //start esc pulses
  start_time = micros(); 
  end_time = start_time + 2000;
  PORTB |= B00001111;

  //p.K_D_yaw = map_d((double)receiver_input_channel_5, 975.0, 2000.0, 0.0, 0.1);
  //p.K_P_yaw = map_d((double)receiver_input_channel_6, 975.0, 2000.0, 0.0, 1.5);

  //map inputs to angles
  if (!disable_sticks){
    rad_roll = map_d((double)receiver_input_channel_1,1000.0, 2000.0, -REF_MAX_HORIZON, REF_MAX_HORIZON);
    rad_pitch = map_d((double)receiver_input_channel_2,1000.0, 2000.0, -REF_MAX_HORIZON, REF_MAX_HORIZON);
    rad_yaw = map_d((double)receiver_input_channel_4,1000.0, 2000.0, -REF_MAX_ACRO, REF_MAX_ACRO);
  }else{
    rad_roll = 0.0;
    rad_pitch = 0.0;
    rad_yaw = 0.0;
  }
  //calculate pids
  timed = (double)t/1000000.0;

  pid_pitch_stab.update(&pitch_stab, rad_pitch, im.ypr[1], timed, 1.0);
  pid_roll_stab.update(&roll_stab, rad_roll, im.ypr[1], timed, 1.0);

  pid_pitch_rate.update(&front, (double)pitch_stab, im.y_gyr*RAD_TO_DEG, timed, -1.0);
  pid_roll_rate.update(&left, (double)roll_stab, im.x_gyr*RAD_TO_DEG, timed, 1.0);
  pid_yaw_rate.update(&cw, rad_yaw, -im.z_gyr*RAD_TO_DEG, timed, -1.0);

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

  //update only the gyroscope on the imu
  imu_update_acro(&im, t); 

  //start esc pulses
  start_time = micros(); 
  end_time = start_time + 2000;
  PORTB |= B00001111;

  //p.K_D_yaw = map_d((double)receiver_input_channel_5, 975.0, 2000.0, 0.0, 0.1);
  //p.K_P_yaw = map_d((double)receiver_input_channel_6, 975.0, 2000.0, 0.0, 1.5);

  //compensate for mistimings
  if (receiver_input_channel_1 > 1485 && receiver_input_channel_1 < 1515) receiver_input_channel_1 = 1500;
  if (receiver_input_channel_2 > 1485 && receiver_input_channel_2 < 1515) receiver_input_channel_2 = 1500;
  if (receiver_input_channel_4 > 1485 && receiver_input_channel_4 < 1515) receiver_input_channel_4 = 1500;

  //p.K_tmp = map_d((double)receiver_input_channel_6,1000.0, 2000.0, 0.0, 0.8);

  //map inputs to anglerates
  if (!disable_sticks){
    rad_roll = map_d((double)receiver_input_channel_1,1000.0, 2000.0, -REF_MAX_ACRO, REF_MAX_ACRO);
    rad_pitch = map_d((double)receiver_input_channel_2,1000.0, 2000.0, -REF_MAX_ACRO, REF_MAX_ACRO);
    rad_yaw = map_d((double)receiver_input_channel_4,1000.0, 2000.0, -REF_MAX_YAW, REF_MAX_YAW);
  }else{
    rad_roll = 0.0;
    rad_pitch = 0.0;
    rad_yaw = 0.0;
  }
  //calculate pids
  timed = (double)t/1000000.0;
  pid_pitch_rate.update(&front, rad_pitch, im.y_gyr*RAD_TO_DEG, timed, -1.0);
  pid_roll_rate.update(&left, rad_roll, im.x_gyr*RAD_TO_DEG, timed, 1.0);
  pid_yaw_rate.update(&cw, rad_yaw, -im.z_gyr*RAD_TO_DEG, timed, -1.0);
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

  time_diff = micros() - time_last;
  time_last = micros();

  rad_throttle = map(receiver_input_channel_3, 1108, 1876, 1000, 2000);

  //disable joysticks if low throttle
  if (rad_throttle < 1050){
    disable_sticks = true;
  }else{
    disable_sticks = false;
  }


  //horizon stabilization or acrobatic mode
  if (receiver_input_channel_5 < 1300){
    flight_mode = MODE_HORIZON;    
  }else if(receiver_input_channel_5 > 1600){
    flight_mode = MODE_ACRO;
  }
  

  if (motors_on){
    //change on/off state
    if (disable_sticks && receiver_input_channel_4 > 1710){
      motors_on = false;
    }
    //update according to set flight mode
    if(flight_mode == MODE_HORIZON){
      update_horizon(time_diff);
    }else if (flight_mode == MODE_ACRO){
      update_acro(time_diff);
    }    
    //apply new speed to motors
    set_motor_speeds();
  }else{
    //change on/off
    if (disable_sticks && receiver_input_channel_4 < 1150){
      motors_on = true;  
    }
    //keep motors updated
    set_motor_speeds_min();
  }

  //print_data(time_diff);
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


