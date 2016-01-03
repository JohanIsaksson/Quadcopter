#pragma GCC optimize("-O3")

#include "imu.h"
#include "pid.h"

#define SPEED_MIN 1250
#define SPEED_MAX 1750

#define REF_MAX_HORIZON 20.0
#define REF_MAX_ACRO 90.0
#define REF_MAX_YAW 90.0

#define RADIO_THROTTLE 5 //6 in future

#define LOST_CONNECTION_COUNT 100
#define EMERGENCY_THROTTLE 0

#define MODE_HORIZON 0
#define MODE_ACRO 1


//#define YPR_DATA
//#define COMP_DATA
#define PID_DATA
//#define MAG_DATA
//#define JAVA_DATA
//#define RADIO_DATA
//#define ESC_DATA

imu im;
pid p;

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

//output
int count;

//safety stuff
int radio_off_counter;

//time keeping
uint32_t time_diff;
uint32_t time_last;

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
    end_time = start_time + 1750;
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
  end_time = start_time + 1750;
  PORTB |= B00001111;

  esc_time = micros();

  while(end_time > esc_time){                         //Start the pulse after 1250 micro seconds.
    if (esc_time - start_time >= throttle[0]) PORTB &= B11111110; //front left
    if (esc_time - start_time >= throttle[1]) PORTB &= B11111101; //front left
    if (esc_time - start_time >= throttle[2]) PORTB &= B11111011; //front left
    if (esc_time - start_time >= throttle[3]) PORTB &= B11110111; //front left
    esc_time = micros();
  }
  PORTB &= B11110000; //turn all pulses off for safety

    
  while(micros() - time_last < 4500);
}

void setup(){


  Serial.begin(38400);
  Serial.println("Starting up");

	imu_init(&im);

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
  receiver_input_channel_3 = 1250;
  receiver_input_channel_4 = 1500;
  receiver_input_channel_5 = 1000;
  receiver_input_channel_6 = 1000;

	

  init_pid(&p);

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


  time_last = micros();
  Serial.println("Running...");

  PCICR |= (1 << PCIE2);    // set PCIE2 to enable PCMSK0 scan
}

void update_horizon(uint32_t t){

  //update all sensor on the imu
  imu_update_horizon(&im, t); 

  //start esc pulses
  start_time = micros(); 
  end_time = start_time + 1750;
  PORTB |= B00001111;

  /* calculate pid */
  //pid_pitch(&p, &front, im.ypr[1], 0.0);
  //pid_roll(&p, &left, im.ypr[2], 0.0);
  //pid_yaw_temp(&p, &cw, -im.z_gyr*RAD_TO_DEG, 0.0);


  //p.K_D_yaw = map_d((double)receiver_input_channel_5, 975.0, 2000.0, 0.0, 0.1);
  //p.K_P_yaw = map_d((double)receiver_input_channel_6, 975.0, 2000.0, 0.0, 1.5);

  rad_throttle = receiver_input_channel_3;
  //map inputs to angles
  if (!disable_sticks){
    rad_roll = map_d((double)receiver_input_channel_1,1250.0, 1750.0, -REF_MAX_HORIZON, REF_MAX_HORIZON);
    rad_pitch = map_d((double)receiver_input_channel_2,1250.0, 1750.0, -REF_MAX_HORIZON, REF_MAX_HORIZON);
    rad_yaw = map_d((double)receiver_input_channel_4,1250.0, 1750.0, -REF_MAX_ACRO, REF_MAX_ACRO);
  }else{
    rad_roll = 0.0;
    rad_pitch = 0.0;
    rad_yaw = 0.0;
  }
  //calculate pids
  pid_pitch(&p, &front, im.ypr[1], rad_pitch, t);
  pid_roll(&p, &left, im.ypr[2], rad_roll, t);
  pid_yaw_temp(&p, &cw, -im.z_gyr*RAD_TO_DEG, rad_yaw);
}

void update_acro(uint32_t t){

  //update only the gyroscope on the imu
  imu_update_acro(&im, t); 

  //start esc pulses
  start_time = micros(); 
  end_time = start_time + 1750;
  PORTB |= B00001111;

  /* calculate pid */
  //pid_pitch(&p, &front, im.ypr[1], 0.0);
  //pid_roll(&p, &left, im.ypr[2], 0.0);
  //pid_yaw_temp(&p, &cw, -im.z_gyr*RAD_TO_DEG, 0.0);

  //p.K_D_yaw = map_d((double)receiver_input_channel_5, 975.0, 2000.0, 0.0, 0.1);
  //p.K_P_yaw = map_d((double)receiver_input_channel_6, 975.0, 2000.0, 0.0, 1.5);

  //compensate for mistimings
  if (receiver_input_channel_1 > 1485 && receiver_input_channel_1 < 1515) receiver_input_channel_1 = 1500;
  if (receiver_input_channel_2 > 1485 && receiver_input_channel_2 < 1515) receiver_input_channel_2 = 1500;
  if (receiver_input_channel_4 > 1485 && receiver_input_channel_4 < 1515) receiver_input_channel_4 = 1500;

  rad_throttle = receiver_input_channel_3;
  p.K_tmp = map_d((double)receiver_input_channel_6,1000.0, 2000.0, 0.0, 4.0);

  //map inputs to anglerates
  if (!disable_sticks){
    rad_roll = map_d((double)receiver_input_channel_1,1250.0, 1750.0, -REF_MAX_ACRO, REF_MAX_ACRO);
    rad_pitch = map_d((double)receiver_input_channel_2,1250.0, 1750.0, -REF_MAX_ACRO, REF_MAX_ACRO);
    rad_yaw = map_d((double)receiver_input_channel_4,1250.0, 1750.0, -REF_MAX_YAW, REF_MAX_YAW);
  }else{
    rad_roll = 0.0;
    rad_pitch = 0.0;
    rad_yaw = 0.0;
  }
  //calculate pids
  pid_pitch_rate(&p, &front, im.y_gyr*RAD_TO_DEG, rad_pitch, t); //y_gyr may need to be reversed
  pid_roll_rate(&p, &left, im.x_gyr*RAD_TO_DEG, rad_roll, t);    //x_gyr may need to be reversed
  pid_yaw_temp(&p, &cw, -im.z_gyr*RAD_TO_DEG, rad_yaw);
}

void loop(){
  time_diff = micros() - time_last;
  time_last = micros();

  //disable joysticks if low throttle
  if (rad_throttle < 1270){
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
    if (disable_sticks && receiver_input_channel_4 < 1290){
      motors_on = true;  
    }
    //keep motors updated
    set_motor_speeds_min();
  }

  print_data(time_diff);
} 

/* DEBUG DATA OUTPUT */
void print_data(uint32_t time){ 

  count++;
  if (count == 40){

    #ifdef ESC_DATA

      //Serial.print(esc_time - start_time); 
      Serial.print("\t");
      Serial.print(throttle[0]); 
      Serial.print("\t");


    #endif


    #ifdef YPR_DATA
      /*
      Serial.print(im.ypr_rad[0]);
      Serial.print("\t");
      Serial.print(im.ypr_rad[1]);
      Serial.print("\t");
      Serial.print(im.ypr_rad[2]);
      Serial.print("\t");*/
      Serial.print(im.ypr[0]);
      Serial.print(",");
      Serial.print(im.ypr[1]);
      Serial.print(",");
      Serial.print(im.ypr[2]);
    #endif

    #ifdef RADIO_DATA
      Serial.print(receiver_input_channel_1, DEC);
      Serial.print(",");
      Serial.print(receiver_input_channel_2, DEC);
      Serial.print(",");
      Serial.print(receiver_input_channel_3, DEC);
      Serial.print(",");
      Serial.print(receiver_input_channel_4, DEC);
      Serial.print(",");
      Serial.print(receiver_input_channel_5, DEC);
      Serial.print(",");
      Serial.print(receiver_input_channel_6, DEC);
    #endif


    #ifdef PID_DATA
      Serial.print(p.K_tmp,5);
      Serial.print(",");

    #endif



    #ifdef COMP_DATA
      Serial.print(im.ypr[0]); 
      Serial.print("\t");

      Serial.print(im.ypr[1]);
      Serial.print("\t");

      Serial.print(im.ypr[2]);
      Serial.print("\t");

      Serial.print(rad_yaw);
      Serial.print("\t");

      Serial.print(rad_pitch);
      Serial.print("\t");

      Serial.print(rad_roll);
      Serial.print("\t");
      Serial.print("\t");

      for (int i = 0; i < 4; i++){      
        Serial.print(throttle[i]);
        Serial.print("\t");
      }
      

    #endif


    #ifdef MAG_DATA
      Serial.print("xyz:\t");
      Serial.print(im.mx);
      Serial.print(",");
      Serial.print(im.my);
      Serial.print(",");
      Serial.print(im.mz);
    #endif

    #ifdef JAVA_DATA
      Serial.print(im.ypr[0]);
      Serial.print(",");
      Serial.print(rad_yaw);
      Serial.print(",");

      Serial.print(im.ypr[1]);
      Serial.print(",");
      Serial.print(rad_pitch);
      Serial.print(",");

      Serial.print(im.ypr[2]);
      Serial.print(",");
      Serial.print(rad_roll);
      Serial.print(",");

      for (int i = 0; i < 4; i++){      
        Serial.print(throttle[i]);
        Serial.print(",");
      }

      Serial.print(im.height);
      Serial.print(",");
      Serial.print(im.vertical_speed);
      Serial.print(",");
      Serial.print(im.vertical_acc);
      Serial.print(",");
      
      Serial.print(p.pitch_p);
      Serial.print(",");
      Serial.print(p.pitch_i);
      Serial.print(",");
      Serial.print(p.pitch_d);
      //Serial.print(",");


      /*Serial.print(",");

      Serial.print(",");

      Serial.print(",");

      Serial.print(",");*/



    #endif

    Serial.print(",");
    Serial.println(time);

    if (flight_mode == MODE_HORIZON){
      Serial.print("horizon");
    }else{
      Serial.println("acro");
    }



    count = 0;
  }
}

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


