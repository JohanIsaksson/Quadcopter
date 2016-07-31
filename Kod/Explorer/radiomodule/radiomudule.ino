#pragma GCC optimize("-O3")

#include "Wire.h"
#include "I2Cdev.h"
#include "kalman.h"

#define LOST_CONNECTION_COUNT 200
#define EMERGENCY_THROTTLE 0

#define MODE_HORIZON 0
#define MODE_ACRO 1


#define FLAG_OTHER 3
#define FLAG_M 4
#define FLAG_OVERHEAD 5



//safety stuff
uint8_t radio_off_counter;
bool radio_lost;

//radio variables
byte last_channel_1, 
      last_channel_2, 
      last_channel_3, 
      last_channel_4,
      last_channel_5, 
      last_channel_6;

uint16_t receiver_input_channel_1, 
    receiver_input_channel_2, 
    receiver_input_channel_3, 
    receiver_input_channel_4, 
    receiver_input_channel_5, 
    receiver_input_channel_6;

uint32_t us, timer_tmp,
					timer_1, 
          timer_2, 
          timer_3, 
          timer_4,
          timer_5,
          timer_6;

uint16_t receiver_roll,
          receiver_pitch,
          receiver_throttle,
          receiver_yaw;
;

//motor control variables
bool motors_on, disable_sticks;

//flight mode control
uint8_t flight_mode;

//I2C stuff
uint8_t I2C_cur;
uint8_t I2C_buffer[2];

//altitude parameters
double baro_height, acc_vertical;
int8_t altitude_throttle;
bool measurement_ready;
Kalman k;

//time keeping
uint32_t time_diff;
uint32_t time_last;



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

  I2C_cur = 0;

  //wait for esc init;
  Serial.println("Initializing ESCs...");

  //initialize escs
  motors_on = false;

  //init safety parameters
  radio_off_counter = 0;
  radio_lost = false;

  k.init();

  Serial.println("Running...");

  Wire.begin(8);                // join i2c bus with address #8
   
  Wire.onRequest(send_data); // register i2c events
  Wire.onReceive(get_data);

  PCICR |= (1 << PCIE2);    // set PCIE2 to enable PCMSK0 scan
}



// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void send_data() {

  I2C_cur++;

  uint8_t I2C_buffer[2] = {0,0};

  I2C_buffer[1] = (I2C_cur << FLAG_OVERHEAD);
  I2C_buffer[1] += (motors_on << FLAG_M) + (1 << FLAG_OTHER) + flight_mode;





  switch(I2C_cur) {
    case 1:
      I2C_buffer[0] = (receiver_roll - 1000) >> 2;    
    break;

    case 2:
      I2C_buffer[0] = (receiver_pitch - 1000) >> 2;
    break;

    case 3:
      I2C_buffer[0] = (receiver_throttle - 1000) >> 2;
    break;

    case 4:
      I2C_buffer[0] = (receiver_yaw - 1000) >> 2;
    break;

    case 5:
      I2C_buffer[0] = (receiver_input_channel_6 - 1000) >> 2;
      I2C_cur = 0;
    break;

    default:
      I2C_cur = 0;

    //I2C_buffer[10] = altitude_throttle;

  }

  Wire.write(I2C_buffer, 2);
}


void get_data(int n){
  
  uint8_t I2C_buffer[3] = {0,0,0};
  int i = 0;
  while (Wire.available()) { // slave may send less than requested
    I2C_buffer[i] = Wire.read(); // receive a byte
    i++;
  }

  uint16_t data = I2C_buffer[0];// something wrong here
  
  data = data << 8;
  data += I2C_buffer[1];


  baro_height = ((double)((int16_t)data))/100.0; //convert to metres
  Serial.println(data);

  
  acc_vertical = ((double)((int8_t)I2C_buffer[2]))/32.0; // +-4g max
  measurement_ready = true;
}



void loop(){

  time_diff = micros() - time_last;
  time_last = micros();

	//if (receiver_input_channel_3 < 1400) rad_throttle = 1000; //calibration purposes
	//if (receiver_input_channel_3 > 1600) rad_throttle = 2000;

  if (measurement_ready){
    //uint32_t a = micros();
    k.update(baro_height, acc_vertical, (double)time_diff/1000000.0);
    //Serial.println(micros() - a);
    measurement_ready = false;
    Serial.println(k.get_altitude(),10);
  }

  //check if we lost contact with radio
  radio_off_counter++;
  radio_lost = (radio_off_counter > LOST_CONNECTION_COUNT);

  //disable controls if throttle is low
  if (receiver_input_channel_3 < 1125){
    disable_sticks = true;
  }else{
    disable_sticks = false;
  }

  if (disable_sticks){
    receiver_roll = 1500;
    receiver_pitch = 1500;
    receiver_throttle = 1000;
    receiver_yaw = 1500;
  }else{
    receiver_roll = receiver_input_channel_1;
    receiver_pitch = receiver_input_channel_2;
    receiver_throttle = map(receiver_input_channel_3, 1108, 1876, 1000, 2000);
    receiver_yaw = receiver_input_channel_4;
  }


  //horizon stabilization or acrobatic mode
  if (receiver_input_channel_5 < 1300){
    flight_mode = MODE_HORIZON;    
  }else if(receiver_input_channel_5 > 1600){
    flight_mode = MODE_ACRO;
  }
  

  if (motors_on && !radio_lost){
    //change on/off state
    if (disable_sticks && receiver_input_channel_4 > 1710){
      motors_on = false;
    }  
    
  }else{
    //change on/off
    if (disable_sticks && receiver_input_channel_4 < 1290 && !radio_lost){
      motors_on = true;  
    }else{
      motors_on = false;
    }

  }

  //Serial.println(receiver_input_channel_3);
  //Serial.print("\t");
  //Serial.println(radio_off_counter);
  
  while(micros() - time_last < 2000);

} 



//This routine is called every time input 8, 9, 10 or 11 changed state
ISR(PCINT2_vect){

  //reset safety counter
  radio_off_counter = 0;

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
  if(last_channel_5 == 0 && PIND & B01000000 ){         //Input 12 changed from 0 to 1
    last_channel_5 = 1;                                 //Remember current input state
    timer_5 = us;                                 //Set timer_5 to micros()
  }
  else if(last_channel_5 == 1 && !(PIND & B01000000)){  //Input 12 changed from 1 to 0
    last_channel_5 = 0;                                 //Remember current input state
    receiver_input_channel_5 = us - timer_5;      //Channel 5 is micros() - timer_5
  }

  //Channel 6=========================================
  if(last_channel_6 == 0 && PIND & B10000000 ){         //Input 13 changed from 0 to 1
    last_channel_6 = 1;                                 //Remember current input state
    timer_6 = us;                                 //Set timer_6 to micros()
  }
  else if(last_channel_6 == 1 && !(PIND & B10000000)){  //Input 13 changed from 1 to 0
    last_channel_6 = 0;                                 //Remember current input state
    receiver_input_channel_6 = us - timer_6;      //Channel 6 is micros() - timer_6
  }
}


