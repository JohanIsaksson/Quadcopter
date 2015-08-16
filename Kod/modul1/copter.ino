#include "imu.h"
#include "radio.h"
#include <ServoTimer2.h>
#include "pid.h"

#define SPEED_MIN_FRONT 1500
#define SPEED_MAX_FRONT 2000
#define SPEED_MIN_BACK 1250
#define SPEED_MAX_BACK 1750

#define RADIO_THROTTLE 5 //6 in future

#define CALIBRATION_COMP 10

#define LOST_CONNECTION_COUNT 500
#define EMERGENCY_THROTTLE 150


//#define YPR_DATA
//#define COMP_DATA
#define PID_DATA
//#define MAG_DATA

radio rad;
imu im;
pid p;

//motors
ServoTimer2 motors[4];   
//0 = front left
//1 = front right
//2 = back left
//3 = back right

int throttle[4];

//pid
int front, back;
int left, right;
int cw, ccw;

//output
int count;

//safety stuff
int radio_off_counter;

//time keeping
uint32_t time_diff;
uint32_t time_last;




void setup(){


  Serial.begin(38400);

	bool b = init_imu(&im);

	init_radio(&rad);
  radio_off_counter = 0;

	//asign motors to pins
  motors[0].attach(3);
  motors[1].attach(4);
  motors[2].attach(5);
  motors[3].attach(6);

  //set init speed to motors
  motors[0].write(SPEED_MIN_FRONT);
  motors[1].write(SPEED_MIN_FRONT);
  motors[2].write(SPEED_MIN_BACK);
  motors[3].write(SPEED_MIN_BACK);

  init_pid(&p);

  front = 0;
  back = 0;
  left = 0;
  right = 0;
  cw = 0;
  ccw = 0;


  count = 0;

  //wait for esc init;
  delay(3000);

  time_last = millis();
}


void loop(){
  time_diff = millis() - time_last;
  time_last = millis();


  read_imu(&im, time_diff);
  
  //check for lost radio connetion
  if (!read_message(&rad)){    
    if (radio_off_counter >= LOST_CONNECTION_COUNT){
      rad.buffer[RADIO_THROTTLE] = EMERGENCY_THROTTLE;
    }else{
      radio_off_counter++;
    }

  }else{
    radio_off_counter = 0;
  }
  


  /* calculate pid */
  pid_pitch(&p, &front, &back, im.ypr[1], (double)rad.buffer[3]);
  pid_roll(&p, &left, &right, im.ypr[2], (double)rad.buffer[2]);
  pid_yaw(&p, &cw, &ccw, -im.z_gyr*RAD_TO_DEG, (double)rad.buffer[4]);
  //pid_yaw(&p, &cw, &ccw, im.ypr[0], (int)((((uint16_t)rad.buffer[4]) << 8) + (uint8_t)rad.buffer[5]);

  /* calculate final motor speed */
  //front left
  throttle[0] = (uint8_t)rad.buffer[RADIO_THROTTLE] + front + left + cw + CALIBRATION_COMP;

  //front right
  throttle[1] = (uint8_t)rad.buffer[RADIO_THROTTLE] + front + right + ccw + CALIBRATION_COMP;

  //back left
  throttle[2] = (uint8_t)rad.buffer[RADIO_THROTTLE] + back + left + ccw - CALIBRATION_COMP;

  //back right
  throttle[3] = (uint8_t)rad.buffer[RADIO_THROTTLE] + back + right + cw - CALIBRATION_COMP;

  //check and set speeds
  for (int i = 0; i < 4; i++){
    if (throttle[i] > 255){
      throttle[i] = 255;
    }else if (throttle[i] < 0){
      throttle[i] = 0;
    }         
  }
  motors[0].write(map(throttle[0], 0, 255, SPEED_MIN_FRONT, SPEED_MAX_FRONT));
  motors[1].write(map(throttle[0], 0, 255, SPEED_MIN_BACK, SPEED_MAX_BACK));
  motors[2].write(map(throttle[0], 0, 255, SPEED_MIN_BACK, SPEED_MAX_BACK));
  motors[3].write(map(throttle[0], 0, 255, SPEED_MIN_BACK, SPEED_MAX_BACK));














  count++;
  if (count == 8){

    Serial.print(time_diff);
    Serial.print("\t\t");

    #ifdef PID_DATA
      for (int i = 0; i < 4; i++){      
        Serial.print(throttle[i]);
        Serial.print("\t");
      }
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
      Serial.print("\t");
      Serial.print(im.ypr[1]);
      Serial.print("\t");
      Serial.print(im.ypr[2]);
    #endif



    #ifdef COMP_DATA
      Serial.print("Pitch:\t");
      Serial.print(im.ypr[1]);      

      Serial.print("\t");
      Serial.print(im.x_acc);
      Serial.print("\t");
      Serial.print(im.y_gyr);

      Serial.print("\t\tRoll\t");
      Serial.print(im.ypr[2]);
      Serial.print("\t");
      Serial.print(im.y_acc);
      Serial.print("\t");
      Serial.print(im.x_gyr);
/*
      Serial.print("\t\t");

      for (int i = 0; i < 4; i++){      
        Serial.print(throttle[i]);
        Serial.print("\t");
      }
      //Serial.println();

      Serial.print("\t\t");
      Serial.print(front);
      Serial.print("\t");
      Serial.println(left);
*/

    #endif


    #ifdef MAG_DATA
      Serial.print("xyz:\t");
      Serial.print(im.mx);
      Serial.print("\t");
      Serial.print(im.my);
      Serial.print("\t");
      Serial.print(im.mz);
    #endif


    Serial.println();



    count = 0;
  }
}
