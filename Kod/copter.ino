#include "imu.h"
#include "radio.h"
#include <ServoTimer2.h>
#include "pid.h"

#define SPEED_MIN 1500
#define SPEED_MAX 2000

#define YPR_DATA
//#define COMP_DATA
//#define PID_DATA

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

//test
int count;

void setup(){

  Serial.begin(38400);

	bool b = init_imu(&im);

	init_radio(&rad);

	//asign motors to pins
  motors[0].attach(3);
  motors[1].attach(4);
  motors[2].attach(5);
  motors[3].attach(6);

  //set init speed to motors
  for (int i = 0; i < 4; i++){
    throttle[i] = 0;
    motors[i].write(1500);
  }

  init_pid(&p);

  count = 0;

  //wait for esc init;
  delay(4000);
}


void loop(){

  delay(1);

  read_imu(&im);
  read_message(&rad);


  /* calculate pid */
  pid_pitch(&p, &front, &back, im.ypr[1], -(double)rad.buffer[3]);
  pid_roll(&p, &left, &right, im.ypr[2], (double)rad.buffer[2]);
  pid_yaw(&p, &cw, &ccw, im.ypr[0], (int)((((uint16_t)rad.buffer[4]) << 8) + (uint8_t)rad.buffer[5]);

  /* calculate final motor speed */
  //front left
  throttle[0] = (uint8_t)rad.buffer[6] + front + left + cw;

  //front right
  throttle[1] = (uint8_t)rad.buffer[6] + front + right + ccw;

  //back left
  throttle[2] = (uint8_t)rad.buffer[6] + back + left + ccw;

  //back right
  throttle[3] = (uint8_t)rad.buffer[6] + back + right + cw;

  //check and set speeds
  for (int i = 0; i < 4; i++){
    if (throttle[i] > 255){
      throttle[i] = 255;
    }else if (throttle[i] < 0){
      throttle[i] = 0;
    }
    //motors[i].write(map(throttle[i], 0, 255, SPEED_MIN, SPEED_MAX));
  }
  count++;
  if (count == 50){
    #ifdef PID_DATA
      for (int i = 0; i < 4; i++){      
        Serial.print(throttle[i]);
        Serial.print("\t");
      }
    #endif

    #ifdef YPR_DATA
      Serial.print(im.ypr[0]);
      Serial.print("\t");
      Serial.print(im.ypr[1]);
      Serial.print("\t");
      Serial.println(im.ypr[2]);
    #endif



    #ifdef COMP_DATA
      Serial.print("throttle: ");
      for (int i = 0; i < 4; i++){      
        Serial.print(throttle[i]);
        Serial.print("\t");
      }

      Serial.print("\tRoll: ");
      Serial.print(im.ypr[2]);
      Serial.print("\t");
      Serial.print(rad.buffer[2]);
      Serial.print("\t");

      Serial.print("\tPitch: ");
      Serial.print(im.ypr[1]);
      Serial.print("\t");
      Serial.print(rad.buffer[3]);
      Serial.print("\t");

      
      Serial.println("");
    #endif
    count = 0;
  }
}
