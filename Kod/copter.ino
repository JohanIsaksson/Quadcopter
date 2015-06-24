#include "gyro.h"
#include "radio.h"
#include <ServoTimer2.h>
#include "pid.h"

#define SPEED_MIN 1500
#define SPEED_MAX 2000

radio rad;
gyro gyr;
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

//test
int count;

void setup(){

  Serial.begin(38400);

	bool b = init_gyro(&gyr);

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

  read_gyro(&gyr);
  read_message(&rad);


  /* calculate pid */
  pid_pitch(&p, &front, &back, gyr.ypr[1], -(double)rad.buffer[3]);
  pid_roll(&p, &left, &right, gyr.ypr[2], (double)rad.buffer[2]);

  /* calculate final motor speed */
  //front left
  throttle[0] = (uint8_t)rad.buffer[5] + front + left;

  //front right
  throttle[1] = (uint8_t)rad.buffer[5] + front + right;

  //back left
  throttle[2] = (uint8_t)rad.buffer[5] + back + left;

  //back right
  throttle[3] = (uint8_t)rad.buffer[5] + back + right;

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
  if (count == 100){

    Serial.print("throttle: ");
    for (int i = 0; i < 4; i++){      
      Serial.print(throttle[i]);
      Serial.print("\t");
    }

    Serial.print("\tRoll: ");
    Serial.print(gyr.ypr[2]);
    Serial.print("\t");
    Serial.print(rad.buffer[2]);
    Serial.print("\t");

    Serial.print("\tPitch: ");
    Serial.print(gyr.ypr[1]);
    Serial.print("\t");
    Serial.print(rad.buffer[3]);
    Serial.print("\t");

    count = 0;
    Serial.println("");
  }

}
