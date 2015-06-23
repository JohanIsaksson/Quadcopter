#include "gyro.h"
#include "radio.h"
#include <ServoTimer2.h>
#include "pid.h"

#define SPEED_MIN 800
#define SPEED_MAX 2200

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
    motors[i].write(SPEED_MIN);
  }

  init_pid(&p);

  //wait for esc init;
  delay(2000);
}


void loop(){

  delay(1);

  read_gyro(&gyr);
  read_message(&rad);


  /* calculate pid */
  pid_pitch(&p, &front, &back, gyr.ypr[1], -(double)rad.buffer[3]);

  /* calculate final motor speed */
  //front left
  throttle[0] = rad.buffer[2] + front + left;

  //front right
  throttle[1] = rad.buffer[2] + front + right;

  //back left
  throttle[2] = rad.buffer[2] + back + left;

  //back right
  throttle[3] = rad.buffer[2] + back + right;

  //check and set speeds
  for (int i = 0; i < 4; i++){
    if (throttle[i] > 255){
      throttle[i] = 255;
    }else if (throttle[i] < 0){
      throttle[i] = 0;
    }
    //motors[i].write(map(throttle[i], 0, 255, SPEED_MIN, SPEED_MAX));
  }

  for (int i = 0; i < 4; i++){
    Serial.print(throttle[i]);
    Serial.print("\t");
  }
  Serial.println("");

}
