#include "radio.h"
#include <ServoTimer2.h>

#define SPEED_MIN 800
#define SPEED_MAX 2200

radio rad;

//motors
ServoTimer2 motor;

int throttle;


void setup(){

  Serial.begin(38400);

	init_radio(&rad);

	//asign motors to pins
  motor.attach(3);

  //set init speed to motors
  throttle = 0;
  motor.write(SPEED_MIN);

}


void loop(){

  read_message(&rad);

  throttle = rad.buffer[2];
  
  motor.write(map(throttle, 0, 255, SPEED_MIN, SPEED_MAX));
  
  Serial.print("throttle:\t");
  Serial.println(throttle);

}
