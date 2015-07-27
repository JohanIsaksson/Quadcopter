#include "radio.h"
#include <ServoTimer2.h>

#define SPEED_MIN 1250
#define SPEED_MAX 1750

radio rad;

//motors
ServoTimer2 motor;

int throttle, mout, count;


void setup(){

  Serial.begin(38400);

	init_radio(&rad);

	//asign motors to pins
  motor.attach(3);

  //set init speed to motors
  throttle = 0;
  mout = 0;
  count = 0;
  motor.write(SPEED_MAX);

}


void loop(){

  read_message(&rad);

  throttle = ((uint8_t)rad.buffer[5]);
  mout = map(throttle, 0, 255, SPEED_MIN, SPEED_MAX);
  count++;

  motor.write(mout);
  
  if (count == 200){
    Serial.print("throttle:\t");
    Serial.print(throttle);
    Serial.print("\t");
    Serial.println(mout);
    count = 0;
  }

}
