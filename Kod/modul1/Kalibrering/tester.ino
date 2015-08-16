#include "radio.h"
#include <ServoTimer2.h>

#define SPEED_MIN_FRONT 1500
#define SPEED_MAX_FRONT 2000
#define SPEED_MIN_BACK 1250
#define SPEED_MAX_BACK 1750

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
  motor.write(SPEED_MAX_BACK);

}


void loop(){

  read_message(&rad);

  throttle = ((uint8_t)rad.buffer[5]);
  mout = map(throttle, 0, 255, SPEED_MIN_BACK, SPEED_MAX_BACK);
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
