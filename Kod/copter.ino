#include "gyro.h"
#include "radio.h"
#include <ServoTimer2.h>

radio rad;
gyro gyr;

//motors
ServoTimer2 front_left;
ServoTimer2 front_right;
ServoTimer2 back_left;
ServoTimer2 back_right;


void setup(){

	init_gyro(&gyr);

	init_radio(&rad);

	//asign motors to pins
    front_left.attach(2);
    front_right.attach(3);
    back_left.attach(4);
    back_right.attach(9);

}

void loop(){

	read_message(&rad);

	int throttle = map((uint8_t)(rad.buffer[5]), 0, 255, 1000, 1999);
  front_left.write(throttle);
  front_right.write(throttle);
  back_left.write(throttle);
  back_right.write(throttle);

}
