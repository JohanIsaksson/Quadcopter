#include "gyro.h"
#include "radio.h"
#include <ServoTimer2.h>
#include "pid.h"

radio rad;
gyro gyr;

//motors
ServoTimer2 front_left;
ServoTimer2 front_right;
ServoTimer2 back_left;
ServoTimer2 back_right;

void setup(){

	bool b = init_gyro(&gyr);

	init_radio(&rad);

	//asign motors to pins
  front_left.attach(3);
  front_right.attach(4);
  back_left.attach(5);
  back_right.attach(6);

  Serial.begin(38400);    
}


void PID(){

}

void loop(){

  delay(1);

  read_gyro(&gyr);

  /* print it */
  Serial.print("pitch/roll: ");
  Serial.print(gyr.ypr[1], 5);
  Serial.print("\t");
  Serial.println(gyr.ypr[2], 5);

	read_message(&rad);

	/*int throttle = map((uint8_t)(rad.buffer[5]), 0, 255, 1000, 1999);
  front_left.write(throttle);
  front_right.write(throttle);
  back_left.write(throttle);
  back_right.write(throttle);*/
}
