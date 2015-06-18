#include "gyro.h"
#include "radio.h"
#include <ServoTimer2.h>
#include "pid.h"

radio rad;
gyro gyr;
pid p;

//motors
ServoTimer2 front_left;
ServoTimer2 front_right;
ServoTimer2 back_left;
ServoTimer2 back_right;

//pid
int front, back;

void setup(){

  Serial.begin(38400);

	bool b = init_gyro(&gyr);

	init_radio(&rad);

	//asign motors to pins
  front_left.attach(3);
  front_right.attach(4);
  back_left.attach(5);
  back_right.attach(6);

  init_pid(&p);
}


void PID(){

}

void loop(){

  delay(1);

  read_gyro(&gyr);
  read_message(&rad);

  /* print it */
  Serial.print("pid: ");

  pid_pitch(&p, &front, &back, gyr.ypr[1], -(double)rad.buffer[3]);

  Serial.println(front);

	

	/*int throttle = map((uint8_t)(rad.buffer[5]), 0, 255, 1000, 1999);
  front_left.write(throttle);
  front_right.write(throttle);
  back_left.write(throttle);
  back_right.write(throttle);*/
}
