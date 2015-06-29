
#include "pid.h"

/* Initializes gyro and sets parameters */
void init_pid(pid* p){
	//set pid constants
	p->Kp = 0.1;
	p->Ki = 0.0001;
	p->Kd = 0.5;

	//set integral
	p->roll_integral = 0.0;
	p->pitch_integral = 0.0;
}

/* Performs PID calculation for pitch */
void pid_pitch(pid* p, int* front, int* back, double gyro_pitch, int ref_pitch){

	p->pitch_error = ref_pitch - gyro_pitch; // anlge error

	p->pitch_integral += p->pitch_error; 

	p->pitch_u = (int)(p->Kp*p->pitch_error + p->Ki*p->pitch_integral + p->Kd*(p->pitch_error-p->pitch_error_prev));

	//limit pitch
	if (p->pitch_u > 0.0){
		if (p->pitch_u > PID_MAX){
			p->pitch_u = PID_MAX;
		}
	}else{
		if (-p->pitch_u > PID_MAX){
			p->pitch_u = -PID_MAX;
		}
	}

	//set values
	*front = p->pitch_u;
	*back = -p->pitch_u;
}

/* Performs PID calculation for roll */
void pid_roll(pid* p, int* left, int* right, double gyro_roll, int ref_roll){

	p->roll_error = ref_roll - gyro_roll; // anlge error

	p->roll_integral += p->roll_error; 

	p->roll_u = (int)(p->Kp*p->roll_error + p->Ki*p->roll_integral + p->Kd*(p->roll_error-p->roll_error_prev));

	//limit roll
	if (p->roll_u > 0.0){
		if (p->roll_u > PID_MAX){
			p->roll_u = PID_MAX;
		}
	}else{
		if (-p->roll_u > PID_MAX){
			p->roll_u = -PID_MAX;
		}
	}

	//set values
	*left = p->roll_u;
	*right = -p->roll_u;
}

/* Performs PID calculation for yaw */
void pid_yaw(pid* p, int* cw, int* ccw, double gyro_yaw, int ref_yaw){

}