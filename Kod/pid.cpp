
#include "pid.h"

void init_pid(pid* p){
	//set pid constants
	p->Kp = 0.5;
	p->Ki = 0.001;
	p->Kd = 2;

	//set integral
	p->roll_integral = 0.0;
	p->pitch_integral = 0.0;

}



void pid_pitch(pid* p, int* front, int* back, double gyro_pitch, int ref_pitch){

	p->pitch_error = ref_pitch - gyro_pitch; // anlge error

	p->pitch_integral += p->pitch_error; 

	p->pitch_u = p->Kp*p->pitch_error + p->Ki*p->pitch_integral + p->Kd*(p->pitch_error-p->pitch_error_prev);

	*front = p->pitch_u;
	*back = -p->pitch_u;

	// TODO: add max u and test for better Kp,Ki,Kd
}



void pid_roll(pid* p, int* left, int* right, double gyro_roll, int ref_roll){

	p->roll_error = ref_roll - gyro_roll; // anlge error

	p->roll_integral += p->roll_error; 

	p->roll_u = p->Kp*p->roll_error + p->Ki*p->roll_integral + p->Kd*(p->roll_error-p->roll_error_prev);

	*left = p->roll_u;
	*right = -p->roll_u;

}