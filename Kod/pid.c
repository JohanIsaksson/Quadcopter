
#include "pid.h"


void pid_pitch(pid* p, int* front, int* back, double gyro_pitch, int ref_pitch){

	p->pitch_error = ref_pitch - gyro_pitch; // anlge error

	p->pitch_integral += p->pitch_error; 

	p->pitch_u = p->Kp*p->pitch_error + p->Ki*p->pitch_integral + p->Kd*(p->pitch_error-p->pitch_error_prev);

	*front = p->pitch_u;
	*back = -p->pitch_u;

	// TODO: add max u and test for better Kp,Ki,Kd
}



void pid_roll(pid* p, int* left, int* right, double gyro_roll, int ref_roll){

}