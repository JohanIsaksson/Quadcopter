
#include "pid.h"


void Pid::init(){
	integral = 0.0;
	error_prev = 0.0;
	enable_integral = true;
}

void Pid::set_constants(double KP_, double KI_, double KD_){

}

void Pid::update(int* out, double ref, double mea, double dt, double scale){
	error = ref - mea;

	integral += error*dt;

	p = KP * error;
	i = KI * integral;
	d = KD * (error - error_prev) / dt;

	output =  p + i + d;

	//limit pitch
	if (output > 0.0){
		if (output > PID_MAX){
			output = PID_MAX;
		}
	}else{
		if (-output > PID_MAX){
			output = -PID_MAX;
		}
	}
	error_prev = error;

	//set values
	*out = -(int)output;
}