
#include "pid.h"


void Pid::Init(){
	integral = 0.0;
	error_prev = 0.0;
	enable_integral = true;
}

void Pid::SetConstants(double KP_, double KI_, double KD_, double INT_MAX_){
	KP = KP_;
	KI = KI_;
	KD = KD_;
	INTEGRAL_MAX = INT_MAX_;
}

void Pid::Calculate(double* out, double ref, double mea, double dt, double scale){
	error = ref - mea;

	//TODO:
	//	windup handling - done
	//	D filtering

	if (error < 20.0) integral += error*dt;

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
	*out = scale*output;
}

void Pid::Update(int* out, double ref, double mea, double dt, double scale){
	double output;
	Calculate(&output, ref, mea, dt, scale);

	//truncate value
	*out = (int)output;
}

void Pid::Update(double* out, double ref, double mea, double dt, double scale){
	Calculate(out, ref, mea, dt, scale);
}

