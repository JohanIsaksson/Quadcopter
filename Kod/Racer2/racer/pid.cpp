
#include "pid.h"


void Pid::init(){
	integral = 0.0;
	error_prev = 0.0;
	enable_integral = true;
}

void Pid::set_constants(double KP_, double KI_, double KD_, double INT_MAX_){
	KP = KP_;
	KI = KI_;
	KD = KD_;
	INTEGRAL_MAX = INT_MAX_;
}

// approx. time consumption = 1216 cc (clock cycles)
//							= 76 us
void Pid::update(int* out, double ref, double mea, double dt, double scale){
	error = ref - mea; //9*16  cc

	//TODO:
	//	windup handling
	//	D filtering

	integral += error*dt; //(9+5)*16 cc

	p = KP * error; //5*16 cc
	i = KI * integral; //5*16 cc
	d = KD * (error - error_prev) / dt; //(9+5+5)*16 cc

	output =  p + i + d; //(5+5)*16

	//limit pitch
	if (output > 0.0){	//9*16 cc
		if (output > PID_MAX){ //9*16 cc
			output = PID_MAX; 	//4 cc
		}
	}else{
		if (-output > PID_MAX){
			output = -PID_MAX;
		}
	}
	error_prev = error; //4 cc

	//set values
	*out = (int)(scale*output); //5*16 + 4 cc
}



// faster by using integers aritmetics instead of floats
// approx. time consume = 42 cc
//						= 3 us
/*void Pid::update_(int* out, int ref, int mea, int scale){
	error = ref - mea; //2 cc

	//TODO:
	//	windup handling
	//	D filtering

	integral += error; //2 cc

	p = KP * error; // 4 + 2 cc
	i = KI * integral; //4 + 2 cc
	d = KD * (error - error_prev); //2 + 4 + 2 cc

	output =  p + i + d; // 4 cc

	//limit pitch
	if (output > 0){ //2 cc
		if (output > PID_MAX){ //2 cc
			output = PID_MAX; // 2 cc
		}
	}else{
		if (-output > PID_MAX){
			output = -PID_MAX;
		}
	}
	error_prev = error; //2 cc

	//set values
	*out = scale*output; // 4 +2 cc 
}*/