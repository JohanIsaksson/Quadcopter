#include <arduino.h>

#ifndef PID_H
#define PID_H

#define PID_MAX 254.0
#define INTEGRAL_MAX 150.0

//Horizon mode pid values
#define KP 3.0//
#define KI 0.02//
#define KD 0.1//

//Acrobatic mode pid values
#define KP_A 2.5 //2.86
#define KI_A 0.2 //0.5
#define KD_A 0.005

//yaw pid values
#define KP_Y 2.0
#define KI_Y 0.01
#define KD_Y 0.0//0.02



class Pid{
	

	//help variables
	double p;
	double i;
	double d;
	double error;
	double error_prev;
	double integral;
	double output;

	//double KP, KI, KD;
	bool enable_integral;
public:
	void init();
	void set_integration(bool b);
	void set_constants(double KP_, double KI_, double KD_);
	void update(int* out, double ref, double mea, double dt, double scale);


	//testing variable
	double K_tmp;
};


#endif