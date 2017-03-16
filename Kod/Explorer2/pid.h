#include <arduino.h>

#ifndef PID_H
#define PID_H

#define PID_MAX 254.0

class Pid{
	
	double KP, KI, KD, INTEGRAL_MAX;

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
	void set_constants(double KP_, double KI_, double KD_, double INT_MAX_);
	void update(int* out, double ref, double mea, double dt, double scale);


	//testing variable
	double K_tmp;
};


#endif