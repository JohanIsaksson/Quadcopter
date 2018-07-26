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

  void Calculate(double* out, double ref, double mea, double dt, double scale);
 
public:
	void Init();
	void SetIntegration(bool b);
	void SetConstants(double KP_, double KI_, double KD_, double INT_MAX_);
	void Update(int* out, double ref, double mea, double dt, double scale);
	void Update(double* out, double ref, double mea, double dt, double scale);

	//testing variable
	double K_tmp;
};


#endif
