#include <arduino.h>

#ifndef PID_H
#define PID_H

#define PID_MAX 254.0

//Horizon mode pid values
#define KP 0.0//0.65//0.75
#define KI 0.0//0.012
#define KD 0.0//0.07

//Acrobatic mode pid values
#define KP_A 0.0//0.4 //0.5//1.06
#define KI_A 0.0//0.1
#define KD_A 0.0//0.001

//yaw pid values
#define KP_Y 0.0//1.80
#define KI_Y 0.0//0.0001
#define KD_Y 0.0//0.02

//Rate pid impact constant for horizon mode
#define RI 4.0

/* PID structure containing necessary info */
struct pid{

	double K_tmp;

	//help variables
	double p;
	double i;
	double d;
	double error;
	double error_rate;
	double output;

	//roll
	double roll_error_prev, roll_rate_error_prev;
	double roll_integral, roll_rate_integral; 

	//pitch
	double pitch_error_prev, pitch_rate_error_prev;
	double pitch_integral, pitch_rate_integral;

	/* yaw */
	int yaw_u;
	double yaw_error, yaw_error_prev;
	double yaw_integral;

};
typedef struct pid pid;

/* Initializes gyro and sets parameters */
void init_pid(pid* p);

/* Performs PID calculation for pitch */
void pid_pitch(pid* p, int* front, double gyro_rate, double gyro_pitch, double ref_pitch, double t);

/* Performs PID calculation for pitch  rate*/
void pid_pitch_rate(pid* p, int* front, double gyro_rate, double ref_rate, double t);

/* Performs PID calculation for roll */
void pid_roll(pid* p, int* left, double gyro_rate, double gyro_roll, double ref_roll, double t);

/* Performs PID calculation for roll rate */
void pid_roll_rate(pid* p, int* left, double gyro_rate, double ref_rate, double t);

/* Performs PID calculation for yaw */
void pid_yaw(pid* p, int* cw, int* ccw, double gyro_yaw, int ref_yaw);

/* Performs PID calculation for yaw rate */
void pid_yaw_rate(pid* p, int* cw, double gyro_yaw, double ref_yaw);




#endif