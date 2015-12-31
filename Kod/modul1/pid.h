#include <arduino.h>

#ifndef PID_H
#define PID_H

#define PID_MAX 254.0

#define KP 0.7 //0.75
#define KI 0.012
#define KD 0.06 //0.07

#define KP_A 0.4 
#define KI_A 0.00
#define KD_A 0.05

/* PID structure containing necessary info */
struct pid{

	//roll
	int roll_u;

	double roll_p;
	double roll_i;
	double roll_d;

	double roll_error, roll_error_prev;
	double roll_integral; 

	//pitch
	int pitch_u;

	double pitch_p;
	double pitch_i;
	double pitch_d;

	double pitch_error, pitch_error_prev;
	double pitch_integral;

	/* yaw */
	int yaw_u;
	double yaw_error, yaw_error_prev;
	double yaw_integral;

};
typedef struct pid pid;

/* Initializes gyro and sets parameters */
void init_pid(pid* p);

/* Performs PID calculation for pitch */
void pid_pitch(pid* p, int* front, double gyro_pitch, double ref_pitch, uint32_t t);

/* Performs PID calculation for pitch  rate*/
void pid_pitch_rate(pid* p, int* front, double gyro_rate, double ref_rate, uint32_t t);

/* Performs PID calculation for roll */
void pid_roll(pid* p, int* left, double gyro_roll, double ref_roll, uint32_t t);

/* Performs PID calculation for roll rate */
void pid_roll_rate(pid* p, int* left, double gyro_rate, double ref_rate, uint32_t t);

/* Performs PID calculation for yaw */
void pid_yaw(pid* p, int* cw, int* ccw, double gyro_yaw, int ref_yaw);

/* Performs PID calculation for yaw - takes only gyro data into account */
void pid_yaw_temp(pid* p, int* cw, double gyro_yaw, double ref_yaw);




#endif