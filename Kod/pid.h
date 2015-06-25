#ifndef PID_H
#define PID_H

#define PID_MAX 254.0

/* PID structure containing necessary info */
struct pid{
	//PID regulation for tilting
	double Kp, Ki, Kd;

	//roll
	int roll_u;
	double roll_error, roll_error_prev;
	double roll_integral; 

	//pitch
	int pitch_u;
	double pitch_error, pitch_error_prev;
	double pitch_integral;

};
typedef struct pid pid;

/* Initializes gyro and sets parameters */
void init_pid(pid* p);

/* Performs PID calculation for pitch */
void pid_pitch(pid* p, int* front, int* back, double gyro_pitch, int ref_pitch);

/* Performs PID calculation for roll */
void pid_roll(pid* p, int* left, int* right, double gyro_roll, int ref_roll);





#endif