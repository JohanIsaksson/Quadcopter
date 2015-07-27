#ifndef PID_H
#define PID_H

#define PID_MAX 254.0
#define KP 1.5
#define KI 0.0001
#define KD 0.5

/* PID structure containing necessary info */
struct pid{

	//roll
	int roll_u;
	double roll_error, roll_error_prev;
	double roll_integral; 

	//pitch
	int pitch_u;
	double pitch_error, pitch_error_prev;
	double pitch_integral;

	/* yaw */
	int yaw_u;
	double yaw_error, yaw_error_prev;

};
typedef struct pid pid;

/* Initializes gyro and sets parameters */
void init_pid(pid* p);

/* Performs PID calculation for pitch */
void pid_pitch(pid* p, int* front, int* back, double gyro_pitch, double ref_pitch);

/* Performs PID calculation for roll */
void pid_roll(pid* p, int* left, int* right, double gyro_roll, double ref_roll);

/* Performs PID calculation for yaw */
void pid_yaw(pid* p, int* cw, int* ccw, double gyro_yaw, int ref_yaw);




#endif