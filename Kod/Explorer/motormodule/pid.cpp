
#include "pid.h"
#include "imu.h"

/* Initializes gyro and sets parameters */
void init_pid(pid* p){
	//set integral
	p->roll_integral = 0.0;
	p->pitch_integral = 0.0;
	p->yaw_integral = 0.0;
	p->K_tmp = 0.0;
}

/* Performs PID calculation for pitch angle */
void pid_pitch(pid* p, int* front, double gyro_rate, double gyro_pitch, double ref_pitch, double t){

	p->error = ref_pitch - gyro_pitch; // anlge error

	p->pitch_integral += p->error*t;

	p->p = KP * p->error;
	p->i = KI * p->pitch_integral;
	p->d = KD * (p->error - p->pitch_error_prev) / t;

	p->output =  (int)(p->p + p->i + p->d);

	//add extra rate stabilization
	p->error_rate = -gyro_rate; //always 0 as referens
	p->p = KP_A * p->error_rate;
	p->d = KD_A * (p->error_rate - p->pitch_rate_error_prev) / t;
	//p->output += (1.0/(1.0 + RI*abs(p->error))) * (p->p + p->d);

	p->output += ((0.25*sin(3.3*p->error)/p->error)+0.18) * (p->p + p->d);


	//limit pitch
	if (p->output > 0.0){
		if (p->output > PID_MAX){
			p->output = PID_MAX;
		}
	}else{
		if (-p->output > PID_MAX){
			p->output = -PID_MAX;
		}
	}
	p->pitch_rate_error_prev = p->error_rate;
	p->pitch_error_prev = p->error;

	//set values
	*front = -p->output;
}

/* Performs PID calculation for pitch rate*/
void pid_pitch_rate(pid* p, int* front, double gyro_rate, double ref_rate, double t){

	p->error = ref_rate - gyro_rate; // anlge rate error

	p->pitch_rate_integral += p->error * t;

	p->p = KP_A * p->error;
	p->i = KI_A * p->pitch_rate_integral;
	p->d = KD_A * (p->error - p->pitch_rate_error_prev) / t;

	p->output =  (int)(p->p + p->i + p->d);

	//limit pitch
	if (p->output > 0.0){
		if (p->output > PID_MAX){
			p->output = PID_MAX;
		}
	}else{
		if (-p->output > PID_MAX){
			p->output = -PID_MAX;
		}
	}

	p->pitch_rate_error_prev = p->error;

	//set values
	*front = -p->output;
}

/* Performs PID calculation for roll */
void pid_roll(pid* p, int* left, double gyro_rate, double gyro_roll, double ref_roll, double t){

	p->error = ref_roll - gyro_roll; // anlge error

	p->roll_integral += p->error * t;

	p->p = KP * p->error;
	p->i = KI * p->roll_integral;
	p->d = KD * (p->error - p->roll_error_prev) / t;

	p->output =  (int)(p->p + p->i + p->d);

	//add extra rate stabilization
	p->error_rate = -gyro_rate; //always 0 as referens
	p->p = KP_A * p->error_rate;
	p->d = KD_A * (p->error_rate - p->roll_rate_error_prev) / t;
	p->output += (1.0/(1.0 + RI*abs(p->error))) * (p->p + p->d);

	//limit roll
	if (p->output > 0.0){
		if (p->output > PID_MAX){
			p->output = PID_MAX;
		}
	}else{
		if (-p->output > PID_MAX){
			p->output = -PID_MAX;
		}
	}

	p->roll_rate_error_prev = p->error_rate;
	p->roll_error_prev = p->error;

	//set values
	*left = p->output;
}

/* Performs PID calculation for roll rate */
void pid_roll_rate(pid* p, int* left, double gyro_rate, double ref_rate, double t){

	p->error = ref_rate - gyro_rate; // anlge error

	p->roll_rate_integral += p->error * t;

	p->p = KP_A * p->error;
	p->i = KI_A * p->roll_rate_integral;
	p->d = KD_A * (p->error - p->roll_rate_error_prev) / t;

	p->output =  (int)(p->p + p->i + p->d);

	//limit roll
	if (p->output > 0.0){
		if (p->output > PID_MAX){
			p->output = PID_MAX;
		}
	}else{
		if (-p->output > PID_MAX){
			p->output = -PID_MAX;
		}
	}

	p->roll_rate_error_prev = p->error;

	//set values
	*left = p->output;
}

/* special error calculation for yaw */
int get_yaw_error(int gyro_yaw, int ref_yaw){
	if (gyro_yaw >= 0 && ref_yaw < 0){

		if ((gyro_yaw-ref_yaw) > (360-gyro_yaw+ref_yaw)){
			return 360-gyro_yaw+ref_yaw;
		}else{
			return ref_yaw-gyro_yaw;
		}

	}
	else if (gyro_yaw < 0 && ref_yaw >= 0){

		if ((ref_yaw-gyro_yaw) > (360-ref_yaw+gyro_yaw)){
			return -360+ref_yaw-gyro_yaw;
		}else{
			return ref_yaw-gyro_yaw;
		}

	}
	else{

		return ref_yaw-gyro_yaw;

	}
}

/* Performs PID calculation for yaw */
void pid_yaw(pid* p, int* cw, int* ccw, double gyro_yaw, int ref_yaw){
	p->yaw_error = get_yaw_error((int)gyro_yaw, ref_yaw); // anlge error

	/* Just PD here as no external forces should be present */
	p->yaw_u = (int)(KP*p->yaw_error + KD*(p->yaw_error-p->yaw_error_prev));

	//limit yaw
	if (p->yaw_u > 0.0){
		if (p->yaw_u > PID_MAX){
			p->yaw_u = PID_MAX;
		}
	}else{
		if (-p->yaw_u > PID_MAX){
			p->yaw_u = -PID_MAX;
		}
	}

	p->yaw_error_prev = p->yaw_error;

	//set values
	*cw = -p->yaw_u;
	*ccw = p->yaw_u;
}

void pid_yaw_rate(pid* p, int* cw, double gyro_yaw, double ref_yaw){
	p->error = ref_yaw - gyro_yaw; // anlge error

	p->yaw_integral += p->error; 

	p->p = KP_Y * p->error;
	p->i = KI_Y * p->yaw_integral;
	p->d = KD_Y * p->error - p->yaw_error_prev;

	p->output = p->p + p->i + p->d;


	//limit pitch
	if (p->output > 0.0){
		if (p->output > PID_MAX){
			p->output = PID_MAX;
		}
	}else{
		if (-p->output > PID_MAX){
			p->output = -PID_MAX;
		}
	}

	p->yaw_error_prev = p->yaw_error;

	//set values
	*cw = -p->output;

}