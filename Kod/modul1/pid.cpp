
#include "pid.h"

/* Initializes gyro and sets parameters */
void init_pid(pid* p){
	//set integral
	p->roll_integral = 0.0;
	p->pitch_integral = 0.0;
	p->K_tmp = 0.0;
}

/* Performs PID calculation for pitch */
void pid_pitch(pid* p, int* front, double gyro_pitch, double ref_pitch, uint32_t t){

	p->pitch_error = ref_pitch - gyro_pitch; // anlge error

	p->pitch_integral += p->pitch_error*((double)t)/1000000.0;

	p->pitch_p = KP * p->pitch_error;
	p->pitch_i = KI * p->pitch_integral;
	p->pitch_d = KD * (p->pitch_error - p->pitch_error_prev) / (((double)t)/1000000.0);

	p->pitch_u =  (int)(p->pitch_p + p->pitch_i + p->pitch_d);

	//limit pitch
	if (p->pitch_u > 0.0){
		if (p->pitch_u > PID_MAX){
			p->pitch_u = PID_MAX;
		}
	}else{
		if (-p->pitch_u > PID_MAX){
			p->pitch_u = -PID_MAX;
		}
	}

	p->pitch_error_prev = p->pitch_error;

	//set values
	*front = -p->pitch_u;
}

/* Performs PID calculation for pitch rate*/
void pid_pitch_rate(pid* p, int* front, double gyro_rate, double ref_rate, uint32_t t){

	p->pitch_error = ref_rate - gyro_rate; // anlge rate error

	p->pitch_integral += p->pitch_error*((double)t)/1000000.0;

	p->pitch_p = KP_A * p->pitch_error;
	p->pitch_i = KI_A * p->pitch_integral;
	p->pitch_d = KD_A * ((p->pitch_error - p->pitch_error_prev) / (((double)t)/1000000.0));

	p->pitch_u =  (int)(p->pitch_p + p->pitch_i + p->pitch_d);

	//limit pitch
	if (p->pitch_u > 0.0){
		if (p->pitch_u > PID_MAX){
			p->pitch_u = PID_MAX;
		}
	}else{
		if (-p->pitch_u > PID_MAX){
			p->pitch_u = -PID_MAX;
		}
	}

	p->pitch_error_prev = p->pitch_error;

	//set values
	*front = -p->pitch_u;
}

/* Performs PID calculation for roll */
void pid_roll(pid* p, int* left, double gyro_roll, double ref_roll, uint32_t t){

	p->roll_error = ref_roll - gyro_roll; // anlge error

	p->roll_integral += p->roll_error*((double)t)/1000000.0;

	p->roll_p = KP * p->roll_error;
	p->roll_i = KI * p->roll_integral;
	p->roll_d = KD * (p->roll_error - p->roll_error_prev) / (((double)t)/1000000.0);

	p->roll_u =  (int)(p->roll_p + p->roll_i + p->roll_d);

	//limit roll
	if (p->roll_u > 0.0){
		if (p->roll_u > PID_MAX){
			p->roll_u = PID_MAX;
		}
	}else{
		if (-p->roll_u > PID_MAX){
			p->roll_u = -PID_MAX;
		}
	}

	p->roll_error_prev = p->roll_error;

	//set values
	*left = p->roll_u;
}

/* Performs PID calculation for roll rate */
void pid_roll_rate(pid* p, int* left, double gyro_rate, double ref_rate, uint32_t t){

	p->roll_error = ref_rate - gyro_rate; // anlge error

	p->roll_integral += p->roll_error*((double)t)/1000000.0;

	p->roll_p = KP_A * p->roll_error;
	p->roll_i = KI_A * p->roll_integral;
	p->roll_d = KD_A * (p->roll_error - p->roll_error_prev) / (((double)t)/1000000.0);

	p->roll_u =  (int)(p->roll_p + p->roll_i + p->roll_d);

	//limit roll
	if (p->roll_u > 0.0){
		if (p->roll_u > PID_MAX){
			p->roll_u = PID_MAX;
		}
	}else{
		if (-p->roll_u > PID_MAX){
			p->roll_u = -PID_MAX;
		}
	}

	p->roll_error_prev = p->roll_error;

	//set values
	*left = p->roll_u;
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

void pid_yaw_temp(pid* p, int* cw, double gyro_yaw, double ref_yaw){
	p->yaw_error = ref_yaw - gyro_yaw; // anlge error

	//p->yaw_integral += p->yaw_error; 

	p->yaw_u = (int)(KP_Y*p->yaw_error + KD_Y*(p->yaw_error-p->yaw_error_prev)); // + KI_Y*p->yaw_integral

	//limit pitch
	if (p->pitch_u > 0.0){
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

}