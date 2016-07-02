/* 2016-06-06
 * Johan Isaksson
 * Kalman filter for altitude estimation
 * 
 *
 *
*/


#include "matrix_lib.h"





struct kalman{
	matrix<3,3> P, P_p, Q, I;
	matrix<3,2> K;
	matrix<2,2> R;
	matrix<3,1> x, w, x_p;
	matrix<2,1> y;
	double dt;
	double u;

	
};

typedef struct kalman kalman;

void kalman_init(kalman* k);

void kalman_update(kalman* k, double baro, double acc, double dt);