/* 2016-06-06
 * Johan Isaksson
 * Kalman filter for altitude estimation
 * 
 *
 *
*/


#include "matrix_lib.h"





class Kalman{
	matrix A, At, B, H, Ht, P, P_p, Q, I, K, R, x, w, x_p, y;
	double u;

	
public:

	void init();

	void update(double baro, double acc, double dt);

	double get_altitude();

};