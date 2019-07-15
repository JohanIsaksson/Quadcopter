/* 2016-06-06
 * Johan Isaksson
 * Kalman filter for altitude estimation
 * 
 *
 *
*/

#ifndef KALMAN_H
#define KALMAN_H

#include "matrix_lib.h"





class Kalman{
	// Kalman matrices
	matrix A, At, B, H, Ht, P, P_p, Q, I, K, R, x, w, x_p, y;
	value dataA[9], dataAt[9], dataB[3], dataH[6], dataHt[6], 
		   dataP[9], dataP_p[9], dataQ[9], dataI[9], dataK[6], 
		   dataR[4], datax[3], dataw[3], datax_p[3], datay[2];

	// Help matrices
	matrix Ax, Bu, Buw, AP, APAt, PHt, HPHt, HPHtR, HPHtR_, Hx, yHx, KyHx, KH, IKH;
	value dataAx[3], dataBu[3], dataBuw[3], dataAP[9], dataAPAt[9], 
		   dataPHt[6], dataHPHt[4], dataHPHtR[4], dataHPHtR_[4], dataHx[2], 
		   datayHx[2], dataKyHx[3], dataKH[9], dataIKH[9];

	value u;

	
public:

	void Print();

	void InitPreset();

	void Update(value baro, value acc, value dt);

	double GetAltitude();
	double GetVerticalSpeed();
	double GetVerticalAcceleration();


};

#endif /* KALMAN_H */

