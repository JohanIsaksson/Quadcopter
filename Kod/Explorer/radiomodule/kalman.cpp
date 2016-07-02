#include "kalman.h"



 void kalman_init(kalman* k){
 	k->u = 9.82; //gravity


	double dataP[9] = {0.0084,    0.0023,    0.0001,
					0.0023,    0.0099,    0.0000,
					0.0001,    0.0000,    0.1321};

    k->P = matrix_create<3,3>(dataP);
    
    double dataQ[9] = {0.00016,		0.0000000016,		0.0016777216,
						0.0000000016,	0.00001,			0.0001048576,
						0.0016777216,	0.00010485760011,	10.48576};
	k->Q = matrix_create<3,3>(dataQ);

    double dataR[4] = {0.3943, 0.0028,
    					0.0028, 0.1338};
    k->R = matrix_create<2,2>(dataR);

    k->I = identity_matrix<3,3>();




 }

 void kalman_update(kalman* k, double baro, double acc, double dt){

 	// set A and B matrices
 	matrix<3,3> A = identity_matrix<3,3>(); 
 	A.data[1][2] = dt;
 	A.data[2][3] = dt;
 	A.data[1][3] = dt*dt/2;

 	matrix<3,3>At = matrix_transpose<3,3>(A);

 	matrix<3,1> B;
 	B.data[1][1] = dt*dt/2;
 	B.data[2][1] = dt;
 	B.data[3][1] = 1.0;





 	// Predict new state 
 	// x_p = A*x + B*u + w
 	matrix<3,1> Ax = matrix_multiply<3,3,3,1>(A,k->x);
 	matrix<3,1> Bu = matrix_scale<3,1>(B,k->u);
 	k->x_p = matrix_add<3,1>(Ax, Bu);

 	// P_p = A*P*At + Q
 	matrix<3,3> AP = matrix_multiply<3,3,3,3>(A, k->P);
 	matrix<3,3> APAt = matrix_multiply<3,3,3,3>(AP, At);
 	k->P_p = matrix_add<3,3>(APAt, k->Q);



 	// Measurment input 
	matrix<2,1> y;
	y.data[1][1] = baro;
	y.data[2][1] = acc;

 	matrix<2,3> H;
 	H.data[1][1] = 1.0;
 	H.data[2][3] = 1.0;
 	matrix<3, 2> Ht = matrix_transpose<2, 3>(H);



 	// Calculate gain and new state
 	// K = P_p*Ht/(H*P_p*Ht + R)
 	matrix<3,2> PHt = matrix_multiply<3,3,3,2>(k->P_p, Ht);
 	matrix<2,2> HPHt = matrix_multiply<2,3,3,2>(H, PHt);
 	matrix<2,2> HPHtR = matrix_add<2,2>(HPHt, k->R);
 	matrix<2,2> HPHtR_ = matrix_inverse<2,2>(HPHtR);
 	k->K = matrix_multiply<3,2,2,2>(PHt, HPHtR_);

 	// x = x_p + K*(y - H*x_p)
 	matrix<2,1> Hx = matrix_multiply<2,3,3,1>(H, k->x_p);
 	matrix<2,1> yHx = matrix_subtract<2,1>(y, Hx);
 	matrix<3,1> KyHx = matrix_multiply<3,2,2,1>(k->K, yHx);
 	k->x = matrix_add<3,1>(k->x_p, KyHx);



 	// Calculate uncertanty 
 	// P = (I - K*H)*P_p
 	matrix<3,3> KH = matrix_multiply<3,2,2,3>(k->K, H);
 	matrix<3,3> IKH = matrix_subtract<3,3>(k->I, KH);
 	k->P = matrix_multiply<3,3,3,3>(IKH, k->P_p);


 	/****** Output **********************/
 }