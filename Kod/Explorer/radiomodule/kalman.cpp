#include "kalman.h"



 void Kalman::init(){
 	u = 9.82; //gravity



	double dataP[9] = {0.0084,    0.0023,    0.0001,
					0.0023,    0.0099,    0.0000,
					0.0001,    0.0000,    0.1321};

    P = matrix_create(3,3,dataP);
    
    double dataQ[9] = {0.00016,		    0.0000000016,		0.0016777216,
						0.0000000016,	0.00001,			0.0001048576,
						0.0016777216,	0.00010485760011,	10.48576};
	Q = matrix_create(3,3,dataQ);

    double dataR[4] = {0.3943, 0.0028,
    					0.0028, 0.1338};
    R = matrix_create(2,2,dataR);

    I = matrix_create_identity(3,3); 

    A = matrix_create_identity(3,3);
    At = matrix_transpose(A); 

    double dataB[3] = {0.0, 0.0, 1.0};
    B = matrix_create(3,1, dataB);

    double dataH[6] = {1.0, 0.0,
    					0.0, 0.0,
    					0.0, 1.0};
    H = matrix_create(2, 3, dataH);
    Ht = matrix_transpose(H);


 }

 void Kalman::update(double baro, double acc, double dt){

 	// set A and B matrices 	
 	*A.data[1] = dt;
 	*A.data[5] = dt;
 	*A.data[2] = dt*dt/2.0;

 	*B.data[0] = dt*dt/2.0;
 	*B.data[1] = dt;


 	// Predict new state 
 	x_p = A*x + B*u + w;
 	P_p = A*P*At + Q;


 	// Measurment input 	
	*y.data[0] = baro;
	*y.data[1] = acc;


 	// Calculate gain and new state
 	K = P_p*Ht/(H*P_p*Ht + R);
 	x = x_p + K*(y - H*x_p);


 	// Calculate uncertanty 
  	P = (I - K*H)*P_p;
 	
}

double Kalman::get_altitude(){
    return *x.data[0];
}