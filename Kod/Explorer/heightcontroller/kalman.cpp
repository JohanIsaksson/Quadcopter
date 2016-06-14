/* 2016-06-06
 * Johan Isaksson
 * Kalman filter for altitude estimation
 * 
 *
 *
*/


#include "matLib.h"





struct kalman{
	matrix *A, *At, *P_p, *Q, *R, *I, *H, *Ht; 		//matrices
	matrix *x, *B, *Bu, *y, *w, *x_p;	//vectors
	value dt;
	value u;

	matrix *tmp33, *tmp31, *tmp32, *tmp23, *tmp22, *tmp21;
};

typedef struct kalman kalman;

kalman k;

 void init_filter(){
 	k.dt = 0.004; //4 ms
 	k.u = 9.82; //gravity

 	value[3] v_data;
 	value[9] m_data;

	k.x = create_zero_matrix(3,1);
	

	k.A = create_matrix(3,3);
	m_data = {1.0, dt, dt*dt/2,
						0.0, 1.0, dt,
						0.0, 0.0, 1.0};
	insert_array(m_data, k.A);

	k.At = matrix_transpose_with_return(k.A);

	k.P = 

	k.B = create_matrix(3,1);
	v_data = {}


	k.tmp31 = create_zero_matrix(3,1);
	k.tmp33 = create_zero_matrix(3,3);

 }

 void kalman_step(){
 	/****** Predict new state *******/
 	multiply_matrices(k.A, k.x, k.tmp31);
 	add_matrices(k.tmp31, k.Bu, k.tmp31;
 	add_matrices(k.tmp31, k.w, k.x_p);

 	multiply_matrices(k.A, k.P, k.tmp33);
 	multiply_matrices(k.tmp33, k.At, k.tmp33);
 	add_matrices(k.tmp33, k.Q, k.P_p);


 	/****** Measurment input  *******/
 	insert_value(barometer_data, 1, 1, k.y);
 	insert_value(vertical_velocity, 2, 1, k.y);


 	/****** Calculate gain and new state *******/
 	multiply_matrices(k.P_p, k.Ht, k.tmp32);
 	multiply_matrices(k.H, k.tmp32, k.tmp22);

 	add_matrices(k.tmp22, k.R, k.tmp22);
 	get_inverse(k.tmp22, k.tmp22);
 	multiply_matrices(k.tmp32, k.tmp22, k.K);

 	multiply_matrices(k.H, k.x_p, k.tmp21);
 	subtract_matrices(k.y, k.tmp21, k.tmp21);
 	add_matrices(k.x_p, k.tmp21, k.x);


 	/****** Calculate uncertanty ******/
 	multiply_matrices(k.K, k.H, k.tmp33);
 	subtract_matrices(k.I, K.tmp22, k.tmp33);
 	multiply_matrices(k.tmp33, k.P_p, k.P);


 	/****** Output **********************/
 }