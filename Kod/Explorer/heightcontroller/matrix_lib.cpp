/* 2016-06-06
 * Johan Isaksson
 * Simple matrix library
 *
 *
 *
*/

typedef double[3][3] matrix;
typedef double[3][1] vector;

 matrix matrix_matrix_multiply(matrix A, matrix B){
 	matrix m;
 	for (int i = 0; i < 3; ++i){
 		for (int j = 0; i < 3; ++j){
 			double s = 0;
 			for (int k = 0; i < 3; ++k){
 				s += A[i][k]*B[k][j];
 			}
 			m[i][j] = s;
 		}
 	}
 	return m;
 }

 vector matrix_vector_multiply(matrix A, vector B){
 	vector m;
 	for (int i = 0; i < 3; ++i){
		double s = 0;
		for (int k = 0; i < 3; ++k){
			s += A[i][k]*B[k][0];
		}
		m[i][j] = s; 		
 	}
 	return m;
 }



 matrix matrix_add(matrix A, matrix B){
 	matrix m;
 	for (int i = 0; i < 3; ++i){
 		for (int j = 0; i < 3; ++j){
 			m[i][j] = A[i][j] + B[i][j];
 		}
 	}
 	return m;
 }

matrix matrix_subtract(matrix A, matrix B){
 	matrix m;
 	for (int i = 0; i < 3; ++i){
 		for (int j = 0; i < 3; ++j){
 			m[i][j] = A[i][j] - B[i][j];
 		}
 	}
 	return m;
 } 

 matrix matrix_transpose(matrix A, matrix B){
 	matrix m;
 	for (int i = 0; i < 3; ++i){
 		for (int j = 0; i < 3; ++j){
 			m[i][j] = A[i][j] + B[i][j];
 		}
 	}
 	return m;
 }