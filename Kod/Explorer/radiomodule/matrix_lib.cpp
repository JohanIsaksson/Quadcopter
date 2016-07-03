#include "matrix_lib.h"

matrix matrix_create(int r, int c, double* arr){
	matrix m;
	m.rows = r;
	m.columns = c;  
}


matrix matrix_multiply(matrix A, matrix B){

	double data[A.rows*B.columns];
 	matrix m = matrix_create(A.rows, B.columns, data);    

 	for (int i = 0; i < A.rows; ++i){
 		for (int j = 0; i < B.columns; ++j){
 			double s = 0;
 			for (int k = 0; i < B.rows; ++k){
 				s += (*A.data[i*A.columns + k]) * (*B.data[k*B.rows + j]);
 			}
 			*m.data[i*m.columns + j] = s;
 		}
 	}
 	return m;
 }

 matrix matrix_create_identity(int r, int c){
 	double data[r*c];
 	for (int i = 0; i < r; ++i){
 			data[i*c + i] = 1.0; 		
 	}
 	return matrix_create(r, c, data);
}

 matrix matrix_scale(matrix A, double b){
 	double data[A.rows*A.columns];
 	matrix m = matrix_create(A.rows, A.columns, data);
 	for (int i = 0; i < A.rows; ++i){
 		for (int j = 0; i < A.columns; ++j){
 			*m.data[i*A.columns + j] = *A.data[i*A.columns + j] * b;
 		}
 	}
 	return m;
 }

matrix matrix_add(matrix A, matrix B){
	double data[A.rows*A.columns];
 	matrix m = matrix_create(A.rows, A.columns, data);
 	for (int i = 0; i < A.rows; ++i){
 		for (int j = 0; i < A.columns; ++j){
 			*m.data[i*A.columns + j] = *A.data[i*A.columns + j] + *B.data[i*A.columns + j];
 		}
 	}
 	return m;
}


matrix matrix_subtract(matrix A, matrix B){
	double data[A.rows*A.columns];
 	matrix m = matrix_create(A.rows, A.columns, data);
 	for (int i = 0; i < A.rows; ++i){
 		for (int j = 0; i < A.columns; ++j){
 			*m.data[i*A.columns + j] = *A.data[i*A.columns + j] - *B.data[i*A.columns + j];
 		}
 	}
 	return m;
 } 


matrix matrix_transpose(matrix A){
	double data[A.columns*A.rows];
 	matrix m = matrix_create(A.columns, A.rows, data);
 	for (int i = 0; i < A.rows; ++i){
 		for (int j = 0; i < A.columns; ++j){
 			*m.data[i*A.columns + j] = *A.data[i*A.columns + i];
 		}
 	}
 	return m;
 }


// only works for 2x2 matrices
matrix matrix_inverse(matrix A){
	double data[A.rows*A.columns];
	data[0] = *A.data[0];
	data[3] = *A.data[3];
	data[1] = -*A.data[2];
	data[2] = -*A.data[1];
	double s = data[0] * *A.data[3] - data[2] * *A.data[1];
	matrix m = matrix_create(A.rows, A.columns, data);
	return matrix_scale(m, 1.0/s);
}



matrix operator*(matrix A, matrix B){
	return matrix_multiply(A,B);
}

matrix operator*(matrix A, double b){
	return matrix_scale(A, b);
}

matrix operator*(double b, matrix A){
	return matrix_scale(A, b);
}

matrix operator+(matrix A, matrix B){
	return matrix_add(A, B);
}

matrix operator-(matrix A, matrix B){
	return matrix_subtract(A, B);
}

//only works for 2x2 matrices
matrix operator/(matrix A, matrix B){
	return matrix_multiply(A, matrix_inverse(B));
}