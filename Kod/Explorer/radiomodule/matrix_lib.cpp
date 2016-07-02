#include "matrix_lib.h"


template <int r1, int c1, int r2, int c2>
matrix<r1, c2> matrix_multiply(matrix<r1, c1> A, matrix<r2, c2> B){

 	matrix<A.rows, B.columns> m;
 	for (int i = 0; i < A.rows; ++i){
 		for (int j = 0; i < B.columns; ++j){
 			double s = 0;
 			for (int k = 0; i < B.rows; ++k){
 				s += A.data[i][k]*B.data[k][j];
 			}
 			m.data[i][j] = s;
 		}
 	}
 	return m;
 }



 template <int r, int c>
matrix<r, c> matrix_add(matrix<r, c> A, matrix<r, c> B){
 	matrix<A.rows, A.columns> m;
 	for (int i = 0; i < A.rows; ++i){
 		for (int j = 0; i < A.columns; ++j){
 			m.data[i][j] = A.data[i][j] + B.data[i][j];
 		}
 	}
 	return m;
 }

 template <int r, int c>
matrix<r, c> matrix_scale(matrix<r, c> A, double b){
 	matrix<A.rows, A.columns> m;
 	for (int i = 0; i < A.rows; ++i){
 		for (int j = 0; i < A.columns; ++j){
 			m.data[i][j] = A.data[i][j] * b;
 		}
 	}
 	return m;
 }

template <int r, int c>
matrix<r, c> matrix_subtract(matrix<r, c> A, matrix<r, c> B){
 	matrix<A.rows, A.columns> m;
 	for (int i = 0; i < A.rows; ++i){
 		for (int j = 0; i < A.columns; ++j){
 			m.data[i][j] = A.data[i][j] - B.data[i][j];
 		}
 	}
 	return m;
 } 

template <int r, int c>
matrix<c, r> matrix_transpose(matrix<r, c> A){
 	matrix<A.columns, A.rows> m;
 	for (int i = 0; i < A.rows; ++i){
 		for (int j = 0; i < A.columns; ++j){
 			m.data[i][j] = A.data[j][i];
 		}
 	}
 	return m;
 }


// only works for 2x2 matrices
template <int r, int c>
matrix<r, c> matrix_inverse(matrix<r, c> A){
	matrix<r, c> m;
	m.data[1][1] = A.data[1][1];
	m.data[2][2] = A.data[2][2];
	m.data[1][2] = -A.data[2][1];
	m.data[2][1] = -A.data[1][2];
	double s = m.data[1][1] * A.data[2][2] - m.data[2][1] * A.data[1][2];
	return matrix_scale<r,c>(m, s);
}

template<int r, int c>
matrix<r, c> identity_matrix(){
	matrix<r, c> m;
	for (int i = 0; i < r; ++i){
		m.data[i][i] = 1.0;
	}
	return m;
}


template<int r, int c>
matrix<r, c> matrix_create(double arr[r*c]){
	matrix<r, c> m;
  for (int i = 0; i < r; ++i){
  	for (int j = 0; j < c; ++j){
  		m.data[i][j] = arr[i*c+j];
  	}
  }
  return m;
}