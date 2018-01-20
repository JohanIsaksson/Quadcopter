#include "matrix_lib.h"

matrix matrix_create(int r, int c, double* d){
  matrix m;
  m.rows = r;
  m.columns = c;
  m.data = d;
  return m; 
}

void matrix_zeroes(matrix A){
  for (int i = 0; i < A.rows; ++i){
    for (int j = 0; j < A.columns; ++j){
      A.data[i*A.columns + j] = 0.0;
    }
  }
}

void matrix_identity(matrix A){
  for (int i = 0; i < A.rows; ++i){
    for (int j = 0; j < A.columns; ++j){
      if (i == j){
        A.data[i*A.columns + j] = 1.0;
      }else{
        A.data[i*A.columns + j] = 0.0;
      }
    }
  }
}

void matrix_multiply(matrix R, matrix A, matrix B){
  double s;
  for (int i = 0; i < A.rows; ++i){
    for (int j = 0; j < B.columns; ++j){
      s = 0;
      for (int k = 0; k < B.rows; ++k){
        s += (A.data[i*A.columns + k]) * (B.data[k*B.rows + j]);
      }
      R.data[i*R.columns + j] = s;
    }
  }
}

void matrix_scale(matrix R, matrix A, double b){
  for (int i = 0; i < A.rows; ++i){
    for (int j = 0; j < A.columns; ++j){
      R.data[i*A.columns + j] = A.data[i*A.columns + j] * b;
    }
  }
}

void matrix_add(matrix R, matrix A, matrix B){
  for (int i = 0; i < A.rows; ++i){
    for (int j = 0; j < A.columns; ++j){
      R.data[i*A.columns + j] = A.data[i*A.columns + j] + B.data[i*A.columns + j];
    }
  }
}


void matrix_subtract(matrix R, matrix A, matrix B){
  for (int i = 0; i < A.rows; ++i){
    for (int j = 0; j < A.columns; ++j){
      R.data[i*A.columns + j] = A.data[i*A.columns + j] - B.data[i*A.columns + j];
    }
  }
} 


void matrix_transpose(matrix R, matrix A){
  for (int i = 0; i < A.rows; ++i){
    for (int j = 0; j < A.columns; ++j){
      R.data[i*A.columns + j] = A.data[j*A.columns + i];
    }
  }
}


// only works for 2x2 matrices
void matrix_inverse(matrix R, matrix A){
  R.data[0] = A.data[3];
  R.data[3] = A.data[0];
  R.data[1] = -A.data[2];
  R.data[2] = -A.data[1];
  double s = A.data[0] * A.data[3] - A.data[2] * A.data[1];
  matrix_scale(R, R, 1.0/s);
}


/*
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
  matrix C = matrix_inverse(B);
  matrix D = matrix_multiply(A, C);
  matrix_delete(C);
  return D;
}
*/
