#include "matrix_lib.h"
#include <stdlib.h>

matrix matrix_create(int r, int c, double* d){
  matrix m;
  m.rows = r;
  m.columns = c;
  m.data = d;
  return m; 
}

void matrix_delete(matrix A){
  free(A.data);
}

matrix matrix_multiply(matrix A, matrix B){
  double* data = (double*)malloc(A.rows*B.columns*sizeof(double));
  matrix m = matrix_create(A.rows, B.columns, data);    

  for (int i = 0; i < A.rows; ++i){
    for (int j = 0; j < B.columns; ++j){
      double s = 0;
      for (int k = 0; k < B.rows; ++k){
        s += (A.data[i*A.columns + k]) * (B.data[k*B.rows + j]);
      }
      m.data[i*m.columns + j] = s;
    }
  }
  return m;
}

matrix matrix_create_identity(int r, int c){
  double* data = (double*)malloc(r*c*sizeof(double));
  for (int i = 0; i < r; ++i){
    for (int j = 0; j < c; ++j){
      if (j == i){
        data[i*c + j] = 1.0;
      }else{
        data[i*c + j] = 0.0;
      }
    }
  }
  return matrix_create(r, c, data);
}

matrix matrix_scale(matrix A, double b){
  double* data = (double*)malloc(A.rows*A.columns*sizeof(double));
  matrix m = matrix_create(A.rows, A.columns, data);
  for (int i = 0; i < A.rows; ++i){
    for (int j = 0; j < A.columns; ++j){
      m.data[i*A.columns + j] = A.data[i*A.columns + j] * b;
    }
  }
  return m;
}

matrix matrix_add(matrix A, matrix B){
  double* data = (double*)malloc(A.rows*A.columns*sizeof(double));
  matrix m = matrix_create(A.rows, A.columns, data);
  for (int i = 0; i < A.rows; ++i){
    for (int j = 0; j < A.columns; ++j){
      m.data[i*A.columns + j] = A.data[i*A.columns + j] + B.data[i*A.columns + j];
    }
  }
  return m;
}


matrix matrix_subtract(matrix A, matrix B){
  double* data = (double*)malloc(A.rows*A.columns*sizeof(double));
  matrix m = matrix_create(A.rows, A.columns, data);
  for (int i = 0; i < A.rows; ++i){
    for (int j = 0; j < A.columns; ++j){
      m.data[i*A.columns + j] = A.data[i*A.columns + j] - B.data[i*A.columns + j];
    }
  }
  return m;
} 


matrix matrix_transpose(matrix A){
  double* data = (double*)malloc(A.rows*A.columns*sizeof(double));
  matrix m = matrix_create(A.columns, A.rows, data);
  for (int i = 0; i < A.rows; ++i){
    for (int j = 0; j < A.columns; ++j){
      m.data[i*A.columns + j] = A.data[j*A.columns + i];
    }
  }
  return m;
}


// only works for 2x2 matrices
matrix matrix_inverse(matrix A){
  double* data = (double*)malloc(A.rows*A.columns*sizeof(double));
  data[0] = A.data[0];
  data[3] = A.data[3];
  data[1] = -A.data[2];
  data[2] = -A.data[1];
  double s = data[0] * A.data[3] - data[2] * A.data[1];
  matrix B = matrix_create(A.rows, A.columns, data);
  matrix C = matrix_scale(B, 1.0/s);
  matrix_delete(B);
  return C;
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
  matrix C = matrix_inverse(B);
  matrix D = matrix_multiply(A, C);
  matrix_delete(C);
  return D;
}