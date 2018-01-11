#include "matrix_lib.h"
#include <stdio.h>


void print_matrix(matrix A){
  printf("Rows: %d\nColumns: %d\n", A.rows, A.columns);


  for (int i = 0; i < A.rows; i++){
    for (int j = 0; j < A.columns; j++){
      printf("%f\t", A.data[i*A.columns + j]);
    }
    printf("\n");
  }
}

int main(){
  printf("---------------------\n");
  printf("Starting matrix test.\n");
  printf("---------------------\n");

  double dataA[9] = {1.0,    2.0,    3.0,
                     4.0,    5.0,    6.0,
                     7.0,    8.0,    9.0};
  matrix A = matrix_create(3,3,dataA);
  print_matrix(A);
    
  double dataB[9] = {1.0,  4.0,  7.0,
                     2.0,  5.0,  8.0,
                     3.0,  6.0,  9.0};
  matrix B = matrix_create(3,3,dataB);
  print_matrix(B);


  matrix C = A + B;
  matrix D = A * B;
  matrix E = matrix_transpose(A);

  matrix F = matrix_create_identity(3,3);
  matrix G = F * E;

  print_matrix(C);
  print_matrix(D);
  print_matrix(E);
  print_matrix(F);
  print_matrix(G);

  printf("---------------------\n");
  printf("Deleting matrices.\n");
  printf("---------------------\n");

  /*matrix_delete(A);
  matrix_delete(B);*/
  matrix_delete(C);
  matrix_delete(D);
  matrix_delete(E);
  matrix_delete(F);
  matrix_delete(G);

  printf("---------------------\n");
  printf("Ending matrix test.\n");
  printf("---------------------\n");

  

  return 0;
}