#include "matrix_lib.h"
#include <stdio.h>
#include <iostream>
#include <string>


void print_matrix(matrix A, std::string s){
  printf("%s: (%d,%d)\n", s.c_str(), A.rows, A.columns);

  for (int i = 0; i < A.rows; i++){
    for (int j = 0; j < A.columns; j++){
      printf("%f\t", A.data[i*A.columns + j]);
    }
    printf("\n");
  }
  printf("----------------------------------------\n");
}

int main(){
  printf("---------------------\n");
  printf("Starting matrix test.\n");
  printf("---------------------\n");

  double dataA[9] = {1.0,    2.0,    3.0,
                     4.0,    5.0,    6.0,
                     7.0,    8.0,    9.0};
  matrix A = matrix_create(3,3,dataA);
  print_matrix(A, "A");
    
  double dataB[9] = {1.0,  4.0,  7.0,
                     2.0,  5.0,  8.0,
                     3.0,  6.0,  9.0};
  matrix B = matrix_create(3,3,dataB);
  print_matrix(B, "B");

  double dataC[9] = {0.0,  0.0,  0.0,
                     0.0,  0.0,  0.0,
                     0.0,  0.0,  0.0};
  matrix C = matrix_create(3,3,dataC);

  double dataD[9] = {0.0,  0.0,  0.0,
                     0.0,  0.0,  0.0,
                     0.0,  0.0,  0.0};
  matrix D = matrix_create(3,3,dataD);

  double dataE[9] = {0.0,  0.0,  0.0,
                     0.0,  0.0,  0.0,
                     0.0,  0.0,  0.0};
  matrix E = matrix_create(3,3,dataE);

  double dataF[6] = {1.0,  2.0,  3.0,
                     4.0,  5.0,  6.0};
  matrix F = matrix_create(2,3,dataF);

  double dataG[6] = {0.0,  0.0,  
                     0.0,  0.0,  
                     0.0,  0.0};
  matrix G = matrix_create(3,2,dataG);

  matrix_add(C, A, B);
  matrix_multiply(D, A, B);
  matrix_transpose(E, A);

  //matrix F = matrix_create_identity(3,3);
  matrix_transpose(G, F);

  print_matrix(C, "C");
  print_matrix(D, "D");
  print_matrix(E, "E");
  print_matrix(F, "F");
  print_matrix(G, "G");


  printf("---------------------\n");
  printf("Ending matrix test.\n");
  printf("---------------------\n");

  

  return 0;
}