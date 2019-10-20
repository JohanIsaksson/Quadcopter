#include "kalman.h"

void fillMatrix(value v0, value v1, value v2, 
  value v3, value v4, value v5, 
  value v6, value v7, value v8,
  value* data){
  data[0] = v0;
  data[1] = v1;
  data[2] = v2;
  data[3] = v3;
  data[4] = v4;
  data[5] = v5;
  data[6] = v6;
  data[7] = v7;
  data[8] = v8;
}

void fillMatrix(value v0, value v1, 
  value v2, value v3,
  value* data){
  data[0] = v0;
  data[1] = v1;
  data[2] = v2;
  data[3] = v3;
}

void fillMatrix(value v0, value v1, value v2, 
  value* data){
  data[0] = v0;
  data[1] = v1;
  data[2] = v2;
}

void fillMatrix(value v0, value v1, value v2, 
  value v3, value v4, value v5, 
  value* data){
  data[0] = v0;
  data[1] = v1;
  data[2] = v2;
  data[3] = v3;
  data[4] = v4;
  data[5] = v5;
}

void fillMatrix(value v0, value v1,
  value* data){
  data[0] = v0;
  data[1] = v1;
}

//#define PLATFORM_WIN

#ifdef PLATFORM_WIN
#include <stdio.h>
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
#endif




void Kalman::Print(){
  #ifdef PLATFORM_WIN
  print_matrix(A,"A");
  print_matrix(At,"At");
  print_matrix(B,"B");
  print_matrix(H,"H");
  print_matrix(Ht,"Ht");
  print_matrix(P,"P");
  print_matrix(P_p,"P_p");
  print_matrix(Q,"Q");
  print_matrix(I,"I");
  print_matrix(K,"K");
  print_matrix(R,"R");
  print_matrix(x,"x");
  print_matrix(w,"w");
  print_matrix(x_p,"x_p");
  print_matrix(y,"y");

  print_matrix(Ax,"Ax");
  print_matrix(Bu,"Bu");
  print_matrix(Buw,"Buw");
  print_matrix(AP,"AP");
  print_matrix(APAt,"APAt");
  print_matrix(PHt,"PHt");
  print_matrix(HPHt,"HPHt");
  print_matrix(HPHtR,"HPHtR");
  print_matrix(HPHtR_,"HPHtR_");
  print_matrix(Hx,"Hx");
  print_matrix(yHx,"yHx");
  print_matrix(KyHx,"KyHx");
  print_matrix(KH,"KH");
  print_matrix(IKH,"IKH");
  #endif

}

void Kalman::InitPreset(){

  // Init kalman matrices

  A = matrix_create(3,3, dataA);
  matrix_identity(A);

  At = matrix_create(3,3, dataAt);
  matrix_transpose(At, A); 

  fillMatrix(0.0, 
             0.0, 
             1.0,
             dataB);
  B = matrix_create(3,1, dataB);

  fillMatrix(1.0, 0.0, 0.0, 
             0.0, 0.0, 1.0,
             dataH);
  H = matrix_create(2, 3, dataH);

  Ht = matrix_create(3,2, dataHt);
  matrix_transpose(Ht, H);

  fillMatrix(0.000992,    0.005000,    0.000126,
             0.000023,    0.000107,    0.000535,
             0.000069,    0.001700,    0.012100,
             dataP);
  P = matrix_create(3,3,dataP);

  P_p = matrix_create(3,3,dataP_p);
  matrix_zeroes(P_p);

  fillMatrix(0.0,       0.0001,     0.00001,
             0.0,       0.0,        0.0,
             0.0,       0.0001,     0.001,
             dataQ);
  Q = matrix_create(3,3,dataQ); 

  I = matrix_create(3,3,dataI);
  matrix_identity(I);

  K = matrix_create(3,2,dataK);
  matrix_zeroes(K);

  fillMatrix(0.05,       0.00044675,
             0.00044675, 0.1338,
             dataR);
  R = matrix_create(2,2,dataR);

  x = matrix_create(3,1,datax);
  matrix_zeroes(x);
  
  w = matrix_create(3,1,dataw);
  matrix_zeroes(w);

  x_p = matrix_create(3,1,datax_p);
  matrix_zeroes(x_p);

  y = matrix_create(2,1,datay);
  matrix_zeroes(y);


  // Init help matrices
  Ax = matrix_create(3,1,dataAx);
  matrix_zeroes(Ax);
  
  Bu = matrix_create(3,1,dataBu);
  matrix_zeroes(Bu);

  Buw = matrix_create(3,1,dataBuw);
  matrix_zeroes(Buw);

  AP = matrix_create(3,3,dataAP);
  matrix_zeroes(AP);

  APAt = matrix_create(3,3,dataAPAt);
  matrix_zeroes(APAt);
  
  PHt = matrix_create(3,2,dataPHt);
  matrix_zeroes(PHt);

  HPHt = matrix_create(2,2,dataHPHt);
  matrix_zeroes(HPHt);

  HPHtR = matrix_create(2,2,dataHPHtR);
  matrix_zeroes(HPHtR);

  HPHtR_ = matrix_create(2,2,dataHPHtR_);
  matrix_zeroes(HPHtR_);
  
  Hx = matrix_create(2,1,dataHx);
  matrix_zeroes(Hx);

  yHx = matrix_create(2,1,datayHx);
  matrix_zeroes(yHx);

  KyHx = matrix_create(3,1,dataKyHx);
  matrix_zeroes(KyHx);

  KH = matrix_create(3,3,dataKH);
  matrix_zeroes(KH);
  
  IKH = matrix_create(3,3,dataIKH);
  matrix_zeroes(IKH);

  // Gravity
  u = -0.98*9.82;
}

void Kalman::Update(value baro, value acc, value dt){

  // set A and B matrices 	
  A.data[1] = dt;
  A.data[5] = dt;
  A.data[2] = dt*dt/2.0;
  matrix_transpose(At, A);


  B.data[0] = dt*dt/2.0;
  B.data[1] = dt;
  B.data[2] = 1.0;


  // Predict new state 
  //x_p = A*x + B*u + w;
  matrix_multiply(x_p/*Ax*/, A, x);  // Ax = 3*1
  //matrix_scale(Bu, B, u);     // Bu = 3*1
  //matrix_add(Buw, Bu, w);     // Buw = 3*1
  //matrix_add(x_p, Ax, Bu);   // 3,1

  //P_p = A*P*At + Q;
  matrix_multiply(AP, A, P);      // 3,3
  matrix_multiply(APAt, AP, At);  // 3,3
  matrix_add(P_p, APAt, Q);       // 3,3


  // Measurment input 	
  y.data[0] = baro;
  y.data[1] = acc*9.82;


  // Calculate gain and new state
  //K = P_p*Ht/(H*P_p*Ht + R);
  matrix_multiply(PHt, P_p, Ht);  // 3,2
  matrix_multiply(HPHt, H, PHt);  // 2,2
  matrix_add(HPHtR, HPHt, R);     // 2,2
  matrix_inverse(HPHtR_, HPHtR);  // 2,2
  matrix_multiply(K, PHt, HPHtR_);// 3,2

  //x = x_p + K*(y - H*x_p);
  matrix_multiply(Hx, H, x_p);    // 2,1
  matrix_subtract(yHx, y, Hx);    // 2,1
  matrix_multiply(KyHx, K, yHx);  // 3,1
  matrix_add(x, x_p, KyHx);       // 3,1

  // Calculate uncertainty 
  //P = (I - K*H)*P_p;
  matrix_multiply(KH, K, H);      // 3,3
  matrix_subtract(IKH, I, KH);    // 3,3
  matrix_multiply(P, IKH, P_p);   // 3,3
}

void Kalman::Reset(bool hard)
{
  if (hard)
  {
    InitPreset();
  }
  else
  {
    x.data[0] = 0;
    x.data[1] = 0;
    x.data[2] = 0;
  }
}

double Kalman::GetAltitude(){
  return x.data[0];
}

double Kalman::GetVerticalSpeed(){
  return x.data[1];
}

double Kalman::GetVerticalAcceleration(){
  return x.data[2];
}

