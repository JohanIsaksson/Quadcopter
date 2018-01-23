#include "matrix_lib.h"
#include <stdio.h>
#include <iostream>
#include <string>
#include "kalman.h"



int main(){
  printf("---------------------\n");
  printf("Starting kalman test.\n");
  printf("---------------------\n");

  Kalman k;
  k.InitPreset();
  k.Print();

  for (int i = 0; i < 100; ++i)
  {
    k.Update(0.0, 0.98, 0.0004);
    printf("\n");
    printf("\n");
    printf("%d\n",i);
    printf("\n");
    k.Print();
  }


  printf("---------------------\n");
  printf("Ending matrix test.\n");
  printf("---------------------\n");

  

  return 0;
}