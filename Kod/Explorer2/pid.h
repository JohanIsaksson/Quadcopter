#include <arduino.h>

#ifndef PID_H
#define PID_H

#define PID_MAX 254.0

class Pid
{
  float KP, KI, KD, INTEGRAL_MAX;

  //help variables
  float p;
  float i;
  float d;
  float error;
  float error_prev;
  float integral;
  float output;
  bool enable_integral;

public:
  void Init();
  void SetIntegration(bool b);
  void SetConstants(float KP_, float KI_, float KD_, float INT_MAX_);
  float Update(float ref, float mea, float dt, float scale);

  //testing variable
  double K_tmp;
};


#endif

