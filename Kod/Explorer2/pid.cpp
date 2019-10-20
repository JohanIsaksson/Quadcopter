
#include "pid.h"


void Pid::Init()
{
  integral = 0.0;
  error_prev = 0.0;
  enable_integral = true;
}

void Pid::SetConstants(float KP_, float KI_, float KD_, float INT_MAX_)
{
  KP = KP_;
  KI = KI_;
  KD = KD_;
  INTEGRAL_MAX = INT_MAX_;
}

float Pid::Update(float ref, float mea, float dt, float scale)
{
  error = ref - mea;

  //TODO:
  //  windup handling - done
  //  D filtering

  if (error < 20.0) integral += error*dt;

  p = KP * error;
  i = KI * integral;
  d = KD * (error - error_prev) / dt;

  output =  p + i + d;

  //limit pitch
  if (output > 0.0){
    if (output > PID_MAX){
      output = PID_MAX;
    }
  }else{
    if (-output > PID_MAX){
      output = -PID_MAX;
    }
  }
  error_prev = error;

  //set values
  return scale*output;
}


