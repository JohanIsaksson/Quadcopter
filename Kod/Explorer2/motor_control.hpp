#include <Arduino.h>

class MotorControl
{
public:
  MotorControl();
  void Setup();

  void SetSpeedFrontLeft(uint16_t speed);
  void SetSpeedFrontRight(uint16_t speed);
  void SetSpeedBackLeft(uint16_t speed);
  void SetSpeedFBackLeft(uint16_t speed);

private:
  volatile uint16_t updatePeriod;
};
