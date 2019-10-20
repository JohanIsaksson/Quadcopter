
#pragma GCC optimize("-O3")

#include "imu.h"
#include "pid.h"
#include "sbus_decoder.hpp"
#include <Arduino.h>

#define SPEED_MIN 1000
#define SPEED_MAX 2000

#define REF_MAX_HORIZON 25.0
#define REF_MAX_ACRO 90.0
#define REF_MAX_YAW 180.0

#define INTEGRAL_MAX 150.0

#define PARAM_ADDR 0

#define LOST_CONNECTION_COUNT 100
#define LOST_CONNECTION_COUNT2 12000
#define EMERGENCY_THROTTLE 0

// Flight modes
#define MODE_HORIZON 0
#define MODE_ACRO 1
#define MODE_ALT_HOLD 2

// Arming modes
#define UNARMED 0
#define ARMED 1
#define UNSAFE 2


class Explorer
{
private:
  // Debug mode (prints via serial)
  //#define DEBUG

  // Inertial measurement unit
  IMU imu;

  // Motor signals
  int throttle[4]; /* 0 = front left
                      1 = front right
                      2 = back left
                      3 = back right
                    */

  // Motor pins
  #define fl_pin 10 //front left
  #define fr_pin 11 //front right
  #define bl_pin 12 //back left
  #define br_pin 13 //back right

  // Faster IO for samd21
  #ifdef _VARIANT_ARDUINO_ZERO_
    volatile uint32_t *set_fl = &PORT->Group[g_APinDescription[fl_pin].ulPort].OUTSET.reg;
    volatile uint32_t *clr_fl = &PORT->Group[g_APinDescription[fl_pin].ulPort].OUTCLR.reg;
    const uint32_t  fl_MASK = (1ul << g_APinDescription[fl_pin].ulPin);

    volatile uint32_t *set_fr = &PORT->Group[g_APinDescription[fr_pin].ulPort].OUTSET.reg;
    volatile uint32_t *clr_fr = &PORT->Group[g_APinDescription[fr_pin].ulPort].OUTCLR.reg;
    const uint32_t  fr_MASK = (1ul << g_APinDescription[fr_pin].ulPin);

    volatile uint32_t *set_bl = &PORT->Group[g_APinDescription[bl_pin].ulPort].OUTSET.reg;
    volatile uint32_t *clr_bl = &PORT->Group[g_APinDescription[bl_pin].ulPort].OUTCLR.reg;
    const uint32_t  bl_MASK = (1ul << g_APinDescription[bl_pin].ulPin);

    volatile uint32_t *set_br = &PORT->Group[g_APinDescription[br_pin].ulPort].OUTSET.reg;
    volatile uint32_t *clr_br = &PORT->Group[g_APinDescription[br_pin].ulPort].OUTCLR.reg;
    const uint32_t  br_MASK = (1ul << g_APinDescription[br_pin].ulPin);
  #endif

  // PID outputs for each axis
  int front;
  int left;
  int cw;

  // Horizon mode stabilzation outputs
  float pitch_stab;
  float roll_stab;
  float yaw_stab;

  // PID controllers for each axis
  Pid pitchRatePID, rollRatePID, yawRatePID;
  Pid pitchStabilityPID, rollStabilityPID, yawStabilityPID;

  // PID parameters
  float P_pitch_a, I_pitch_a, D_pitch_a,
          P_pitch_h, I_pitch_h, D_pitch_h,
          P_roll_a, I_roll_a, D_roll_a,
          P_roll_h, I_roll_h, D_roll_h,
          P_yaw, I_yaw, D_yaw;

  // Altitude hold PID
  Pid pid_altitude_hold, pid_altitude_rate;
  float P_altitude_r, I_altitude_r, D_altitude_r,
          P_altitude_h, I_altitude_h, D_altitude_h;

  // PID outputs for altitude hold
  int altitude_rate_out;
  float altitude_hold_out;

  // altitude hold variables
  float altitude_setpoint;
  int previous_input_channel_3;
  bool altitude_startup;

  // Maximum angle and rotation sppeds
  uint16_t max_pitch, max_roll, max_yaw;
  uint16_t max_p, max_r, max_y;

  // L.O.S. safety counter
  int radio_off_counter;
  bool failSafe; 

  // TODO: Air mode throttle
  int air_mode_throttle;
  //#define AIR_MODE

  // Time keeping
  uint32_t time_diff;
  uint32_t time_last;
  float timed;

  // Radio variables
  SbusDecoder rx;

  float rad_roll, rad_pitch, rad_yaw;
  volatile uint16_t rad_throttle;

  // Motor control variables
  bool motors_on, disable_sticks;
  uint32_t esc_time, start_time, end_time;

  // Arming control
  uint8_t arm_mode;
  uint8_t next_arm_mode;

  // Flight mode control
  uint8_t flight_mode;
  uint8_t previous_flight_mode;

  // Yaw control
  bool set_yaw;

  // Serial bus for SAMD21
  #define DEBUG
  #ifdef DEBUG
  #define SerialPort SerialUSB
  int print_cnt = 0;
  #endif

  float map_f(float x, float in_min, float in_max, float out_min, float out_max);
  void init_motors();
  uint16_t limit(uint16_t x, uint16_t a, uint16_t b);

  void start_motor_pulses();
  void stop_motor_pulses();
  void set_motor_speeds();

  void set_motor_speeds_min();
  void set_motor_speeds_max();

  void Update_horizon(float t);
  void Update_acro(float t);
  void Update_altitude_hold(float t);

  void Print();


public:
  Explorer();
  void Update();


};
