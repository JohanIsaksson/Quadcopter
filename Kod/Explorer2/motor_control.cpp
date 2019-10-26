#include "motor_control.hpp"

//----------------------------------------------------------------------------
MotorControl::MotorControl()
{
}

//----------------------------------------------------------------------------
void MotorControl::Setup()
{
  // put your setup code here, to run once:
  #define fl_pin 10 //front left
  #define fr_pin 11 //front right
  #define bl_pin 12 //back left
  #define br_pin 13 //back right
  pinMode(fl_pin, OUTPUT);
  pinMode(fr_pin, OUTPUT);
  pinMode(bl_pin, OUTPUT);
  pinMode(br_pin, OUTPUT);

  // Configure generic clocks to a frequency of 1 MHz for TCC0 and TCC2
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(48) | GCLK_GENDIV_ID(4);
  // Wait for synchronization
  while (GCLK->STATUS.bit.SYNCBUSY);

  // Enable GCLK4 and GCLK5, set the 48MHz clock source, select GCLK4
  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |
                     GCLK_GENCTRL_GENEN |
                     GCLK_GENCTRL_SRC_DFLL48M |
                     GCLK_GENCTRL_ID(4);
  // Wait for synchronization
  while (GCLK->STATUS.bit.SYNCBUSY);
  
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(48) | GCLK_GENDIV_ID(5);
  // Wait for synchronization
  while (GCLK->STATUS.bit.SYNCBUSY);
  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |
                     GCLK_GENCTRL_GENEN |
                     GCLK_GENCTRL_SRC_DFLL48M |
                     GCLK_GENCTRL_ID(5);;
  // Wait for synchronization
  while (GCLK->STATUS.bit.SYNCBUSY);

  // Enable the port multiplexer for the digital pin
  PORT->Group[g_APinDescription[fl_pin].ulPort].PINCFG[g_APinDescription[fl_pin].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[fr_pin].ulPort].PINCFG[g_APinDescription[fr_pin].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[bl_pin].ulPort].PINCFG[g_APinDescription[bl_pin].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[br_pin].ulPort].PINCFG[g_APinDescription[br_pin].ulPin].bit.PMUXEN = 1;
  
  //Connect the TCC0 timer to digital output
  PORT->Group[g_APinDescription[fl_pin].ulPort].PMUX[g_APinDescription[fl_pin].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;
  PORT->Group[g_APinDescription[fr_pin].ulPort].PMUX[g_APinDescription[fr_pin].ulPin >> 1].reg = PORT_PMUX_PMUXO_E | PORT_PMUX_PMUXE_E;
  PORT->Group[g_APinDescription[bl_pin].ulPort].PMUX[g_APinDescription[bl_pin].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;
  PORT->Group[g_APinDescription[br_pin].ulPort].PMUX[g_APinDescription[br_pin].ulPin >> 1].reg = PORT_PMUX_PMUXO_E | PORT_PMUX_PMUXE_E;

  // Feed GCLK4 to TCC0
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TTC1
  // Wait for synchronization
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Feed GCLK4 to TCC2
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0
                     GCLK_CLKCTRL_GEN_GCLK5 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC2_TC3;   // Feed GCLK4 to TCC0 and TC3
  // Wait for synchronization
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Set for Single slope PWM operation and reverse the output polarity on all outputs
  REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;
  while (TCC0->SYNCBUSY.bit.WAVE);
  REG_TCC2_WAVE |= TCC_WAVE_WAVEGEN_NPWM;
  while (TCC2->SYNCBUSY.bit.WAVE);

  // Set PWM frequency to 1000000 / 3000
  REG_TCC0_PER = 3000;
  // Wait for synchronization
  while (TCC0->SYNCBUSY.bit.PER);
  REG_TCC2_PER = 3000;
  // Wait for synchronization
  while (TCC2->SYNCBUSY.bit.PER);
  
  // Set initial PWM signal duty cycle to 33% (motors are turned off)
  REG_TCC0_CC2 = 1000;
  while (TCC0->SYNCBUSY.bit.CC2);
  REG_TCC2_CC0 = 1700;
  while (TCC2->SYNCBUSY.bit.CC0);
  REG_TCC0_CC3 = 1300;
  while (TCC0->SYNCBUSY.bit.CC3);
  REG_TCC2_CC1 = 2000;
  while (TCC2->SYNCBUSY.bit.CC1);

  // Set prescaler and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |
                    TCC_CTRLA_ENABLE;
  // Wait for synchronization
  while (TCC0->SYNCBUSY.bit.ENABLE);

  REG_TCC2_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |
                    TCC_CTRLA_ENABLE;
  // Wait for synchronization
  while (TCC2->SYNCBUSY.bit.ENABLE);
}

//----------------------------------------------------------------------------
void MotorControl::SetSpeedFrontLeft(uint16_t speed)
{
  REG_TCC0_CC2 = speed;
  while (TCC0->SYNCBUSY.bit.CC2);
}
//----------------------------------------------------------------------------
void MotorControl::SetSpeedFrontRight(uint16_t speed)
{
  REG_TCC2_CC0 = speed;
  while (TCC2->SYNCBUSY.bit.CC0);
}
//----------------------------------------------------------------------------
void MotorControl::SetSpeedBackLeft(uint16_t speed)
{
  REG_TCC0_CC3 = speed;
  while (TCC0->SYNCBUSY.bit.CC3);
}
//----------------------------------------------------------------------------
void MotorControl::SetSpeedFBackLeft(uint16_t speed)
{
  REG_TCC2_CC1 = speed;
  while (TCC2->SYNCBUSY.bit.CC1);
}