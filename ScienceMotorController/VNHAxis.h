/*
VNHAxis
Mars Rover Design Team
Missouri S&T, 2022
Faye Squires

This class is designed to drive a single axis actuator using a VNH5019A Motor Controller with feedback from an encoder using the
RoveUsDigiMa3Pwm library.
*/

#ifndef VNH_AXIS
#define VNH_AXIS

#include "RoveUsDigiMa3Pwm.h"
#include "RoveStmVnhPwm.h"

#include "RoveComm.h"

#include <stdint.h>

class VNHAxis{
  public:
  void home();
  float computePID();
  void cycle();
  void updateInputs();
  void brake();
  void releaseBrake();

  uint8_t homingState; // 0: Not homing, 1: Homing low limit, 2: Homing high limit
  bool isLoLimit;
  bool isHiLimit;

  uint8_t loLimitPin;
  uint8_t hiLimitPin;

  RoveStmVnhPwm motor;
  RoveUsDigiMa3Pwm enc;
  
  int16_t pos;
  int16_t vel;
  int16_t homePos;
  int16_t hiLimitPos;
  int16_t encAngle;
  int16_t lastEncAngle;
  int16_t schmovement;
  int16_t targetPos;
  int16_t eIntegral;
  int16_t deadZone;
  int16_t lastError;
  uint32_t cycleTime;
  uint32_t lastTime;

  int16_t homingSpeedHi;
  int16_t homingSpeedLo;
  int16_t brakePower;

  float kP;
  float kI;
  float kD;

  VNHAxis( uint8_t motorInA, // Motor A pin
           uint8_t motorInB, // Motor B pin
           uint8_t motorPWM, // Motor PWM output Pin
           uint8_t encIn, // Encoder PWM input pin
           uint8_t loLimit, // Input pin for limit switch on low end
           uint8_t hiLimit, // Input pin for limit switch on high end
           float kP = 1.0, 
           float kI = 0.0, 
           float kD = 0.0,
           int16_t homingSpeedHi = 1000,
           int16_t homingSpeedLo = 200,
           int16_t brakePower = 1000 ) {
    this -> loLimitPin = loLimit;
    this -> hiLimitPin = hiLimit;

    this -> homingSpeedHi = homingSpeedHi;
    this -> homingSpeedLo = homingSpeedLo;
    
    this -> kP = kP;
    this -> kI = kI;
    this -> kD = kD;

    pos = 0;
    cycleTime = 0;
    lastTime = 0;

    motor.attach(motorInA, motorInB, motorPWM);
    enc.attach(encIn);

    home();
  }

};

#endif