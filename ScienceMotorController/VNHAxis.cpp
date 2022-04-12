/*
VNHAxis
Mars Rover Design Team
Missouri S&T, 2022
Faye Squires

This class is designed to drive a single axis actuator using a VNH5019A Motor Controller with feedback from an encoder using the
RoveUsDigiMa3Pwm library.
*/

#include "VNHAxis.h"

#include "RoveStmVnhPwm.h"
#include "RoveUsDigiMa3Pwm.h"

#include "Energia.h"

#include <stdlib.h>

void VNHAxis::home() {
  homingState = 1;
}

float VNHAxis::computePID() {
  int16_t error = (targetPos - map(pos, 0, hiLimitPos, 0, 1000)) / 1000;
  int16_t derivative = (error - lastError) / cycleTime;
  eIntegral = (abs(error) < deadZone) ? 0 : eIntegral + (error * cycleTime);
  return (kP * error) + (kI * eIntegral) - (kD * derivative);
}

void VNHAxis::updateInputs() {
  isLoLimit = digitalRead(loLimitPin);
  isHiLimit = digitalRead(hiLimitPin);
  encAngle = enc.readDegrees();
  if(encAngle - lastEncAngle > 180000) schmovement = encAngle - lastEncAngle - 360000;
  else if(encAngle - lastEncAngle < -180000) schmovement = encAngle - lastEncAngle + 360000;
  else schmovement = encAngle - lastEncAngle;
  pos += schmovement;
}

void VNHAxis::cycle() {
  cycleTime = (lastTime == 0) ? 0 : millis() - lastTime;
  lastTime = millis();
  updateInputs();
  switch(homingState) {
    case 1: {
      if(isLoLimit) {
        homingState = 2;
        motor.brake(brakePower);
        pos = 0;
      } else motor.drive(-homingSpeedHi);
    }

    case 2: {
      if(pos >= 360000) {
        homingState = 3;
        motor.brake(brakePower);
      }
    }
  }
    motor.drive(-homingSpeedHi);
  } else if(homingState == 1 && isLoLimit) {
    homingState = 2;
    motor.brake(brakePower);
    pos = 0;
  } else if(homingState == 2 && !isHiLimit) {
    motor.drive(homingSpeedHi);
  } else if(homingState ==2 && isHiLimit) {
    homingState = 0;
    motor.brake(brakePower);
    hiLimitPos = pos;
  } else if(homingState == 0) {
    motor.drive(computePID());
  }
}