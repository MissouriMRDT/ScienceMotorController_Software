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
  int16_t error = (targetPos - pos) / 1000;
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
  posEncTicks += schmovement;
  pos = map(posEncTicks, 0, hiLimitPos, 0, 1000);
}

void VNHAxis::cycle() {
  cycleTime = (lastTime == 0) ? 0 : millis() - lastTime;
  lastTime = millis();
  updateInputs();
  switch(homingState) {
    case 0: vel = computePID();

    case 1: {
      if(isLoLimit) {
        homingState = 2;
        vel = 0;
        posEncTicks = 0;
      } else vel = -homingSpeedHi;
    }

    case 2: {
      if(posEncTicks >= 360000) {
        homingState = 3;
        vel = 0;
      } else vel = homingSpeedHi;
    }

    case 3: {
      if(isLoLimit) {
        homingState = 4;
        vel = 0;
        posEncTicks = 0;
      } else vel = -homingSpeedLo;
    }

    case 4: {
      if(isHiLimit) {
        homingState = 5;
        vel = 0;
        hiLimitPos = posEncTicks;
      } else vel = homingSpeedHi;
    }

    case 5: {
      if(posEncTicks <= hiLimitPos - 360000) {
        homingState = 6;
        vel = 0;
      } else vel = -homingSpeedHi;
    }

    case 6: {
      if(isHiLimit) {
        homingState = 0;
        isCalibrated = true;
        vel = 0;
        hiLimitPos = posEncTicks;
      } else vel = homingSpeedLo;
    }
  }

  if(isLoLimit && vel < 0) vel = 0;
  if(isHiLimit && vel > 0) vel = 0;

  if(vel != 0) motor.drive(vel);
  else motor.brake(brakePower);
}