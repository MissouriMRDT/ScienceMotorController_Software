#ifndef SCIENCEACTUATION_H
#define SCIENCEACTUATION_H

#include <RoveComm.h>
#include <RoveHBridge.h>
#include <LimitSwitch.h>
#include <RovePIDController.h>
#include <RoveJoint.h>

#include <Servo.h>

#include <cstdint>


// RoveComm declarations
RoveCommEthernet RoveComm;
EthernetServer TCPServer(RC_ROVECOMM_ETHERNET_TCP_PORT);

// Watchdog declarations
#define WATCHDOG_TIMEOUT 300000
IntervalTimer Watchdog;

#define TELEMETRY_PERIOD 150000
IntervalTimer Telemetry;



// Motor Pins
#define FWD_PWM_1   25
#define RVS_PWM_1   24
#define FWD_PWM_2   10
#define RVS_PWM_2   9
#define FWD_PWM_3   12
#define RVS_PWM_3   11
#define FWD_PWM_4   8
#define RVS_PWM_4   7
#define PUMP_PWM    6

// Servo Pins
#define SERVO_1     18
#define SERVO_2     19
#define SERVO_3     22
#define SERVO_4     23

// Encoder Pins
#define ENC_1   0
#define ENC_2   1
#define ENC_3   2
#define ENC_4   3
#define ENC_5   4

// Limit Swtich Pins
#define LIM_1   40
#define LIM_2   39
#define LIM_3   38
#define LIM_4   37
#define LIM_5   36
#define LIM_6   35
#define LIM_7   34
#define LIM_8   33

// Manual Switch Pins
#define SW_FWD  16
#define SW_RVS  17

// Motor Button pins
#define MOTOR_SW_1  32
#define MOTOR_SW_2  31
#define MOTOR_SW_3  30
#define MOTOR_SW_4  29
#define MOTOR_SW_5  28

// Servo Button pins
#define SERVO_SW_1  15
#define SERVO_SW_2  14
#define SERVO_SW_3  13
#define SERVO_SW_4  41


// Servos
Servo Servo1, Servo2, Servo3, Servo4;

#define Scoop Servo1
#define PumpMUX Servo2


// Motors
RoveHBridge Motor1(FWD_PWM_1, RVS_PWM_1);
RoveHBridge Motor2(FWD_PWM_2, RVS_PWM_2);
RoveHBridge Motor3(FWD_PWM_3, RVS_PWM_3);
RoveHBridge Motor4(FWD_PWM_4, RVS_PWM_4);

// Limit Switches
LimitSwitch LS1(LIM_1);
LimitSwitch LS2(LIM_2);
LimitSwitch LS3(LIM_3);
LimitSwitch LS4(LIM_4);
LimitSwitch LS5(LIM_5);
LimitSwitch LS6(LIM_6);
LimitSwitch LS7(LIM_7);
LimitSwitch LS8(LIM_8);

// PID Controllers


// Joints
RoveJoint ScoopX(&Motor1);
RoveJoint ScoopZ(&Motor3);
RoveJoint ScienceZ(&Motor2);
#define MICROSCOPE Motor4


// Variables
int16_t decipercents[4] = {0, 0, 0, 0};
bool pumpOutput = 0;
uint8_t limitSwitchValues = 0;
uint16_t jointAngles[3] = {0, 0, 0};
uint8_t scoopTarget = 120;
uint8_t pumpMUXTarget = 90;
bool manualButtons[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
bool lastManualButtons[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
bool manualForward = true;

// Methods
void updateLimitSwitchValues();
//void updateJointAngles();

void telemetry();
void estop();
void feedWatchdog();

#endif
