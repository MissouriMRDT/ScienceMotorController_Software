#ifndef SCIENCEACTUATION_H
#define SCIENCEACTUATION_H

#include <RoveComm.h>
#include <RoveHBridge.h>
#include <MA3PWM.h>
#include <LimitSwitch.h>
#include <RovePIDController.h>
#include <RoveJoint.h>

#include <Servo.h>

#include <cstdint>


// RoveComm declarations
RoveCommEthernet RoveComm;
EthernetServer TCPServer(RC_ROVECOMM_SCIENCEACTUATIONBOARD_PORT);

// Watchdog declarations
#define WATCHDOG_TIMEOUT 300000
IntervalTimer Watchdog;



// Motor Pins
#define FWD_PWM_1   7
#define RVS_PWM_1   6
#define FWD_PWM_2   5
#define RVS_PWM_2   4
#define FWD_PWM_3   3
#define RVS_PWM_3   2
#define FWD_PWM_4   1
#define RVS_PWM_4   0

// Servo Pins
#define SERVO_1     8
#define SERVO_2     9
#define SERVO_3     10
#define SERVO_4     13
#define SERVO_5     12
#define SERVO_6     11

// Encoder Pins
#define ENC_1   14
#define ENC_2   15
#define ENC_3   18
#define ENC_4   19

// Limit Swtich Pins
#define LIM_1   23
#define LIM_2   22
#define LIM_3   21
#define LIM_4   20
#define LIM_5   17
#define LIM_6   16
#define LIM_7   41
#define LIM_8   40
#define LIM_9   39

// Manual Switch Pins
#define SW_FWD  38
#define SW_RVS  37

// Motor Button pins
#define MOTOR_SW_1  27
#define MOTOR_SW_2  26
#define MOTOR_SW_3  25
#define MOTOR_SW_4  24

// Servo Button pins
#define SERVO_SW_1  36
#define SERVO_SW_2  35
#define SERVO_SW_3  34
#define SERVO_SW_4  33
#define SERVO_SW_5  31
#define SERVO_SW_6  32


// Servos
Servo Servo1, Servo2, Servo3, Servo4, Servo5, Servo6;

// Motors
RoveHBridge Motor1(FWD_PWM_1, RVS_PWM_1);
RoveHBridge Motor2(FWD_PWM_2, RVS_PWM_2);
RoveHBridge Motor3(FWD_PWM_3, RVS_PWM_3);
RoveHBridge Motor4(FWD_PWM_4, RVS_PWM_4);

// Encoders
MA3PWM Encoder1(ENC_1);
MA3PWM Encoder2(ENC_2);
MA3PWM Encoder3(ENC_3);
MA3PWM Encoder4(ENC_4);

// Limit Switches
LimitSwitch LS1(LIM_1);
LimitSwitch LS2(LIM_2);
LimitSwitch LS3(LIM_3);
LimitSwitch LS4(LIM_4);
LimitSwitch LS5(LIM_5);
LimitSwitch LS6(LIM_6);
LimitSwitch LS7(LIM_7);
LimitSwitch LS8(LIM_8);
LimitSwitch LS9(LIM_9);

// PID Controllers


// Joints
RoveJoint ScoopX(&Motor1);
RoveJoint ScoopZ(&Motor2);
RoveJoint ScienceZ(&Motor3);


// Variables
uint8_t limitSwitchValues = 0;
uint16_t jointAngles[3] = {0, 0, 0};
uint8_t scoopTarget = 120;
bool manualButtons[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
bool lastManualButtons[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
bool manualForward = true;

// Methods
void updateLimitSwitchValues();
void updateJointAngles();
void updateManualButtons();

void estop();
void feedWatchdog();

#endif
