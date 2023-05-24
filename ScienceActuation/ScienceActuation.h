#ifndef SCIENCEACTUATION_H
#define SCIENCEACTUATION_H

#include "PinAssignments.h"

#include <RoveComm.h>
#include <RoveHBridge.h>
#include <LimitSwitch.h>
#include <RoveQuadEncoder.h>
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

// Encoders
RoveQuadEncoder Encoder1(ENC_1, ENC_2, 360.0);
RoveQuadEncoder Encoder2(ENC_3, ENC_4, 360.0);
RoveQuadEncoder Encoder3(ENC_5, LIM_8, 360.0);
ParallaxFeedback Encoder4(SERVO_4);

// PID Controllers
RovePIDController PID1(0.5, 0, 0);
RovePIDController PID2(0.5, 0, 0);
RovePIDController PID3(0, 0, 0);
RovePIDController PID4(0, 0, 0);

// Joints
RoveJoint ScoopX(&Motor1);
RoveJoint ScoopZ(&Motor3);
RoveJoint ScienceZ(&Motor2);
#define MICROSCOPE Motor4



// Closed loop
uint8_t scoopCalibrationMode = 0; // 0 for not calibrating, 1 for calibrating ScoopX, 2 for calibrating ScoopZ
bool scoopCalibrated = false;
bool scoopClosedLoopActive = false;
uint8_t scoopClosedLoopMode = 255;
enum ScoopClosedLoopMode {
    GROUND = 0,
    POSITION_1 = 1, // Funnels 1 and 2
    POSITION_2 = 2, // Funnels 3 and 4
    POSITION_3 = 3, // Funnels 5 and 6
    POSITION_4 = 4, // Funnels 7 and 8
    POSITION_5 = 5, // Funnels 9 and 10
    POSITION_6 = 6, // Funnels 11 and 12
    CALIBRATE = 7
};

#define SCOOP_DROP_HEIGHT 1500
#define SCOOP_DOWN_HEIGHT 18800
#define SCOOP_OUT_THRESHOLD -4000


bool pumpMUXClosedLoopActive = false;
uint8_t pumpMUXClosedLoopMode = 255;
bool pumpMUXCalibrated = false;


// Open Loop
int16_t decipercents[4] = {0, 0, 0, 0};
bool pumpOutput = 0;
uint8_t scoopTarget = 50;
int8_t pumpMUXOutput = 0;
bool manualButtons[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
bool lastManualButtons[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};


// Methods
void telemetry();
void estop();
void feedWatchdog();

#endif
