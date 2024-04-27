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
uint8_t watchdogStatus = 0;
uint8_t watchdogOverride = 0;

#define TELEMETRY_PERIOD 300000
IntervalTimer Telemetry;



// Servos
Servo Servo1, Servo2;

// Motors
RoveHBridge Motor1(MOCO1_FWD, MOCO1_RVS);
RoveHBridge Motor2(MOCO2_FWD, MOCO2_RVS);
RoveHBridge Motor3(MOCO3_FWD, MOCO3_RVS);
RoveHBridge Motor4(MOCO4_FWD, MOCO4_RVS);

// Limit Switches
LimitSwitch LS1(LIMITSWITCH1);
LimitSwitch LS2(LIMITSWITCH2);
LimitSwitch LS3(LIMITSWITCH3);
LimitSwitch LS4(LIMITSWITCH4);
LimitSwitch LS5(LIMITSWITCH5);
LimitSwitch LS6(LIMITSWITCH6);

// Encoders
RoveQuadEncoder Encoder1(ENCODER_1A, ENCODER_1B, 360);
RoveQuadEncoder Encoder2(ENCODER_2A, ENCODER_2B, 360);
RoveQuadEncoder Encoder3(ENCODER_3A, ENCODER_3B, 360);

// Joints
RoveJoint ScoopAxis(&Motor2); 
RoveJoint SensorAxis(&Motor3);
#define Auger Motor4
#define SpareMotor Motor1
#define Microscope Servo2
#define SpareServo Servo1

// Control variables
int16_t ScoopAxisDecipercent = 0;
int16_t SensorAxisDecipercent = 0;
int16_t AugerDecipercent = 0;
uint8_t MicroscopePosition = 90;


// Methods
void telemetry();
void estop();
void feedWatchdog();

#endif
