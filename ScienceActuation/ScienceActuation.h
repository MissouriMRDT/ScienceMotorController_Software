#ifndef SCIENCEACTUATION_H
#define SCIENCEACTUATION_H

#include "PinAssignments.h"
#include "CachedServo.h"

#include <RoveComm.h>
#include <RoveHBridge.h>
#include <RoveSwitch.h>
#include <LimitSwitch.h>
#include <RoveQuadEncoder.h>
#include <RovePIDController.h>
#include <RoveJoint.h>

#include <cstdint>

// Gimbal Servo Max/Mins
#define GIMBAL_TILT_MIN     45
#define GIMBAL_TILT_MAX     135
#define GIMBAL_PAN_MIN      40
#define GIMBAL_PAN_MAX      180

#define HUMIDITY_ADC_MIN        10
#define HUMIDITY_ADC_MAX        972
#define HUMIDITY_MAPPED_MIN     100.0
#define HUMIDITY_MAPPED_MAX     0.0

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
#define GIMBAL_PAN_PIN SERVO_1
#define GIMBAL_TILT_PIN SERVO_2
CachedServo GimbalPan(90, GIMBAL_PAN_MIN, GIMBAL_PAN_MAX);
CachedServo GimbalTilt(90, GIMBAL_TILT_MIN, GIMBAL_TILT_MAX);

// Motors
RoveHBridge MotorSpare(MOCO1_FWD, MOCO1_RVS);
RoveHBridge MotorAugerAxis(MOCO2_FWD, MOCO2_RVS);
RoveHBridge MotorSensorAxis(MOCO3_FWD, MOCO3_RVS);
RoveHBridge MotorAuger(MOCO4_FWD, MOCO4_RVS);

// cursed af hack
class CompoundSwitch : public RoveSwitch {
private:
    RoveSwitch &m_sw1;
    RoveSwitch &m_sw2;
public:
    CompoundSwitch(RoveSwitch &sw1, RoveSwitch &sw2) : m_sw1(sw1), m_sw2(sw2) {}
    RoveSwitch &Sw1() const { return m_sw1; }
    RoveSwitch &Sw1() { return m_sw1; }
    RoveSwitch &Sw2() const { return m_sw2; }
    RoveSwitch &Sw2() { return m_sw2; }
    bool read() const override {
        return m_sw1.read() || m_sw2.read();
    }
};

// Limit Switches
LimitSwitch LimitSensorAxisRVS(LIMITSWITCH1);
LimitSwitch LimitAugerAxisFWD(LIMITSWITCH2);
LimitSwitch LimitSensorAxisFWD(LIMITSWITCH3);
LimitSwitch LimitSpareRVS(LIMITSWITCH4);
LimitSwitch LimitLaserFWD(LIMITSWITCH5); // LimitSpareFWD
LimitSwitch LimitAugerAxisRVS(LIMITSWITCH6);

CompoundSwitch LimitSensorOrLaserFWD(LimitSensorAxisFWD, LimitLaserFWD);

// Encoders
RoveQuadEncoder EncoderAugerAxis(ENCODER_1A, ENCODER_1B, 360);
RoveQuadEncoder EncoderAuger(ENCODER_2A, ENCODER_2B, 360);
RoveQuadEncoder EncoderSpare(ENCODER_3A, ENCODER_3B, 360);
RoveQuadEncoder EncoderSensorAxis(ENCODER_4A, ENCODER_4B, 360);

// Joints
RoveJoint AugerAxis(&MotorAugerAxis); 
RoveJoint SensorAxis(&MotorSensorAxis);
#define Auger MotorAuger
#define SpareMotor MotorSpare

// Control variables
int16_t augerAxisDecipercent = 0;
int16_t sensorAxisDecipercent = 0;
int16_t augerDecipercent = 0;

uint8_t gimbalPanStartPosition = (GIMBAL_PAN_MAX-GIMBAL_PAN_MIN)/2+GIMBAL_PAN_MIN;
uint8_t gimbalTiltStartPosition = (GIMBAL_TILT_MAX-GIMBAL_TILT_MIN)/2+GIMBAL_TILT_MIN;


// Methods
float analogMap(uint16_t measurement, uint16_t fromADC, uint16_t toADC, float fromAnalog, float toAnalog);
void telemetry();
void estop();
void feedWatchdog();

#endif
