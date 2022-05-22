// Science Motor Controller Software
// 2022 Prometheus, Mars Rover Design Team
// Written by Faye Squires
// #RoveSoHard

#include "ScienceMotorController.h"

void setup()
{
    Serial.begin(SERIAL_BAUD);
    RoveComm.begin(RC_SCIENCEACTUATIONBOARD_FOURTHOCTET, &TCPServer);

    // attach motor controllers to pins
    sensZ.attach(MOTOR_1_IN_A, MOTOR_1_IN_B, MOTOR_1_PWM);
    gantX.attach(MOTOR_2_IN_A, MOTOR_2_IN_B, MOTOR_2_PWM);
    gantZ.attach(MOTOR_3_IN_A, MOTOR_3_IN_B, MOTOR_3_PWM);

    // start watchdog for mocos
    watchdog.attach(eStop);
    watchdog.start(WATCHDOG_TIMEOUT);

    // attach PID controllers
    // gantXPID.attach(-1000f, 1000f, GANTX_KP, GANTX_KI, GANTX_KD);

    // start encoder trackers
    sensZEnc.attach(ENC_1);
    sensZEnc.start();
    gantXEnc.attach(ENC_2);
    gantXEnc.start();
    gantZEnc.attach(ENC_3);
    gantZEnc.start();

    // set output pin modes for servos and solenoids
    pinMode(SERVO_1, OUTPUT);
    pinMode(SERVO_2, OUTPUT);

    for (int i = 0; i < 9; i++)
    {
        pinMode(sol[i], OUTPUT);
    }
}

void loop()
{
    packet = RoveComm.read();

    switch (packet.data_id)
    {
    case RC_SCIENCEACTUATIONBOARD_SENSORAXIS_DATA_ID:
    {
        // set sensor axis target value
        sensZTarget = ((int16_t *)packet.data)[0];

        watchdog.clear();
        break;
    }
    case RC_SCIENCEACTUATIONBOARD_WATER_DATA_ID:
    {
        uint16_t solTarget = ((uint16_t *)packet.data)[0];
        // increment through 3 solenoid groups
        for (int i = 0; i < 9; i++)
        {
            // check corresponding bit then shift right 1 bit
            digitalWrite(sol[i], (solTarget & (1 << i)) >= 1);
        }

        watchdog.clear();
        break;
    }
    case RC_SCIENCEACTUATIONBOARD_XOOPAXIS_DATA_ID:
    {
        gantXTarget = ((int16_t *)packet.data)[0]; // set gantry X axis target value

        // gantXTarget = gantXPID.incrementPid(gantXPos[((int16_t*)packet.data)[0]], gantXEnc.readDegrees(), GANTX_TOL);
        watchdog.clear();
        break;
    }
    case RC_SCIENCEACTUATIONBOARD_ZOOPAXIS_DATA_ID:
    {
        gantZTarget = ((int16_t *)packet.data)[0]; // set gantry Z axis target value

        watchdog.clear();
        break;
    }
    case RC_SCIENCEACTUATIONBOARD_SCOOPGRABBER_DATA_ID:
    {
        scoopAngle = ((uint8_t *)packet.data)[0];

        // check if scoop needs to change angle (repeatedly setting servo to the same value caused shakiness in testing)
        if (scoopAngle != lastScoopAngle)
        {
            analogWrite(SERVO_2, scoopAngle == 0 ? SCOOP_OPEN_VALUE : SCOOP_CLOSED_VALUE); // set PWM pin to appropriate value
            lastScoopAngle = scoopAngle;
        }

        watchdog.clear();
        break;
    }
    default:
    {
        break;
    }
    }

    // TODO: add something to stop axes drifting due to gravity when target set to 0 using encoders (PID?)
    // set each moco to target
    sensZ.drive(sensZTarget);
    gantX.drive(gantXTarget);
    gantZ.drive(gantZTarget);

    // set limitStates bitmask
    limitStates = 0;
    for (int i = 0; i < 6; i++)
    {
        // increment bitmask by 1 if current limit is triggered then shift left 1 bit
        if (digitalRead(lim[i]))
            limitStates += 1;
        limitStates << 1;
    }
}
//
//// send telemetry data
// void telemetry()
//{
//   // TODO: encoder position telemetry
//   RoveComm.writeReliable(RC_SCIENCEACTUATIONBOARD_LIMITSWITCHTRIGGERED_DATA_ID, 1, limitStates);  // send limitStates bitmask
//   telemTimer.clear();
// }

// stop all motors
void eStop()
{
    sensZTarget = 0;
    gantXTarget = 0;
    gantZTarget = 0;
    watchdog.clear();
}
