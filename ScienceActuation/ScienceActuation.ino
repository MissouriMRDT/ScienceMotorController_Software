#include "ScienceActuation.h"

void setup() {
    Serial.begin(115200);

    pinMode(SW1, INPUT);
    pinMode(SW2, INPUT);
    pinMode(SW3, INPUT);
    pinMode(SW4, INPUT);
    pinMode(SW5, INPUT);
    pinMode(SW6, INPUT);
    pinMode(SW7, INPUT);
    pinMode(SW8, INPUT);
    pinMode(DIR_SW, INPUT);

    pinMode(PUMP1, OUTPUT);
    pinMode(PUMP2, OUTPUT);

    ScoopAxis.attachHardLimits(&LS1, &LS2);
    SensorAxis.attachHardLimits(&LS3, &LS4);

    LS1.configInvert(false);
    LS2.configInvert(false);
    LS3.configInvert(false);
    LS4.configInvert(false);
    LS5.configInvert(false);
    LS6.configInvert(false);

    Motor1.configInvert(false);
    Motor2.configInvert(false);
    Motor3.configInvert(false);

    Encoder1.configInvert(false);
    Encoder2.configInvert(false);
    Encoder3.configInvert(false);

    Encoder1.begin([]{ Encoder1.handleInterrupt(); });
    Encoder2.begin([]{ Encoder2.handleInterrupt(); });
    Encoder3.begin([]{ Encoder3.handleInterrupt(); });

    ScoopAxis.attachEncoder(&Encoder1);
    SensorAxis.attachEncoder(&Encoder2);

    Motor1.configMaxOutputs(-900, 900);
    Motor2.configMaxOutputs(-900, 900);
    Motor3.configMaxOutputs(-900, 900);

    Motor1.configMinOutputs(0, 0);
    Motor2.configMinOutputs(0, 0);
    Motor3.configMinOutputs(0, 0);

    Servo1.attach(SERVO_1, 500, 2500);
    Servo2.attach(SERVO_2, 500, 2500);

    RoveComm.begin(RC_SCIENCEACTUATIONBOARD_FIRSTOCTET, RC_SCIENCEACTUATIONBOARD_SECONDOCTET, RC_SCIENCEACTUATIONBOARD_THIRDOCTET, RC_SCIENCEACTUATIONBOARD_FOURTHOCTET, &TCPServer);
    Telemetry.begin(telemetry, TELEMETRY_PERIOD);
    feedWatchdog();
}

void loop() {
    // Parse RoveComm packets
    rovecomm_packet packet = RoveComm.read();
    switch (packet.data_id) {

        // Open loop control of ScienceZ
        case RC_SCIENCEACTUATIONBOARD_SENSORAXIS_DATA_ID:
        {
            int16_t data = ((int16_t*) packet.data)[0];

            decipercents[2] = data;

            feedWatchdog();
            break;
        }

        // Turn perisaltic pump on or off
        case RC_SCIENCEACTUATIONBOARD_WATERPUMP_DATA_ID:
        {
            uint8_t data = ((uint8_t*) packet.data)[0];

            pumpOutput = data;

            feedWatchdog();
            break;
        }

        // Override joint limit switches
        case RC_SCIENCEACTUATIONBOARD_LIMITSWITCHOVERRIDE_DATA_ID:
        {
            uint8_t data = ((uint8_t*) packet.data)[0];

            ScoopZ.overrideForwardHardLimit(data & (1<<5));
            ScoopZ.overrideReverseHardLimit(data & (1<<4));
            ScoopX.overrideReverseHardLimit(data & (1<<3));
            ScoopX.overrideForwardHardLimit(data & (1<<2));
            ScienceZ.overrideForwardHardLimit(data & (1<<1));
            ScienceZ.overrideReverseHardLimit(data & (1<<0));
            
            break;
        }

        // Open loop control of microscope
        case RC_SCIENCEACTUATIONBOARD_MICROSCOPEFOCUS_DATA_ID:
        {
            int16_t data = ((int16_t*) packet.data)[0];

            decipercents[3] = data;

            feedWatchdog();
            break;
        }

    }


    bool direction = digitalRead(DIR_SW);

    
    // ScoopX
    if (digitalRead(MOTOR_SW_1)) ScoopX.drive((direction ? -900 : 900));
    else ScoopX.drive(decipercents[0]);
        
    // ScoopZ
    if (digitalRead(MOTOR_SW_2)) ScoopZ.drive((direction ? -900 : 900));
    else ScoopZ.drive(decipercents[1]);
    
    // Spare Servos
    if (digitalRead(SERVO_SW_3)) Servo3.write((direction? 0 : 180));
    else Servo3.write(90);
    
    if (digitalRead(SERVO_SW_4)) Servo4.write((direction? 0 : 180));
    else Servo4.write(90);
}


void telemetry() {
    float positions[2] = { Encoder1.readDegrees(), Encoder2.readDegrees() };
    RoveComm.write(RC_SCIENCEACTUATIONBOARD_ENCODERPOSITIONS_DATA_ID, RC_SCIENCEACTUATIONBOARD_ENCODERPOSITIONS_DATA_COUNT, positions);

    uint8_t limitSwitchValues = (ScoopZ.atForwardHardLimit() << 5) | (ScoopZ.atReverseHardLimit() << 4) | (ScoopX.atReverseHardLimit() << 3) 
                                | (ScoopX.atForwardHardLimit() << 2) | (ScienceZ.atForwardHardLimit() << 1) | (ScienceZ.atReverseHardLimit() << 0);
    RoveComm.write(RC_SCIENCEACTUATIONBOARD_LIMITSWITCHTRIGGERED_DATA_ID, RC_SCIENCEACTUATIONBOARD_LIMITSWITCHTRIGGERED_DATA_COUNT, limitSwitchValues);
}


void estop() {

}

void feedWatchdog() {
    Watchdog.begin(estop, WATCHDOG_TIMEOUT);
}
