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
        case RC_SCIENCEACTUATIONBOARD_SCOOPAXIS_OPENLOOP_DATA_ID:
        {
            int16_t data = *((int16_t*) packet.data);

            ScoopAxisDecipercent = data;

            feedWatchdog();

            break;
        }
        
        case RC_SCIENCEACTUATIONBOARD_SENSORAXIS_OPENLOOP_DATA_ID:
        {
            int16_t data = *((int16_t*) packet.data);

            SensorAxisDecipercent = data;

            feedWatchdog();

            break;
        }

        case RC_SCIENCEACTUATIONBOARD_SCOOPAXIS_SETPOSITION_DATA_ID:
        {




            break;
        }

        case RC_SCIENCEACTUATIONBOARD_SENSORAXIS_SETPOSITION_DATA_ID:
        {




            break;
        }

        case RC_SCIENCEACTUATIONBOARD_SCOOPAXIS_INCREMENTPOSITION_DATA_ID:
        {




            break;
        }

        case RC_SCIENCEACTUATIONBOARD_SENSORAXIS_INCREMENTPOSITION_DATA_ID:
        {




            break;
        }

        case RC_SCIENCEACTUATIONBOARD_AUGER_DATA_ID:
        {
            int16_t data = *((int16_t*) packet.data);
            
            AugerDecipercent = data;

            feedWatchdog();

            break;
        }

        case RC_SCIENCEACTUATIONBOARD_MICROSCOPE_DATA_ID:
        {




            break;
        }

        // Turn perisaltic pump on or off
        case RC_SCIENCEACTUATIONBOARD_WATERPUMP_DATA_ID:
        {




            break;
        }

        // Override joint limit switches
        case RC_SCIENCEACTUATIONBOARD_LIMITSWITCHOVERRIDE_DATA_ID:
        {
            uint8_t data = ((uint8_t*) packet.data)[0];

            ScoopAxis.overrideForwardHardLimit(data & (1<<0));
            ScoopAxis.overrideReverseHardLimit(data & (1<<1));
            SensorAxis.overrideForwardHardLimit(data & (1<<2));
            SensorAxis.overrideReverseHardLimit(data & (1<<3));
            
            break;
        }

    }


    bool direction = digitalRead(DIR_SW);

    
    // ScoopAxis
    if (digitalRead(SW1)) ScoopAxis.drive((direction ? -900 : 900));
    else ScoopAxis.drive(ScoopAxisDecipercent);
        
    // SensorAxis
    if (digitalRead(SW2)) SensorAxis.drive((direction ? -900 : 900));
    else SensorAxis.drive(SensorAxisDecipercent);

    // Auger
    if (digitalRead(SW3)) Auger.drive((direction ? -900 : 900));
    else Auger.drive(AugerDecipercent);

    // Spare Motor Port
    if (digitalRead(SW4)) Motor4.drive((direction ? -900 : 900));
    else Motor4.drive(0);

    // Spare Servos
    if (digitalRead(SW5)) Servo1.write((direction? 0 : 180));
    else Servo1.write(90);
    
    if (digitalRead(SW6)) Servo2.write((direction? 0 : 180));
    else Servo2.write(90);
    
}


void telemetry() {
    float positions[2] = { Encoder1.readDegrees(), Encoder2.readDegrees() };
    RoveComm.write(RC_SCIENCEACTUATIONBOARD_POSITIONS_DATA_ID, RC_SCIENCEACTUATIONBOARD_POSITIONS_DATA_COUNT, positions);

    uint8_t limitSwitchValues = (ScoopAxis.atForwardHardLimit() << 0) | (ScoopAxis.atReverseHardLimit() << 1) | (SensorAxis.atForwardHardLimit() << 2) | (SensorAxis.atReverseHardLimit() << 3);
    RoveComm.write(RC_SCIENCEACTUATIONBOARD_LIMITSWITCHTRIGGERED_DATA_ID, RC_SCIENCEACTUATIONBOARD_LIMITSWITCHTRIGGERED_DATA_COUNT, limitSwitchValues);
}


void estop() {
    ScoopAxis.drive(0);
    SensorAxis.drive(0);
    Auger.drive(0);
    ScoopAxisDecipercent = 0;
    SensorAxisDecipercent = 0;
    AugerDecipercent = 0;
}

void feedWatchdog() {
    Watchdog.begin(estop, WATCHDOG_TIMEOUT);
}
