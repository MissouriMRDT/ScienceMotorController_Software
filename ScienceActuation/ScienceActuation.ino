#include "ScienceActuation.h"

void setup() {
    Serial.begin(115200);

    pinMode(SW1, INPUT_PULLUP);
    pinMode(SW2, INPUT_PULLUP);
    pinMode(SW3, INPUT_PULLUP);
    pinMode(SW4, INPUT_PULLUP);
    pinMode(SW5, INPUT_PULLUP);
    pinMode(SW6, INPUT_PULLUP);
    //pinMode(SW7, INPUT);
    //pinMode(SW8, INPUT);
    pinMode(DIR_SW, INPUT);

    //pinMode(PUMP1, OUTPUT);
    //pinMode(PUMP2, OUTPUT);
    //digitalWrite(PUMP1, LOW);
    //digitalWrite(PUMP2, LOW);

    ScoopAxis.attachEncoder(&Encoder1);
    SensorAxis.attachEncoder(&Encoder2);

    ScoopAxis.attachHardLimits(&LS1, &LS2);
    SensorAxis.attachHardLimits(&LS6, &LS3);

    ScoopAxis.ForwardHardLimit()->configInvert(false);
    ScoopAxis.ReverseHardLimit()->configInvert(false);
    ScoopAxis.Motor()->configRampRate(5000);

    SensorAxis.ForwardHardLimit()->configInvert(false);
    SensorAxis.ReverseHardLimit()->configInvert(false);
    SensorAxis.Motor()->configRampRate(5000);

    ScoopAxis.Motor()->configInvert(false);
    SensorAxis.Motor()->configInvert(false);
    Auger.configRampRate(5000);

    ScoopAxis.Encoder()->configInvert(false);
    SensorAxis.Encoder()->configInvert(false);

    Encoder1.begin([]{ Encoder1.handleInterrupt(); });
    Encoder2.begin([]{ Encoder2.handleInterrupt(); });
    Encoder3.begin([]{ Encoder3.handleInterrupt(); });

    ScoopAxis.Motor()->configMaxOutputs(-1000, 1000);
    SensorAxis.Motor()->configMaxOutputs(-1000, 1000);

    ScoopAxis.Motor()->configMinOutputs(0,0);
    SensorAxis.Motor()->configMinOutputs(0,0);

    GimbalTilt.attach(SERVO_1);
    GimbalPan.attach(SERVO_2);

    GimbalTilt.write(GimbalTiltPosition);
    GimbalPan.write(GimbalPanPosition);

    Serial.println("RoveComm Initializing...");
    RoveComm.begin(RC_SCIENCEACTUATIONBOARD_FIRSTOCTET, RC_SCIENCEACTUATIONBOARD_SECONDOCTET, RC_SCIENCEACTUATIONBOARD_THIRDOCTET, RC_SCIENCEACTUATIONBOARD_FOURTHOCTET, &TCPServer);
    Telemetry.begin(telemetry, TELEMETRY_PERIOD);
    Serial.println("Complete");
}

void loop() {
    // Parse RoveComm packets
    rovecomm_packet packet = RoveComm.read();
    switch (packet.data_id) {

        case RC_SCIENCEACTUATIONBOARD_SCOOPAXIS_OPENLOOP_DATA_ID:
        {
            ScoopAxisDecipercent = *((int16_t*) packet.data);
            feedWatchdog();

            break;
        }
        
        case RC_SCIENCEACTUATIONBOARD_SENSORAXIS_OPENLOOP_DATA_ID:
        {
            SensorAxisDecipercent = *((int16_t*) packet.data);
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

        case RC_SCIENCEACTUATIONBOARD_LIMITSWITCHOVERRIDE_DATA_ID:
        {
            uint8_t data = ((uint8_t*) packet.data)[0];

            ScoopAxis.overrideForwardHardLimit(data & (1<<0));
            ScoopAxis.overrideReverseHardLimit(data & (1<<1));
            SensorAxis.overrideForwardHardLimit(data & (1<<2));
            SensorAxis.overrideReverseHardLimit(data & (1<<3));
            
            break;
        }

        case RC_SCIENCEACTUATIONBOARD_AUGER_DATA_ID:
        {
            AugerDecipercent = *((int16_t*) packet.data);
            feedWatchdog();

            break;
        }

        /*
        case RC_SCIENCEACTUATIONBOARD_MICROSCOPE_DATA_ID:
        {
            MicroscopePosition = *((uint8_t*) packet.data);
            
            break;
        }
        */

        case RC_SCIENCEACTUATIONBOARD_WATCHDOGOVERRIDE_DATA_ID:
        {
            watchdogOverride = *((uint8_t*) packet.data);

            break;
        }
        //increment science pan and tilt gimbals by (-180, 180)
        case RC_SCIENCEACTUATIONBOARD_SCIENCEGIMBALINCREMENT_DATA_ID:
            Serial.write("moving Gimbal");
            int16_t* data = (int16_t*) packet.data;
            GimbalPan.target += data[0];
            GimbalTilt.target += data[1];
            break;
    }


    bool direction = digitalRead(DIR_SW);

    
    // AugerAxis
    if (!digitalRead(SW4)) ScoopAxis.drive((direction ? -900 : 900));
    else ScoopAxis.drive(ScoopAxisDecipercent);
        
    // SensorAxis
    if (!digitalRead(SW2)) SensorAxis.drive((direction ? -900 : 900));
    else SensorAxis.drive(SensorAxisDecipercent);

    // Auger
    if (!digitalRead(SW1)) Auger.drive((direction ? -900 : 900));
    else Auger.drive(AugerDecipercent);

     // Spare Motor
    if (!digitalRead(SW5)) SpareMotor.drive((direction ? -900 : 900));
    else SpareMotor.drive(0);

    // Pan Gimbal
    if (!digitalRead(SW6)) GimbalPan.write((direction? 0 : 180));
    else GimbalPan.write((GIMBAL_PAN_MAX-GIMBAL_PAN_MIN)/2+GIMBAL_PAN_MIN);
    
    // Tilt Gimbal
    if (!digitalRead(SW3)) GimbalTilt.write((direction? 0 : 180));
    else GimbalTilt.write((GIMBAL_TILT_MAX-GIMBAL_TILT_MIN)/2+GIMBAL_TILT_MIN);

    GimbalPan.write();
    GimbalTilt.write();
}


void telemetry() {
    // Encoder positions
    float positions[2] = { Encoder1.readDegrees(), Encoder2.readDegrees() };
    RoveComm.write(RC_SCIENCEACTUATIONBOARD_POSITIONS_DATA_ID, RC_SCIENCEACTUATIONBOARD_POSITIONS_DATA_COUNT, positions);

    // Limit switches
    uint8_t limitSwitchValues = (ScoopAxis.atForwardHardLimit() << 0) | (ScoopAxis.atReverseHardLimit() << 1) | (SensorAxis.atForwardHardLimit() << 2) | (SensorAxis.atReverseHardLimit() << 3);
    RoveComm.write(RC_SCIENCEACTUATIONBOARD_LIMITSWITCHTRIGGERED_DATA_ID, RC_SCIENCEACTUATIONBOARD_LIMITSWITCHTRIGGERED_DATA_COUNT, limitSwitchValues);

    // humidity
    float humidity = analogRead(HUMIDITY);
    RoveComm.write(RC_SCIENCEACTUATIONBOARD_HUMIDITY_DATA_ID, RC_SCIENCEACTUATIONBOARD_HUMIDITY_DATA_COUNT, &humidity);

    // Watchdog status
    RoveComm.write(RC_SCIENCEACTUATIONBOARD_WATCHDOGSTATUS_DATA_ID, RC_SCIENCEACTUATIONBOARD_WATCHDOGSTATUS_DATA_COUNT, &watchdogStatus);
}


void estop() {
    watchdogStatus = 1;

    if (!watchdogOverride) {
        ScoopAxis.drive(0);
        SensorAxis.drive(0);
        Auger.drive(0);

        ScoopAxisDecipercent = 0;
        SensorAxisDecipercent = 0;
        AugerDecipercent = 0;
    }
}

void feedWatchdog() {
    watchdogStatus = 0;
    Watchdog.begin(estop, WATCHDOG_TIMEOUT);
}
