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

    AugerAxis.attachEncoder(&EncoderAugerAxis);
    SensorAxis.attachEncoder(&EncoderSensorAxis);

    AugerAxis.attachHardLimits(&LimitAugerAxisRVS, &LimitAugerAxisFWD);
    SensorAxis.attachHardLimits(&LimitSensorAxisRVS, &LimitSensorOrLaserFWD);

    AugerAxis.Motor()->configRampRate(5000);
    SensorAxis.Motor()->configRampRate(5000);
    Auger.configRampRate(5000);

    EncoderAugerAxis.begin([]{ EncoderAugerAxis.handleInterrupt(); });
    EncoderAuger.begin([]{ EncoderAuger.handleInterrupt(); });
    EncoderSensorAxis.begin([]{ EncoderSensorAxis.handleInterrupt(); });

    AugerAxis.Motor()->configMaxOutputs(-1000, 1000);
    SensorAxis.Motor()->configMaxOutputs(-1000, 1000);

    AugerAxis.Motor()->configMinOutputs(0,0);
    SensorAxis.Motor()->configMinOutputs(0,0);

    GimbalPan.attach(GIMBAL_PAN_PIN);
    GimbalTilt.attach(GIMBAL_TILT_PIN);

    GimbalPan.write(gimbalPanStartPosition);
    GimbalTilt.write(gimbalTiltStartPosition);

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
            augerAxisDecipercent = *((int16_t*) packet.data);
            feedWatchdog();

            break;
        }
        
        case RC_SCIENCEACTUATIONBOARD_SENSORAXIS_OPENLOOP_DATA_ID:
        {
            sensorAxisDecipercent = *((int16_t*) packet.data);
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

            AugerAxis.overrideForwardHardLimit(data & (1<<0));
            AugerAxis.overrideReverseHardLimit(data & (1<<1));
            SensorAxis.overrideForwardHardLimit(data & (1<<2));
            SensorAxis.overrideReverseHardLimit(data & (1<<3));
            
            break;
        }

        case RC_SCIENCEACTUATIONBOARD_AUGER_DATA_ID:
        {
            augerDecipercent = *((int16_t*) packet.data);
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
        {
            //Serial.write("moving Gimbal");
            int16_t* data = (int16_t*) packet.data;
            GimbalPan.target += data[0];
            GimbalTilt.target += data[1];
            break;
        }
        
        case RC_SCIENCEACTUATIONBOARD_REQUESTHUMIDITY_DATA_ID:
        {
            // humidity
            float humidity = analogMap(analogRead(HUMIDITY), HUMIDITY_ADC_MIN, HUMIDITY_ADC_MAX, HUMIDITY_MAPPED_MIN, HUMIDITY_MAPPED_MAX);
            RoveComm.write(RC_SCIENCEACTUATIONBOARD_HUMIDITY_DATA_ID, RC_SCIENCEACTUATIONBOARD_HUMIDITY_DATA_COUNT, humidity);
            break;
        }
    }

    bool direction = digitalRead(DIR_SW);

    // AugerAxis
    if (!digitalRead(SW4)) AugerAxis.drive((direction ? -900 : 900));
    else AugerAxis.drive(augerAxisDecipercent);

    // SensorAxis
    if (!digitalRead(SW2)) SensorAxis.drive((direction ? -900 : 900));
    else SensorAxis.drive(sensorAxisDecipercent);

    // Auger
    if (!digitalRead(SW1)) Auger.drive((direction ? -900 : 900));
    else Auger.drive(augerDecipercent);

     // Spare Motor
    if (!digitalRead(SW5)) SpareMotor.drive((direction ? -900 : 900));
    else SpareMotor.drive(0);

    // Pan Gimbal
    if (!digitalRead(SW6)) GimbalPan.target +=((direction? -10 : 10));
    
    // Tilt Gimbal
    if (!digitalRead(SW3)) GimbalTilt.target +=((direction? -10 : 10));

    GimbalPan.write();
    GimbalTilt.write();
}

float analogMap(uint16_t measurement, uint16_t fromADC, uint16_t toADC, float fromAnalog, float toAnalog) {
  float slope = (toAnalog - fromAnalog) / (toADC - fromADC);
  return (measurement - fromADC) * slope + fromAnalog;
}

void telemetry() {
    // Encoder positions
    float positions[2] = { EncoderAugerAxis.readDegrees(), EncoderSensorAxis.readDegrees() };
    RoveComm.write(RC_SCIENCEACTUATIONBOARD_POSITIONS_DATA_ID, RC_SCIENCEACTUATIONBOARD_POSITIONS_DATA_COUNT, positions);

    // Limit switches
    uint8_t limitSwitchValues = (AugerAxis.atForwardHardLimit() << 0) | (AugerAxis.atReverseHardLimit() << 1) | (SensorAxis.atForwardHardLimit() << 2) | (SensorAxis.atReverseHardLimit() << 3);
    RoveComm.write(RC_SCIENCEACTUATIONBOARD_LIMITSWITCHTRIGGERED_DATA_ID, RC_SCIENCEACTUATIONBOARD_LIMITSWITCHTRIGGERED_DATA_COUNT, limitSwitchValues);

    // Watchdog status
    RoveComm.write(RC_SCIENCEACTUATIONBOARD_WATCHDOGSTATUS_DATA_ID, RC_SCIENCEACTUATIONBOARD_WATCHDOGSTATUS_DATA_COUNT, watchdogStatus);
}


void estop() {
    watchdogStatus = 1;

    if (!watchdogOverride) {
        AugerAxis.drive(0);
        SensorAxis.drive(0);
        Auger.drive(0);

        augerAxisDecipercent = 0;
        sensorAxisDecipercent = 0;
        augerDecipercent = 0;
    }
}

void feedWatchdog() {
    watchdogStatus = 0;
    Watchdog.begin(estop, WATCHDOG_TIMEOUT);
}
