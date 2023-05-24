#include "ScienceActuation.h"

void setup() {
    Serial.begin(115200);

    pinMode(SW_FWD, INPUT);
    pinMode(SW_RVS, INPUT);

    pinMode(MOTOR_SW_1, INPUT);
    pinMode(MOTOR_SW_2, INPUT);
    pinMode(MOTOR_SW_3, INPUT);
    pinMode(MOTOR_SW_4, INPUT);
    pinMode(MOTOR_SW_5, INPUT);

    pinMode(SERVO_SW_1, INPUT);
    pinMode(SERVO_SW_2, INPUT);
    pinMode(SERVO_SW_3, INPUT);
    pinMode(SERVO_SW_4, INPUT);

    pinMode(PUMP_PWM, OUTPUT);

    ScoopX.attachHardLimits(&LS2, &LS1);
    ScoopZ.attachHardLimits(&LS3, &LS4);
    ScienceZ.attachHardLimits(&LS5, &LS6);

    LS1.configInvert(true);
    LS2.configInvert(true);
    LS3.configInvert(true);
    LS4.configInvert(false);
    LS5.configInvert(false);
    LS6.configInvert(false);
    LS7.configInvert(false);

    Motor1.configInvert(false);
    Motor2.configInvert(true);
    Motor3.configInvert(true);
    Motor4.configInvert(false); // TODO zoom when +

    Encoder1.configInvert(false);
    Encoder2.configInvert(true);
    Encoder3.configInvert(true);

    Encoder1.begin([]{ Encoder1.handleInterrupt(); });
    Encoder2.begin([]{ Encoder2.handleInterrupt(); });
    Encoder3.begin([]{ Encoder3.handleInterrupt(); });

    ScoopX.attachEncoder(&Encoder1);
    ScoopZ.attachEncoder(&Encoder2);
    ScienceZ.attachEncoder(&Encoder3);

    ScoopX.attachPID(&PID1);
    ScoopZ.attachPID(&PID2);
    ScienceZ.attachPID(&PID3);

    Motor1.configMaxOutputs(-900, 900);
    Motor2.configMaxOutputs(-900, 900);
    Motor3.configMaxOutputs(-900, 900);
    Motor4.configMaxOutputs(-900, 900);

    Motor1.configMinOutputs(-70, 70);
    Motor2.configMinOutputs(-170, 70);
    Motor3.configMinOutputs(-50, 50);
    Motor4.configMinOutputs(-50, 50);

    Servo1.attach(SERVO_1, 500, 2500);
    Servo2.attach(SERVO_2);
    Servo3.attach(SERVO_3);
    Servo4.attach(SERVO_4);

    Scoop.write(scoopTarget);

    RoveComm.begin(RC_SCIENCEACTUATIONBOARD_FIRSTOCTET, RC_SCIENCEACTUATIONBOARD_SECONDOCTET, RC_SCIENCEACTUATIONBOARD_THIRDOCTET, RC_SCIENCEACTUATIONBOARD_FOURTHOCTET, &TCPServer);
    Telemetry.begin(telemetry, TELEMETRY_PERIOD);
    feedWatchdog();
}

void loop() {
    float timestamp = ((float) millis()) / 1000.0;

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

        // Open loop control of pump MUX
        case RC_SCIENCEACTUATIONBOARD_WATERSELECTOR_DATA_ID:
        {
            int8_t data = ((int8_t*) packet.data)[0];
            
            pumpMUXOutput = data;
            pumpMUXClosedLoopActive = false;
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

        // Open loop control of ScoopX
        case RC_SCIENCEACTUATIONBOARD_XOOPAXIS_DATA_ID:
        {
            int16_t data = ((int16_t*) packet.data)[0];
            decipercents[0] = data;

            if (scoopClosedLoopActive) {
                scoopClosedLoopActive = false;
                decipercents[1] = 0;
            }

            feedWatchdog();
            break;
        }

        // Open loop control of ScoopZ
        case RC_SCIENCEACTUATIONBOARD_ZOOPAXIS_DATA_ID:
        {
            int16_t data = ((int16_t*) packet.data)[0];
            decipercents[1] = data;
            
            if (scoopClosedLoopActive) {
                scoopClosedLoopActive = false;
                decipercents[0] = 0;
            }

            feedWatchdog();
            break;
        }

        // Open or close scoop servo
        case RC_SCIENCEACTUATIONBOARD_SCOOPGRABBER_DATA_ID:
        {
            uint8_t data = ((uint8_t*) packet.data)[0];

            switch (data) {
                case 0:  scoopTarget = 100;  break;
                case 1:  scoopTarget = 50;   break;
                case 2:  scoopTarget = 75;   break;
            }
            
            break;
        }

        // Closed loop control of Scoop
        case RC_SCIENCEACTUATIONBOARD_GOTOPOSITION_DATA_ID:
        {
            uint8_t data = ((uint8_t*) packet.data)[0];

            if (!scoopClosedLoopActive || scoopClosedLoopMode != data) {
                if ((scoopClosedLoopMode == GROUND) || (Encoder1.readDegrees() > SCOOP_OUT_THRESHOLD)) scoopCalibrationMode = 1;
                else scoopCalibrationMode = 2;

                scoopClosedLoopMode = data;
                scoopClosedLoopActive = true;
            }
            
            feedWatchdog();
            break;
        }

        // Increment scoop target position
        case RC_SCIENCEACTUATIONBOARD_INCREMENTALSCOOP_DATA_ID:
        {
            int8_t data = ((int8_t*) packet.data)[0];
            int16_t tmp = scoopTarget + data;

            if (tmp > 100) tmp = 100;
            else if (tmp < 50) tmp = 50;
            scoopTarget = tmp;

            feedWatchdog();
            break;
        }

        // TODO: Open and close servo scoop in quick succession
        case RC_SCIENCEACTUATIONBOARD_BUMPSCOOP_DATA_ID:
        {
            uint8_t data = ((uint8_t*) packet.data)[0];

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

        // TODO: Closed loop control of pump MUX
        case 80085: {
            uint8_t data = ((uint8_t*) packet.data)[0];

            pumpMUXClosedLoopMode = data;
            pumpMUXClosedLoopActive = true;
        }
        
    }


    bool direction = digitalRead(SW_RVS);

    if (scoopClosedLoopActive) {

        if (scoopCalibrationMode == 1) {
            ScoopX.drive(500, timestamp);
            ScoopZ.drive(0, timestamp);
            if (ScoopX.atForwardHardLimit()) {
                Encoder1.setDegrees(0);
                scoopCalibrationMode = 2;
            }
        }
        else if (scoopCalibrationMode == 2) {
            ScoopX.drive(0, timestamp);
            ScoopZ.drive(-600, timestamp);
            if (ScoopZ.atReverseHardLimit()) {
                Encoder2.setDegrees(0);
                scoopCalibrationMode = 0;
                scoopCalibrated = true;
            }
        }
        else if (scoopClosedLoopMode == CALIBRATE) {
            scoopCalibrationMode = 1;
        }
        else if (scoopCalibrated) {
            // ScoopX
            float scoopXTarget;
            switch (scoopClosedLoopMode) {
                case GROUND:      scoopXTarget = -100;    break;
                case POSITION_1:  scoopXTarget = -7100;   break;
                case POSITION_2:  scoopXTarget = -10200;  break;
                case POSITION_3:  scoopXTarget = -13100;  break;
                case POSITION_4:  scoopXTarget = -16150;  break;
                case POSITION_5:  scoopXTarget = -19550;  break;
                case POSITION_6:  scoopXTarget = -22500;  break;
            }
            ScoopX.setAngle(scoopXTarget, timestamp);

            // ScoopZ
            float scoopZTarget;
            switch (scoopClosedLoopMode) {
                case GROUND:
                    scoopZTarget = SCOOP_DOWN_HEIGHT;
                    break;
                case POSITION_1: case POSITION_2: case POSITION_3: case POSITION_4: case POSITION_5: case POSITION_6:
                    scoopZTarget = SCOOP_DROP_HEIGHT;
                    break;
            }
            if (abs(PID1.error()) < 800) ScoopZ.setAngle(scoopZTarget, timestamp);
            else ScoopZ.drive(0, timestamp);
        }
        else {
            ScoopX.drive(0, timestamp);
            ScoopZ.drive(0, timestamp);
        }

    }
    else {
        // ScoopX
        if (digitalRead(MOTOR_SW_1)) ScoopX.drive((direction ? -900 : 900), timestamp);
        else ScoopX.drive(decipercents[0], timestamp);
        
        // ScoopZ
        if (digitalRead(MOTOR_SW_2)) ScoopZ.drive((direction ? -900 : 900), timestamp);
        else ScoopZ.drive(decipercents[1], timestamp);
    }
    
    // ScienceZ
    if (digitalRead(MOTOR_SW_3)) ScienceZ.drive((direction ? -900 : 900), timestamp);
    else ScienceZ.drive(decipercents[2], timestamp);
    
    // Microscope
    if (digitalRead(MOTOR_SW_4)) MICROSCOPE.drive((direction ? -900 : 900), timestamp);
    else MICROSCOPE.drive(decipercents[3], timestamp);

    // Perisaltic Pump
    if (digitalRead(MOTOR_SW_5)) digitalWrite(PUMP_PWM, 1);
    else digitalWrite(PUMP_PWM, pumpOutput);
    
    // Scoop
    if (digitalRead(SERVO_SW_1)) scoopTarget = (direction? 100 : 50);
    Scoop.write(scoopTarget);
    
    // Pump MUX
    if (pumpMUXClosedLoopActive) {
        if (!pumpMUXCalibrated || (pumpMUXClosedLoopMode == 0)) { // calibrate
            drivePumpMux(-50);
            //if limit switch pressed, zero
        }
        else {
            float pumpMUXTarget = 0;
            switch (pumpMUXClosedLoopMode) {
                case 1:   pumpMUXTarget = 0;   break;
                case 2:   pumpMUXTarget = 0;   break;
                case 3:   pumpMUXTarget = 0;   break;
                case 4:   pumpMUXTarget = 0;   break;
                case 5:   pumpMUXTarget = 0;   break;
                case 6:   pumpMUXTarget = 0;   break;
                case 7:   pumpMUXTarget = 0;   break;
                case 8:   pumpMUXTarget = 0;   break;
                case 9:   pumpMUXTarget = 0;   break;
                case 10:  pumpMUXTarget = 0;   break;
                case 11:  pumpMUXTarget = 0;   break;
                case 12:  pumpMUXTarget = 0;   break;
            }

            float output = PID4.calculate(pumpMUXTarget, Encoder4.readDegrees(), timestamp);
            if (output > 90) output = 90;
            else if (output < -90) output = -90;

            PumpMUX.write(90 + output);
        }
    }
    else {
        if (digitalRead(SERVO_SW_2)) drivePumpMux(direction? -90 : 90);
        else drivePumpMux(pumpMUXOutput);
    }
    
    // Spare Servos
    if (digitalRead(SERVO_SW_3)) Servo3.write((direction? 0 : 180));
    else Servo3.write(90);
    
    if (digitalRead(SERVO_SW_4)) Servo4.write((direction? 0 : 180));
    else Servo4.write(90);
}

// -90 to 90
void drivePumpMux(int8_t output) {
    // check limit switch
    PumpMUX.write(90 + output)
}


void telemetry() {
    float positions[3] = { Encoder1.readDegrees(), Encoder2.readDegrees(), Encoder4.readDegrees() };
    RoveComm.write(RC_SCIENCEACTUATIONBOARD_ENCODERPOSITIONS_DATA_ID, RC_SCIENCEACTUATIONBOARD_ENCODERPOSITIONS_DATA_COUNT, positions);

    uint8_t limitSwitchValues = (ScoopZ.atForwardHardLimit() << 5) | (ScoopZ.atReverseHardLimit() << 4) | (ScoopX.atReverseHardLimit() << 3) 
                                | (ScoopX.atForwardHardLimit() << 2) | (ScienceZ.atForwardHardLimit() << 1) | (ScienceZ.atReverseHardLimit() << 0);
    RoveComm.write(RC_SCIENCEACTUATIONBOARD_LIMITSWITCHTRIGGERED_DATA_ID, RC_SCIENCEACTUATIONBOARD_LIMITSWITCHTRIGGERED_DATA_COUNT, limitSwitchValues);
}


void estop() {
    scoopClosedLoopActive = false;
    scoopCalibrationMode = 0;
    for (int i = 0; i < 4; i++) {
        decipercents[i] = 0;
    }
    pumpOutput = 0;
    pumpMUXClosedLoopActive = false;
    pumpMUXOutput = 0;
}

void feedWatchdog() {
    Watchdog.begin(estop, WATCHDOG_TIMEOUT);
}
