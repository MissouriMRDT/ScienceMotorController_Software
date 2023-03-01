#include "ScienceActuation.h"

void setup() {
    Serial.begin(115200);

    pinMode(SW_FWD, INPUT);
    pinMode(SW_RVS, INPUT);

    pinMode(MOTOR_SW_1, INPUT);
    pinMode(MOTOR_SW_2, INPUT);
    pinMode(MOTOR_SW_3, INPUT);
    pinMode(MOTOR_SW_4, INPUT);

    pinMode(SERVO_SW_1, INPUT);
    pinMode(SERVO_SW_2, INPUT);
    pinMode(SERVO_SW_3, INPUT);
    pinMode(SERVO_SW_4, INPUT);
    pinMode(SERVO_SW_5, INPUT);
    pinMode(SERVO_SW_6, INPUT);

    Encoder1.begin([]{Encoder1.handleInterrupt();});
    Encoder2.begin([]{Encoder2.handleInterrupt();});
    Encoder3.begin([]{Encoder3.handleInterrupt();});
    Encoder4.begin([]{Encoder4.handleInterrupt();});

    ScoopX.attachEncoder(&Encoder1);
    ScoopZ.attachEncoder(&Encoder2);
    ScienceZ.attachEncoder(&Encoder3);

    ScoopX.attachHardLimits(&LS6, &LS3);
    ScoopZ.attachHardLimits(&LS2, &LS5);
    ScienceZ.attachHardLimits(&LS1, &LS4);

    LS2.configInvert(true);
    LS3.configInvert(true);
    LS6.configInvert(true);

    Servo1.attach(SERVO_1);
    Servo2.attach(SERVO_2);
    Servo3.attach(SERVO_3);
    Servo4.attach(SERVO_4);
    Servo5.attach(SERVO_5);
    Servo6.attach(SERVO_6);

    RoveComm.begin(RC_SCIENCEACTUATIONBOARD_FOURTHOCTET, &TCPServer, RC_ROVECOMM_SCIENCEACTUATIONBOARD_MAC);
    feedWatchdog();
}


uint32_t lastWriteTimestamp = millis();

void loop() {
    uint32_t timestamp = millis();

    // Parse RoveComm packets
    rovecomm_packet packet = RoveComm.read();
    switch (packet.data_id) {

        // Open loop control of ScienceZ
        case RC_SCIENCEACTUATIONBOARD_SENSORAXIS_DATA_ID:
        {
            int16_t data = ((int16_t*) packet.data)[0];
            ScienceZ.drive(data);
            feedWatchdog();
            break;
        }

        // TODO Set position of selector servo
        case RC_SCIENCEACTUATIONBOARD_WATERSELECTOR_DATA_ID:
        {
            int8_t data = ((int8_t*) packet.data)[0];
            feedWatchdog();
            break;
        }

        // TODO Set output of pump servo
        case RC_SCIENCEACTUATIONBOARD_WATERPUMP_DATA_ID:
        {
            uint8_t data = ((uint8_t*) packet.data)[0];
            feedWatchdog();
            break;
        }

        // TODO
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
            ScoopX.drive(data);
            feedWatchdog();
            break;
        }

        // Open loop control of ScoopZ
        case RC_SCIENCEACTUATIONBOARD_ZOOPAXIS_DATA_ID:
        {
            int16_t data = ((int16_t*) packet.data)[0];
            ScoopZ.drive(data);
            feedWatchdog();
            break;
        }

        // Open or close scoop servo
        case RC_SCIENCEACTUATIONBOARD_SCOOPGRABBER_DATA_ID:
        {
            uint8_t data = ((uint8_t*) packet.data)[0];
            if (data) scoopTarget = 120;
            else scoopTarget = 0;
            feedWatchdog();
            break;
        }

        // TODO Closed loop control of ScoopX
        case RC_SCIENCEACTUATIONBOARD_GOTOPOSITION_DATA_ID:
        {
            uint8_t data = ((uint8_t*) packet.data)[0];
            feedWatchdog();
            break;
        }

        // Increment scoop target position
        case RC_SCIENCEACTUATIONBOARD_INCREMENTALSCOOP_DATA_ID:
        {
            int8_t data = ((int8_t*) packet.data)[0];
            int16_t tmp = scoopTarget + data;

            if (tmp > 180) tmp = 180;
            else if (tmp < 0) tmp = 0;

            scoopTarget = tmp;
            feedWatchdog();
            break;
        }

        // Open and close servo scoop in quick succession
        case RC_SCIENCEACTUATIONBOARD_BUMPSCOOP_DATA_ID:
        {
            uint8_t data = ((uint8_t*) packet.data)[0];
            feedWatchdog();
            break;
        }

        // Default
        default:
        {
            break;
        }
    }


    // Periodically write telemetry
    if ((timestamp - lastWriteTimestamp) >= ROVECOMM_UPDATE_RATE*4) {
        updateJointAngles();
        RoveComm.write(RC_SCIENCEACTUATIONBOARD_ENCODERPOSITIONS_DATA_ID, RC_SCIENCEACTUATIONBOARD_ENCODERPOSITIONS_DATA_COUNT, jointAngles);
        updateLimitSwitchValues();
        RoveComm.write(RC_SCIENCEACTUATIONBOARD_LIMITSWITCHTRIGGERED_DATA_ID, RC_SCIENCEACTUATIONBOARD_LIMITSWITCHTRIGGERED_DATA_COUNT, limitSwitchValues);
        lastWriteTimestamp = timestamp;
    }


    updateManualButtons();

    Servo1.write(scoopTarget);
}


void updateLimitSwitchValues() {
    limitSwitchValues = (ScoopZ.atForwardHardLimit() << 5) | (ScoopZ.atReverseHardLimit() << 4) | (ScoopX.atReverseHardLimit() << 3) 
                        | (ScoopX.atForwardHardLimit() << 2) | (ScienceZ.atForwardHardLimit() << 1) | (ScienceZ.atReverseHardLimit() << 0);
}

void updateJointAngles() {
    jointAngles[0] = Encoder3.readDegrees();
    jointAngles[1] = Encoder1.readDegrees();
    jointAngles[2] = Encoder2.readDegrees();
}

void updateManualButtons() {
    for (int i = 0; i < 10; i++) {
        lastManualButtons[i] = manualButtons[i];
    }

    manualButtons[0] = digitalRead(SERVO_SW_1);
    manualButtons[1] = digitalRead(SERVO_SW_2);
    manualButtons[2] = digitalRead(SERVO_SW_3);
    manualButtons[3] = digitalRead(SERVO_SW_4);
    manualButtons[4] = digitalRead(SERVO_SW_5);
    manualButtons[5] = digitalRead(SERVO_SW_6);

    manualButtons[6] = digitalRead(MOTOR_SW_1);
    manualButtons[7] = digitalRead(MOTOR_SW_2);
    manualButtons[8] = digitalRead(MOTOR_SW_3);
    manualButtons[9] = digitalRead(MOTOR_SW_4);

    manualForward = digitalRead(SW_FWD);

    // Servo buttons
    if (manualButtons[0]) scoopTarget = 120;
    else if (lastManualButtons[0]) scoopTarget = 0;
    
    if (manualButtons[1]) Servo2.write((manualForward? 180 : 0));
    else if (lastManualButtons[1]) Servo2.write(90);
    
    if (manualButtons[2]) Servo3.write(90);
    else if (lastManualButtons[2]) Servo3.write(0);
    
    if (manualButtons[3]) Servo4.write(90);
    else if (lastManualButtons[3]) Servo4.write(0);
    
    if (manualButtons[4]) Servo5.write(90);
    else if (lastManualButtons[4]) Servo5.write(0);
    
    if (manualButtons[5]) Servo6.write(90);
    else if (lastManualButtons[5]) Servo6.write(0);
    

    // Motor buttons
    if (manualButtons[6]) Motor1.drive( (manualForward? 800 : -800) );
    else if (lastManualButtons[6]) Motor1.drive(0);
    
    if (manualButtons[7]) Motor2.drive( (manualForward? 800 : -800) );
    else if (lastManualButtons[7]) Motor2.drive(0);
    
    if (manualButtons[8]) Motor3.drive( (manualForward? 950 : -950) );
    else if (lastManualButtons[8]) Motor3.drive(0);
    
    if (manualButtons[9]) Motor4.drive( (manualForward? 950 : -950) );
    else if (lastManualButtons[9]) Motor4.drive(0);

/*
    for (int i = 0; i < 10; i++) {
        if (manualButtons[i]) {
            feedWatchdog();
            break;
        }
    }
*/
}


void estop() {
    Motor1.drive(0);
    Motor2.drive(0);
    Motor3.drive(0);
    Motor4.drive(0);
}

void feedWatchdog() {
    Watchdog.begin(estop, WATCHDOG_TIMEOUT);
}
