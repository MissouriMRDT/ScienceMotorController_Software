#include "ScienceMotorController.h"

void setup()
{
  Serial.begin(9600);
  RoveComm.begin(RC_SCIENCEACTUATIONBOARD_FOURTHOCTET, &TCPServer);
  sensZ.attach(MOTOR_1_IN_A, MOTOR_1_IN_B, MOTOR_1_PWM);
  gantX.attach(MOTOR_2_IN_A, MOTOR_2_IN_B, MOTOR_2_PWM);
  gantZ.attach(MOTOR_3_IN_A, MOTOR_3_IN_B, MOTOR_3_PWM);
  pinMode(SERVO_2, OUTPUT);
  for(int i = 0; i < 3; i ++) {
    for(int a = 0; a < 3; a ++) {
      pinMode(sol[a][i], OUTPUT);
    }
  }
}

void loop()
{
  packet = RoveComm.read();

  switch(packet.data_id)
  {
    case RC_SCIENCEACTUATIONBOARD_SENSORAXIS_DATA_ID: {
      sensZTarget = ((int16_t*)packet.data)[0];
      break;
    }
    
    case RC_SCIENCEACTUATIONBOARD_WATER_DATA_ID: {
      int8_t solTarget = ((int8_t*)packet.data)[0];
      for(int i = 0; i < 3; i ++) {
        solStates[i] = solTarget % 2;
        solTarget >> 1;
        for(int a = 0; a < 3; a ++) {
          digitalWrite(sol[i][a], solStates[i]);
        }
      }
      break;
    }

    case RC_SCIENCEACTUATIONBOARD_XOOPAXIS_DATA_ID: {
      gantXTarget = ((int16_t*)packet.data)[0];
      break;
    }

    case RC_SCIENCEACTUATIONBOARD_ZOOPAXIS_DATA_ID: {
      gantZTarget = ((int16_t*)packet.data)[0];
      break;
    }

    case RC_SCIENCEACTUATIONBOARD_SCOOPGRABBER_DATA_ID: {
      scoopAngle = ((int16_t*)packet.data)[0];
      Serial.println(scoopAngle);
      if(scoopAngle != lastScoopAngle) {
        analogWrite(SERVO_2, scoopAngle == 0 ? 115 : 150);
        lastScoopAngle = scoopAngle;
      }
      break;
    }

    default: {
      break;
    }
  }

  sensZ.drive(sensZTarget);
  gantX.drive(gantXTarget);
  gantZ.drive(gantZTarget);
  
}
