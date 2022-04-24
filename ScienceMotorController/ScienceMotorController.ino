#include "ScienceMotorController.h"

void setup()
{
  RoveComm.begin(RC_SCIENCEACTUATIONBOARD_FOURTHOCTET, &TCPServer);

  // attack motor controllers to pins
  sensZ.attach(MOTOR_1_IN_A, MOTOR_1_IN_B, MOTOR_1_PWM);
  gantX.attach(MOTOR_2_IN_A, MOTOR_2_IN_B, MOTOR_2_PWM);
  gantZ.attach(MOTOR_3_IN_A, MOTOR_3_IN_B, MOTOR_3_PWM);

  // set output pin modes for servos and solenoids
  pinMode(SERVO_1, OUTPUT);
  pinMode(SERVO_2, OUTPUT);

  for(int i = 0; i < 3; i ++)
  {
    for(int a = 0; a < 3; a ++)
    {
      pinMode(sol[a][i], OUTPUT);
    }
  }
}

void loop()
{
  packet = RoveComm.read();

  switch(packet.data_id)
  {
    case RC_SCIENCEACTUATIONBOARD_SENSORAXIS_DATA_ID:
    {
      // set sensor axis target value
      sensZTarget = ((int16_t*)packet.data)[0];
      break;
    }
    
    case RC_SCIENCEACTUATIONBOARD_WATER_DATA_ID:
    {
      int8_t solTarget = ((int8_t*)packet.data)[0];
      // TODO: add control for individual solenoids once RoveComm Manifest is updated for it

      // increment through 3 solenoid groups
      for(int i = 0; i < 3; i ++)
      {
        // check corresponding bit then shift right 1 bit
        solStates[i] = solTarget % 2;
        solTarget >> 1;

        // increment though each solenoid in group and set to correct state
        for(int a = 0; a < 3; a ++)
        {
          digitalWrite(sol[i][a], solStates[i]);
        }
      }
      break;
    }

    case RC_SCIENCEACTUATIONBOARD_XOOPAXIS_DATA_ID:
    {
      // TODO: add PID control based off encoder position once installed
      gantXTarget = ((int16_t*)packet.data)[0];  // set gantry X axis target value
      break;
    }

    case RC_SCIENCEACTUATIONBOARD_ZOOPAXIS_DATA_ID:
    {
      gantZTarget = ((int16_t*)packet.data)[0];  // set gantry Z axis target value
      break;
    }

    case RC_SCIENCEACTUATIONBOARD_SCOOPGRABBER_DATA_ID:
    {
      scoopAngle = ((int16_t*)packet.data)[0];

      // check if scoop needs to change angle (repeatedly setting servo to the same value caused shakiness in testing)
      if(scoopAngle != lastScoopAngle)
      {
        analogWrite(SERVO_2, scoopAngle == 0 ? SCOOP_OPEN_VALUE : SCOOP_CLOSED_VALUE);  // set PWM pin to appropriate value
        lastScoopAngle = scoopAngle;
      }
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
  for(int i = 0; i < 6; i ++)
  {
    // increment bitmask by 1 if current limit is triggered then shift left 1 bit
    if(digitalRead(enc(i))) limitStates += 1;
    limitStates << 1;
  }

  // check if enough time has passed to send telemetry
  if(millis() - lastUpdateTime >= ROVECOMM_UPDATE_RATE)
  {
    // TODO: encoder position telemetry
    RoveComm.writeReliable(RC_SCIENCEACTUATIONBOARD_LIMITSWITCHTRIGGERED_DATA_ID, 1, limitStates);  // send limitStates bitmask
  }
  
}
