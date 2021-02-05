#include "RoveComm.h"
#include "RoveStmVnhPwm.h"
#include "SRAActuation.h"

#include "RoveWatchdog.h"

void setup()
{
  Serial.begin(115200);


  RoveComm.begin(RC_SRAACTUATIONBOARD_FOURTHOCTET, &TCPServer);

  Chem1Motor.attach(CHEM1_INA, CHEM1_INB, CHEM1_PWM);  
  Chem2Motor.attach(CHEM2_INA, CHEM2_INB, CHEM2_PWM);
  Chem3Motor.attach(CHEM3_INA, CHEM3_INB, CHEM3_PWM);
  GenevaMotor.attach(GENEVA_INA, GENEVA_INB, GENEVA_PWM); 
  Z_AxisMotor.attach(Z_AXIS_INA, Z_AXIS_INB, Z_AXIS_PWM);
  GenevaEncoder.attach(GENEVA_PWM, 7);
  Z_AxisEncoder.attach(Z_AXIS_PWM, 7);
  GenevaEncoder.start();
  Z_AxisEncoder.start();

  for(int16_t i = 0; i < 4; i++)
  {
    pinMode(motorButtons[i], INPUT);
  }
  
  //attach estop to watchdog, cut motors after 150 ms of no comms (control rate is 100ms)
  Watchdog.attach(Estop);
  Watchdog.start(150);
}

void loop()
{
  ParsePackets();
  CheckButtons();
  DriveMotors();
}

void Estop()
{
  Serial.println("Estop triggered");
  
  for(int16_t i = 0; i < 4; i++)
  {
    if(!digitalRead(motorButtons[i]))
    {
      if(i == 3)
      {
        geneva_Speed = 0;
      }
      else
      {
        chem_Speeds[i] = 0;
      }
    }
  }
  
  Serial.println("Watchdog cleared");
  Watchdog.clear();
}

void SetMotorSpeed(int16_t Speed)
{
  int16_t* motorSpeed = (int16_t*)packet.data;
  Serial.println(motorSpeed[0]);
  Speed = motorSpeed[0];
  Watchdog.clear();
}

void SetMotorSpeed(int16_t Speed[])
{
  int16_t* motorSpeed = (int16_t*)packet.data;
  Serial.println(motorSpeed[0]);
  Serial.println(motorSpeed[1]);
  Serial.println(motorSpeed[2]);
  for(uint8_t i = 0; i < 3; i++)
  {
    Speed[i] = motorSpeed[i];
  }
  Watchdog.clear();
}

void CheckButtons()
{
  for(int16_t i = 0; i < 4; i++)
  {
    if(digitalRead(motorButtons[i]))
    {
      if(i == 3)
      {
        geneva_Speed = 140;
      }
      else
      {
        chem_Speeds[i] = 140;
      }
      Watchdog.clear();
    }
  }
}

void ParsePackets()
{
  packet = RoveComm.read();
  switch (packet.data_id)
  {
    case RC_SRAACTUATION_GENEVA_OPENLOOP_DATAID:
      Serial.println("Geneva Position: ");
      SetMotorSpeed(geneva_Speed);
      break;

    case RC_SRAACTUATION_CHEMICALS_DATAID:
      Serial.println("Chemical Speeds Position: ");
      SetMotorSpeed(chem_Speeds);
      break;

      case RC_SRAACTUATION_Z_AXIS_DATAID:
      Serial.println("Z-Axis Position: ");
      SetMotorSpeed(z_Axis_Speed);
      break;

      case RC_SRAACTUATION_GENEVA_TO_POS_DATAID:
      
      Watchdog.clear();
      break;

      case RC_SRAACTUATION_GENEVA_INC_POS_DATAID:
      
      Watchdog.clear();
      break;
  }
}

void DriveMotors()
{
  Chem1Motor.drive(chem_Speeds[0]);
  Chem2Motor.drive(chem_Speeds[1]);
  Chem3Motor.drive(chem_Speeds[2]); 
  GenevaMotor.drive(geneva_Speed);
  Z_AxisMotor.drive(z_Axis_Speed);
}