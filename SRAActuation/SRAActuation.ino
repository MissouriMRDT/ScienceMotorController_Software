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
  GenevaMotor.attach(GENEVA_INA, GENEVA_INB, GENEVA_PWM); //motor 4
  ElevatorMotor.attach(ELEVATOR_INA, ELEVATOR_INB, ELEVATOR_PWM);
  GenevaEncoder.attach(GENEVA_PWM, 7);
  ElevatorEncoder.attach(ELEVATOR_PWM, 7);

  for(int i = 0; i < 4; i++)
  {
    pinMode(motorButtons[i], INPUT);
  }
  
  //attach estop to watchdog, cut motors after 150 ms of no comms (control rate is 100ms)
  Watchdog.attach(Estop);
  Watchdog.start(150);
}

void loop()
{
  packet = RoveComm.read();
  //Serial.println(packet.data_id);
  switch (packet.data_id)
  {
    case RC_SRAACTUATION_GENEVA_OPENLOOP_DATAID:
      int16_t* genevaSpeed;
      genevaSpeed = (int16_t*)packet.data;
      Serial.println("Geneva Position: ");
      Serial.println(genevaSpeed[0]);
      geneva_Speed = genevaSpeed[0];
      Watchdog.clear();
      break;
    case RC_SRAACTUATION_CHEMICALS_DATAID:
      int16_t* chemicalSpeeds;
      chemicalSpeeds = (int16_t*)packet.data;
      Serial.println("Chemical Speeds Position: ");
      Serial.println(chemicalSpeeds[0]);
      Serial.println(chemicalSpeeds[1]);
      Serial.println(chemicalSpeeds[2]);
      for(uint8_t i = 0; i < 3; i++)
      {
        chem_Speeds[i] = chemicalSpeeds[i];
      }
      Watchdog.clear();
      break;
      case RC_SRAACTUATION_Z_AXIS_DATAID:
      int16_t* elevatorSpeed;
      elevatorSpeed = (int16_t*)packet.data;
      Serial.println("Geneva Position: ");
      Serial.println(elevatorSpeed[0]);
      elevator_Speed = elevatorSpeed[0];
      Watchdog.clear();
      break;
  }

  for(int i = 0; i < 4; i++)
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
  
  Chem1Motor.drive(chem_Speeds[0]);
  Chem2Motor.drive(chem_Speeds[1]);
  Chem3Motor.drive(chem_Speeds[2]); 
  GenevaMotor.drive(geneva_Speed);
  ElevatorMotor.drive(elevator_Speed);
}

void Estop()
{
  Serial.println("Estop triggered");
  
  for(int i = 0; i < 4; i++)
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
