#include "RoveComm.h"
#include "RoveStmVnhPwm.h"
#include "SRAActuation.h"
#include "RoveWatchdog.h"

void setup()
{
  BeginSerial();
  BeginRoveComm();
  AttachChemMotors();
  AttachGenevaMotors();
  InitButtons();
  AttachWatchdog();
  BeginWatchdog();
}

void loop()
{
  ParsePackets();
  CheckButtons();
}

void BeginSerial()
{
  Serial.begin(115200);
}

void BeginRoveComm()
{
  RoveComm.begin(RC_SCIENCEACTUATIONBOARD_FOURTHOCTET, &TCPServer);
}

void AttachChemMotors()
{
  Chem1Motor.attach(CHEM1_PIN1, CHEM1_PIN2, CHEM1_PIN3);  
  Chem2Motor.attach(CHEM2_PIN1, CHEM2_PIN2, CHEM2_PIN3);
  Chem3Motor.attach(CHEM3_PIN1, CHEM3_PIN2, CHEM3_PIN3);
}

void AttachGenevaMotors()
{
  GenevaMotor.attach(GENEVA_PIN1, GENEVA_PIN2, GENEVA_PIN3);
}

void InitButtons()
{
  for(int i = 0; i < 4; i++)
  {
    pinMode(motorButtons[i], INPUT);
  }
}

void AttachWatchdog()
{
  Watchdog.attach(Estop);
}

void BeginWatchdog()
{
  Watchdog.start(150);
}

void ParsePackets()
{
  packet = RoveComm.read();
  switch (packet.data_id)
  {
    case RC_SCIENCEACTUATIONBOARD_GENEVAOPENLOOP_DATA_ID:
      ScienceActuationBoard_GenevaOpenLoop();
      break;
    case RC_SCIENCEACTUATIONBOARD_CHEMICALS_DATA_ID:
      ScienceActuationBoard_Chemicals();
      break;
  }
}

void ScienceActuationBoard_GenevaOpenLoop()
{
  int16_t* genevaSpeed = ReceiveGenevaSpeed();
  PrintGenevaSpeed(genevaSpeed);
  DriveGenevaMotor(genevaSpeed);
  WatchdogClear();
}

void ScienceActuationBoard_Chemicals()
{
  int16_t* chemicalSpeeds = ReceiveChemSpeeds();
  PrintChemSpeed(chemicalSpeeds);
  DriveChemMotor(chemicalSpeeds);
  WatchdogClear();
}

int16_t* ReceiveGenevaSpeed()
{
  return (int16_t*)packet.data;
}

void PrintGenevaSpeed(int16_t* genevaSpeed)
{
  Serial.println("Geneva Position: ");
  Serial.println(genevaSpeed[0]);
}

void DriveGenevaMotor(int16_t* genevaSpeed)
{
  GenevaMotor.drive(geneva_Speed[0]);
}

int16_t* ReceiveChemSpeeds()
{
  return (int16_t*)packet.data;
}

void PrintChemSpeed(int16_t* chemicalSpeeds)
{
  Serial.println("Chemical Speeds Position: ");
  Serial.println(chemicalSpeeds[0]);
  Serial.println(chemicalSpeeds[1]);
  Serial.println(chemicalSpeeds[2]);
}

void DriveChemMotor(int16_t* chemicalSpeeds)
{
  Chem1Motor.drive(chemicalSpeeds[0]);
  Chem2Motor.drive(chemicalSpeeds[1]);
  Chem3Motor.drive(chemicalSpeeds[2]);
}

void CheckButtons()
{
  for(int i = 0; i < 4; i++)
  {
    if(digitalRead(motorButtons[i]))
    {
      if(i == 3)
      {
        GenevaMotor.drive(140);
      }
      else
      {
        Chem1Motor.drive(140);
        Chem2Motor.drive(140);
        Chem3Motor.drive(140);
      }
      WatchdogClear();
    }
  }
}

void CheckButtonsEstop()
{
  for(int i = 0; i < 4; i++)
  {
    if(!digitalRead(motorButtons[i]))
    {
      if(i == 3)
      {
        GenevaMotor.drive(0);
      }
      else
      {
        Chem1Motor.drive(0);
        Chem2Motor.drive(0);
        Chem3Motor.drive(0);
      }
    }
  }
}

void Estop()
{
  Serial.println("Estop triggered");
  CheckButtonsEstop();
  Serial.println("Watchdog cleared");
  WatchdogClear();
}

void WatchdogClear()
{
  Watchdog.clear();
}