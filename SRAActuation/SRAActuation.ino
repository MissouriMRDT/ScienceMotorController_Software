#include "RoveComm.h"
#include "RoveStmVnhPwm.h"
#include "SRAActuation.h"

#include "RoveWatchdog.h"

void setup()
{
  Serial.begin(115200);


  RoveComm.begin(RC_SRAACTUATIONBOARD_FOURTHOCTET, RC_ROVECOMM_ETHERNET_SRA_ACTUATIONBOARD_PORT);

  Chem1Motor.attach(CHEM1_INA, CHEM1_INB, CHEM1_PWM);  
  Chem2Motor.attach(CHEM2_INA, CHEM2_INB, CHEM2_PWM);
  Chem3Motor.attach(CHEM3_INA, CHEM3_INB, CHEM3_PWM);
  GenevaMotor.attach(GENEVA_INA, GENEVA_INB, GENEVA_PWM); //motor 4
  
  //attach estop to watchdog, cut motors after 150 ms of no comms (control rate is 100ms)
  Watchdog.attach(Estop);
  Watchdog.start(1000, 99999);
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
      GenevaMotor.drive(genevaSpeed[0]);
      Watchdog.clear();
      break;
    case RC_SRAACTUATION_CHEMICALS_DATAID:
      int16_t* chemicalSpeeds;
      chemicalSpeeds = (int16_t*)packet.data;
      Serial.println("Chemical Speeds Position: ");
      Serial.println(chemicalSpeeds[0]);
      Serial.println(chemicalSpeeds[1]);
      Serial.println(chemicalSpeeds[2]);

      Chem1Motor.drive(chemicalSpeeds[0]);
      Chem2Motor.drive(chemicalSpeeds[1]);
      Chem3Motor.drive(chemicalSpeeds[2]);
      Watchdog.clear();
      break;
  }
}

void Estop()
{
  Serial.println("Estop triggered");
  Chem1Motor.drive(0);
  Chem2Motor.drive(0);
  Chem3Motor.drive(0);
  GenevaMotor.drive(0);

}
