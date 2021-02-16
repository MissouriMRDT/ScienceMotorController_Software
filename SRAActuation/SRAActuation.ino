#include "RoveComm.h"
#include "RoveStmVnhPwm.h"
#include "SRAActuation.h"

#include "RoveWatchdog.h"

void setup()
{
  Serial.begin(115200);


  RoveComm.begin(RC_SCIENCEACTUATIONBOARD_FOURTHOCTET, &TCPServer);

  Chem1Motor.attach(CHEM1_INA, CHEM1_INB, CHEM1_PWM);  
  Chem2Motor.attach(CHEM2_INA, CHEM2_INB, CHEM2_PWM);
  Chem3Motor.attach(CHEM3_INA, CHEM3_INB, CHEM3_PWM);
  GenevaMotor.attach(GENEVA_INA, GENEVA_INB, GENEVA_PWM); 
  Z_AxisMotor.attach(Z_AXIS_INA, Z_AXIS_INB, Z_AXIS_PWM);
  GenevaEncoder.attach(GENEVA_ENC, 7);
  Z_AxisEncoder.attach(Z_AXIS_ENC, 7);
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
  packet = RoveComm.read();
  switch (packet.data_id)
  {
    case RC_SCIENCEACTUATIONBOARD_GENEVAOPENLOOP_DATA_ID:
      Serial.println("Geneva Position: ");
      int16_t* geneva_Speed; 
      geneva_Speed = (int16_t*)packet.data;
      Serial.println(geneva_Speed[0]);
      GenevaMotor.drive(geneva_Speed[0]);
      Watchdog.clear();
      break;

    case RC_SCIENCEACTUATIONBOARD_CHEMICALS_DATA_ID:
      Serial.println("Chemical Speeds Position: ");
      int16_t* chem_Speeds; 
      chem_Speeds = (int16_t*)packet.data;
      Serial.println(chem_Speeds[0]);
      Serial.println(chem_Speeds[1]);
      Serial.println(chem_Speeds[2]);
      Chem1Motor.drive(chem_Speeds[0]);
      Chem2Motor.drive(chem_Speeds[1]);
      Chem3Motor.drive(chem_Speeds[2]); 
      Watchdog.clear();
      break;

    case RC_SCIENCEACTUATIONBOARD_ZAXIS_DATA_ID:
      Serial.println("Z-Axis Position: ");
      int16_t* z_Axis_Speed; 
      z_Axis_Speed = (int16_t*)packet.data;
      Serial.println(z_Axis_Speed[0]);
      Z_AxisMotor.drive(-z_Axis_Speed[0]);
      Watchdog.clear();
      break;

    case RC_SCIENCEACTUATIONBOARD_GENEVATOPOSITION_DATA_ID:
      Watchdog.clear();
      break;

    case RC_SCIENCEACTUATIONBOARD_GENEVAINCREMENTPOSITION_DATA_ID:
      GenevaIncPos();
      Watchdog.clear();
      break;
  }

  for(int16_t i = 0; i < 4; i++)
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
      Watchdog.clear();
    }
  }
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
  
  Serial.println("Watchdog cleared");
  Watchdog.clear();
}

void GenevaIncPos()
{
  int8_t* inc = (int8_t*)packet.data;
  currentAngle = GenevaEncoder.readDegrees();
  if (inc[0] > 0)
  {
    uint16_t lastAngle = currentAngle;
    while (currentAngle < (lastAngle + 28))
    {
      GenevaMotor.drive(140);
      currentAngle = GenevaEncoder.readDegrees();
    }
    genevaPos++;
  }
  else 
  {
    uint16_t lastAngle = currentAngle;
    while (currentAngle > (lastAngle - 32))
    {
      GenevaMotor.drive(-140);
      currentAngle = GenevaEncoder.readDegrees();
    }
    genevaPos--;
  }
}