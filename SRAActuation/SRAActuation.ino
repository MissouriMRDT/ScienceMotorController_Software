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
  GenevaEncoder.attach(GENEVA_ENC);
  Z_AxisEncoder.attach(Z_AXIS_ENC);
  GenevaEncoder.start();
  Z_AxisEncoder.start();

  for (uint8_t i = 0; i < 4; i++)
  {
    pinMode(motorButtons[i], INPUT);
  }

  //attach estop to watchdog, cut motors after 150 ms of no comms (control rate is 100ms)
  Watchdog.attach(Estop);
  Watchdog.start(150);
  
  genevaPos = static_cast<uint8_t>(GenevaEncoder.readDegrees() / 30.0 + 1.5); //Initialize the absolute geneva position
}

void loop()
{
  packet = RoveComm.read();
  if(packet.data_id != 6){
    Serial.println("Actual");
    Serial.println(packet.data_id);
    Serial.println("Desired");
    Serial.println(RC_SCIENCEACTUATIONBOARD_GENEVATOPOSITION_DATA_ID);
    Serial.println("Data ID:");
    Serial.println(((int8_t*)packet.data)[0]);
  }
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
      GenevaToPos();
      Watchdog.clear();
      break;

    case RC_SCIENCEACTUATIONBOARD_GENEVAINCREMENTPOSITION_DATA_ID:
      GenevaIncPos();
      Watchdog.clear();
      break;
  }
  CheckButtons();
  RoveComm.write(RC_SCIENCEACTUATIONBOARD_GENEVACURRENTPOSITION_DATA_ID, RC_SCIENCEACTUATIONBOARD_GENEVACURRENTPOSITION_DATA_COUNT, genevaPos);
}

void Estop()
{
  //Serial.println("Estop triggered");
  //Serial.println("Watchdog cleared");
  Chem1Motor.drive(0);
  Chem2Motor.drive(0);
  Chem3Motor.drive(0);
  GenevaMotor.drive(0);
  Z_AxisMotor.drive(0);
  Watchdog.clear();
}

void GenevaIncPos()
{
  genevaPos = static_cast<uint8_t>(GenevaEncoder.readDegrees() / 30.0 + 1.5); //Update the absolute geneva position

  int8_t geneva_inc = ((int8_t*)packet.data)[0];
  if(abs(geneva_inc) >= NUM_TEST_TUBES){ //Input out of bounds
    Watchdog.clear();
    return;
  }
  
  uint8_t goal_geneva_pos = genevaPos + geneva_inc + NUM_TEST_TUBES; //Goal position read in from the packet
  if(goal_geneva_pos > NUM_TEST_TUBES){
    goal_geneva_pos %= NUM_TEST_TUBES;
  }
  
  uint16_t goal_geneva_angle = (goal_geneva_pos - 1) * TARGET_DEGREE; //The angle to move to

  int8_t clockwise_distance = (goal_geneva_pos - genevaPos + NUM_TEST_TUBES) % NUM_TEST_TUBES; //how far the geneva would have to turn clockwise
  int8_t counterclockwise_distance = NUM_TEST_TUBES - clockwise_distance; // " " " counterclockwise
  
  int16_t geneva_movement = (clockwise_distance > counterclockwise_distance ? -1 : 1) * GENEVA_SPEED; //Finds the best direction to spin
  
  while(abs(currentAngle - goal_geneva_angle) > DEGREE_TOLERANCE && 360 - abs(currentAngle - goal_geneva_angle) > DEGREE_TOLERANCE)
  {
    GenevaMotor.drive(geneva_movement);
    currentAngle = GenevaEncoder.readDegrees();
  }
  
  GenevaMotor.drive(0);
  genevaPos = static_cast<uint8_t>(GenevaEncoder.readDegrees() / 30.0 + 1.5); //Update the absolute geneva position
  Watchdog.clear();
}

void GenevaToPos()
{
  genevaPos = static_cast<uint8_t>(GenevaEncoder.readDegrees() / 30.0 + 1.5); //Update the absolute geneva position
  
  uint8_t goal_geneva_pos = ((int8_t*)packet.data)[0]; //Goal position read in from the packet
  uint16_t goal_geneva_angle = (goal_geneva_pos - 1) * TARGET_DEGREE; //The angle to move to
  

  if(goal_geneva_pos == 0 || goal_geneva_pos > NUM_TEST_TUBES){ //The specified goal is out of bounds.
    Watchdog.clear();
    return;
  }

  int8_t clockwise_distance = (goal_geneva_pos - genevaPos + NUM_TEST_TUBES) % NUM_TEST_TUBES; //how far the geneva would have to turn clockwise
  int8_t counterclockwise_distance = NUM_TEST_TUBES - clockwise_distance; // " " " counterclockwise
  
  int16_t geneva_movement = (clockwise_distance > counterclockwise_distance ? -1 : 1) * GENEVA_SPEED; //Finds the best direction to spin
  
  while(abs(currentAngle - goal_geneva_angle) > DEGREE_TOLERANCE && 360 - abs(currentAngle - goal_geneva_angle) > DEGREE_TOLERANCE)
  {
    GenevaMotor.drive(geneva_movement);
    currentAngle = GenevaEncoder.readDegrees();
  }
  
  GenevaMotor.drive(0);
  genevaPos = static_cast<uint8_t>(GenevaEncoder.readDegrees() / 30.0 + 1.5); //Update the absolute geneva position
  Watchdog.clear();
}

void CheckButtons()
{
  for (uint8_t i = 0; i < 4; i++)
  {
    if (digitalRead(motorButtons[i]))
    {
      if (i == 0)
      {
        Chem1Motor.drive(CHEM_SPEED);
      }
      else if (i == 1)
      {
        Chem2Motor.drive(CHEM_SPEED);
      }
      else if (i == 2)
      {
        Chem3Motor.drive(CHEM_SPEED);
      }
      else if (i == 3)
      {
        GenevaMotor.drive(GENEVA_SPEED);
      }
      Watchdog.clear();
    }
  }
}
