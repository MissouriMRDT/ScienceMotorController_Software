#include "ScienceMotorControllers.h"

void setup()
{
  Serial.begin(115200);
  RoveComm.begin(RC_SCIENCEACTUATIONBOARD_FOURTHOCTET, &TCPServer);
  gantX = new VNHAxis(MOTOR_1_IN_A, MOTOR_1_IN_B , MOTOR_1_PWM, ENC_1, LIM_SWITCH_1, LIM_SWITCH_2);
  gantZ.attach(MOTOR_2_IN_A, MOTOR_2_IN_B, MOTOR_2_PWM);
  sensZ.attach(MOTOR_3_IN_A, MOTOR_3_IN_B, MOTOR_3_PWM);
  uint8_t* tempEnc = (uint8_t*)ENC_1;
  for(int i = 0; i < 6; i ++){
    encoder[i].attach(tempEnc[i]);
  }
}

void loop()
{
  packet = RoveComm.read();

  Serial.println("Data id: ");
  Serial.println(packet.data_id);

  switch(packet.data_id)
  {
    case RC_SCIENCEACTUATIONBOARD_SENSORAXIS_DATA_ID:
      int16_t* sensor_speed;
      sensor_speed = (int16_t*)packet.data;
      Serial.println(sensor_speed[0]);
      sensZ.drive(sensor_speed[0]);
      break;
    case RC_SCIENCEACTUATIONBOARD_WATER_DATA_ID:
      int16_t* water_speeds;
      water_speeds = (int16_t*)packet.data;
      for(int i=0;i<RC_SCIENCEACTUATIONBOARD_WATER_DATA_COUNT;i++)
      {
        Serial.println(water_speeds[i]);
        for(int a = 0; a < 3; a ++){
          digitalWrite(sol[a][i], floor(water_speeds[i] / 1000));
        }
      }
      break;
    case RC_SCIENCEACTUATIONBOARD_XOOPAXIS_DATA_ID:
      int16_t* xmotor_speed;
      xmotor_speed = (int16_t*)packet.data;
      Serial.println(xmotor_speed[0]);
      gantX.drive(xmotor_speed[0]);
      break;
    case RC_SCIENCEACTUATIONBOARD_ZOOPAXIS_DATA_ID:
      int16_t* zmotor_speed;
      zmotor_speed = (int16_t*)packet.data;
      Serial.println(zmotor_speed[0]);
      gantZ.drive(zmotor_speed[0]);
      break;
    case RC_SCIENCEACTUATIONBOARD_SCOOPGRABBER_DATA_ID:
      int16_t* scoop_degrees;
      scoop_degrees = (int16_t*)packet.data;
      Serial.println(scoop_degrees[0]);
      scoop.write(scoop_degrees[0]);
    default:
      break;
  }
}