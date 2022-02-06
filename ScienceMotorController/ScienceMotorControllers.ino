#include "ScienceMotorControllers.h"

void setup()
{
  Serial.begin(115200);
  RoveComm.begin(RC_SCIENCEACTUATIONBOARD_FOURTHOCTET, &TCPServer);
  RoveStmVnhPwm water[3] = {water_1, water_2, water_3};
  gantry_x_axis.attach(MOTOR_1_IN_A, MOTOR_1_IN_B , MOTOR_1_PWM);
  gantry_z_axis.attach(MOTOR_2_IN_A, MOTOR_2_IN_B, MOTOR_2_PWM);
  sensors_z_axis.attach(MOTOR_3_IN_A, MOTOR_3_IN_B, MOTOR_3_PWM);
  water_1.attach(MOTOR_4_IN_A, MOTOR_4_IN_B, MOTOR_4_PWM);
  water_2.attach(MOTOR_5_IN_A, MOTOR_5_IN_B, MOTOR_5_PWM);
  water_3.attach(MOTOR_6_IN_A, MOTOR_6_IN_B, MOTOR_6_PWM);
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
      sensors_z_axis.drive(sensor_speed[0]);
      break;
    case RC_SCIENCEACTUATIONBOARD_WATER_DATA_ID:
      int16_t* water_speeds;
      water_speeds = (int16_t*)packet.data;
      for(int i=0;i<RC_SCIENCEACTUATIONBOARD_WATER_DATA_COUNT;i++)
      {
        Serial.println(motor_speed[i]);
        water[i].drive(water_speeds[i]);
      }
      break;
    case RC_SCIENCEACTUATIONBOARD_XOOPAXIS_DATA_ID:
      int16_t* xmotor_speed;
      xmotor_speed = (int16_t*)packet.data;
      Serial.println(xmotor_speed[0]);
      gantry_x_axis.drive(xmotor_speed[0]);
      break;
    case RC_SCIENCEACTUATIONBOARD_ZOOPAXIS_DATA_ID:
      int16_t* zmotor_speed;
      zmotor_speed = (int16_t*)packet.data;
      Serial.println(zmotor_speed[0]);
      gantry_z_axis.drive(zmotor_speed[0]);
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