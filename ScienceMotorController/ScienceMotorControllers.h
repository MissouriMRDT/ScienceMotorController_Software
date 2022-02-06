#ifndef _SRAActuation
#define _SRAActuation

#include "RoveComm.h"
#include "RoveStmVnhPwm.h"
#include "RoveUsDigiMa3Pwm.h"
#include "RoveWatchdog.h"
#include "Servo.h"

// motor pin declarations

#define MOTOR_1_PWM PF_1
#define MOTOR_1_IN_A A1_5
#define MOTOR_1_IN_B A1_6

#define MOTOR_2_PWM PF_2
#define MOTOR_2_IN_A A1_7
#define MOTOR_2_IN_B A1_8

#define MOTOR_3_PWM PF_3
#define MOTOR_3_IN_A A1_9
#define MOTOR_3_IN_B A1_10

// motors 4-6 are solenoids

#define MOTOR_4_PWM PG_1
#define MOTOR_4_IN_A A2_5
#define MOTOR_4_IN_B A2_6

#define MOTOR_5_PWM PK_4
#define MOTOR_5_IN_A A2_7
#define MOTOR_5_IN_B A2_8

#define MOTOR_6_PWM PK_5
#define MOTOR_6_IN_A A2_9
#define MOTOR_6_IN_B A2_10

// servo pin declaration

#define SERVO_1 PM_0

// limit switch pin declarations

#define LIM_SWITCH_1 PP_5
#define LIM_SWITCH_2 PQ_2
#define LIM_SWITCH_3 PQ_3
#define LIM_SWITCH_4 PH_0
#define LIM_SWITCH_5 PH_1
#define LIM_SWITCH_6 PK_6

// encoder pin declarations

#define ENC_1 PM_1
#define ENC_2 PM_2
#define ENC_3 PA_7

RoveCommEthernet RoveComm;
EthernetServer TCPServer(RC_ROVECOMM_SCIENCEACTUATIONBOARD_PORT);
RoveWatchdog Watchdog;
rovecomm_packet packet;

RoveStmVnhPwm gantry_x_axis;
RoveStmVnhPwm gantry_z_axis;
RoveStmVnhPwm sensors_z_axis;
RoveStmVnhPwm water_1;
RoveStmVnhPwm water_2;
RoveStmVnhPwm water_3;

Servo scoop;

#endif
