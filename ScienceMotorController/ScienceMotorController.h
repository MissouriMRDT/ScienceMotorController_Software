#ifndef _SRAActuation
#define _SRAActuation

#include "RoveComm.h"
#include "RoveStmVnhPwm.h"
#include "RoveUsDigiMa3Pwm.h"
#include "RoveWatchdog.h"
#include "Servo.h"

// motor pin declarations

#define MOTOR_1_PWM PD_1
#define MOTOR_1_IN_A PL_0
#define MOTOR_1_IN_B PL_1
#define MOTOR_1_CS PE_0

#define MOTOR_2_PWM PM_4
#define MOTOR_2_IN_A PK_4
#define MOTOR_2_IN_B PG_1
#define MOTOR_2_CS PE_1

#define MOTOR_3_PWM PM_5
#define MOTOR_3_IN_A PF_2
#define MOTOR_3_IN_B PF_1
#define MOTOR_3_CS PE_2

#define MOTOR_4_PWM PL_4
#define MOTOR_4_IN_A PH_0
#define MOTOR_4_IN_B PH_1
#define MOTOR_4_CS PE_3

// servo pin declaration

#define SERVO_1 PB_2
#define SERVO_2 PB_3

// limit switch pin declarations

#define LIM_SWITCH_1 PE_5
#define LIM_SWITCH_2 PC_4
#define LIM_SWITCH_3 PC_5
#define LIM_SWITCH_4 PC_6
#define LIM_SWITCH_5 PD_3
#define LIM_SWITCH_6 PC_7

// encoder pin declarations

#define ENC_1 PA_6
#define ENC_2 PD_2
#define ENC_3 PD_5
#define ENC_4 PD_3
#define ENC_5 PA_4
#define ENC_6 PA_5

// solenoid pins

#define SOL_1 PB_4
#define SOL_2 PP_0
#define SOL_3 PB_5
#define SOL_4 PP_1
#define SOL_5 PK_0
#define SOL_6 PD_4
#define SOL_7 PK_1
#define SOL_8 PK_2
#define SOL_9 PK_3

// error LED pins

#define ERR_1 PQ_0
#define ERR_2 PP_4

// parameters

#define SCOOP_OPEN_VALUE 115
#define SCOOP_CLOSED_VALUE 150

RoveCommEthernet RoveComm;
EthernetServer TCPServer(RC_ROVECOMM_SCIENCEACTUATIONBOARD_PORT);
RoveWatchdog Watchdog;
rovecomm_packet packet;

RoveStmVnhPwm sensZ;
RoveStmVnhPwm gantX;
RoveStmVnhPwm gantZ;
int16_t gantXTarget;
int16_t gantZTarget;
int16_t sensZTarget;
RoveStmVnhPwm spare;
RoveUsDigiMa3Pwm encoder[6];
bool solStates[3];
uint8_t sol[3][3] = {{SOL_1, SOL_2, SOL_3}, {SOL_4, SOL_5, SOL_6}, {SOL_7, SOL_8, SOL_9}};
int16_t scoopAngle = 0;
int16_t lastScoopAngle = 1;
#endif
