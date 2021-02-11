#ifndef _SRAActuation
#define _SRAActuation

#include "RoveComm.h"
#include "RoveStmVnhPwm.h"
#include "RoveUsDigiMa3Pwm.h"
#include "RoveWatchdog.h"


#define CHEM1_INA   PC_6
#define CHEM1_INB   PE_5
#define CHEM1_PWM   PF_1

#define CHEM2_INA   PD_3
#define CHEM2_INB   PC_7
#define CHEM2_PWM   PF_2

#define CHEM3_INA   PB_2
#define CHEM3_INB   PB_3
#define CHEM3_PWM   PF_3

#define GENEVA_INA   PD_4
#define GENEVA_INB   PD_5
#define GENEVA_PWM   PG_1

#define Z_AXIS_INA   PQ_0
#define Z_AXIS_INB   PP_4
#define Z_AXIS_PWM   PK_4

#define Z_UPPER _LIM PP_5
#define Z_LOWER_LIM  PA_7
#define GENEVA_HOME  PQ_2
#define GENEVA_SET   PQ_3

#define GENEVA_ENC   PM_1
#define Z_AXIS_ENC   PM_2

#define CHEM1_SW    PG_0
#define CHEM2_SW    PL_4
#define CHEM3_SW    PE_1
#define GENEVA_SW   PE_2

#define TUBE_1      0
#define TUBE_2      30
#define TUBE_3      60
#define TUBE_4      90
#define TUBE_5      120
#define TUBE_6      150
#define TUBE_7      180
#define TUBE_8      210
#define TUBE_9      240
#define TUBE_10     270
#define TUBE_11     300
#define TUBE_12     330

RoveCommEthernet RoveComm;
EthernetServer TCPServer(RC_ROVECOMM_SCIENCEACTUATIONBOARD_PORT);

RoveStmVnhPwm Chem1Motor;
RoveStmVnhPwm Chem2Motor;
RoveStmVnhPwm Chem3Motor;
RoveStmVnhPwm GenevaMotor;
RoveStmVnhPwm Z_AxisMotor;

RoveUsDigiMa3Pwm GenevaEncoder;
RoveUsDigiMa3Pwm Z_AxisEncoder;

int16_t chem_Speeds[3] = { 0, 0, 0};
int16_t geneva_Speed = 0;
int16_t z_Axis_Speed = 0;
uint8_t motorButtons[4] = {CHEM1_SW, CHEM2_SW, CHEM3_SW, GENEVA_SW};
uint8_t genevaPos = 0;
uint16_t currentAngle = 0;

RoveWatchdog  Watchdog;

rovecomm_packet packet;

void Estop();
void SetMotorSpeed(int16_t& Speed);
void SetMotorSpeed(int16_t Speed[]);
void CheckButtons();
void ParsePackets();
void DriveMotors();
void GenevaIncPos();

#endif
