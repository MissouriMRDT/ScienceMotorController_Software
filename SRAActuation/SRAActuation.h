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

#define GENEVA_ENC   PM_4
#define Z_AXIS_ENC   PM_2

#define CHEM1_SW     PG_0
#define CHEM2_SW     PL_4
#define CHEM3_SW     PE_1
#define GENEVA_SW    PE_2

#define NUM_TEST_TUBES      12
#define TARGET_DEGREE       360/NUM_TEST_TUBES
#define DEGREE_TOLERANCE    15
#define GENEVA_SPEED        500
#define CHEM_SPEED          140

RoveCommEthernet RoveComm;
EthernetServer TCPServer(RC_ROVECOMM_SCIENCEACTUATIONBOARD_PORT);

RoveStmVnhPwm Chem1Motor;
RoveStmVnhPwm Chem2Motor;
RoveStmVnhPwm Chem3Motor;
RoveStmVnhPwm GenevaMotor;
RoveStmVnhPwm Z_AxisMotor;

RoveUsDigiMa3Pwm GenevaEncoder;
RoveUsDigiMa3Pwm Z_AxisEncoder;

uint8_t motorButtons[4] = {CHEM1_SW, CHEM2_SW, CHEM3_SW, GENEVA_SW};
uint8_t genevaPos = 0;
float currentAngle = 0;

RoveWatchdog  Watchdog;

rovecomm_packet packet;

void Estop();
void GenevaIncPos();
void GenevaToPos();
void CheckButtons();

#endif
