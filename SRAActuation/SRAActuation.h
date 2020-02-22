#ifndef _SRAActuation
#define _SRAActuation

#include "RoveComm.h"
#include "RoveStmVnhPwm.h"
#include "RoveWatchdog.h"

#define CHEM1_INA   PL_0
#define CHEM1_INB   PL_1
#define CHEM1_PWM   PF_1

#define CHEM2_INA   PL_2
#define CHEM2_INB   PL_3
#define CHEM2_PWM   PF_2

#define CHEM3_INA   PQ_2
#define CHEM3_INB   PQ_3
#define CHEM3_PWM   PK_4

#define GENEVA_INA   PP_3
#define GENEVA_INB   PQ_1
#define GENEVA_PWM   PG_1

RoveCommEthernet RoveComm;

RoveStmVnhPwm Chem1Motor;
RoveStmVnhPwm Chem2Motor;
RoveStmVnhPwm Chem3Motor;

RoveStmVnhPwm GenevaMotor;

RoveWatchdog  Watchdog;

rovecomm_packet packet;

void Estop();

#endif
