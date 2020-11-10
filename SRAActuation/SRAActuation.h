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

#define CHEM1_SW    PG_0
#define CHEM2_SW    PL_4
#define CHEM3_SW    PE_1
#define GENEVA_SW   PE_2

RoveCommEthernet RoveComm;
EthernetServer TCPServer(RC_ROVECOMM_ETHERNET_SRA_ACTUATIONBOARD_PORT);

RoveStmVnhPwm Chem1Motor;
RoveStmVnhPwm Chem2Motor;
RoveStmVnhPwm Chem3Motor;

RoveStmVnhPwm GenevaMotor;

uint8_t chemicalSpeeds[3] = { 0, 0, 0};
uint8_t genevaSpeed[1] = {0};
uint8_t motorButtons[4] = {CHEM1_SW, CHEM2_SW, CHEM3_SW, GENEVA_SW};

RoveWatchdog  Watchdog;

rovecomm_packet packet;

void Estop();

#endif
