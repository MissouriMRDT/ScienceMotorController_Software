#ifndef _SRAActuation
#define _SRAActuation

#include "RoveComm.h"
#include "RoveStmVnhPwm.h"
#include "RoveWatchdog.h"

#define CHEM1_PIN1  CHEM_PIN1_1
#define CHEM_PIN1_1 CHEM_1_INA
#define CHEM1_PIN2  CHEM_PIN2_1
#define CHEM_PIN2_1 CHEM_1_INB
#define CHEM1_PIN3  CHEM_PIN3_1
#define CHEM_PIN3_1 CHEM_1_PWM
#define CHEM_1_INA  CHEM1_INA
#define CHEM_1_INB  CHEM1_INB
#define CHEM_1_PWM  CHEM1_PWM
#define CHEM1_INA   PL_0
#define CHEM1_INB   PL_1
#define CHEM1_PWM   PF_1

#define CHEM2_PIN1  CHEM_PIN1_2
#define CHEM_PIN1_2 CHEM_2_INA
#define CHEM2_PIN2  CHEM_PIN2_2
#define CHEM_PIN2_2 CHEM_2_INB
#define CHEM2_PIN3  CHEM_PIN3_2
#define CHEM_PIN3_2 CHEM_2_PWM
#define CHEM_2_INA  CHEM2_INA
#define CHEM_2_INB  CHEM2_INB
#define CHEM_2_PWM  CHEM2_PWM
#define CHEM2_INA   PL_2
#define CHEM2_INB   PL_3
#define CHEM2_PWM   PF_2

#define CHEM3_PIN1  CHEM_PIN1_3
#define CHEM_PIN1_3 CHEM_3_INA
#define CHEM3_PIN2  CHEM_PIN2_3
#define CHEM_PIN2_3 CHEM_3_INB
#define CHEM3_PIN3  CHEM_PIN3_3
#define CHEM_PIN3_3 CHEM_3_PWM
#define CHEM_3_INA  CHEM3_INA
#define CHEM_3_INB  CHEM3_INB
#define CHEM_3_PWM  CHEM3_PWM
#define CHEM3_INA   PQ_2
#define CHEM3_INB   PQ_3
#define CHEM3_PWM   PK_4

#define GENEVA_PIN1  GENEVA_PIN1_
#define GENEVA_PIN1_ GENEVA__INA
#define GENEVA_PIN2  GENEVA_PIN2_
#define GENEVA_PIN2_ GENEVA__INB
#define GENEVA_PIN3  GENEVA_PIN3_
#define GENEVA_PIN3_ GENEVA__PWM
#define GENEVA__INA   GENEVA_INA
#define GENEVA__INB   GENEVA_INB
#define GENEVA__PWM   GENEVA_PWM
#define GENEVA_INA   PP_3
#define GENEVA_INB   PQ_1
#define GENEVA_PWM   PG_1

#define SWITCH_1    CHEM1_SW
#define SWITCH_2    CHEM2_SW
#define SWITCH_3    CHEM3_SW
#define SWITCH_4    GENEVA_SW
#define CHEM1_SW    PG_0
#define CHEM2_SW    PL_4
#define CHEM3_SW    PE_1
#define GENEVA_SW   PE_2

RoveCommEthernet RoveComm;
EthernetServer TCPServer(RC_ROVECOMM_SCIENCEACTUATIONBOARD_PORT);

RoveStmVnhPwm Chem1Motor;
RoveStmVnhPwm Chem2Motor;
RoveStmVnhPwm Chem3Motor;
RoveStmVnhPwm GenevaMotor;

int16_t chem_Speeds[3] = { 0, 0, 0};
int16_t geneva_Speed[1] = {0};
uint8_t motorButtons[4] = {SWITCH_1, SWITCH_2, SWITCH_3, SWITCH_4};

RoveWatchdog  Watchdog;
rovecomm_packet packet;

void BeginSerial();
void BeginRoveComm();
void AttachChemMotors();
void AttachGenevaMotors();
void InitButtons();
void AttachWatchdog();
void BeginWatchdog();
void ParsePackets();
void ScienceActuationBoard_GenevaOpenLoop();
void ScienceActuationBoard_Chemicals();
int16_t* ReceiveGenevaSpeed();
void PrintGenevaSpeed(int16_t* genevaSpeed);
void DriveGenevaMotor(int16_t* genevaSpeed);
int16_t* ReceiveChemSpeeds();
void CheckButtons();
void CheckButtonsEstop();
void Estop();
void WatchdogClear();

#endif
