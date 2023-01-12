#ifndef _SRAActuation
#define _SRAActuation

#include "RoveComm.h"
#include "RoveStmVnhPwm.h"
#include "RoveUsDigiMa3Pwm.h"
#include "RoveWatchdog.h"


//Motor controller pin connections
#define FWD_PWM_1   7
#define RVS_PWM_1   6
#define FWD_PWM_2   5
#define RVS_PWM_2   4
#define FWD_PWM_3   3
#define RVS_PWM_3   2
#define FWD_PWM_4   0
#define RVS_PWM_4   1

//Servo pin connections
#define Servo_1     8
#define Servo_2     9
#define Servo_3     10
#define Servo_4     13
#define Servo_5     12
#define Servo_6     11

//Limit Swtich pin connections
#define Limit_Switch_1  23
#define Limit_Switch_2  22
#define Limit_Switch_3  21
#define Limit_Switch_4  20
#define Limit_Switch_5  17
#define Limit_Switch_6  16
#define Limit_Switch_7  41
#define Limit_Switch_8  40
#define Limit_Switch_9  39

//Encoder pin connections
#define Encoder_PWM_1   14
#define Encoder_PWM_2   15
#define Encoder_PWM_3   18
#define Encoder_PWM_4   19

//Manual Switch pin connections
#define Manual_Control_Right    38
#define Manual_Control_Left     37

//Motor Controller Button pin connections
#define Motor_1_SW  27
#define Motor_2_SW  26
#define Motor_3_SW  25
#define Motor_4_SW  24

//Servo Button pin connections
#define Servo_1_SW  36
#define Servo_2_SW  35
#define Servo_3_SW  34
#define Servo_4_SW  33
#define Servo_5_SW  31
#define Servo_6_SW  32

#define NUM_TEST_TUBES      12
#define TARGET_DEGREE       360/NUM_TEST_TUBES
#define DEGREE_TOLERANCE    10
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
void GenevaIncPos();
void GenevaToPos();
void CheckButtons();

#endif
