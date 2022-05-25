#ifndef _SMoco
#define _SMoco

#include "RoveComm.h"
#include "RoveStmVnhPwm.h"
#include "RoveUsDigiMa3Pwm.h"
#include "RoveWatchdog.h"
#include "RovePid.h"

// motor pin declarations

#define MOTOR_1_PWM PD_1 // sensor Z axis moco
#define MOTOR_1_IN_A PL_0
#define MOTOR_1_IN_B PL_1
#define MOTOR_1_CS PE_0

#define MOTOR_2_PWM PM_4 // gantry X axis moco
#define MOTOR_2_IN_A PK_4
#define MOTOR_2_IN_B PG_1
#define MOTOR_2_CS PE_1

#define MOTOR_3_PWM PM_5 // gantry Z axis moco
#define MOTOR_3_IN_A PF_2
#define MOTOR_3_IN_B PF_1
#define MOTOR_3_CS PE_2

#define MOTOR_4_PWM PL_4 // spare moco
#define MOTOR_4_IN_A PH_0
#define MOTOR_4_IN_B PH_1
#define MOTOR_4_CS PE_3

// servo pin declaration

#define SERVO_1 PB_2
#define SERVO_2 PB_3

// limit switch pin declarations

#define LIM_SWITCH_1_TOP PE_5 // sensor Z axis limits
#define LIM_SWITCH_1_BOTTOM PC_4
#define LIM_SWITCH_2_TOP PC_5 // gantry X axis limits
#define LIM_SWITCH_2_BOTTOM PC_6
#define LIM_SWITCH_3_TOP PD_3 // gantry Z axis limits
#define LIM_SWITCH_3_BOTTOM PC_7

// encoder pin declarations

#define ENC_1 PA_6 // sensor Z axis enc
#define ENC_2 PD_2 // gantry X axis enc
#define ENC_3 PD_5 // gantry Z axis enc
#define ENC_4 PD_3 // spare enc
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

#define SCOOP_OPEN_VALUE 115   // PWM value (0-255) corresponding to scoop open position
#define SCOOP_CLOSED_VALUE 150 // PWM value (0-255) corresponding to scoop closed position
#define SCOOP_MAX_ANGLE 180
#define SCOOP_MIN_ANGLE 0
#define SCOOP_THRESHOLD 2

#define GANTX_POS_0 0
#define GANTX_POS_1 360.0
#define GANTX_POS_2 720.0
#define GANTX_POS_3 1080.0

#define SERIAL_BAUD

#define WATCHDOG_TIMEOUT 500 // ms without recieving new packet before estop

// PID parameters

// #define SENSZ_KP 1f
// #define SENSZ_KI 1f
// #define SENSZ_KD 0.5f
// #define SENSZ_TOL 30f

//#define GANTX_KP 1f
//#define GANTX_KI 1f
//#define GANTX_KD 0.5f
//#define GANTX_TOL 30f

// #define GANTZ_KP 1f
// #define GANTZ_KI 1f
// #define GANTZ_KD 0.5f
// #define GANTZ_TOL 30f

// RoveComm object declarations

RoveCommEthernet RoveComm;
EthernetServer TCPServer(RC_ROVECOMM_SCIENCEACTUATIONBOARD_PORT);
rovecomm_packet packet;

// motor controller object declarations

RoveStmVnhPwm sensZ;
RoveStmVnhPwm gantX;
RoveStmVnhPwm gantZ;
RoveStmVnhPwm spare;

int16_t sensZTarget;
int16_t gantXTarget;
int16_t gantZTarget;

float gantXPos[4] = {GANTX_POS_0, GANTX_POS_1, GANTX_POS_2, GANTX_POS_3};

// RovePidFloats sensZPID;
RovePidFloats gantXPID;
// RovePidFloats gantZPID;

RoveWatchdog watchdog;

bool solStates[9];

uint8_t sol[9] = {SOL_1, SOL_2, SOL_3, SOL_4, SOL_5, SOL_6, SOL_7, SOL_8, SOL_9};

uint8_t lim[6] = {LIM_SWITCH_1_TOP, LIM_SWITCH_1_BOTTOM, LIM_SWITCH_2_TOP, LIM_SWITCH_2_BOTTOM, LIM_SWITCH_3_TOP, LIM_SWITCH_3_BOTTOM};

int16_t scoopAngle = 0;

uint8_t limitStates;

void eStop();

// class to track encoder degrees over multiple rotations
class EncPosTracker
{
public:
    RoveUsDigiMa3Pwm enc;

    uint8_t encPos;
    uint8_t lastEncPos;

    int16_t pos;
    int16_t lastPos;

    void attach(uint8_t pin)
    {
        enc.attach(pin);
    }

    void start()
    {
        enc.start();
    }

    void stop()
    {
        enc.stop();
    }

    int16_t readDegrees()
    {
        encPos = enc.readDegrees();
        int encDelta = encPos - lastEncPos;

        // if there was a jump from very low to very high or vice versa, the encoder likely completed a rotation, so delta needs to be adjusted
        if (abs(encDelta) > 180)
        {
            pos += encPos > lastEncPos ? encDelta - 360 : encDelta + 360; // if the pos went from low to high, subtract 360 from the reading, otherwise add 360, then increment pos value
        }
        else
            pos += encDelta;

        return pos;
    }
};

EncPosTracker sensZEnc;
EncPosTracker gantXEnc;
EncPosTracker gantZEnc;

#endif
