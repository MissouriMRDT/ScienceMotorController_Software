#ifndef PINASSIGNMENTS_H
#define PINASSIGNMENTS_H


// Motor Pins
#define MOCO1_FWD    29 // Spare FWD (mag wired from 32)
#define MOCO1_RVS    28 // Spare RVS (mag wired from 31)
#define MOCO2_FWD    9 // Auger Axis FWD
#define MOCO2_RVS    8 // Auger Axis RVS
#define MOCO3_FWD    11 // Sensor Axis FWD
#define MOCO3_RVS    10 // Sensor Axis RVS
#define MOCO4_FWD    6 // Auger FWD
#define MOCO4_RVS    7 // Auger RVS

// Servo Pins
#define SERVO_1    22 // Science Pan
#define SERVO_2    23 // Science Tilt

// Encoder Pins
#define ENCODER_1A    40 // Auger Axis A
#define ENCODER_1B    41 // Auger Axis B
#define ENCODER_2A    18 // Auger A
#define ENCODER_2B    19 // Auger B
#define ENCODER_3A    21 // Spare A
#define ENCODER_3B    20 // Spare B
#define ENCODER_4A    14 // Sensor Axis A
#define ENCODER_4B    13 // Sensor Axis B

// Limit Swtich Pins
#define LIMITSWITCH1    33 // Sensor Axis RVS
#define LIMITSWITCH2    34 // Sensor Axis FWD
#define LIMITSWITCH3    36 // Auger Axis FWD
#define LIMITSWITCH4    37 // Spare RVS
#define LIMITSWITCH5    38 // Spare FWD
#define LIMITSWITCH6    35 // Auger Axis RVS

// Manual Switch Pins
#define DIR_SW    17

// Motor Button pins
#define SW1    2 // Auger
#define SW2    0 // Auger Axis
#define SW3    4 // Sensor Axis
#define SW4    1 // Spare Motor
#define SW5    3 // Science Pan
#define SW6    5 // Science Tilt

// Laser
#define LAS    31

// Temp and Humidity 
#define TEMP    15
#define HUMIDITY 16

// Peltier Module
#define PELTIER    27


#endif