// Motors.h
// Runs on TM4C123 for CECS347 Project 2


/////////////////////////////////////////////////////////////////////////////
// Course Number: CECS 347
// Assignment: Example project for Hardware PWM controlled Car
// Description: 
/////////////////////////////////////////////////////////////////////////////
#define WHEEL_DIR (*((volatile unsigned long *)0x40005330)) // PB7632 are the four direction pins for L298


// Constant definitions based on the following hardware interface:
// PB5432 are used for direction control on L298.
// Motor 1 is connected to the left wheel, Motor 2 is connected to the right wheel.
#define FORWARD 0x88
#define BACKWARD 0x44
#define LEFTPIVOT 0x48
#define RIGHTPIVOT 0x84

//////////////////////1. Declarations Section////////////////////////////////
////////// Function Prototypes //////////
// Dependency: None
// Inputs: None
// Outputs: None
// Description: Initializes PB5432 for use with L298N motor driver direction

// configure the system to get its clock from the PLL
void Motors_Init(void);
/////////////////////////////////////////////////////////////////////////////