// TrackExplorer.c
// Runs on TM4C123
// This is the starter file for CECS 347 Project 2 - A Track Explorer
// This project uses hardware PWM to control two DC Motors, 
// ADC to collect analog inputs from three Sharp IR sensors.
// The three Sharp analog IR distance sensors (GP2Y0A21YK0F) are used
// to allowthe robot to navigate through a track with two walls: 
// one mounted looking directly forward to avoid a head-on collision, 
// the other two looking forward to the left and to the right to detect  
// the distances between the car and the two walls. The goal is to 
// control power to each wheel so the left and right distances to the 
// walls are equal.
// If an object is detected too close to the robot, 
// the robot should be able to avoid it.
/*
    ------------------------------------------wall---------
                      /
                     /
                    / 
                   /
         -----------
         |         |
         | Robot   | ---> direction of motion and third sensor
         |         |
         -----------
                   \
                    \
                     \
                      \
    ------------------------------------------wall---------
*/
// The original project is designed by Dr. Daniel Valvano, Jonathan Valvano
// September 12, 2013
// Modifications are made by Dr. Min He.

// PE1 connected to forward facing IR distance sensor
// PE4 connected to right IR distance sensor
// PE5 connected to left IR distance sensor

#include "tm4c123gh6pm.h"
#include "Sensors.h"
#include "Motors.h"
#include "LEDSW.h"
#include "PLL.h"
#include "stdint.h"
#include "PWM.h"


// basic functions defined at end of startup.s
extern void DisableInterrupts(void); // Disable interrupts
extern void EnableInterrupts(void);  // Enable interrupts
extern void WaitForInterrupt(void);  // low power mode


// You use datasheet to calculate the following ADC values
// then test your sensors to adjust the values 
#define CRASH             IR15CM// if there is less than this distance ahead of the robot, it will immediately stop
#define IR15CM            2233  // ADC output for 15cm:1.8v -> (1.8/3.3)*4095=2233 
#define IR20CM            1724  // ADC output for 20cm:1.39v -> (1.39/3.3)*4095=1724
#define IR30CM            1116  // ADC output for 30cm:0.9v -> (0.9/3.3)*4095=1116
#define IR40CM            918   // ADC output for 40cm:0.74v -> (0.74/3.3)*4095=918
#define IR80CM            496   // ADC output for 80cm:0.4v -> (0.4/3.3)*4095=496
                                // with equal power to both motors (LeftH == RightH), the robot still may not drive straight
                                // due to mechanical differences in the motors, so bias the left wheel faster or slower than
                                // the constant right wheel
#define LEFTPOWER        	0.5*PERIOD   // duty cycle of left wheel 
#define RIGHTPOWER        0.5*PERIOD    // duty cycle of left wheel 
#define GREEN           0x08 // 
#define BLUE            0x04 // 
#define RED       			0x02 // 
#define LED             GPIO_PORTF_DATA_R
#define NVIC_EN0_PORTF 0x40000000
#define SWITCH1 (*((volatile unsigned long *)0x40004004)) // Address of SW1 (let switch)
#define SWITCH2 (*((volatile unsigned long *)0x40004008)) // Address of SW2 (right switch)
#define SW1 0x10
#define SW2 0x01
int tracker = 0;
//void SysTick_Init(void);
//void Buttons_Init(void);
void System_Init(void);
void PORTF_Init_BLUE(void);
void PORTF_Init_GREEN(void);
void PORTF_Init_RED(void);
void steering(uint16_t ahead_dist,uint16_t right_dist, uint16_t left_dist);
void Debounce(void);
void GPIOPortF_Handler(void);
//void ReadSensorsMedianFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8);uint16_t median(uint16_t u1, uint16_t u2, uint16_t u3);
//void ReadSensorsIIRFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8);
//void ReadSensorsFIRFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8);
//void Sensors_In(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8);
//void Sensors_Init(void);
int main(void){
  uint16_t left, right, ahead;
  LEDSW_Init();
	Switch_Init();
  DisableInterrupts();  // disable interrupts while initializing
  System_Init();
  EnableInterrupts();   // enable after all initialization are done
	PLL_Init();                         // 80 MHz
	PORTF_Init_BLUE();
	PORTF_Init_GREEN();
	PORTF_Init_RED();
	Motors_Init();
  PWM_PB54_Init();
  PWM_PB54_Duty(START_SPEED, START_SPEED);
	

//	
  // TODO: Calibrate the sensors: read at least 5 times from the sensor 
	// before the car starts to move: this will allow software to filter the sensor outputs.	


	// TODO: start with moving forward, LED green 
		//PWM_PB54_Duty(8000,8000);
		PWM_PB54_Duty(10000,10000);
	  LED = GREEN;
		WHEEL_DIR = FORWARD;
		PWM0_ENABLE_R |= 0x0000000C; // enable both wheels

	
  while(1){
		// choose one of the following three software filter methods
		for(int x = 0 ; x < 9 ; x++){
		ReadSensorsMedianFilter(&ahead, &right, &left);
		}
//		ReadSensorsIIRFilter(&ahead, &right, &left);
//		ReadSensorsFIRFilter(&ahead, &right, &left);
		ReadSensorsMedianFilter(&ahead, &right, &left);
		steering(ahead,right,left); //put a breakpoint here// and possible delay here
		
  }
}

void System_Init(void) {
  PLL_Init();           // bus clock at 80 MHz
  Sensors_Init();        // initialize ADC to sample AIN2 (PE1), AIN9 (PE4), AIN8 (PE5)
  LEDSW_Init();         // configure onboard LEDs and push buttons
	PWM_PB54_Init();
  Motors_Init();         // Initialize signals for the two DC Motors
}

void GPIOPortF_Handler(void){
	
	if(GPIO_PORTF_RIS_R&SW1){
		
		GPIO_PORTF_ICR_R = SW1;
		tracker =1 ;
		LED = BLUE;
	}
	if (GPIO_PORTF_RIS_R &SW2){
		GPIO_PORTF_ICR_R = SW2;
		tracker =0;
		LED = RED;
	}
}
	
	
	//	Debounce();
//	if(GPIO_PORTF_RIS_R & SW1){
//		PWM0_ENABLE_R |= 0x0000000C;
//		GPIO_PORTF_ICR_R = SW1;
//	}
//	if(GPIO_PORTF_RIS_R & SW2){
//		PWM0_ENABLE_R &= ~0x0000000C;
//		GPIO_PORTF_ICR_R = SW2;
//	}
		
// had a }

void steering(uint16_t ahead_dist,uint16_t right_dist, uint16_t left_dist){
	// Suggest the following simple control as starting point:
  // 1. Take care of off center positions:
	//    If left off center track, turn right; 
	//    If right off center track, turn left
  // 2. Take care of turning track: 
	//    make a right turn if left opening detected;
	//    make a left turn if right opening detected;
  // 3. Optional: Take care of crach ahead: stop if crash ahead
	// 4.	go straight if moving in the center area
  // Feel free to add more controlls to fine tune your robot car.
  // Make sure to take care of both wheel movements and LED display.
	uint16_t left, right, ahead;
	PWM0_ENABLE_R |= 0x0000000C;
	if (tracker == 1){
		
	ReadSensorsMedianFilter(&ahead, &right, &left);
		if((ahead_dist > IR40CM && left_dist > IR40CM && right_dist > IR40CM))
	{
		LED = RED;
		//PWM_PB54_Duty(8000,8000);
		PWM_PB54_Duty(10000,10000);
		WHEEL_DIR = BACKWARD;
		PWM0_ENABLE_R |= 0x0000000C;
	}
	else if((left_dist > IR40CM && ahead_dist > IR40CM) || (left_dist > IR40CM))
	{
		LED = RED;
		//PWM_PB54_Duty(9000,9000);
		PWM_PB54_Duty(10000,10000);
		WHEEL_DIR = LEFTPIVOT;
		PWM0_ENABLE_R |= 0x0000000C; // Enable both wheels
//		PWM0_ENABLE_R |= 0x00000004; // Enable right wheel
//		PWM0_ENABLE_R &= ~0x00000008;
	}
	else if((right_dist > IR40CM && ahead_dist > IR40CM) || (right_dist > IR40CM))
	{
		LED = RED;
		//PWM_PB54_Duty(9000,9000);
		PWM_PB54_Duty(10000,10000);
		WHEEL_DIR = RIGHTPIVOT;
		PWM0_ENABLE_R |= 0x0000000C; // Enable both wheels
//		PWM0_ENABLE_R |= 0x00000008; // Enable right wheel
//		PWM0_ENABLE_R &= ~0x00000004;
	}
	else if(ahead_dist < IR80CM && left_dist < IR80CM && right_dist < IR80CM)
	{
		LED = BLUE;
		PWM0_ENABLE_R &= ~0x0000000C;
	}
	else
	{
		LED = GREEN;
		//PWM_PB54_Duty(8500, 8500);
		PWM_PB54_Duty(10000,10000);
		WHEEL_DIR = FORWARD;
		PWM0_ENABLE_R |= 0x0000000C;
	} 
}else if (tracker == 0 ){
		PWM0_ENABLE_R &= ~0x0000000C;
	LED &= 0x00;
	}
}
//void SysTick_Init(void){
//	NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
//  NVIC_ST_RELOAD_R = NVIC_ST_RELOAD_M;  // maximum reload value 
//  NVIC_ST_CURRENT_R = 0;                // any write to current clears it
//                                        // enable SysTick with core clock
//  
//	NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R & 0x00FFFFFF) | 0x20000000;
//  NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE+NVIC_ST_CTRL_CLK_SRC+NVIC_ST_CTRL_INTEN;
//}
//void Buttons_Init(void)
//{  
//  GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY; 	// unlock GPIO Port F
//  GPIO_PORTF_CR_R |= 0x11;         		// allow changes to PF4,0
//  GPIO_PORTF_DIR_R &= ~0x11;    			// (c) make PF4,0 in (built-in button)
//  GPIO_PORTF_AFSEL_R &= ~0x11;  			//     disable alt funct on PF4,0
//  GPIO_PORTF_DEN_R |= 0x11;     			//     enable digital I/O on PF4,0
//  GPIO_PORTF_PCTL_R &= ~0x000F000F; 	//  configure PF4,0 as GPIO
//  GPIO_PORTF_AMSEL_R &= ~0x11;  			//     disable analog functionality on PF4,0
//  GPIO_PORTF_PUR_R |= 0x11;     			//     enable weak pull-up on PF4,0
//  GPIO_PORTF_IS_R &= ~0x11;     			// (d) PF4,PF0 is edge-sensitive
//  GPIO_PORTF_IBE_R &= ~0x11;    			//     PF4,PF0 is not both edges
//  GPIO_PORTF_IEV_R &= ~0x11;    			//     PF4,PF0 falling edge event
//  GPIO_PORTF_ICR_R = 0x11;      			// (e) clear flags 4,0
//  GPIO_PORTF_IM_R |= 0x11;      			// (f) arm interrupt on PF4,PF0
//  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF1FFFFF)|0x00400000; // (g) bits:23-21 for PORTF, set priority to 2
//  NVIC_EN0_R |= NVIC_EN0_PORTF;      // (h) enable interrupt 30 in NVIC
//}

//void PORTF_Init_RED(void){
//  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // activate port F
//	while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOF)!=SYSCTL_RCGC2_GPIOF){}
//		
//  GPIO_PORTF_DIR_R |= 0x02;             // make PF2 out (built-in LED)
//  GPIO_PORTF_AFSEL_R &= ~0x02;          // disable alt funct on PF2
//  GPIO_PORTF_DEN_R |= 0x02;             // enable digital I/O on PF2
//                                        // configure PF2 as GPIO
//  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF0FF)+0x00000000;
//  GPIO_PORTF_AMSEL_R = 0;               // disable analog functionality on PF
//}
//void PORTF_Init_GREEN(void){
//  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // activate port F
//	while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOF)!=SYSCTL_RCGC2_GPIOF){}
//		
//  GPIO_PORTF_DIR_R |= 0x08;             // make PF2 out (built-in LED)
//  GPIO_PORTF_AFSEL_R &= ~0x08;          // disable alt funct on PF2
//  GPIO_PORTF_DEN_R |= 0x08;             // enable digital I/O on PF2
//                                        // configure PF2 as GPIO
//  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF0FF)+0x00000000;
//  GPIO_PORTF_AMSEL_R = 0;               // disable analog functionality on PF
//}
//void PORTF_Init_YELLOW(void){
//  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // activate port F
//	while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOF)!=SYSCTL_RCGC2_GPIOF){}
//		
//  GPIO_PORTF_DIR_R |= 0x0A;             // make PF2 out (built-in LED)
//  GPIO_PORTF_AFSEL_R &= ~0x0A;          // disable alt funct on PF2
//  GPIO_PORTF_DEN_R |= 0x0A;             // enable digital I/O on PF2
//                                        // configure PF2 as GPIO
//  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF0FF)+0x00000000;
//  GPIO_PORTF_AMSEL_R = 0;               // disable analog functionality on PF
//}
//void PORTF_Init_BLUE(void){
//  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // activate port F
//	while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOF)!=SYSCTL_RCGC2_GPIOF){}
//		
//  GPIO_PORTF_DIR_R |= 0x04;             // make PF2 out (built-in LED)
//  GPIO_PORTF_AFSEL_R &= ~0x04;          // disable alt funct on PF2
//  GPIO_PORTF_DEN_R |= 0x04;             // enable digital I/O on PF2
//                                        // configure PF2 as GPIO
//  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF0FF)+0x00000000;
//  GPIO_PORTF_AMSEL_R = 0;               // disable analog functionality on PF
//}
void Debounce(void) {
	unsigned long j;
	for (j = 0; j < 199999; j++) {
	}
}