/**
 * @file TableTrafficLight.c
 * @brief Runs on LM4F120 or TM4C123
 *  A Moore finite state machine is used to operate a traffic light. 
 * @author Michelle Tran, Kevin Martinez, Grecia Francisco, Jesus Perez
 * @date 10-06-2022
 */

/* Pin Map
east - west
PB5 RED 
PB4 YELLOW
PB3 GREEN
north - south
PB2 RED
PB1 YELLOW
PB0 GREEN
PF1 RED (TURN)
PF3 GREEN (TURN)
PF1 + PF3 YELLOW (TURN)
sensor
PA3 north-south
PA2 east-west
PF4 right turn
*/

#include "SysTick.h" // used for the delay between light switching
#include "tm4c123gh6pm.h" // header file that defines the entire board

//***** PROTOTYPE *****//
void PortF_Init(void);
void PortA_Init(void);
void PortB_Init(void);

// Port Definitions
#define LIGHT_PORTB             (*((volatile unsigned long *)0x400050FC)) // port B data register
#define LIGHT_PORTF		(*((volatile unsigned long *)0x40025028)) // port F data register
#define SENSOR_PORTA            (*((volatile unsigned long *)0x40004030)) // port A data register
#define SENSOR_PORTF		(*((volatile unsigned long *)0x40025040)) // port F data register
 
// Linked data structure
struct State {
  unsigned long Out; 
  unsigned long Time;  
  unsigned long Next[8];
};
typedef const struct State STyp;
#define goEast   	0
#define waitSouth	1
#define goSouth  	2
#define waitEast 	3
#define turnEast 	4
#define waitTurn 	5
STyp FSM[6] =
{
	// goEast 1111 0000
	{0xF0, 6, {goEast, goEast, waitEast, waitEast, waitEast, waitEast, waitEast, waitEast}},
	// waitSouth 1111 1010
	{0xFA, 8, {goEast, goEast, goEast, goEast, turnEast, goEast, turnEast, goEast}},
	// goSouth 1111 1001
	{0xF9, 6, {goSouth, waitSouth, goSouth, waitSouth, turnEast, waitSouth, turnEast, waitSouth}},
	// waitEast 1110 1000
	{0xE8, 8, {goSouth, goSouth, goSouth, goSouth, turnEast, turnEast, turnEast, turnEast}},
	// turnEast 0000 1000
	{0x08, 6, {goEast, goEast, goSouth, goEast, goEast, goEast, goSouth, goEast}},
	// waitTrun 0000 1010
	{0x0A, 8, {goEast, goEast, goSouth, goEast, goEast, goEast, goSouth, goEast}}
};

unsigned long S;  // index to the current state 
unsigned long Input; 

int main(void)
{ 
	volatile unsigned long delay;
	SysTick_Init();
	PortF_Init();
  	SYSCTL_RCGC2_R |= 0x23;      // 1) B A F clocks
  	delay = SYSCTL_RCGC2_R;      // 2) no need to unlock
	
  	// define initial state
	S = goEast;	
  	while(1)
  	{
		// set light
		LIGHT_PORTB = FSM[S].Out & 0x3F;
		LIGHT_PORTF = FSM[S].Out & 0x0A; 
		
		// time delay in seconds
		SysTick_WaitSecond(FSM[S].Time);
		
		// read btn input PF4, PA2, PA3
		Input = (SENSOR_PORTA >> 2) + (SENSOR_PORTF >> 4);
		
		// set new state
		S = FSM[S].Next[Input];      
  	}
}

/***** FUNCTIONS *****/
// Subroutine to initialize port F pins for input and output
// PF4 and PF0 are input SW1 and SW2 respectively
// PF3,PF2,PF1 are outputs to the LED
// Notes: These five pins are connected to hardware on the LaunchPad
void PortF_Init(void)
{ 
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0       
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 input, PF3,PF2,PF1 output (direction)  
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) no alternate function
  GPIO_PORTF_PUR_R = 0x11;          // enable pullup resistors on PF4,PF0       
  GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital pins PF4-PF0        
}

// POSITIVE LOGIC
// PA2 & PA3 are external btns on breadboard
void PortA_Init(void)
{
	GPIO_PORTA_AMSEL_R &= ~0x0C; 	// disable analog function
	GPIO_PORTA_PCTL_R &= ~0xFF;   	// GPIO clear bit PCTL
	GPIO_PORTA_DIR_R |= 0xF3; 	// PA2 & PA3 are input (0 - input) 1111_0011
	GPIO_PORTA_AFSEL_R &= ~0x0C; 	//  no alternate function
  	GPIO_PORTA_PUR_R &= ~0x0C; 	//  diable pull up resistors
	GPIO_PORTA_PDR_R |= 0x0C; 	//  enable pull down resistors
  	GPIO_PORTA_DEN_R |= 0x0C; 	//  enable digital pins PF4-PF0
}

/*
* three LED interface that implements positive logic for the South road heading straight
* three LED interface that implements negative logic for the West road
* east - west (negative logic)
* PB5 RED 
* PB4 YELLOW
* PB3 GREEN
* north - south (positive logic)
* PB2 RED
* PB1 YELLOW
* PB0 GREEN
*/
void PortB_Init(void)
{
	GPIO_PORTB_AMSEL_R &= ~0x3F; 	// disable analog function
	GPIO_PORTB_PCTL_R &= ~0xFF;   // GPIO clear bit PCTL
	GPIO_PORTB_DIR_R |= 0x3F; 	  // all output 0011 1111 (1 - output)
	GPIO_PORTB_AFSEL_R &= ~0x3F; //  no alternate function
  	GPIO_PORTB_PUR_R &= ~0x3F; 	 //  diable pull up resistors
	GPIO_PORTB_PDR_R &= ~0x3F; 	 //  disable pull down resistors
  	GPIO_PORTB_DEN_R |= 0x3F; 	 //  enable digital pins
}
