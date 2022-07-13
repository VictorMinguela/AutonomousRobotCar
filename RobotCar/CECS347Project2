// WallFollower.c

#include <stdint.h>
#include "ADCSWTrigger.h"
#include "PLL.h"
#include "tm4c123gh6pm.h"
#include "PWM1.h"

#define PERIOD 16000 //PWM period
#define FORWARD 0x09 //Motors rotate forward (PB3 - PB0)
#define BACKWARD 0x06 //Motors rotate backward (PB3 - PB0)

#define direction      GPIO_PORTB_DATA_R
// basic functions defined at end of startup.s
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

void PortF_Init(void);
void GPIOPortF_Handler(void);
void PortB_Init(void);

int ADCtoCM(unsigned long ADCvalue);
void steer(void);
void LED_Init(void);
void GPIOPortF_Handler(void);
void SysTick_Init(unsigned long period);
int potSpeed(unsigned long pot);
void SysTick_Handler(void);
void ReadADCMedianFilter(unsigned long *ain2, unsigned long *ain9, unsigned long *ain8 );
unsigned long median(unsigned long u1, unsigned long u2, unsigned long u3);

int stop, start;
int Ldistance, Rdistance;
int duty;
int error;
unsigned long left, right, pot;//ADC values

int main(void){
	PLL_Init();                           // 80 MHz
  ADC_Init298();         // ADC initialization PE1, PE4 
	SysTick_Init(4000000);        // initialize SysTick timer
	//Initializations
	PWM1A_Init(PERIOD);
  PWM1B_Init(PERIOD);
	PortB_Init();
	PortF_Init();
	
	direction = FORWARD; //Set forward direction
	GPIO_PORTF_DATA_R = 0x0A; //Yellow light
	EnableInterrupts();
  while(1){
		//Converts IR sensors values into CM
		Ldistance = ADCtoCM(left);
		Rdistance = ADCtoCM(right);
		//Converts potentiometer value into a duty cycle value
		duty = potSpeed(pot);
		//Starts robot when start button is pressed
		if(start){
			GPIO_PORTF_DATA_R = 0x02;
			//Stops robot when at end of track
			if(Ldistance >70 && Rdistance > 70){
				GPIO_PORTF_DATA_R = 0x06;
				PWM1A_Duty(0);
				PWM1B_Duty(0);
			}
			//Will perform steer function to control robot
			else
				steer();
			}
		//Will stop of stop button is pressed
		if(stop){
			GPIO_PORTF_DATA_R = 0x04;
			PWM1A_Duty(0);
			PWM1B_Duty(0);
		}
		WaitForInterrupt();
  }
}
//converts voltage from pot to a pwm duty cycle
int potSpeed(unsigned long value){
	int speed;
	double ratio = 3.9; // PWM duty cycle(16000) divided by Max ADC value(4095)
	speed = value * ratio;
	return speed;
}
//Function keeps robot car in middle as possible
//Computes error of both sensors
void steer(void){
	error = (Ldistance - (Rdistance-1));
	int turn;
	double increase = 1.33;//Multiplier I used
	turn = duty * increase;
	//Right turn
	//Checks if empty space on its right
	//Will also check to see if too close to left wall
	if(Rdistance >70 || Ldistance <10){
		GPIO_PORTF_DATA_R = 0x02;
		PWM1A_Duty(turn);
		PWM1B_Duty(0);
	}
	//Left turn
	//Checks if empty space on its Left
	//Will also check to see if too close to right wall
	else if(Ldistance > 70 || Rdistance < 10){
		GPIO_PORTF_DATA_R = 0x02;
		PWM1A_Duty(0);
		PWM1B_Duty(turn);
	}
	//close to left wall
	//decreases speed of right wheel by 50%
	else if(error < 0){
		GPIO_PORTF_DATA_R = 0x08;
		PWM1A_Duty(duty);  //remain the same
		PWM1B_Duty(duty/2); //decrease
	}
	//close to right wall
	//decreases speed of left wheel by 50%
	else if(error>0){
		GPIO_PORTF_DATA_R = 0x04;
		PWM1A_Duty(duty/2);//decrease
		PWM1B_Duty(duty);//remain the same
	}
	//Robot car is in center
	//Both wheels will have same speed
	else{
		GPIO_PORTF_DATA_R = 0x00;
		PWM1A_Duty(duty);
		PWM1B_Duty(duty);
	}
}

//Converts adc value from sensors to a value in centimeter
int ADCtoCM(unsigned long ADCvalue){
	int centimeters;
	float a = 43520.143;
	float b = 5.673;
	centimeters = (a/ADCvalue)-b;//inverse formula
	return centimeters;
}
// Initialize RGB LED pins and onboard buttons
void PortF_Init(void){unsigned long volatile delay;
  SYSCTL_RCGC2_R |= 0x20; // (a) activate clock for port F
  delay = SYSCTL_RCGC2_R;
	GPIO_PORTF_LOCK_R = 0x4C4F434B; // unlock GPIO Port F
	GPIO_PORTF_CR_R = 0x11;
	GPIO_PORTF_DIR_R &= ~0x11;    // (c) make PF4,0 in (built-in button)
	GPIO_PORTF_DIR_R |= 0x0E;
  GPIO_PORTF_AFSEL_R &= ~0x1F;  // disable alt funct on PF4,2,0
	GPIO_PORTF_AMSEL_R &= ~0x1F;  //     disable analog functionality on PF4,2,0
  GPIO_PORTF_DEN_R |= 0x1F;     //     enable digital I/O on PF4,2,0
  GPIO_PORTF_PCTL_R &= ~0x000FFFFF; //  configure PF4,2,0 as GPIO
	GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4,0
  GPIO_PORTF_IS_R &= ~0x11;     // (d) PF4,PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    //     PF4,PF0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    //     PF4,PF0 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      // (e) clear flags 4,0
  GPIO_PORTF_IM_R |= 0x11;      // (f) arm interrupt on PF4,PF0
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF1FFFFF)|0x00400000; // (g) bits:23-21 for PORTF, set priority to 2
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
}

//Push buttons to stop or start robot car
void GPIOPortF_Handler(void){
	if(GPIO_PORTF_RIS_R&0x01){//SW2 touched
		GPIO_PORTF_ICR_R = 0x01;  // acknowledge flag0
		stop = 1;
		start = 0;
	}
	if(GPIO_PORTF_RIS_R&0x10){//SW1 touch
		GPIO_PORTF_ICR_R = 0x10;  // acknowledge flag4
		start = 1;
		stop = 0;
	}
}
//Initialized for direction of motors
void PortB_Init(void){
	volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x00000002;
	delay = SYSCTL_RCGC2_R;				//delay
	GPIO_PORTB_AMSEL_R &=  0x0F;
	GPIO_PORTB_DIR_R   |=  0x0F;  //PB0,1,2,3 output
	GPIO_PORTB_AFSEL_R &= ~0x0F;  //Regular I/O
	GPIO_PORTB_PCTL_R  &= ~0x0F;  //GPIO
	GPIO_PORTB_DEN_R   |=  0x0F;
}
void ReadADCMedianFilter(unsigned long *ain2, unsigned long *ain9, unsigned long *ain8){
  //                   x(n-2)        x(n-1)
  static unsigned long ain2oldest=0, ain2middle=0;
  static unsigned long ain9oldest=0, ain9middle=0;
  static unsigned long ain8oldest=0, ain8middle=0;
  // save some memory; these do not need to be 'static'
  //            x(n)
  unsigned long ain2newest;
  unsigned long ain9newest;
  unsigned long ain8newest;
  ADC_In298(&ain2newest, &ain9newest, &ain8newest); // sample AIN2(PE1), AIN9 (PE4), AIN8 (PE5)
  *ain2 = median(ain2newest, ain2middle, ain2oldest);
  *ain9 = median(ain9newest, ain9middle, ain9oldest);
  *ain8 = median(ain8newest, ain8middle, ain8oldest);
  ain2oldest = ain2middle; ain9oldest = ain9middle; ain8oldest = ain8middle;
  ain2middle = ain2newest; ain9middle = ain9newest; ain8middle = ain8newest;
}

unsigned long median(unsigned long u1, unsigned long u2, unsigned long u3){
unsigned long result;
  if(u1>u2)
    if(u2>u3)   result=u2;     // u1>u2,u2>u3       u1>u2>u3
      else
        if(u1>u3) result=u3;   // u1>u2,u3>u2,u1>u3 u1>u3>u2
        else      result=u1;   // u1>u2,u3>u2,u3>u1 u3>u1>u2
  else
    if(u3>u2)   result=u2;     // u2>u1,u3>u2       u3>u2>u1
      else
        if(u1>u3) result=u1;   // u2>u1,u2>u3,u1>u3 u2>u1>u3
        else      result=u3;   // u2>u1,u2>u3,u3>u1 u2>u3>u1
  return(result);
}
//Systick initialization with interrupt
void SysTick_Init(unsigned long period){
  NVIC_ST_CTRL_R = 0;           // disable SysTick during setup
  NVIC_ST_RELOAD_R = period - 1; // reload value for 500us
  NVIC_ST_CURRENT_R = 0;        // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; // priority 2
  NVIC_ST_CTRL_R = 0x00000007;  // enable with core clock and interrupts
}
//Reads ADC values from IR sensors and potentiometer
void SysTick_Handler(void){
	ReadADCMedianFilter(&left,&right, &pot);
}

