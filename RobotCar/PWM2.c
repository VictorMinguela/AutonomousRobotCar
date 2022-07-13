// PWM.c
#include <stdint.h>
#include "tm4c123gh6pm.h"
void PWM1A_Init(uint16_t period){
	volatile unsigned long delay;
	SYSCTL_RCGCPWM_R  |= 0x01;           // 1) activate M0
  SYSCTL_RCGCGPIO_R |= 0x02;           // 2) activate port B
	delay = SYSCTL_RCGCGPIO_R;
	//while((SYSCTL_PRGPIO_R&0x01) == 0){}
		
	GPIO_PORTB_AFSEL_R |=  0x40;         //PB6 alt funct	
	GPIO_PORTB_PCTL_R  &= ~0x0F000000;  
	GPIO_PORTB_PCTL_R	 |=  0x04000000;	 //PB6 as PWM0
	
	GPIO_PORTB_AMSEL_R &= ~0x40;         //disable analog PB6
	GPIO_PORTB_DEN_R   |=  0x40;				 //digital I/O PB6
	
	SYSCTL_RCC_R = 0x00100000 |          // 3) use PWM divider
      (SYSCTL_RCC_R & (~0x000E0000));  // configure for /2 divider	
			
	PWM0_0_CTL_R  = 0;									 //reload down-count mode									
	PWM0_0_GENA_R = 0xC8;                //low on LOAD, high on CMPA
		
	PWM0_0_LOAD_R  = period - 1;         
	PWM0_0_CMPA_R  = 0 - 1;	
	PWM0_0_CTL_R  |= 0x00000001;				 //start PWM0
	PWM0_ENABLE_R |= 0x00000001;         //enable PB6, M0PWM0
}
void PWM1A_Duty (uint16_t duty){
	PWM0_0_CMPA_R = duty-1;
}
void PWM1B_Init(uint16_t period){
	volatile unsigned long delay;
	SYSCTL_RCGCPWM_R  |= 0x01;
	SYSCTL_RCGCGPIO_R |= 0x02;
	delay = SYSCTL_RCGCGPIO_R;
	
	GPIO_PORTB_AFSEL_R |=  0x80;				 //PB7 alt funct
	GPIO_PORTB_PCTL_R  &= ~0xF0000000;   
	GPIO_PORTB_PCTL_R  |=  0x40000000;   //PB7 as PWM0
	GPIO_PORTB_AMSEL_R &= ~0x80;         //disable analog PB7
	GPIO_PORTB_DEN_R   |=  0x80;         //digital I/O PB7
	
	SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV; // 3) use PWM divider
  SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M; //    clear PWM divider field
  SYSCTL_RCC_R += SYSCTL_RCC_PWMDIV_2;  //    configure for /2 divider
	
	PWM0_0_CTL_R  = 0;                    //reload down-count mode	
	PWM0_0_GENB_R = 0xC08;                //low on LOAD, high on CMPA
	
	PWM0_0_LOAD_R  = period - 1;
	PWM0_0_CMPB_R  = 0 -1;
	PWM0_0_CTL_R  |= 0x00000001;				 	//start PWM0
	PWM0_ENABLE_R |= 0x00000002;         	//enable PB7, M0PWM1
}

void PWM1B_Duty (uint16_t duty){
	PWM0_0_CMPB_R = duty-1;
}