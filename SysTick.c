#include "stm32l476xx.h"
#include <stdint.h>

//Input: ticks = number of tickes between two interrupts
void SysTick_Init(uint32_t ticks){
	//Set the clock
	//RCC->CFGR
	
	//Disable Systick IRQ and SysTick counter
	SysTick->CTRL = 0;
	
	//Set reload register
	SysTick->LOAD = ticks -1;
	
	//Set interrupt priority of SysTick
	//Make Systick least urgent (i.e., highest priority number)
	// __NVIC_PRIO_BITS: number of bits for prioruty levels, defined inCMSIS
	NVIC_SetPriority(SysTick_IRQn, (1<<__NVIC_PRIO_BITS)-1);
	
	//Reset the Systick counter value
	SysTick->VAL = 0;
	
	//Select processor clock
	// 1 - processor clock; 0 = external clock
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk; 
	
	//Enables SysTick exception request
	// 1= counting down to zero asserts the SysTick exception request
	// 0= counting down to zero does not assert the SysTick exception request
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	
	//Enable SysTick timer
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

