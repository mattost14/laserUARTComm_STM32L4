#include "stm32l476xx.h"
#include "LCD.h"
#include<stdio.h>
#include<string.h>
#include<math.h>
#include "SysTick.h"


#define BufferSize 32

uint8_t USART1_Buffer_Rx[BufferSize];	//	arrays to recieve and send data via UARTS
uint8_t USART2_Buffer_Rx[BufferSize];
uint8_t USART4_Buffer_Rx[BufferSize];

volatile uint32_t Rx1_Counter = 0;	//	recieve and send counters for UART 1
volatile uint32_t Tx1_Counter = 0;

volatile uint32_t Rx2_Counter = 0;	//	recieve and send counters for UART 2
volatile uint32_t Tx2_Counter = 0;

volatile uint32_t Rx4_Counter = 0;	//	recieve and send counters for UART 4
volatile uint32_t Tx4_Counter = 0;

volatile uint32_t TimeDelay;	//	variables for SysTick
volatile int tick;


void Delay(uint32_t nTime){
	//nTime: specifies the delay time length
	TimeDelay = nTime;
	while(TimeDelay != 0); //Busy wait
}



void SysTick_Handler(void){
	tick++;
	
	if(TimeDelay >0)
			TimeDelay--;
}


void USART_Init (USART_TypeDef * USARTx){
	
	//	Disable USART
	USARTx->CR1 &= ~USART_CR1_UE;
	
	//	Set data length to 8 bits
	//	00 = 8 data bits,	01 = 9 data bits,	10 = 7 data bits
	USARTx->CR1 &= ~USART_CR1_M;
	
	//	Select 1 stop bit
	//	00 = 1 stop bit		01 = 0.5 stop bits
	//	10 = 2 stop bits	11 = 1.5 stop bits
	USARTx->CR2 &= ~USART_CR2_STOP;
	
	//	Set parity controll as no parity
	//	0 = no parity,
	//	1 = parity enabled (then, program PS bit to select Even or Odd parity)
	USARTx->CR1 &= ~USART_CR1_PCE;
	
	//	Oversampling by 16
	//	0 = oversampling by 16, 1 = oversambling by 8
	USARTx->CR1 &= ~USART_CR1_OVER8;
	
	//	Set Baud rate to 9600 using APB frequency (80 MHz)
	//	See Example 1 in Section 22.1.2
	USARTx->BRR = 0x23;//0x1A1;
	
	//	Turn on the interrupts for recieve and transmit
	USARTx->CR1 |= USART_CR1_RXNEIE;	//	recieve register not empty interrupt
	USARTx->CR1 &= ~USART_CR1_TXEIE;	//	transmit register empty interrupt
	
//	//	Set overrun disable bit in CR3
//	USARTx->CR3 |= USART_CR3_OVRDIS;
	
	//	Enabble transmission and reception
	USARTx->CR1 |= (USART_CR1_TE | USART_CR1_RE);
	
	//	Enable USART
	USARTx->CR1 |= USART_CR1_UE;
	
	//	Verify that USART is ready for transmission
	//	TEACK: Transmit enable acknowledge flag. Hardware sets or resets it. 
	while ((USARTx->ISR & USART_ISR_TEACK) == 0);
	
	//	Verify that USART is ready for reception
	//	REACK: Recieve enable acknowledge flag. Hardware sets or resets it. 
	while ((USARTx->ISR & USART_ISR_REACK) == 0);
}

void USART1_Init(void){
	
	//	Enable GPIO clock and configure the Tx pin and Rx pin as: 
	//	Alternate function, High Speed, Push-Pull, Pull-up
	
	//--------------- GPIO Initialization for USART 1 -------------------
	//	PB.6 = AF7 (USART1_TX), PB.7 = AF7 (USART1_RX), See Appendix I
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;	//	Enable GPIO prot B clock
	
	//	00 = Input,	01 = Output, 10 = Alternate Fucntion, 11 = Analog
	GPIOB->MODER &= ~(0xF << (2*6));	//	Clear mode bits for pin 6 and 7
	GPIOB->MODER |= 0xA << (2*6);			//	Select Alternate Function Mode
	
	//	Alternative function 7 = USART 1
	//	Appendix I shows all alternate functions
	GPIOB->AFR[0] |= 0x77 << (4*6);		//	Set pin 6 and 7 to AF 7 (0111)
	
	//	GPIO Speed: 00 = Low speed,		01 = Medium Speed
	//							10 = Fast Speed, 	11 = High Speed
	GPIOB->OSPEEDR |= 0xF << (2*6);		//	Set pin 6 and 7 to High Speed
	
	//	GPIO Push-Pull:	00 = No pull-up/pull-down,	01 = Pull-up
	//									10 = Pull-Down,							11 = Reserved
	GPIOB->PUPDR &= ~(0xF << (2*6));	//	Clear PUPDR bits for 6 and 7
	GPIOB->PUPDR |= 0x5 << (2*6);			//	Select Pull-Up for 6 and 7
	
	//	GPIO Output Type: 0 = push-pull,	1 = open drain
	GPIOB->OTYPER &=	~(0x3 << 6);	//	Set bit 6 to push-pull
}

void USART2_Init(void){
	
	//	Enable GPIO clock and configure the Tx pin and Rx pin as: 
	//	Alternate function, High Speed, Push-Pull, Pull-up
	
	//--------------- GPIO Initialization for USART 2 -------------------
	//	PD.5 = AF7 (USART2_TX), PD.6 = AF7 (USART2_RX), See Appendix I
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;	//	Enable GPIO prot D clock
	
	//	00 = Input,	01 = Output, 10 = Alternate Fucntion, 11 = Analog
	GPIOD->MODER &= ~(0xF << (2*5));	//	Clear mode bits for pin 5 and 6
	GPIOD->MODER |= 0xA << (2*5);			//	Select Alternate Function Mode
	
	//	Alternative function 7 = USART 2
	//	Appendix I shows all alternate functions
	GPIOD->AFR[0] |= 0x77 << (4*5);		//	Set pin 5 and 6 to AF 7 (0111)
	
	//	GPIO Speed: 00 = Low speed,		01 = Medium Speed
	//							10 = Fast Speed, 	11 = High Speed
	GPIOD->OSPEEDR |= 0xF << (2*5);		//	Set pin 5 and 6 to High Speed
	
	//	GPIO Push-Pull:	00 = No pull-up/pull-down,	01 = Pull-up
	//									10 = Pull-Down,							11 = Reserved
	GPIOD->PUPDR &= ~(0xF << (2*5));	//	Clear PUPDR bits for 5 and 6
	GPIOD->PUPDR |= 0x5 << (2*5);			//	Select Pull-Up for 5 and 6
	
	//	GPIO Output Type: 0 = push-pull,	1 = open drain
	GPIOD->OTYPER &=	~(0x3 << 5);	//	Set bit 5 to push-pull
}

void USART4_Init(void){
	
	//	Enable GPIO clock and configure the Tx pin and Rx pin as: 
	//	Alternate function, High Speed, Push-Pull, Pull-up
	
	//--------------- GPIO Initialization for USART 1 -------------------
	//	PA.0 = AF8 (USART4_TX), PA.1 = AF8 (USART4_RX), See Appendix I
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;	//	Enable GPIO port A clock
	
	//	00 = Input,	01 = Output, 10 = Alternate Fucntion, 11 = Analog
	GPIOA->MODER &= ~(0xF);	//	Clear mode bits for pin 0 and 1
	GPIOA->MODER |= 0xA;			//	Select Alternate Function Mode
	
	//	Alternative function 8 = USART 4
	//	Appendix I shows all alternate functions
	GPIOA->AFR[0] |= 0x88;		//	Set pin 0 and 1 to AF 7 (1000)
	
	//	GPIO Speed: 00 = Low speed,		01 = Medium Speed
	//							10 = Fast Speed, 	11 = High Speed
	GPIOA->OSPEEDR |= 0xF;		//	Set pin 0 and 1 to High Speed
	
	//	GPIO Push-Pull:	00 = No pull-up/pull-down,	01 = Pull-up
	//									10 = Pull-Down,							11 = Reserved
	GPIOA->PUPDR &= ~(0xF);	//	Clear PUPDR bits for 0 and 1
	GPIOA->PUPDR |= 0x5;			//	Select Pull-Up for 0 and 1
	
	//	GPIO Output Type: 0 = push-pull,	1 = open drain
	GPIOA->OTYPER &=	~(0x3);	//	Set bit 0 to push-pull
}



void USART_Clock_Init(void){
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;		//	Enable UART 1 clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;	//	Enable UART 2 clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_UART4EN;	//	Enable UART 4 clock
	
	//	Select system clock (SYSCLK) USART clock source of UART 1, 2 and 4
	//	00 = PCLK,	01 = System Clock (SYSCLK)
	//	10 = HSI16,	11 = LSE
	RCC->CCIPR &= 0xFFFFFF30;		//	Clear bits for UART 1, 2, and 4
	RCC->CCIPR |= 0x45;					//	Set UART 1, 2, and 4 to SYSCLK
}


void recieve(USART_TypeDef *USARTx, uint8_t *buffer, uint32_t *pCounter){
	if(USARTx->ISR & USART_ISR_RXNE){		//	Check RXNE event
		buffer[*pCounter] = USARTx->RDR;	//	Reading RDR clears the RXNE flag
		(*pCounter)++;										//	Dereference and update memory value
		if((*pCounter) >= BufferSize){		//	Check buffer overflow
			(*pCounter) = 0;								//	Circular buffer
		}
	}
}


void UART_Send(USART_TypeDef *USARTx, uint8_t *buffer){
	USARTx->CR1 |= USART_CR1_TXEIE;	//	Enable TXE Interrupt
	//	Write to Transmit Data Register (TDR) to start transmission
	//	An interrupt will be initiated after data in TDR has been sent
	USARTx->TDR = buffer[0];
}


void send(USART_TypeDef *USARTx, uint8_t *buffer, uint32_t *pCounter){
	if(USARTx->ISR & USART_ISR_TXE){								//	Check TXE flag
		
		USARTx->TDR = buffer[*pCounter] & 0xFF;		//	writing to TDR clears TXE
		(*pCounter)++;										//	Dereference and update memory value
		if((*pCounter) >= BufferSize){		//	Check buffer overflow
			(*pCounter) = 0;								//	Circular buffer
		}
//	_______________________Below is the Useless code from the book______________________________
//		if(*pCounter <= BufferSize - 1){						//	Transmit the next byte
//			USARTx->TDR = buffer[*pCounter] & 0xFF;		//	writing to TDR clears TXE
//			(*pCounter)++;															//	Bytes that have been sent
//		}
//		else{																				//	Transmission completes
//			(*pCounter) = 0;													//	Clear the counter
//			USARTx->CR1 &= ~USART_CR1_TXEIE;					//	Disable TXE interrupt
//		}
//	____________________________________________________________________________________________
	}
}


void USART1_IRQHandler(void){
	recieve(USART1, USART1_Buffer_Rx, &Rx1_Counter);
	send(USART2, USART1_Buffer_Rx, &Tx2_Counter);
}


void USART2_IRQHandler(void){	
	recieve(USART2, USART2_Buffer_Rx, &Rx2_Counter);
	send(UART4, USART2_Buffer_Rx, &Tx4_Counter);		//	sending data through uart 1 - recieved through uart 4
	//send(USART2, USART2_Buffer_Rx, &Tx1_Counter);
}


//void UART4_IRQHandler(void){
//		recieve(UART4, USART4_Buffer_Rx, &Rx4_Counter);
//		//send(UART4, USART4_Buffer_Rx, &Tx4_Counter);
//		send(USART2, USART2_Buffer_Rx, &Tx1_Counter);	//	sending data back to computer through uart 2
//}


int main(void){	
	SysTick_Init(200);
	
	LCD_Initialization();
	
	USART1_Init();
	USART2_Init();
	USART4_Init();
	
	USART_Clock_Init();
	
	USART_Init(USART2);
	USART_Init(USART1);
	USART_Init(UART4);
	
	NVIC_SetPriority(USART1_IRQn, 0);	//	Set the highest urgency
	NVIC_EnableIRQ(USART1_IRQn);			//	Enable NVIC interrupt
	
	NVIC_SetPriority(USART2_IRQn, 0);	//	Set the highest urgency
	NVIC_EnableIRQ(USART2_IRQn);			//	Enable NVIC interrupt
	
	NVIC_SetPriority(UART4_IRQn, 0);	//	Set the highest urgency
	NVIC_EnableIRQ(UART4_IRQn);			//	Enable NVIC interrupt
	
	LCD_DisplayString("------", 0);	
	
	while(1){
		//memcpy(arraycopy, originalarray, sizeof(arraycopy)); //	useful function for copying arrays (for reference) 
		
		LCD_DisplayString(USART2_Buffer_Rx, 0); //	display data coming in from UART 2
	}
}