#include "stm32l476xx.h"
#include <stdint.h>

void USART_Init(USART_TypeDef * USARTx){
	//Disable USART
	USARTx->CR1 &= ~USART_CR1_UE;
	//Set data length to 8 bits
	// 00=8 data bits, 01 = 9 data bits, 10 = 7 data bits
	USARTx->CR1 &= ~USART_CR1_M;
	//Select 1 stop bit
	// 00=1 stop bit		01=0.5 Stop bit
	// 10=2 stop bits 	11=1.5 Stop bit
	USARTx->CR2 &= ~USART_CR2_STOP;
	//Set parity control as no parity
	//0 = no parity,
	//1 = parity enabled (then, program PS bit to select Even or Odd parity)
	USARTx->CR1 &= ~USART_CR1_PCE;
	// Oversmapling by 16
	//0= oversamlpling by 16	1= by 8
	USARTx->CR1 &= ~USART_CR1_OVER8;
	//Set baud rate to 9600 using APB frequency (80Mhz: 0x208D - 4Mhz: 0x1A1) - baud rate: 1200 - D05
	USARTx->BRR = 0x1A1;
	//Enable transmission and reception
	USARTx->CR1 |= (USART_CR1_TE | USART_CR1_RE);
	//Enable Start
	USARTx->CR1 |= USART_CR1_UE;
	//Verify that USART is ready for transmission
	// TEAK: Transmit enable acknowledge flag. Harware sets or resets it.
	while((USARTx->ISR & USART_ISR_TEACK) ==0);
	//Verify that USART is ready for reception
	//REACK: receive enable acknowledge flag. Harware sets or resets it.
	while((USARTx->ISR & USART_ISR_REACK) ==0);

}

void ConfigGPIO_USART2(){
	//-----GPIO Initialization for USART2 ---------//
	// PD5 = AF7 (USART2_TX), PD6 = AF7 (USART2_RX)
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN; //Enable GPIO port D clock
	
	//00 = input, 01= output, 10=Alternate function, 11=analog
	GPIOD->MODER &= ~(0xF <<(2*5)); //Clear mode bits for pin5 and 6
	GPIOD->MODER |= 0xA << (2*5);		//Select Alternate Function mode
	//Alternate function 7 = USART2
	GPIOD->AFR[0] |= 0x77 << (4*5); // Set pin 5 and 6 to AF7
	//GPIO speed: 00 = low, 01 = medium, 10= fast, 11=high
	GPIOD->OSPEEDR |= 0xF<<(2*5);
	//GPIO push-pull: 00= No pull-up/pull-down, 01=pull-up, 10 = pull-down, 11= reserved
	GPIOD->PUPDR &= ~(0xF<<(2*5));
	GPIOD->PUPDR |= 0x5<<(2*5); //Select pull-up
	//GPIO output type: 0=push-pull, 1=open drain
	GPIOD->OTYPER &= ~(0x3<<5);
	//Enable USART2 clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
	//Select system clock (SYSCLK) USART clock source of UART 2
	// 00= PCLK, 01= System clock (SYSCLK), 10= HSI16, 11=LSE
	RCC->CCIPR &= ~(RCC_CCIPR_USART2SEL);
	RCC->CCIPR |= RCC_CCIPR_USART2SEL_0;

}

void ConfigGPIO_USART4(){
	//-----GPIO Initialization for USART2 ---------//
	// PA0 = AF8 (UART4_TX), PA1 = AF8 (UART4_RX)
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; //Enable GPIO port A clock
	
	//00 = input, 01= output, 10=Alternate function, 11=analog
	GPIOA->MODER &= ~(0xF <<(2*0)); //Clear mode bits for pin 0 and 1
	GPIOA->MODER |= 0xA << (2*0);		//Select Alternate Function mode
	//Alternate function 7 = USART2
	GPIOA->AFR[0] |= 0x88 << (4*0); // Set pin 0 and 1 to AF8
	//GPIO speed: 00 = low, 01 = medium, 10= fast, 11=high
	GPIOA->OSPEEDR |= 0xF<<(2*0);
	//GPIO push-pull: 00= No pull-up/pull-down, 01=pull-up, 10 = pull-down, 11= reserved
	GPIOA->PUPDR &= ~(0xF<<(2*0));
	GPIOA->PUPDR |= 0x5<<(2*0); //Select pull-up
	//GPIO output type: 0=push-pull, 1=open drain
	GPIOA->OTYPER &= ~(0x3<<0);
	//Enable UART4 clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_UART4EN;
	//Select system clock (SYSCLK) USART clock source of UART 2
	// 00= PCLK, 01= System clock (SYSCLK), 10= HSI16, 11=LSE
	RCC->CCIPR &= ~(RCC_CCIPR_UART4SEL);
	RCC->CCIPR |= RCC_CCIPR_UART4SEL_0;
}
void ConfigGPIO_USART1(){
	//-----GPIO Initialization for USART1 ---------//
	// PB6 = AF8 (USART1_TX), PB7 = AF7 (USART1_RX)
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN; //Enable GPIO port B clock
	
	//00 = input, 01= output, 10=Alternate function, 11=analog
	GPIOB->MODER &= ~(0xF <<(2*6)); //Clear mode bits for pin 6 and 7
	GPIOB->MODER |= 0xA << (2*6);		//Select Alternate Function mode
	//Alternate function 7 = USART2
	GPIOB->AFR[0] |= 0x77 << (4*6); // Set pin 0 and 1 to AF8
	//GPIO speed: 00 = low, 01 = medium, 10= fast, 11=high
	GPIOB->OSPEEDR |= 0xF<<(2*6);
	//GPIO push-pull: 00= No pull-up/pull-down, 01=pull-up, 10 = pull-down, 11= reserved
	GPIOB->PUPDR &= ~(0xF<<(2*6));
	GPIOB->PUPDR |= 0x5<<(2*6); //Select pull-up0
	//GPIO output type: 0=push-pull, 1=open drain
	GPIOB->OTYPER &= ~(0x3<<6);
	//Enable USART4 clock
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	//Select system clock (SYSCLK) USART clock source of UART 2
	// 00= PCLK, 01= System clock (SYSCLK), 10= HSI16, 11=LSE
	RCC->CCIPR &= ~(RCC_CCIPR_USART1SEL);
	RCC->CCIPR |= RCC_CCIPR_USART1SEL_0;

}

