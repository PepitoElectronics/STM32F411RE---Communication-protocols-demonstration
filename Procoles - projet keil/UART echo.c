#include "stm32f4xx.h"
void delay(int ms)
{
	int i;
	for(;ms>0;ms--)
		for(i=0;i<3195;i++);
}
int main (void)
{
	//ACtiver la clock
	RCC->AHB1ENR |= 1;
	RCC->AHB1ENR |= 4;
	//RCC->APB1ENR |= 20000;
	RCC->APB1ENR |= 0x00020000;
	//RCC->AHB1ENR |= (1<<0);
	// Config input/output
	GPIOA -> MODER |= 0X00000400;
	GPIOC -> MODER &=~ 0X0C000000;
	//alternate 5-4 bit 1-0
	GPIOA -> MODER |= 0xA0; 
	GPIOA -> MODER &=~ 0x50;
	//enable alternate
	GPIOA -> AFR[0] |= 0X7700;
	//config usart TX
	USART2->CR1 |= 0X2000;
	USART2->CR1 &=~ 0X1000;
	USART2->CR1 |= 0X8;
	USART2->CR2 &=~ 0X3000;
	USART2->CR3 &=~ 0X80;
	USART2->BRR |= 0X0683;
	//config usart RX
	USART2->CR1 |= 0X4;
	
	
	//send data
	USART2 -> DR = 'J';

	//mise à l état haut
	while(1)
	{
		/*if(USART2->SR & 0X40){
			USART2 -> DR = 0X41;
			delay(2000);
		}*/
		if(USART2->SR & 0X20){
			//USART2 -> SR &=~ 0X20;
			/*GPIOA -> ODR |= 0X00000020; // set PA5
			delay(2000);
			GPIOA -> ODR &=~ 0X00000020; // clear PA5
			delay(2000);*/
			while(USART2->SR & 0X40){
				char data;
				data = USART2 -> DR;
				USART2 -> DR = data;
				delay(1);
			}
		
		}
		/*if(GPIOC->IDR & 0X00002000)
			{
					GPIOA -> ODR &=~ 0X00000020; // clear PA5
				}
				else{
					GPIOA -> ODR |= 0X00000020; // set PA5
				
			
			//delay(1000);
			//GPIOA -> ODR &=~ 0X00000020; // clear PA5
			//delay(1000);
		}*/
	}
}

