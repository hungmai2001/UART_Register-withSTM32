
#include "main.h"
#include <stm32f103xb.h>
#define UART_BAUDRATE 115200
#define SystemCoreClock_def 8000000
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void UART1_Init(void);
void TIM2_delay(uint32_t delay_ms);

int main(void)
{
	uint8_t data;
	SystemClock_Config();
	MX_GPIO_Init();
	UART1_Init();
	while (1)
	{
//		//PC13 = 1
//		//GPIOC->ODR |= (1<<13)|(1<<14);
//		GPIOA->ODR |= (1<<11);
//	  	//delay 1s
//		TIM2_delay(1000);
//		//PC13 = 0;
//		//GPIOC->ODR &= ~((1<<13)|(1<<14));
//		GPIOA->ODR &= ~(1<<11);
//		//delay 1s
//		TIM2_delay(1000);

		GPIOA->ODR |= (1<<11);
	  	// Wait for data to be received
	 	 while (!(USART1->SR & USART_SR_RXNE));

	 	 // Read received data
	 	 data = USART1->DR;

	 	 //for(int i=0;i<1000000;i++);
		 // Send a response
		 USART1->DR = data;
	 	 while (!(USART1->SR & USART_SR_TXE));
	 	 GPIOA->ODR &= ~(1<<11);
	 	 TIM2_delay(1000);
	}
}

void SystemClock_Config(void)
{
	//HSE on
	RCC->CR |= RCC_CR_HSEON;
	//wait HSE complete
	while (!(RCC->CR & RCC_CR_HSERDY));
	// configure system clock - HSE
	RCC->CFGR |= RCC_CFGR_SW_HSE;
}

static void MX_GPIO_Init(void)
{
	// Clock for GPIO A, GPIO C
	RCC->APB2ENR |=RCC_APB2ENR_IOPAEN;
	RCC->APB2ENR |=(1<<4);
	//configure mode push-pull for PC13
//	GPIOC->CRH |= GPIO_CRH_MODE13_Msk;
//	GPIOC->CRH |=GPIO_CRH_MODE14_Msk;
//	GPIOC->CRH |= GPIO_CRH_MODE13_0;
	GPIOA->CRH |= GPIO_CRH_MODE11_1;
	GPIOA->CRH &= ~((1<<14)|(1<<15));
	//GPIOA->CRH &= ~(1<<11);
}
void UART1_Init(void)
{
	// Enable USART1 and GPIOA clock
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN;

	// Configure USART1 TX pin (PA9) as alternate function push-pull
	GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9); // Clear CNF and MODE bits for pin 9
	GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9; // Set CNF bits to alternate function push-pull, and MODE bits to 50 MHz

	// Configure USART1 RX pin (PA10) as input floating
	GPIOA->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10); // Clear CNF and MODE bits for pin 10
	GPIOA->CRH |= GPIO_CRH_CNF10_0; // Set CNF bits to input floating

	// Configure USART1 with desired settings
	USART1->BRR = SystemCoreClock_def / UART_BAUDRATE; // Set baud rate
	USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // Enable transmitter, receiver, and USART1
}

void TIM2_delay(uint32_t delay_ms)
{
    // Enable clock for TIM2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Set prescaler and auto-reload values
    TIM2->PSC = 7999; // Prescaler value for 1ms tick with 8MHz sysclk
    TIM2->ARR = delay_ms; // Auto-reload value for desired delay in ms

    // Enable the TIM2 and start the timer
    TIM2->CR1 |= TIM_CR1_CEN;

    // Wait for the timer to reach the desired value
    while(!(TIM2->SR & TIM_SR_UIF));

    // Clear the UIF bit and disable the TIM2
    TIM2->SR &= ~TIM_SR_UIF;
    TIM2->CR1 &= ~TIM_CR1_CEN;
}
