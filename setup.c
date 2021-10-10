/*
 * setup.c
 *
 *  Created on: 04.07.2021
 *      Author: symon
 */

#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "setup.h"
#include "flash.h"
#include "OSD.h"

static void setup_HSE();
static void setup_PLL();
static void setup_FPU();
static void setup_GPIOA(); // GPIOA (pin 2 - TIM2_CH3; pin 3 - TIM2_CH4; pin 4 - CS_SPI1; pin 5 - SCLK_SPI1; pin 6 - MISO_SPI1; pin 7 MOSI1_SPI1; pin 10 - RX USART1)
static void setup_GPIOB(); // GPIOB (pin 0 - TIM3_CH3; pin 1 - TIM3_CH4; pin 4 -  LED; pin 5 - blue LED; 10 - TX USART3)
static void setup_GPIOC(); // GPIOC (pin 4 - EXTI (INT MPU6000); pin 6 - TX USART6; pin 7 - RX USART6)
static void setup_TIM2(); 			// setup TIM2 motors PWM
static void setup_TIM3(); 			// setup TIM3 motors PWM
static void setup_TIM6(); 			// setup TIM6 global time
static void setup_TIM7(); 			// setup TIM7 delay functions
static void setup_PWM();			// if you use PWM for ESC
static void setup_Dshot();			// if you use Dshot for ESC
static void setup_USART1(); 		// USART for radioreceiver
static void setup_USART3();	// USART for communication via (3Dradio or bluetooth) - UNUSED
static void setup_USART6();	// USART for communication via (3Dradio or bluetooth)
static void setup_SPI1();			// SPI for communication with MPU6000
static void setup_DMA();
static void setup_EXTI();

extern uint8_t table_of_bytes_to_sent[2 * ALL_ELEMENTS_TO_SEND + 4];
extern uint8_t read_write_tab[];
extern volatile uint8_t rxBuf[];

void setup() {

	setup_HSE();
	setup_PLL();
	setup_FPU();
	setup_GPIOA();
	setup_GPIOB();
	setup_GPIOC();
	setup_TIM6();
	setup_TIM7();
	setup_TIM2();
	setup_TIM3();
	setup_PWM();
	//setup_Dshot();
	setup_USART1();
	setup_USART3();
	setup_USART6();
	setup_SPI1();
	setup_DMA();
	setup_EXTI();
	setup_FLASH();
	setup_OSD();
}

static void setup_HSE() {
	RCC->CFGR = (RCC->CFGR & (uint32_t) (~RCC_CFGR_SW)) | RCC_CFGR_SW_HSI; /* (2) */
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) {
		;			//wait
	}
	// enable HSE:
	RCC->CR |= RCC_CR_HSEON;
	while (0 == (RCC->CR & RCC_CR_HSERDY)) {
		// waiting until RDY bit is set
	}
	// enable power interface clock
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

}

static void setup_PLL() {

	/* (1) Test if PLL is used as System clock */
	/* (2) Select HSI as system clock */
	/* (3) Wait for HSI switched */
	/* (4) Disable the PLL */
	/* (5) Wait until PLLRDY is cleared */
	/* (6) Configure flash */
	/* (7) Set HSE as PLL source */
	/* (8) Set the PLLM to 4, PLLN to 168, PLLP to 2 */
	/* PLL_freq = PLL_clock_in / PLLM * PLLN / PLLP
	 * Important:
	 * 2 <= PLLM <= 64;
	 * 50 <= PLLN <=432;
	 * PLLP ={2,4,6,8}
	 * In addition:
	 * 	1  [MHz] <= PLL_clock_in / PLLM 		 	  <=  2  [MHz] but 2 [MHz] is preferred
	 * 100 [MHz] <= PLL_clock_in / PLLM * PLLN 		  <= 432 [MHz]
	 *  			PLL_clock_in / PLLM * PLLN / PLLP <= 168 [MHz]
	 * So as I want receive max. freq. 168 [MHz] PLLP has to be 2 (VCO_out 336 [MHz]), PLLN = 336 [MHz]/2 [MHz]= 168 and PLLM is 4 since 8 [MHz]/PLLM = 2 [MHz] */
	/* (9) Enable the PLL */
	/* (10) Wait until PLLRDY is set */
	/* (11) Select PLL as system clock */
	/* (12) Wait until the PLL is switched on */
	if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL) /* (1) */
	{
		RCC->CFGR = (RCC->CFGR & (uint32_t) (~RCC_CFGR_SW)) | RCC_CFGR_SW_HSI; /* (2) */
		while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) /* (3) */
		{
			/* For robust implementation, add here time-out management */
		}
	}
	RCC->CR &= (uint32_t) (~RCC_CR_PLLON);/* (4) */
	while ((RCC->CR & RCC_CR_PLLRDY) != 0) /* (5) */
	{
		/* For robust implementation, add here time-out management */
	}

	FLASH->ACR |= FLASH_ACR_LATENCY_5WS;/* (6) */

	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE; /* (7) */

	RCC->PLLCFGR = (RCC->PLLCFGR
			& (~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP)))
			| (RCC_PLLCFGR_PLLM_2 | RCC_PLLCFGR_PLLN_3 | RCC_PLLCFGR_PLLN_5
					| RCC_PLLCFGR_PLLN_7); /* (8) */
	RCC->CR |= RCC_CR_PLLON; /* (9) */
	while ((RCC->CR & RCC_CR_PLLRDY) == 0) /* (10) */
	{
		/* For robust implementation, add here time-out management */
	}
	RCC->CFGR |= (uint32_t) (RCC_CFGR_SW_PLL); /* (11) */
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) /* (12) */
	{
		/* For robust implementation, add here time-out management */
	}
}

static void setup_FPU() {
	SCB->CPACR |= (3UL << 10 * 2) | (3UL << 11 * 2);
}

static void setup_GPIOA() {
	// enable GPIOA clock:
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	//	set mode (00-input; 01-output; 10-alternate):
	GPIOA->MODER &= ~GPIO_MODER_MODER2;
	GPIOA->MODER |= GPIO_MODER_MODER2_1;

	GPIOA->MODER &= ~GPIO_MODER_MODER3;
	GPIOA->MODER |= GPIO_MODER_MODER3_1;

	GPIOA->MODER &= ~GPIO_MODER_MODER4;
	GPIOA->MODER |= GPIO_MODER_MODER4_0;

	GPIOA->MODER &= ~GPIO_MODER_MODER5;
	GPIOA->MODER |= GPIO_MODER_MODER5_1;

	GPIOA->MODER &= ~GPIO_MODER_MODER6;
	GPIOA->MODER |= GPIO_MODER_MODER6_1;

	GPIOA->MODER &= ~GPIO_MODER_MODER7;
	GPIOA->MODER |= GPIO_MODER_MODER7_1;

	GPIOA->MODER &= ~GPIO_MODER_MODER10;
	GPIOA->MODER |= GPIO_MODER_MODER10_1;

	//	set alternate functions:
	GPIOA->AFR[0] &= ~0xFFF0FF00;
	GPIOA->AFR[0] |= 0x55501100;
	GPIOA->AFR[1] &= ~0x00000F00;
	GPIOA->AFR[1] |= 0x00000700;

	//pull up (01) on PIN4 (SPI1 CS):
	//GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_0;

	//set high on PIN4 (SPI CS)
	GPIOA->BSRR |= GPIO_BSRR_BS4;

	GPIOA->OSPEEDR |= ( GPIO_OSPEEDER_OSPEEDR2_1 | GPIO_OSPEEDER_OSPEEDR2_0 |
	GPIO_OSPEEDER_OSPEEDR3_1 | GPIO_OSPEEDER_OSPEEDR3_0 |
	GPIO_OSPEEDER_OSPEEDR4_1 | GPIO_OSPEEDER_OSPEEDR4_0 |
	GPIO_OSPEEDER_OSPEEDR5_1 | GPIO_OSPEEDER_OSPEEDR5_0 |
	GPIO_OSPEEDER_OSPEEDR6_1 | GPIO_OSPEEDER_OSPEEDR6_0 |
	GPIO_OSPEEDER_OSPEEDR7_1 | GPIO_OSPEEDER_OSPEEDR7_0 |
	GPIO_OSPEEDER_OSPEEDR10_1 | GPIO_OSPEEDER_OSPEEDR10_0);
}

static void setup_GPIOB() {
	// enable GPIOB clock:
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	//	set mode (00-input; 01-output; 10-alternate):
	GPIOB->MODER &= ~GPIO_MODER_MODER0;
	GPIOB->MODER |= GPIO_MODER_MODER0_1;

	GPIOB->MODER &= ~GPIO_MODER_MODER1;
	GPIOB->MODER |= GPIO_MODER_MODER1_1;

	GPIOB->MODER &= ~GPIO_MODER_MODER3;
	GPIOB->MODER |= GPIO_MODER_MODER3_0;

	GPIOB->MODER &= ~GPIO_MODER_MODER4;
	GPIOB->MODER |= GPIO_MODER_MODER4_0;

	GPIOB->MODER &= ~GPIO_MODER_MODER5;
	GPIOB->MODER |= GPIO_MODER_MODER5_0;

	GPIOB->MODER &= ~GPIO_MODER_MODER10;
	GPIOB->MODER |= GPIO_MODER_MODER10_1;

	GPIOB->MODER &= ~GPIO_MODER_MODER12;
	GPIOB->MODER |= GPIO_MODER_MODER12_0;

	//set alternate functions:
	GPIOB->AFR[0] &= ~0x000000FF;
	GPIOB->AFR[0] |= 0x00000022;
	GPIOB->AFR[1] &= ~0x00000F00;
	GPIOB->AFR[1] |= 0x00000700;

	GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0_1 | GPIO_OSPEEDER_OSPEEDR0_0 |
	GPIO_OSPEEDER_OSPEEDR1_1 | GPIO_OSPEEDER_OSPEEDR1_0 |
	GPIO_OSPEEDER_OSPEEDR3_1 | GPIO_OSPEEDER_OSPEEDR3_0 |
	GPIO_OSPEEDER_OSPEEDR4_1 | GPIO_OSPEEDER_OSPEEDR4_0 |
	GPIO_OSPEEDER_OSPEEDR5_1 | GPIO_OSPEEDER_OSPEEDR5_0 |
	GPIO_OSPEEDER_OSPEEDR10_1 | GPIO_OSPEEDER_OSPEEDR10_0 |
	GPIO_OSPEEDER_OSPEEDR12_1 | GPIO_OSPEEDER_OSPEEDR12_0);
}

static void setup_GPIOC() {
	// enable GPIOC clock:
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

	//	set mode ( 00-input; 01-output; 10-alternate):

	GPIOC->MODER &= ~GPIO_MODER_MODER4;

	GPIOC->MODER &= ~GPIO_MODER_MODER6;
	GPIOC->MODER |= GPIO_MODER_MODER6_1;

	GPIOC->MODER &= ~GPIO_MODER_MODER7;
	GPIOC->MODER |= GPIO_MODER_MODER7_1;

	//	set alternate functions:
	GPIOC->AFR[0] &= ~0xFF000000;
	GPIOC->AFR[0] |= 0x88000000;

	// (00 no pull down, no pull up; 10 pull-down ):
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD4_0 | GPIO_PUPDR_PUPD4_1);
	GPIOC->PUPDR |= GPIO_PUPDR_PUPD4_1;

	//(11-max speed):
	GPIOC->OSPEEDR |= ( GPIO_OSPEEDER_OSPEEDR4_1 | GPIO_OSPEEDER_OSPEEDR4_0 |
	GPIO_OSPEEDER_OSPEEDR6_1 | GPIO_OSPEEDER_OSPEEDR6_0 |
	GPIO_OSPEEDER_OSPEEDR7_1 | GPIO_OSPEEDER_OSPEEDR7_0);

}

static void setup_TIM2() {
	// enable TIM2 clock:
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	// register is buffered and only overflow generate interrupt:
	TIM2->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;

	// PWM mode 1 and output compare 3 preload enable:
	TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;
	// PWM mode 1 and output compare 4 preload enable:
	TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;

	//channel 3 enable:
	TIM2->CCER |= TIM_CCER_CC3E;
	//channel 4 enable:
	TIM2->CCER |= TIM_CCER_CC4E;

}

static void setup_TIM3() {
	// enable TIM3 clock:
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	// register is buffered and only overflow generate interrupt:
	TIM3->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;

	// PWM mode 1 and output compare 3 preload enable:
	TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;
	// PWM mode 1 and output compare 4 preload enable:
	TIM3->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;

	//channel 3 enable:
	TIM3->CCER |= TIM_CCER_CC3E;
	//channel 4 enable:
	TIM3->CCER |= TIM_CCER_CC4E;

}

static void setup_TIM6() {
	// enable TIM6 clock:
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	// register is buffered and only overflow generate interrupt:
	TIM6->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;

	TIM6->PSC = 168 - 1; 			// every 1 us 1 count
	TIM6->ARR = 65535; 		// 1 period is 0.065535 [s] long

	//	interrupt enable
	TIM6->DIER |= TIM_DIER_UIE;
	//	TIM6 enabling:

	TIM6->EGR |= TIM_EGR_UG;
	TIM6->CR1 |= TIM_CR1_CEN;
}

static void setup_TIM7() {
	// enable TIM7 clock:
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

	// register is buffered, only overflow generate interrupt:
	TIM7->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;

	TIM7->PSC = 168 - 1; 			// every 1 us 1 count
	TIM7->ARR = 65535; 		// it will be set by user

	//	interrupt enable
	TIM7->DIER |= TIM_DIER_UIE;

	//	TIM7 is enabling in delay functions:

}

static void setup_PWM() {

	TIM2->PSC = 168 - 1; 		// counter count every microsecond
	TIM2->ARR = 20000; 		// 1 period of PWM is 20[ms]

	TIM2->CCR3 = 1000; 			//wypelneinie channel 3
	TIM2->CCR4 = 1000; 			//wypelneinie channel 4

	//	TIM2 enabling:
	TIM2->EGR |= TIM_EGR_UG;
	TIM2->CR1 |= TIM_CR1_CEN;

	TIM3->PSC = 168 - 1; 		// counter count every microsecond
	TIM3->ARR = 20000; 		// 1 period of PWM is 20[ms]

	TIM3->CCR3 = 1000; 			//wypelneinie channel 3
	TIM3->CCR4 = 1000; 			//wypelneinie channel 4

	//	TIM3 enabling:
	TIM3->EGR |= TIM_EGR_UG;
	TIM3->CR1 |= TIM_CR1_CEN;

}

static void setup_Dshot() {

	//	TIM2:

	TIM2->PSC = 168000 / DSHOT_MODE / DSHOT_PWM_FRAME_LENGTH - 1;
	TIM2->ARR = DSHOT_PWM_FRAME_LENGTH;

	TIM2->DIER|=TIM_DIER_CC1DE|TIM_DIER_CC2DE;

	TIM2->CCR1 = DSHOT_PWM_FRAME_LENGTH-1; //	DMA request is send right before PWM generation
	TIM2->CCR2 = DSHOT_PWM_FRAME_LENGTH-1; //	DMA request is send right before PWM generation
	TIM2->CCR3 = 0; 				//wypelneinie channel 3
	TIM2->CCR4 = 0; 				//wypelneinie channel 4

	//	TIM2 enabling:
	TIM2->EGR |= TIM_EGR_UG;
	TIM2->CR1 |= TIM_CR1_CEN;

	//	TIM3:

	TIM3->PSC = 168000 / DSHOT_MODE / DSHOT_PWM_FRAME_LENGTH - 1;
	TIM3->ARR = DSHOT_PWM_FRAME_LENGTH;

	TIM3->DIER|=TIM_DIER_CC1DE|TIM_DIER_CC2DE;

	TIM3->CCR1 = DSHOT_PWM_FRAME_LENGTH-1;	//	DMA request is send right before PWM generation
	TIM3->CCR2 = DSHOT_PWM_FRAME_LENGTH-1; 	//	DMA request is send right before PWM generation
	TIM3->CCR3 = 0; 						//	PWM duration channel 3
	TIM3->CCR4 = 0; 						//	PWM duration channel 4

	//	TIM3 enabling:
	TIM3->EGR |= TIM_EGR_UG;
	TIM3->CR1 |= TIM_CR1_CEN;
}

static void setup_USART1() {
	// enable USART1 clock:
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	USART1->BRR = 0x5B2; // 168 000 000 / 115 200/16 = 91,1458 so 91 is mantissa 0x5B and fraction 16*0.1458=~2 =0x2 so BRR is 0x5B2
	USART1->CR3 |= USART_CR3_DMAR; //uncomment for DMA reading
	USART1->CR1 |= USART_CR1_IDLEIE |
	USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;

}

static void setup_USART3() {
	// enable USART3 clock:
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	USART3->BRR = 0x5B2; // 168 000 000 / 115 200/16 = 91,1458 so 91 is mantissa 0x5B and fraction 16*0.1458=~2 =0x2 so BRR is 0x5B2
	USART3->CR3 |= USART_CR3_DMAT;
	USART3->CR1 |=
	USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_UE;
} //

static void setup_USART6() {
	// enable USART6 clock:
	RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
	USART6->BRR = 0x5B2; // 168 000 000 / 115 200 / 16 = 91,1458 so 91 is mantissa 0x5B and fraction 16*0.1458=~2 =0x2 so BRR is 0x5B2
	USART6->CR3 |= USART_CR3_DMAT;
	USART6->CR1 |= USART_CR1_RXNEIE |
	USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_UE;

} //

static void setup_SPI1() {
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	SPI1->CR1 |= SPI_CR1_BR; //APB2 is 168 [MHz] and max frequency of MPU6000 registers is 1 [MHz] (168/256<1 [MHz]) later (for sensors reading) it can be speed up to 20 [MHz] (168/16=10.5 [MHz])
	SPI1->CR1 |= SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_MSTR | SPI_CR1_CPOL
			| SPI_CR1_CPHA;
	//SPI1->CR2 |= SPI_CR2_SSOE | SPI_CR2_RXDMAEN;

}

static void setup_DMA() {

	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

//	//USART3 TX (from memory to peripheral):
//	DMA1_Stream3->CR = DMA_SxCR_CHSEL_2| DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_DIR_0
//			| DMA_SxCR_TCIE| DMA_SxCR_PL_0;
//	DMA1_Stream3->PAR = (uint32_t)&(USART3->DR);
//	DMA1_Stream3->M0AR = (uint32_t) (& MEMORY ADRESS FOR UASRT3);
//	DMA1_Stream3->NDTR = NUMBER OF ELEMENT TO SEND;

	// DSHOT:
	//motor1:
	DMA1_Stream6->CR |= DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1 | DMA_SxCR_MINC
			| DMA_SxCR_CIRC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE | DMA_SxCR_PL_0;
	DMA1_Stream6->PAR = (uint32_t) (&(TIM2->CCR4));
	DMA1_Stream6->M0AR = (uint32_t) (dshot_buffer_1);
	DMA1_Stream6->NDTR = DSHOT_BUFFER_LENGTH;
	//motor2
	DMA1_Stream7->CR |= DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_2 | DMA_SxCR_MINC
			| DMA_SxCR_CIRC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE | DMA_SxCR_PL_0;
	DMA1_Stream7->PAR = (uint32_t) (&(TIM2->CCR3));
	DMA1_Stream7->M0AR = (uint32_t) (dshot_buffer_2);
	DMA1_Stream7->NDTR = DSHOT_BUFFER_LENGTH;
	//motor3
	DMA1_Stream2->CR |= DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_2 | DMA_SxCR_MINC
			| DMA_SxCR_CIRC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE | DMA_SxCR_PL_0;
	DMA1_Stream2->PAR = (uint32_t) (&(TIM2->CCR4));
	DMA1_Stream2->M0AR = (uint32_t) (dshot_buffer_3);
	DMA1_Stream2->NDTR = DSHOT_BUFFER_LENGTH;
	//motor4
	DMA1_Stream1->CR |= DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1 | DMA_SxCR_MINC
			| DMA_SxCR_CIRC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE | DMA_SxCR_PL_0;
	DMA1_Stream1->PAR = (uint32_t) (&(TIM2->CCR3));
	DMA1_Stream1->M0AR = (uint32_t) (dshot_buffer_4);
	DMA1_Stream1->NDTR = DSHOT_BUFFER_LENGTH;

	//USART6 telemetry TX (from memory to peripheral):
	DMA2_Stream6->CR |= DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_2 | DMA_SxCR_MINC
			| DMA_SxCR_CIRC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE | DMA_SxCR_PL_0;
	DMA2_Stream6->PAR = (uint32_t) (&(USART6->DR));
	DMA2_Stream6->M0AR = (uint32_t) (table_of_bytes_to_sent);
	DMA2_Stream6->NDTR = 2 * ALL_ELEMENTS_TO_SEND + 4;

	//USART1 RX reading:
	DMA2_Stream5->CR |= DMA_SxCR_CHSEL_2 | DMA_SxCR_MINC | DMA_SxCR_CIRC
			| DMA_SxCR_TCIE | DMA_SxCR_PL_1;
	DMA2_Stream5->PAR = (uint32_t) (&(USART1->DR));
	DMA2_Stream5->M0AR = (uint32_t) (&rxBuf[1]);
	DMA2_Stream5->NDTR = 31;

	//IMU reading:
	DMA2_Stream0->CR |= DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0 | DMA_SxCR_MINC
			| DMA_SxCR_CIRC | DMA_SxCR_PL_1;
	DMA2_Stream0->PAR = (uint32_t) (&(SPI1->DR));
	DMA2_Stream0->M0AR = (uint32_t) (read_write_tab);
	DMA2_Stream0->NDTR = 14;

}

static void setup_EXTI() {

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	//set PC4 as EXTI
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PC;

	//disable interrupts mask for IM4 and IM15
	EXTI->IMR &= ~(EXTI_IMR_IM4 | EXTI_IMR_IM15);
	EXTI->IMR |= EXTI_IMR_IM4 | EXTI_IMR_IM15;

	//setting rising edge detection:
	EXTI->RTSR |= EXTI_RTSR_TR4;
}

void setup_NVIC_1() {

	//Only basics interrupts (Global_Time; delay_functions(); failsafe_functions();)

	// nvic interrupt enable (TIM6 interrupt);
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	NVIC_SetPriority(TIM6_DAC_IRQn, 6);

	// nvic interrupt enable (TIM7 interrupt);
	NVIC_EnableIRQ(TIM7_IRQn);
	NVIC_SetPriority(TIM7_IRQn, 7);

	// nvic EXTI interrupt enable:
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	NVIC_SetPriority(EXTI15_10_IRQn, 8);

}
void setup_NVIC_2() {
	// nvic interrupt enable (USART6 interrupt):
	NVIC_EnableIRQ(USART6_IRQn);
	NVIC_SetPriority(USART6_IRQn, 15);

	// nvic interrupt enable (USART3 interrupt):
	NVIC_EnableIRQ(USART3_IRQn);
	NVIC_SetPriority(USART3_IRQn, 14);

	// nvic interrupt enable (USART1 interrupt):
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn, 9);

	//nvic interrupt enable (EXTI interrupt)
	NVIC_EnableIRQ(EXTI4_IRQn);
	NVIC_SetPriority(EXTI4_IRQn, 10);

	// nvic DMA interrupt enable:

	NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	NVIC_SetPriority(DMA1_Stream1_IRQn, 13);

	NVIC_EnableIRQ(DMA1_Stream2_IRQn);
	NVIC_SetPriority(DMA1_Stream2_IRQn, 13);

	NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	NVIC_SetPriority(DMA1_Stream6_IRQn, 13);

	NVIC_EnableIRQ(DMA1_Stream7_IRQn);
	NVIC_SetPriority(DMA1_Stream7_IRQn, 13);

	NVIC_EnableIRQ(DMA2_Stream5_IRQn);
	NVIC_SetPriority(DMA2_Stream5_IRQn, 11);

	NVIC_EnableIRQ(DMA2_Stream6_IRQn);
	NVIC_SetPriority(DMA2_Stream6_IRQn, 12);

}

