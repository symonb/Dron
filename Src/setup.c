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
#include "OSD.h"
#include "adc1.h"
#include "motors.h"
#include "filters.h"
#include "setup.h"
#include "rx.h"

 /* IMPORTANT:
  APB2 max frequency is 84 [MHz], 168 [MHz] only for timers
  APB1 max frequency is 42 [MHz], 84 [MHz] only for timers
  */

static void setup_HSE();
static void setup_PLL(); // clock setting
static void setup_FPU();
static void setup_GPIOA();		 // GPIOA (pin 2 - TIM2_CH3; pin 3 - TIM2_CH4; pin 4 - CS_SPI1; pin 5 - SCLK_SPI1; pin 6 - MISO_SPI1; pin 7 MOSI1_SPI1; pin 8 - BUZZER; pin 10 - RX USART1; pin 11 - D- OTG_USB_FS; pin 12 - D+ OTG_USB_FS  )
static void setup_GPIOB();		 // GPIOB (pin 0 - TIM3_CH3; pin 1 - TIM3_CH4; pin 3 - CS_SPI3; pin 4 -  LED; pin 5 - blue LED; pin 6 - I2C1_SCL; pin 7 - I2C1_SDA; pin 10 - TX USART3, pin 12 - CS_SPI2, pin 13 - SCLK_SPI2, pin 14 - MISO_SPI2, pin 15 - MOSI_SPI2)
static void setup_GPIOC();		 // GPIOC (pin 0 - invert RX; pin 1 - battery voltage (ADC123_IN11) pin 4 - EXTI (INT MPU6000); pin 5 - USB detection; pin 6 - TX USART6; pin 7 - RX USART6; pin 10 - SCLK_SPI3; pin 11 - MISO_SPI3; pin 12 - MOSI_SPI3)
static void setup_TIM5();		 // setup TIM5 global time and delay functions
#if defined(ESC_PROTOCOL_PWM)	
static void setup_PWM();		 // if you use PWM for ESC
#elif defined(ESC_PROTOCOL_BDSHOT)
static void setup_BDshot();		 //	if you use Bidirectional DShot for ESC
#elif defined(ESC_PROTOCOL_DSHOT)
static void setup_Dshot();		 // if you use Dshot for ESC
#elif defined(ESC_PROTOCOL_DSHOT_BURST)
static void setup_Dshot_burst(); // if Dshot is capable of burst transfer
#elif defined(ESC_PROTOCOL_ONESHOT125)
static void setup_OneShot125();	 // if you use OneShot125 for ESC
#endif
static void setup_USART1();		 // USART for radioreceiver
static void setup_USART3();		 // USART for communication via (3Dradio or bluetooth) - UNUSED
static void setup_USART6();		 // USART for communication via (3Dradio or bluetooth)
static void setup_SPI1();		 // SPI for communication with MPU6000
static void setup_SPI2();		 //	SPI for OSD chip
static void setup_SPI3();		 // SPI for FLASH
static void setup_ADC1();
static void setup_DMA();
static void setup_EXTI();
static void setup_I2C1();

extern uint8_t table_of_bytes_to_sent[2 * ALL_ELEMENTS_TO_SEND + 4];
extern uint8_t read_write_tab[];
extern volatile uint8_t rxBuf[];

void setup()
{
	setup_HSE();
	setup_PLL();
	setup_FPU();
	setup_GPIOA();
	setup_GPIOB();
	setup_GPIOC();
	setup_TIM5();
#if defined(ESC_PROTOCOL_PWM)
	setup_PWM();
#elif defined(ESC_PROTOCOL_BDSHOT)
	setup_BDshot();
	preset_bb_Dshot_buffers();
#elif defined(ESC_PROTOCOL_DSHOT)
	setup_Dshot();
#elif defined(ESC_PROTOCOL_DSHOT_BURST)
	// setup_Dshot_burst();		// NIESKONCZONE _ NIE DZIALA
#elif defined(ESC_PROTOCOL_ONESHOT125)
	setup_OneShot125();
#endif
	setup_USART1();
	setup_USART3();
	setup_USART6();
	setup_SPI1();
	setup_SPI2();
	setup_SPI3();
	setup_ADC1();
	setup_EXTI();
	setup_I2C1();
	setup_DMA();
	setup_D_term_filters();
	setup_RX();
}

static void setup_HSE()
{
	RCC->CFGR = (RCC->CFGR & (uint32_t)(~RCC_CFGR_SW)) | RCC_CFGR_SW_HSI; /* (2) */
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
	{
		; // wait
	}
	// enable HSE:
	RCC->CR |= RCC_CR_HSEON;
	while (0 == (RCC->CR & RCC_CR_HSERDY))
	{
		// waiting until RDY bit is set
	}
	// set all prescalers (main clock will be set at 168 MHz via PLL):

	RCC->CFGR |= RCC_CFGR_PPRE2_2;					  // APB2 presc. = 2
	RCC->CFGR |= RCC_CFGR_PPRE1_2 | RCC_CFGR_PPRE1_0; // APB1 presc. = 4
	// AHB presc. = 1 (left at default)

	// enable power interface clock
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
}

static void setup_PLL()
{

	/* (1) Test if PLL is used as System clock */
	/* (2) Select HSI as system clock */
	/* (3) Wait for HSI switched */
	/* (4) Disable the PLL */
	/* (5) Wait until PLLRDY is cleared */
	/* (6) Configure flash */
	/* (7) Set HSE as PLL source */
	/* (8) Set the PLLM to 4, PLLN to 168, PLLP to 2, PLLQ to 7 */
	/* PLL_freq = PLL_clock_in / PLLM * PLLN / PLLP
	 * PLL_48 =PLL_clock_in / PLLM * PLLN / PLLQ = 48 [MHz]
	 * Important:
	 * 2 <= PLLM <= 64;
	 * 50 <= PLLN <= 432;
	 * PLLP = {2,4,6,8}
	 * 2 <= PLLQ <= 15
	 * In addition:
	 * 	1  [MHz] <= PLL_clock_in / PLLM 		 	  <=  2  [MHz] but 2 [MHz] is preferred
	 * 100 [MHz] <= PLL_clock_in / PLLM * PLLN 		  <= 432 [MHz]
	 *  			PLL_clock_in / PLLM * PLLN / PLLP <= 168 [MHz]
	 * So as I want receive max. freq. 168 [MHz] PLLP has to be 2 (VCO_out 336 [MHz]), PLLN = 168 and PLLM is 4 since 8 [MHz]/PLLM = 2 [MHz]
	 * Also PLL_48 has to be 48 [MHz] so PLLQ = 7 -> 336 [MHz] / 48 [MHz] = 7
	 * */
	 /* (9) Enable the PLL */
	 /* (10) Wait until PLLRDY is set */
	 /* (11) Select PLL as system clock */
	 /* (12) Wait until the PLL is switched on */
	if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL) /* (1) */
	{
		RCC->CFGR = (RCC->CFGR & (uint32_t)(~RCC_CFGR_SW)) | RCC_CFGR_SW_HSI; /* (2) */
		while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)				  /* (3) */
		{
			/* For robust implementation, add here time-out management */
		}
	}
	RCC->CR &= (uint32_t)(~RCC_CR_PLLON);  /* (4) */
	while ((RCC->CR & RCC_CR_PLLRDY) != 0) /* (5) */
	{
		/* For robust implementation, add here time-out management */
	}

	FLASH->ACR |= FLASH_ACR_LATENCY_5WS; /* (6) */

	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE; /* (7) */

	RCC->PLLCFGR = (RCC->PLLCFGR & (~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLQ))) | (RCC_PLLCFGR_PLLM_2 | RCC_PLLCFGR_PLLN_3 | RCC_PLLCFGR_PLLN_5 | RCC_PLLCFGR_PLLN_7 | RCC_PLLCFGR_PLLQ_0 | RCC_PLLCFGR_PLLQ_1 | RCC_PLLCFGR_PLLQ_2); /* (8) */

	RCC->CR |= RCC_CR_PLLON;			   /* (9) */
	while ((RCC->CR & RCC_CR_PLLRDY) == 0) /* (10) */
	{
		/* For robust implementation, add here time-out management */
	}
	RCC->CFGR |= (uint32_t)(RCC_CFGR_SW_PLL);			   /* (11) */
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) /* (12) */
	{
		/* For robust implementation, add here time-out management */
	}
}

static void setup_FPU()
{
	SCB->CPACR |= (3UL << 10 * 2) | (3UL << 11 * 2);
}

static void setup_GPIOA()
{
	// enable GPIOA clock:
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	//	set mode (00-input; 01-output; 10-alternate):
#if !defined(ESC_PROTOCOL_BDSHOT)
	GPIOA->MODER &= ~GPIO_MODER_MODER2;
	GPIOA->MODER |= GPIO_MODER_MODER2_1;

	GPIOA->MODER &= ~GPIO_MODER_MODER3;
	GPIOA->MODER |= GPIO_MODER_MODER3_1;
#endif

	GPIOA->MODER &= ~GPIO_MODER_MODER4;
	GPIOA->MODER |= GPIO_MODER_MODER4_0;

	GPIOA->MODER &= ~GPIO_MODER_MODER5;
	GPIOA->MODER |= GPIO_MODER_MODER5_1;

	GPIOA->MODER &= ~GPIO_MODER_MODER6;
	GPIOA->MODER |= GPIO_MODER_MODER6_1;

	GPIOA->MODER &= ~GPIO_MODER_MODER7;
	GPIOA->MODER |= GPIO_MODER_MODER7_1;

	GPIOA->MODER &= ~GPIO_MODER_MODER8;
	GPIOA->MODER |= GPIO_MODER_MODER8_0;

	GPIOA->MODER &= ~GPIO_MODER_MODER10;
	GPIOA->MODER |= GPIO_MODER_MODER10_1;

	GPIOA->MODER &= ~GPIO_MODER_MODER11;
	GPIOA->MODER |= GPIO_MODER_MODER11_1;

	GPIOA->MODER &= ~GPIO_MODER_MODER12;
	GPIOA->MODER |= GPIO_MODER_MODER12_1;

	//	set alternate functions:
	GPIOA->AFR[0] &= ~0xFFF0FF00;
	GPIOA->AFR[0] |= 0x55501100;
	GPIOA->AFR[1] &= ~0x000FFF00;
	GPIOA->AFR[1] |= 0x00077700;

	// pull up (01) pull down (10):

	// set high on PIN4 (SPI CS):
	GPIOA->BSRR |= GPIO_BSRR_BS_4;

	//	output type (0 - push-pull, 1 - open-drain):

	// set speed (11 - max speed):
	GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR2 |
		GPIO_OSPEEDER_OSPEEDR3 |
		GPIO_OSPEEDER_OSPEEDR4 |
		GPIO_OSPEEDER_OSPEEDR5 |
		GPIO_OSPEEDER_OSPEEDR6 |
		GPIO_OSPEEDER_OSPEEDR7 |
		GPIO_OSPEEDER_OSPEEDR8 |
		GPIO_OSPEEDER_OSPEEDR10 |
		GPIO_OSPEEDER_OSPEEDR11 |
		GPIO_OSPEEDER_OSPEEDR12);
}

static void setup_GPIOB()
{
	// enable GPIOB clock:
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	//	set mode (00-input; 01-output; 10-alternate):
#if !defined(ESC_PROTOCOL_BDSHOT)
	GPIOB->MODER &= ~GPIO_MODER_MODER0;
	GPIOB->MODER |= GPIO_MODER_MODER0_1;

	GPIOB->MODER &= ~GPIO_MODER_MODER1;
	GPIOB->MODER |= GPIO_MODER_MODER1_1;
#endif

	GPIOB->MODER &= ~GPIO_MODER_MODER3;
	GPIOB->MODER |= GPIO_MODER_MODER3_0;

	GPIOB->MODER &= ~GPIO_MODER_MODER4;
	GPIOB->MODER |= GPIO_MODER_MODER4_0;

	GPIOB->MODER &= ~GPIO_MODER_MODER5;
	GPIOB->MODER |= GPIO_MODER_MODER5_0;

	GPIOB->MODER &= ~GPIO_MODER_MODER6;
	GPIOB->MODER |= GPIO_MODER_MODER6_1;

	GPIOB->MODER &= ~GPIO_MODER_MODER7;
	GPIOB->MODER |= GPIO_MODER_MODER7_1;

	GPIOB->MODER &= ~GPIO_MODER_MODER10;
	GPIOB->MODER |= GPIO_MODER_MODER10_1;

	GPIOB->MODER &= ~GPIO_MODER_MODER12;
	GPIOB->MODER |= GPIO_MODER_MODER12_0;

	GPIOB->MODER &= ~GPIO_MODER_MODER13;
	GPIOB->MODER |= GPIO_MODER_MODER13_1;

	GPIOB->MODER &= ~GPIO_MODER_MODER14;
	GPIOB->MODER |= GPIO_MODER_MODER14_1;

	GPIOB->MODER &= ~GPIO_MODER_MODER15;
	GPIOB->MODER |= GPIO_MODER_MODER15_1;

	// set alternate functions:
	GPIOB->AFR[0] &= ~0xFF0000FF;
	GPIOB->AFR[0] |= 0x44000022;
	GPIOB->AFR[1] &= ~0xFFF00F00;
	GPIOB->AFR[1] |= 0x55500700;

	// set high on PIN3 (SPI3 CS) and PIN12 (SPI2 CS) :
	GPIOB->BSRR |= GPIO_BSRR_BS_3;
	GPIOB->BSRR |= GPIO_BSRR_BS_12;

	// (00 no pull down, no pull up; 01 pull-up; 10 pull-down ):
	// pull-up SDA and SCL lines 
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR6);
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0;
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR7);
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR7_0;

	//	output type (0 - push-pull, 1 - open-drain)
	//	open-drain for SDA and SCL:
	GPIOB->OTYPER |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7;

	// set speed:
	GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0 |
		GPIO_OSPEEDER_OSPEEDR1 |
		GPIO_OSPEEDER_OSPEEDR3 |
		GPIO_OSPEEDER_OSPEEDR4 |
		GPIO_OSPEEDER_OSPEEDR5 |
		GPIO_OSPEEDER_OSPEEDR10 |
		GPIO_OSPEEDER_OSPEEDR12 |
		GPIO_OSPEEDER_OSPEEDR13 |
		GPIO_OSPEEDER_OSPEEDR14 |
		GPIO_OSPEEDER_OSPEEDR15);
}

static void setup_GPIOC()
{
	// enable GPIOC clock:
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

	//	set mode ( 00-input; 01-output; 10-alternate; 11-analog):

	GPIOC->MODER &= ~GPIO_MODER_MODER0;
	GPIOC->MODER |= GPIO_MODER_MODER0_0;

	GPIOC->MODER &= ~GPIO_MODER_MODER1;
	GPIOC->MODER |= GPIO_MODER_MODER1;

	GPIOC->MODER &= ~GPIO_MODER_MODER4;

	GPIOC->MODER &= ~GPIO_MODER_MODER5;

	GPIOC->MODER &= ~GPIO_MODER_MODER6;
	GPIOC->MODER |= GPIO_MODER_MODER6_1;

	GPIOC->MODER &= ~GPIO_MODER_MODER7;
	GPIOC->MODER |= GPIO_MODER_MODER7_1;

	GPIOC->MODER &= ~GPIO_MODER_MODER10;
	GPIOC->MODER |= GPIO_MODER_MODER10_1;

	GPIOC->MODER &= ~GPIO_MODER_MODER11;
	GPIOC->MODER |= GPIO_MODER_MODER11_1;

	GPIOC->MODER &= ~GPIO_MODER_MODER12;
	GPIOC->MODER |= GPIO_MODER_MODER12_1;

	//	set alternate functions:
	GPIOC->AFR[0] &= ~0xFF000000;
	GPIOC->AFR[0] |= 0x88000000;
	GPIOC->AFR[1] &= ~0x000FFF00;
	GPIOC->AFR[1] |= 0x00066600;

	// (00 no pull down, no pull up; 10 pull-down; 01 pull-up):
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR4);
	GPIOC->PUPDR |= GPIO_PUPDR_PUPDR4_1;
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR5);
	GPIOC->PUPDR |= GPIO_PUPDR_PUPDR5_1;

	//	output type (0 - push-pull, 1 - open-drain):

	//(11-max speed):
	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0 |
		GPIO_OSPEEDER_OSPEEDR1 |
		GPIO_OSPEEDER_OSPEEDR4 |
		GPIO_OSPEEDER_OSPEEDR5 |
		GPIO_OSPEEDER_OSPEEDR6 |
		GPIO_OSPEEDER_OSPEEDR7 |
		GPIO_OSPEEDER_OSPEEDR10 |
		GPIO_OSPEEDER_OSPEEDR11 |
		GPIO_OSPEEDER_OSPEEDR12;
}

static void setup_TIM5(void)
{
	// enable TIM5 clock:
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	// Timer clock is 84 [MHz]
	// 32bit cnt register
	// register is buffered and only overflow generate interrupt:
	TIM5->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;

	TIM5->PSC = 84 - 1;		 // every 1 us 1 count
	TIM5->ARR = 10000000 - 1; // 1 period is 10 [s] long

	//	interrupt enable:
	TIM5->DIER |= TIM_DIER_UIE;

	//	TIM5 enabling:
	TIM5->EGR |= TIM_EGR_UG;
	TIM5->CR1 |= TIM_CR1_CEN;
}

#if defined(ESC_PROTOCOL_PWM)	
static void setup_PWM()
{
	// enable TIM2 clock:
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	// register is buffered and only overflow generate interrupt:
	TIM2->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;

	// PWM mode 1 and output compare 3 preload enable:
	TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;
	// PWM mode 1 and output compare 4 preload enable:
	TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;

	// channel 3 enable:
	TIM2->CCER |= TIM_CCER_CC3E;
	// channel 4 enable:
	TIM2->CCER |= TIM_CCER_CC4E;

	// counter count every 0.5 [us] (typically 1 step is 1 [us]).
	// For better resolution and easier Dshot implementation it is 0.5[us].
	// Notice that lowest motor_value (2000) is still 1[ms] long as in typical PWM):
	TIM2->PSC = 84 / 2 - 1;
	TIM2->ARR = 2000000 / FREQUENCY_ESC_UPDATE - 1; // 1 period of PWM

	TIM2->CCR3 = 2000; // PWM length channel 3 (1 [ms])
	TIM2->CCR4 = 2000; // PWM length channel 4 (1 [ms])

	//	TIM2 enabling:
	TIM2->EGR |= TIM_EGR_UG;
	TIM2->CR1 |= TIM_CR1_CEN;

	// enable TIM3 clock:
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	// register is buffered:
	TIM3->CR1 |= TIM_CR1_ARPE;

	// PWM mode 1 and output compare 3 preload enable:
	TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;
	// PWM mode 1 and output compare 4 preload enable:
	TIM3->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;

	// channel 3 enable:
	TIM3->CCER |= TIM_CCER_CC3E;
	// channel 4 enable:
	TIM3->CCER |= TIM_CCER_CC4E;

	TIM3->PSC = 84 / 2 - 1;								// counter count every 0.5 microsecond (typically 1 step is 1 [us] long but for better resolution and easier Dshot implementation it is 0.5 [us]. Notice that lowest motor_value 2000 step is still 1 [ms] long as in typical PWM)
	TIM3->ARR = 2000000 / FREQUENCY_ESC_UPDATE - 1; // 1 period of PWM

	TIM3->CCR3 = 2000; // PWM length channel 3 (1 [ms])
	TIM3->CCR4 = 2000; // PWM length channel 4 (1 [ms])

	//	TIM3 enabling:
	TIM3->EGR |= TIM_EGR_UG;
	TIM3->CR1 |= TIM_CR1_CEN;
}
#elif defined(ESC_PROTOCOL_BDSHOT)
static void setup_BDshot()
{
	//	TIM1 - only for generating time basement all outputs are set by GPIOs:

	// enable TIM1 clock:

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	// register is buffered and overflow DMA request:
	TIM1->CR1 = 0x0;
	TIM1->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;

	// DMA request:
#if defined(BIT_BANGING_V1)
	TIM1->DIER |= TIM_DIER_CC1DE; // channel 1 request

	TIM1->CCR1 = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS;

	//	TIM1 is 168 [MHz]:
	TIM1->PSC = 168000 / DSHOT_MODE / DSHOT_BB_FRAME_LENGTH - 1;
	TIM1->ARR = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS - 1;

#elif defined(BIT_BANGING_V2)
	TIM1->DIER |= TIM_DIER_CC1DE; // channel 1 request
	TIM1->DIER |= TIM_DIER_CC2DE; // channel 2 request
	TIM1->DIER |= TIM_DIER_CC3DE; // channel 3 request

	TIM1->CCR1 = 0;
	TIM1->CCR2 = DSHOT_BB_0_LENGTH;
	TIM1->CCR3 = DSHOT_BB_1_LENGTH;

	//	TIM1 is 168 [MHz]:
	TIM1->PSC = 168000 / DSHOT_MODE / DSHOT_BB_FRAME_LENGTH - 1;
	TIM1->ARR = DSHOT_BB_FRAME_LENGTH - 1;
#endif

	//	TIM1 enable:
	TIM1->EGR |= TIM_EGR_UG;
	TIM1->CR1 |= TIM_CR1_CEN;

	//	TIM8 - only for generating time basement all outputs are set by GPIOs:

	// enable TIM8 clock:

	RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;

	// register is buffered and overflow DMA request:
	TIM8->CR1 = 0x0;
	TIM8->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;

	// DMA request:
#if defined(BIT_BANGING_V1)
	TIM8->DIER |= TIM_DIER_CC1DE; // channel 1 request

	TIM8->CCR1 = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS;

	//	TIM8 is 168 [MHz]:
	TIM8->PSC = 168000 / DSHOT_MODE / DSHOT_BB_FRAME_LENGTH - 1;
	TIM8->ARR = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS - 1;

#elif defined(BIT_BANGING_V2)
	TIM8->DIER |= TIM_DIER_CC1DE; // channel 1 request
	TIM8->DIER |= TIM_DIER_CC2DE; // channel 2 request
	TIM8->DIER |= TIM_DIER_CC3DE; // channel 3 request

	TIM8->CCR1 = 0;
	TIM8->CCR2 = DSHOT_BB_0_LENGTH;
	TIM8->CCR3 = DSHOT_BB_1_LENGTH;

	//	TIM8 is 168 [MHz]:
	TIM8->PSC = 168000 / DSHOT_MODE / DSHOT_BB_FRAME_LENGTH - 1;
	TIM8->ARR = DSHOT_BB_FRAME_LENGTH - 1;
#endif
	//	TIM8 enable:
	TIM8->EGR |= TIM_EGR_UG;
	TIM8->CR1 |= TIM_CR1_CEN;
}
#elif defined(ESC_PROTOCOL_DSHOT)
static void setup_Dshot()
{
	//	TIM2:

	// enable TIM2 clock:

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	// register is buffered and only overflow generate interrupt:
	TIM2->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;

	// PWM mode 1 and output compare 3 preload enable:
	TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;
	// PWM mode 1 and output compare 4 preload enable:
	TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;

	//	TIM2 is 84 [MHz]:
	TIM2->PSC = 84000 / DSHOT_MODE / DSHOT_PWM_FRAME_LENGTH - 1;
	TIM2->ARR = DSHOT_PWM_FRAME_LENGTH - 1;

	TIM2->DIER |= TIM_DIER_CC3DE | TIM_DIER_CC4DE; //	DMA request enable for 3rd and 4th channel

	TIM2->CCR3 = 10; //	PWM duration channel 3
	TIM2->CCR4 = 10; //	PWM duration channel 4

	// channel 3 output enable:
	TIM2->CCER |= TIM_CCER_CC3E;
	// channel 4 output enable:
	TIM2->CCER |= TIM_CCER_CC4E;

	//	TIM2 enabling:
	TIM2->EGR |= TIM_EGR_UG;
	TIM2->CR1 |= TIM_CR1_CEN;

	//	TIM3:

	// enable TIM3 clock:
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	// register is buffered and only overflow generate interrupt:
	TIM3->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;

	// PWM mode 1 and output compare 3 preload enable:
	TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;
	// PWM mode 1 and output compare 4 preload enable:
	TIM3->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;

	//	TIM3 is 84 [MHz]:
	TIM3->PSC = 84000 / DSHOT_MODE / DSHOT_PWM_FRAME_LENGTH - 1;
	TIM3->ARR = DSHOT_PWM_FRAME_LENGTH - 1;

	TIM3->DIER = 0x0;
	TIM3->DIER |= TIM_DIER_CC3DE | TIM_DIER_CC4DE; //	DMA request enable for 3 and 4 channel

	TIM3->CCR3 = 10; //	PWM duration channel 3
	TIM3->CCR4 = 10; //	PWM duration channel 4

	// channel 3 output enable:
	TIM3->CCER |= TIM_CCER_CC3E;
	// channel 4 output enable:
	TIM3->CCER |= TIM_CCER_CC4E;

	//	TIM3 enabling:
	TIM3->EGR |= TIM_EGR_UG;
	TIM3->CR1 |= TIM_CR1_CEN;
}

#elif defined(ESC_PROTOCOL_DSHOT_BURST)
static void setup_Dshot_burst()
{

	//	TIM2:

	// enable TIM2 clock:
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	TIM2->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;

	// PWM mode 1 and output compare 3 preload enable:
	TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;
	// PWM mode 1 and output compare 4 preload enable:
	TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;

	TIM2->DIER |= TIM_DIER_CC1DE; //	DMA request enable for 1 and 2 channel

	TIM2->PSC = 84 - 1; // 84000 / DSHOT_MODE / DSHOT_PWM_FRAME_LENGTH - 1;
	TIM2->ARR = DSHOT_PWM_FRAME_LENGTH - 1;

	TIM2->CCR2 = DSHOT_PWM_FRAME_LENGTH - 1; //	DMA request is send right before PWM generation
	TIM2->CCR3 = 0;							 //	PWM duration channel 3
	TIM2->CCR4 = 0;							 //	PWM duration channel 4

	// channel 1 enable:
	TIM2->CCER |= TIM_CCER_CC1E;

	// channel 3 output enable:
	TIM2->CCER |= TIM_CCER_CC3E;
	// channel 4 output enable:
	TIM2->CCER |= TIM_CCER_CC4E;

	TIM2->DCR |= TIM_DCR_DBL_0;									//	updating 2 registers CC3 and CC4
	TIM2->DCR |= TIM_DCR_DBA_3 | TIM_DCR_DBA_2 | TIM_DCR_DBA_1; //	offset of register CC3 (0x0E)

	//	TIM2 enabling:
	TIM2->EGR |= TIM_EGR_UG;
	TIM2->CR1 |= TIM_CR1_CEN;

	//	TIM3:

	// enable TIM3 clock:
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	TIM3->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;

	// PWM mode 1 and output compare 3 preload enable:
	TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;
	// PWM mode 1 and output compare 4 preload enable:
	TIM3->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;

	TIM3->DIER |= TIM_DIER_CC1DE; //	DMA request enable for 1

	TIM3->PSC = 84 - 1; // 84000 / DSHOT_MODE / DSHOT_PWM_FRAME_LENGTH - 1;
	TIM3->ARR = DSHOT_PWM_FRAME_LENGTH - 1;

	TIM3->CCR1 = DSHOT_PWM_FRAME_LENGTH - 1; //	DMA request is send right before PWM generation
	TIM3->CCR3 = 0;							 //	PWM duration channel 3
	TIM3->CCR4 = 0;							 //	PWM duration channel 4

	// channel 1 enable:
	TIM3->CCER |= TIM_CCER_CC1E;

	// channel 3 output enable:
	TIM3->CCER |= TIM_CCER_CC3E;
	// channel 4 output enable:
	TIM3->CCER |= TIM_CCER_CC4E;

	TIM3->DCR |= TIM_DCR_DBL_0;									//	updating 2 registers CC3 and CC4
	TIM3->DCR |= TIM_DCR_DBA_3 | TIM_DCR_DBA_2 | TIM_DCR_DBA_1; //	offset of register CC3 (0x0E)

	//	TIM3 enabling:
	TIM3->EGR |= TIM_EGR_UG;
	TIM3->CR1 |= TIM_CR1_CEN;
}

#elif defined(ESC_PROTOCOL_ONESHOT125)
static void setup_OneShot125()
{
	//	Enable TIM2 clock:
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->CR1 |= TIM_CR1_OPM; //	ONE_PULSE_MODE
	//	PWM 1 mode channel 3:
	TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
	//	PWM 1 mode channel 4:
	TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
	//	Channel's polarity -> when channels are active (for PWM 1 mode when CNT<CCR) signal is low:
	TIM2->CCER |= TIM_CCER_CC3P;
	TIM2->CCER |= TIM_CCER_CC4P;
	//	Channel 3 output enable:
	TIM2->CCER |= TIM_CCER_CC3E;
	//	Channel 4 output enable:
	TIM2->CCER |= TIM_CCER_CC4E;
	//	OneShot is 8x faster regular PWM. 
	//	1 step is 1/16 [us] (normally 1 [us]):
	TIM2->PSC = 84 / 16 - 1;
	TIM2->ARR = 3999 + 16;	//	PWM frame length (250+16 [us]) cause min delay required in pulse mode
	TIM2->CCR3 = 4015 - 2000 + 1;	//	PWM duration channel 3 (125 [us])
	TIM2->CCR4 = 4015 - 2000 + 1;	//	PWM duration channel 4 (125 [us])
	//	TIM2 enabling:
	TIM2->EGR |= TIM_EGR_UG;
	TIM2->CR1 |= TIM_CR1_CEN;

	//	TIM3:
	//	Enable TIM3 clock:
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3->CR1 |= TIM_CR1_OPM; //	ONE_PULSE_MODE
	//	PWM 1 mode channel 3:
	TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
	//	PWM 1 mode channel 4:
	TIM3->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
	//	Channel's polarity -> when channels are active (for PWM 1 mode when CNT<CCR) signal is low:
	TIM3->CCER |= TIM_CCER_CC3P;
	TIM3->CCER |= TIM_CCER_CC4P;
	//	Channel 3 output enable:
	TIM3->CCER |= TIM_CCER_CC3E;
	//	Channel 4 output enable:
	TIM3->CCER |= TIM_CCER_CC4E;
	//	OneShot is 8x faster regular PWM. 
	//	1 step is 1/16 [us] (normally 1 [us]):
	TIM3->PSC = 84 / 16 - 1;
	TIM3->ARR = 3999 + 16;	//	PWM frame length (250+1 [us]) cause min delay required in pulse mode
	TIM3->CCR3 = 4015 - 2000 + 1;	//	PWM duration channel 3 (125 [us])
	TIM3->CCR4 = 4015 - 2000 + 1;	//	PWM duration channel 4 (125 [us])
	//	TIM3 enabling:
	TIM3->EGR |= TIM_EGR_UG;
	TIM3->CR1 |= TIM_CR1_CEN;
}
#endif

static void setup_USART1()
{
	// enable USART1 clock:
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	USART1->BRR = 0x2D9;		   // 84 000 000 / 115 200/16 = 45.5729 so 45 is mantissa 0x2D and fraction 16*0.5729=~9 =0x9 so BRR is 0x9
	USART1->CR3 |= USART_CR3_DMAR; // uncomment for DMA reading
	USART1->CR1 |= USART_CR1_IDLEIE |
		USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
}

static void setup_USART3()
{
	// enable USART3 clock:
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	USART3->BRR = 0x16D; // 42 000 000 / 115 200/16 = 22.7865 so 22 is mantissa 0x16 and fraction 16*0.7865=~13 =0xD so BRR is 0x16D
	USART3->CR3 |= USART_CR3_DMAT;
	USART3->CR1 |=
		USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_UE;
} //

static void setup_USART6()
{
	// enable USART6 clock:
	RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
	USART6->BRR = 0x2D9; // 84 000 000 / 115 200/16 = 45.5729 so 45 is mantissa 0x2D and fraction 16*0.5729=~9 =0x9 so BRR is 0x9
	USART6->CR3 |= USART_CR3_DMAT;
	USART6->CR1 |= USART_CR1_RXNEIE |
		USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_UE;

} //

static void setup_SPI1()
{
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	SPI1->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1;											 // APB2 is 84 [MHz] and max frequency of MPU6000 registers is 1 [MHz] (84/128<1 [MHz])
	SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | SPI_CR1_CPOL | SPI_CR1_CPHA; // NSS value of master is set by software (SSM) it has to be high so set  SSI; Master configuration; clock idle is high (CPOL); second edge data capture (CPHA)

	SPI1->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
}

static void setup_SPI2()
{
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

	SPI2->CR1 |= SPI_CR1_BR_1;							   // APB1 is 42 [MHz] and max frequency of OSD registers is 10 [MHz] (42/8<10 [MHz])
	SPI2->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR; // NSS value of master is set by software (SSM) it has to be high so set  SSI; Master configuration; clock idle is low (CPOL not set); first (rising) edge data capture (CPHA not set)
	//	enable SPI2:
	SPI2->CR1 |= SPI_CR1_SPE;
}

static void setup_SPI3()
{
	RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;

	SPI3->CR1 |= SPI_CR1_BR_1 | SPI_CR1_BR_0;											 // should be able to change to  &=~(SPI_CR1_BR); APB1 clock is 42 [MHz] so baudrate is 42/2=21 [MHz] but it is not working the highest is 42/16 [MHz]
	SPI3->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | SPI_CR1_CPOL | SPI_CR1_CPHA; // NSS value of master is set by software (SSM) it has to be high so set  SSI; Master configuration; clock idle is high (CPOL); second edge data capture (CPHA)

	SPI3->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
}

static void setup_ADC1()
{
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	// set prescaler to 6 f_ADC = 84/6 = 14 [MHz] < 36 [MHz]:
	ADC->CCR |= ADC_CCR_ADCPRE_1;
	//	enable temperature sensor:
	ADC->CCR |= ADC_CCR_TSVREFE;

	//	use 12-bit resolution (default), SCAN mode on, overrun interrupt enable:
	ADC1->CR1 |= ADC_CR1_SCAN | ADC_CR1_OVRIE;
	//	EOC after each conversion, DMA on, DDS on:
	ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_DDS;
	// single conversion mode:
	ADC1->CR2 &= ~ADC_CR2_CONT;
	//	right data alignment:
	ADC1->CR2 &= ~ADC_CR2_ALIGN;

	//	sampling 3 cycles for channel11 (default):
	ADC1->SMPR1 &= ~ADC_SMPR1_SMP11;
	//	sampling 144 cycles for channel16:
	ADC1->SMPR1 |= ADC_SMPR1_SMP16_1 | ADC_SMPR1_SMP16_2;

	//	set 2 conversions in sequence:
	ADC1->SQR1 |= ADC_SQR1_L_0;
	//	define channels in sequence (IN11 - bat. volt., IN16 - temp.):
	ADC1->SQR3 |= ADC_SQR3_SQ1_3 | ADC_SQR3_SQ1_1 | ADC_SQR3_SQ1_0 | ADC_SQR3_SQ2_4;

	ADC1->CR2 |= ADC_CR2_ADON;
}

static void setup_DMA()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	// SPI3 - flash:
#if defined(USE_FLASH_BLACKBOX)
#if !defined(USE_I2C1)
	// reading
	DMA1_Stream0->CR = 0x0;
	while (DMA1_Stream0->CR & DMA_SxCR_EN)
	{
		; // wait
	}
	DMA1_Stream0->CR |= DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_PL_0;
	DMA1_Stream0->PAR = (uint32_t)(&(SPI3->DR));
	DMA1_Stream0->M0AR = (uint32_t)(flash_read_buffer);
#endif
	// writing:
	DMA1_Stream5->CR = 0x0;
	while (DMA1_Stream5->CR & DMA_SxCR_EN)
	{
		; // wait
	}
	DMA1_Stream5->CR |= DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE | DMA_SxCR_PL_0;
	DMA1_Stream5->PAR = (uint32_t)(&(SPI3->DR));
	DMA1_Stream5->M0AR = (uint32_t)(flash_write_buffer);

#endif

	// DSHOT:
#if defined(ESC_PROTOCOL_DSHOT)
	//	motor1:
	DMA1_Stream6->CR = 0x0;
	while (DMA1_Stream6->CR & DMA_SxCR_EN)
	{
		; // wait
	}
	DMA1_Stream6->CR |= DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1 | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE | DMA_SxCR_PL_0;
	DMA1_Stream6->PAR = (uint32_t)(&(TIM2->CCR4));
	DMA1_Stream6->M0AR = (uint32_t)(dshot_buffer_1);
	DMA1_Stream6->NDTR = DSHOT_BUFFER_LENGTH;

	//	motor2:
	DMA1_Stream7->CR = 0x0;
	while (DMA1_Stream7->CR & DMA_SxCR_EN)
	{
		; // wait
	}
	DMA1_Stream7->CR |= DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_2 | DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE | DMA_SxCR_PL_0;
	DMA1_Stream7->PAR = (uint32_t)(&(TIM3->CCR3));
	DMA1_Stream7->M0AR = (uint32_t)(dshot_buffer_2);
	DMA1_Stream7->NDTR = DSHOT_BUFFER_LENGTH;

	//	motor3:
	DMA1_Stream2->CR = 0x0;
	while (DMA1_Stream2->CR & DMA_SxCR_EN)
	{
		; // wait
	}
	DMA1_Stream2->CR |= DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_2 | DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE | DMA_SxCR_PL_0;
	DMA1_Stream2->PAR = (uint32_t)(&(TIM3->CCR4));
	DMA1_Stream2->M0AR = (uint32_t)(dshot_buffer_3);
	DMA1_Stream2->NDTR = DSHOT_BUFFER_LENGTH;

	//	motor4:
	DMA1_Stream1->CR = 0x0;
	while (DMA1_Stream1->CR & DMA_SxCR_EN)
	{
		; // wait
	}
	DMA1_Stream1->CR |= DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1 | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE | DMA_SxCR_PL_0;
	DMA1_Stream1->PAR = (uint32_t)(&(TIM2->CCR3));
	DMA1_Stream1->M0AR = (uint32_t)(dshot_buffer_4);
	DMA1_Stream1->NDTR = DSHOT_BUFFER_LENGTH;

	DMA1->LIFCR |= DMA_LIFCR_CTCIF1;
	DMA1->LIFCR |= DMA_LIFCR_CTCIF2;
	DMA1->HIFCR |= DMA_HIFCR_CTCIF6;
	DMA1->HIFCR |= DMA_HIFCR_CTCIF7;

	//-----------------------------END NORMAL DSHOT------------------------------------
	// bidirectional DSHOT:
#elif defined(ESC_PROTOCOL_BDSHOT)
	// for TIM1
	DMA2_Stream6->CR |= DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE | DMA_SxCR_PL_0;
	// all the other parameters will be set afterward

	DMA2_Stream2->CR |= DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE | DMA_SxCR_PL_0;
	// all the other parameters will be set afterward

// DSHOT_BURST:
#elif defined(ESC_PROTOCOL_DSHOT_BURST)

	//	motor 2 and 3:
	DMA1_Stream4->CR = 0x0;
	while (DMA1_Stream4->CR & DMA_SxCR_EN)
	{
		; // wait
	}
	DMA1_Stream4->CR |= DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_2 | DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE | DMA_SxCR_PL_1;
	DMA1_Stream4->PAR = (uint32_t)(&(TIM3->DMAR));
	DMA1_Stream4->M0AR = (uint32_t)(dshot_buffer_2_3);
	DMA1_Stream4->NDTR = DSHOT_BUFFER_LENGTH * 2;

	//	motor 4 and 1:

	TRZEBA TO ZMIENIC NA INNY STREAM !!!!5 jest zajeta przez SPI3 TX

		DMA1_Stream5->CR = 0x0;
	while (DMA1_Stream5->CR & DMA_SxCR_EN)
	{
		; // wait
	}

	DMA1_Stream5->CR |= DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_2 | DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE | DMA_SxCR_PL_1;
	DMA1_Stream5->PAR = (uint32_t)(&(TIM2->DMAR));
	DMA1_Stream5->M0AR = (uint32_t)(dshot_buffer_4_1);
	DMA1_Stream5->NDTR = DSHOT_BUFFER_LENGTH * 2;

	DMA1->HIFCR |= DMA_HIFCR_CTCIF5;
	DMA1->HIFCR |= DMA_HIFCR_CTCIF4;

#endif
	//-----------------------------END DSHOT_BURST-------------------------------------

	// USART6 - telemetry TX (from memory to peripheral):
	DMA2_Stream7->CR |= DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_2 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE | DMA_SxCR_PL_0;
	DMA2_Stream7->PAR = (uint32_t)(&(USART6->DR));
	DMA2_Stream7->M0AR = (uint32_t)(table_of_bytes_to_sent);
	DMA2_Stream7->NDTR = 2 * ALL_ELEMENTS_TO_SEND + 4;

	// USART1 - RX reading:
	DMA2_Stream5->CR |= DMA_SxCR_CHSEL_2 | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_TCIE | DMA_SxCR_PL_1;
	DMA2_Stream5->PAR = (uint32_t)(&(USART1->DR));
	DMA2_Stream5->M0AR = (uint32_t)(&rxBuf[1]);
	DMA2_Stream5->NDTR = 31;

	//	SPI1 - IMU reading:
	DMA2_Stream0->CR |= DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0 | DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_PL_0;
	DMA2_Stream0->PAR = (uint32_t)(&(SPI1->DR));
	// address and number of bytes will be set before each transmission
	DMA2_Stream3->CR |= DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE | DMA_SxCR_PL_0;
	DMA2_Stream3->PAR = (uint32_t)(&(SPI1->DR));

	//	SPI2 - OSD transmission
	DMA1_Stream4->CR |= DMA_SxCR_DIR_0 | DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_PL_0;
	DMA1_Stream4->PAR = (uint32_t)(&(SPI2->DR));
	// address and number of bytes will be set before each transmission

	//	ADC1:
	DMA2_Stream4->CR |= DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_TCIE | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1;
	DMA2_Stream4->PAR = (uint32_t)(&(ADC1->DR));
	DMA2_Stream4->M0AR = (uint32_t)(ADC1_buffer);
	DMA2_Stream4->NDTR = 2;

	// I2C1:
#if defined(USE_I2C1)
	// reading
	DMA1_Stream0->CR |= DMA_SxCR_CHSEL_0 | DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_PL_0;
	DMA1_Stream0->PAR = (uint32_t)(&(I2C1->DR));
	DMA1_Stream0->M0AR = (uint32_t)(I2C1_read_buffer); // it will be set during reception anyway
#endif
}

static void setup_EXTI()
{

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// set PC4,PC5 as EXTI:
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PC | SYSCFG_EXTICR2_EXTI5_PC;

	// disable interrupts mask for IM0, IM1, IM2, IM3, IM4, IM5 and IM15
	EXTI->IMR &= ~(EXTI_IMR_MR4 | EXTI_IMR_MR5 | EXTI_IMR_MR15);
	EXTI->IMR |= EXTI_IMR_MR4 | EXTI_IMR_MR5 | EXTI_IMR_MR15;

	// set rising edge detection for PC4, PC5:
	EXTI->RTSR |= EXTI_RTSR_TR4 | EXTI_RTSR_TR5;
	// set falling edge detection for PC5:
	EXTI->FTSR |= EXTI_FTSR_TR5;
}

static void setup_I2C1()
{
	//	enable I2C1 clock:
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	//	reset I2C:
	I2C1->CR1 |= I2C_CR1_SWRST;
	I2C1->CR1 &= ~I2C_CR1_SWRST;
	I2C1->CR1 &= ~I2C_CR1_NOSTRETCH;
	// peripheral clock frequency (assume max. 42 [MHz]):
	I2C1->CR2 |= 0x2A;
	// according datasheet need to be set high:
	I2C1->OAR1 |= (1UL << 14);

	//	I2C1 SM/FM mode selection (set FM):
	I2C1->CCR |= I2C_CCR_FS;
	// DMA requests enable, NACK at the end of DMA reception:
	I2C1->CR2 |= I2C_CR2_DMAEN | I2C_CR2_LAST;

	/*400kHz setting (datasheet says that main clock should have been multiply of 10 [MHz] but it should work)
	 * APB1 clock has 42 [MHz] -> 1/42 [us] is main clock period
	 * according datasheet:
	 * in FM mode Low time (T_L) = 2*High time (T_H) so whole period is 3*T_H
	 * since we want 1/400[ms]=2.5 [us] period 2.5/3 = T_H
	 * T_H needs to be a multiple of main clock period so CCR=(2.5/3)/(1/42) = 35
	 * so CCR =0x23
	 */
	I2C1->CCR |= 0x23;
	/* max. rise time - I couldn't find much information about it
	 * Only general info about rising/falling time for certain GPIO speed setting (sth. like 2.5-6 [ns] for max. speed)
	 * On the other hand in the Internet I found 300 [ns] for FM,
	 * 1000 [ns] (used for SM) is too much for sure (period for 400 [kHz] is 2.5 [us] so falling and rising would take 2 [us])
	 * so to be safe use 300 [ns] = 0.3 [us] -> 0.3/(1/42) = 13 -> TRISE = 13+1
	 */
	I2C1->TRISE |= 0x0E;

	//	peripheral enable:
	I2C1->CR1 |= I2C_CR1_PE;
}


void setup_NVIC_1()
{
	// Only basics interrupts (Global_Time; delay_functions(); failsafe_functions();)

	//	nvic interrupt enable (TIM5 interrupt);
	NVIC_EnableIRQ(TIM5_IRQn);
	NVIC_SetPriority(TIM5_IRQn, 7);

	//	nvic EXTI interrupt enable:
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	NVIC_SetPriority(EXTI15_10_IRQn, 8);
}

void setup_NVIC_2()
{
	//	nvic interrupt enable (USART6 interrupt):
	NVIC_EnableIRQ(USART6_IRQn);
	NVIC_SetPriority(USART6_IRQn, 17);

	//	nvic interrupt enable (USART1 interrupt):
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn, 9);

	//	nvic interrupt enable (EXTI interrupt)
	NVIC_EnableIRQ(EXTI4_IRQn);
	NVIC_SetPriority(EXTI4_IRQn, 10);

	//	nvic interrupt enable (EXTI interrupt)
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_SetPriority(EXTI9_5_IRQn, 20);

	//	nvic interrupt enable (ADC1 interrupt)
	NVIC_DisableIRQ(ADC_IRQn);
	NVIC_SetPriority(ADC_IRQn, 22);

	//	nvic DMA interrupts enable:
#if defined(USE_FLASH_BLACKBOX)
	NVIC_EnableIRQ(DMA1_Stream0_IRQn);
#endif

	NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	NVIC_SetPriority(DMA1_Stream1_IRQn, 16);

	NVIC_EnableIRQ(DMA1_Stream2_IRQn);
	NVIC_SetPriority(DMA1_Stream2_IRQn, 15);

#if defined(USE_FLASH_BLACKBOX)
	NVIC_EnableIRQ(DMA1_Stream5_IRQn);
#endif
	NVIC_EnableIRQ(DMA1_Stream4_IRQn);
	NVIC_SetPriority(DMA1_Stream4_IRQn, 17);

	NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	NVIC_SetPriority(DMA1_Stream6_IRQn, 14);

	NVIC_EnableIRQ(DMA1_Stream7_IRQn);
	NVIC_SetPriority(DMA1_Stream7_IRQn, 13);

	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	NVIC_SetPriority(DMA2_Stream0_IRQn, 12);

	NVIC_EnableIRQ(DMA2_Stream3_IRQn);
	NVIC_SetPriority(DMA2_Stream3_IRQn, 12);

	NVIC_EnableIRQ(DMA2_Stream4_IRQn);
	NVIC_SetPriority(DMA2_Stream4_IRQn, 18);

	NVIC_EnableIRQ(DMA2_Stream5_IRQn);
	NVIC_SetPriority(DMA2_Stream5_IRQn, 11);

	NVIC_EnableIRQ(DMA2_Stream7_IRQn);
	NVIC_SetPriority(DMA2_Stream7_IRQn, 12);

#if defined(ESC_PROTOCOL_BDSHOT)

	NVIC_EnableIRQ(DMA2_Stream6_IRQn);
	NVIC_SetPriority(DMA2_Stream6_IRQn, 13);
	NVIC_EnableIRQ(DMA2_Stream2_IRQn);
	NVIC_SetPriority(DMA2_Stream2_IRQn, 14);

#endif
}
