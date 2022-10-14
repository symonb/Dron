/*
 * global_functions.c

 *
 *  Created on: 03.05.2021
 *      Author: symon
 */
#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "OSD.h"

flight_mode_e get_Flight_Mode()
{
	return flight_mode;
}

void interrupts_LOCK()
{

	// LOCK ALL INTERRUPTS:

	// nvic interrupt enable (TIM5 interrupt);
	NVIC_DisableIRQ(TIM5_IRQn);

	// nvic EXTI interrupt enable:
	NVIC_DisableIRQ(EXTI15_10_IRQn);

	// nvic interrupt enable (USART6 interrupt):
	NVIC_DisableIRQ(USART6_IRQn);

	// nvic interrupt enable (USART1 interrupt):
	NVIC_DisableIRQ(USART1_IRQn);

	// nvic interrupt enable (EXTI interrupt)
	NVIC_DisableIRQ(EXTI4_IRQn);

	// nvic interrupt enable (EXTI interrupt)
	NVIC_DisableIRQ(EXTI9_5_IRQn);

	// nvic DMA interrupt enable:
#if defined(USE_FLASH_BLACKBOX)
	NVIC_DisableIRQ(DMA1_Stream0_IRQn);
#endif

	NVIC_DisableIRQ(DMA1_Stream1_IRQn);

	NVIC_DisableIRQ(DMA1_Stream2_IRQn);

#if defined(USE_FLASH_BLACKBOX)
	NVIC_EnableIRQ(DMA1_Stream5_IRQn);
#endif

	NVIC_DisableIRQ(DMA1_Stream6_IRQn);

	NVIC_DisableIRQ(DMA1_Stream7_IRQn);

	NVIC_DisableIRQ(DMA2_Stream5_IRQn);

	NVIC_DisableIRQ(DMA2_Stream6_IRQn);

// if oneshot_v2 is ok these 2 nvics conf. are unnecessary:
#if defined(ESC_PROTOCOL_ONESHOT_V1)
	// nvic interrupt enable (TIM3 interrupt);
	NVIC_DisableIRQ(TIM3_IRQn);

	// nvic interrupt enable (TIM2 interrupt);
	NVIC_DisableIRQ(TIM2_IRQn);

#endif
}

void interrupts_UNLOCK()
{

	// UNLOCK ALL INTERRUPTS:

	// nvic interrupt enable (TIM5 interrupt);
	NVIC_EnableIRQ(TIM5_IRQn);

	// nvic EXTI interrupt enable:
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	// nvic interrupt enable (USART6 interrupt):
	NVIC_EnableIRQ(USART6_IRQn);

	// nvic interrupt enable (USART1 interrupt):
	NVIC_EnableIRQ(USART1_IRQn);

	// nvic interrupt enable (EXTI interrupt)
	NVIC_EnableIRQ(EXTI4_IRQn);

	// nvic interrupt enable (EXTI interrupt)
	NVIC_EnableIRQ(EXTI9_5_IRQn);

	// nvic DMA interrupt enable:
#if defined(USE_FLASH_BLACKBOX)
	NVIC_EnableIRQ(DMA1_Stream0_IRQn);
#endif

	NVIC_EnableIRQ(DMA1_Stream1_IRQn);

	NVIC_EnableIRQ(DMA1_Stream2_IRQn);

#if defined(USE_FLASH_BLACKBOX)
	NVIC_EnableIRQ(DMA1_Stream5_IRQn);
#endif

	NVIC_EnableIRQ(DMA1_Stream6_IRQn);

	NVIC_EnableIRQ(DMA1_Stream7_IRQn);

	NVIC_EnableIRQ(DMA2_Stream5_IRQn);

	NVIC_EnableIRQ(DMA2_Stream6_IRQn);

// if oneshot_v2 is ok these 2 nvics conf. are unnecessary:
#if defined(ESC_PROTOCOL_ONESHOT_V1)
	// nvic interrupt enable (TIM3 interrupt);
	NVIC_EnableIRQ(TIM3_IRQn);

	// nvic interrupt enable (TIM2 interrupt);
	NVIC_EnableIRQ(TIM2_IRQn);

#endif
}

void TIM5_IRQHandler()
{
	if (TIM_SR_UIF & TIM5->SR)
	{
		TIM5->SR &= ~TIM_SR_UIF;
		Global_Time += (TIM5->ARR + 1);
	}
}

uint64_t get_Global_Time()
{
	// gives time in [us] with accuracy of 1 [us]
	return (Global_Time + TIM5->CNT);
}

void delay_mili(uint32_t delay_time)
{
	// wait for time [ms]
	uint64_t t = get_Global_Time();
	while ((get_Global_Time() - t) < (uint64_t)delay_time * 1000)
	{
		;
	}
}

void delay_micro(uint16_t delay_time)
{
	uint64_t t = get_Global_Time();
	while (get_Global_Time() - t < delay_time)
	{
		;
	}
}

bool failsafe_PID_loop(timeUs_t *dt)
{

	if (*dt > SEC_TO_US(2. / FREQUENCY_MAIN_LOOP))
	{
		*dt = SEC_TO_US(2. / FREQUENCY_MAIN_LOOP);
		FailSafe_status = PID_LOOP_TIMEOUT;
		EXTI->SWIER |= EXTI_SWIER_SWIER15;
		return true;
	}
	return false;
}

void anti_windup(ThreeF *sum_err, PIDF *R_PIDF, PIDF *P_PIDF, PIDF *Y_PIDF)
{
	if (ARMING_STATUS == ARMED)
	{
		//	limit I_corrections:
		if ((sum_err->roll * R_PIDF->I) > MAX_I_CORRECTION)
		{
			sum_err->roll = MAX_I_CORRECTION / R_PIDF->I;
		}
		else if ((sum_err->roll * R_PIDF->I) < -MAX_I_CORRECTION)
		{
			sum_err->roll = -MAX_I_CORRECTION / R_PIDF->I;
		}
		if ((sum_err->pitch * P_PIDF->I) > MAX_I_CORRECTION)
		{
			sum_err->pitch = MAX_I_CORRECTION / P_PIDF->I;
		}
		else if ((sum_err->pitch * P_PIDF->I) < -MAX_I_CORRECTION)
		{
			sum_err->pitch = -MAX_I_CORRECTION / P_PIDF->I;
		}
		if ((sum_err->yaw * Y_PIDF->I) > MAX_I_CORRECTION)
		{
			sum_err->yaw = MAX_I_CORRECTION / Y_PIDF->I;
		}
		else if ((sum_err->yaw * Y_PIDF->I) < -MAX_I_CORRECTION)
		{
			sum_err->yaw = -MAX_I_CORRECTION / Y_PIDF->I;
		}
	}

	else
	{ // quad is disarmed so turn off I term of corrections
		sum_err->roll = 0;
		sum_err->pitch = 0;
		sum_err->yaw = 0;
	}
}

void set_motors(ThreeF corr)
{
	const uint16_t max_value = 4000;

	//	Make corrections:
	//	right back:
	motor_1_value = Throttle * 2 - corr.roll + corr.pitch - corr.yaw;
	//	right front:
	motor_2_value = Throttle * 2 - corr.roll - corr.pitch + corr.yaw;
	//	left back:
	motor_3_value = Throttle * 2 + corr.roll + corr.pitch + corr.yaw;
	//	left front:
	motor_4_value = Throttle * 2 + corr.roll - corr.pitch - corr.yaw;

	if (motor_1_value < IDLE_VALUE * 2)
	{
		motor_1_value = IDLE_VALUE * 2;
	}
	else if (motor_1_value > max_value)
		motor_1_value = max_value;
	if (motor_2_value < IDLE_VALUE * 2)
	{
		motor_2_value = IDLE_VALUE * 2;
	}
	else if (motor_2_value > max_value)
		motor_2_value = max_value;
	if (motor_3_value < IDLE_VALUE * 2)
	{
		motor_3_value = IDLE_VALUE * 2;
	}
	else if (motor_3_value > max_value)
		motor_3_value = max_value;
	if (motor_4_value < IDLE_VALUE * 2)
	{
		motor_4_value = IDLE_VALUE * 2;
	}
	else if (motor_4_value > max_value)
		motor_4_value = max_value;
}

void turn_ON_BLUE_LED()
{
	GPIOB->BSRRH |= GPIO_BSRR_BS_5;
}
void turn_OFF_BLUE_LED()
{
	GPIOB->BSRRL |= GPIO_BSRR_BS_5;
}
void turn_ON_RED_LED()
{
	GPIOB->BSRRH |= GPIO_BSRR_BS_4;
}
void turn_OFF_RED_LED()
{
	GPIOB->BSRRL |= GPIO_BSRR_BS_4;
}
void toggle_BLUE_LED()
{
	GPIOB->ODR ^= GPIO_ODR_ODR_5;
}
void toggle_RED_LED()
{
	GPIOB->ODR ^= GPIO_ODR_ODR_4;
}

void turn_on_BUZZER()
{ // it has to be buzzer with generator
	GPIOA->ODR |= GPIO_ODR_ODR_8;
}
void turn_off_BUZZER()
{ // it has to be buzzer with generator
	GPIOA->ODR &= ~GPIO_ODR_ODR_8;
}
void toggle_BUZZER()
{ // it has to be buzzer with generator
	GPIOA->ODR ^= GPIO_ODR_ODR_8;
}

void EXTI9_5_IRQHandler()
{
	if ((EXTI->PR & EXTI_PR_PR5))
	{
		EXTI->PR |= EXTI_PR_PR5; // clear this bit setting it high

		USB_detected = GPIOC->IDR & GPIO_IDR_IDR_5;
	}
	if ((EXTI->PR & EXTI_PR_PR6))
	{
		EXTI->PR |= EXTI_PR_PR6; // clear this bit setting it high
	}
	if ((EXTI->PR & EXTI_PR_PR7))
	{
		EXTI->PR |= EXTI_PR_PR7; // clear this bit setting it high
	}
	if ((EXTI->PR & EXTI_PR_PR8))
	{
		EXTI->PR |= EXTI_PR_PR8; // clear this bit setting it high
	}
	if ((EXTI->PR & EXTI_PR_PR9))
	{
		EXTI->PR |= EXTI_PR_PR9; // clear this bit setting it high
	}
}

// FAILSAFE HANDLER:
void EXTI15_10_IRQHandler()
{

	if ((EXTI->PR & EXTI_PR_PR10))
	{
		EXTI->PR |= EXTI_PR_PR10; // clear this bit setting it high
	}
	if ((EXTI->PR & EXTI_PR_PR11))
	{
		EXTI->PR |= EXTI_PR_PR11; // clear this bit setting it high
	}
	if ((EXTI->PR & EXTI_PR_PR12))
	{
		EXTI->PR |= EXTI_PR_PR12; // clear this bit setting it high
	}
	if ((EXTI->PR & EXTI_PR_PR13))
	{
		EXTI->PR |= EXTI_PR_PR13; // clear this bit setting it high
	}
	if ((EXTI->PR & EXTI_PR_PR14))
	{
		EXTI->PR |= EXTI_PR_PR14; // clear this bit setting it high
	}
	// FAILSAFEs
	if ((EXTI->PR & EXTI_PR_PR15))
	{
		OSD_print_warnings();
		static uint16_t err_counter[FAILSAFES_COUNTER];
		EXTI->PR |= EXTI_PR_PR15; // clear(setting 1) this bit (and at the same time bit SWIER15)

		switch (FailSafe_status)
		{
		case NO_FAILSAFE:
			break;

		case INCORRECT_CHANNELS_VALUES: // BAD_CHANNELS_VALUES

			motor_1_value_pointer = &MOTOR_OFF;
			motor_2_value_pointer = &MOTOR_OFF;
			motor_3_value_pointer = &MOTOR_OFF;
			motor_4_value_pointer = &MOTOR_OFF;

			err_counter[INCORRECT_CHANNELS_VALUES]++;
			FailSafe_status = NO_FAILSAFE;
			break;
		case RX_TIMEOUT: // RX_TIMEOUT

			ARMING_STATUS = DISARMED;

			motor_1_value_pointer = &MOTOR_OFF;
			motor_2_value_pointer = &MOTOR_OFF;
			motor_3_value_pointer = &MOTOR_OFF;
			motor_4_value_pointer = &MOTOR_OFF;

			err_counter[RX_TIMEOUT]++;
			FailSafe_status = NO_FAILSAFE;
			break;
		case NO_PREARM:
			err_counter[NO_PREARM]++;
			FailSafe_status = NO_FAILSAFE;
			break;
		case SETUP_ERROR:
			err_counter[SETUP_ERROR]++;
			FailSafe_status = NO_FAILSAFE;
			break;
		case I2C_ERROR:
			err_counter[I2C_ERROR]++;
			FailSafe_status = NO_FAILSAFE;
			break;
		case SPI_IMU_ERROR:
			err_counter[SPI_IMU_ERROR]++;
			FailSafe_status = NO_FAILSAFE;
			break;
		case SPI_FLASH_ERROR:
			err_counter[SPI_FLASH_ERROR]++;
			FailSafe_status = NO_FAILSAFE;
			break;
		case SPI_OSD_ERROR:
			err_counter[SPI_OSD_ERROR]++;
			FailSafe_status = NO_FAILSAFE;
			break;
		case PID_LOOP_TIMEOUT:
			err_counter[PID_LOOP_TIMEOUT]++;
			FailSafe_status = NO_FAILSAFE;
			break;
		}
	}
}

void DMA1_Stream4_IRQHandler(void)
{

	if (DMA1->HISR & DMA_HISR_TCIF4)
	{
		DMA1->HIFCR |= DMA_HIFCR_CTCIF4;
		DMA1_Stream4->CR &= ~DMA_SxCR_EN;
	}
	if (DMA1->HISR & DMA_HISR_HTIF4)
	{
		DMA1->HIFCR |= DMA_HIFCR_CHTIF4;
	}
	if (DMA1->HISR & DMA_HISR_DMEIF4)
	{
		DMA1->HIFCR |= DMA_HIFCR_CDMEIF4;
	}
	if (DMA1->HISR & DMA_HISR_TEIF4)
	{
		DMA1->HIFCR |= DMA_HIFCR_CTEIF4;
	}
}
