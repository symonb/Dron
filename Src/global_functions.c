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

void delay_mili(uint16_t delay_time)
{
	// wait for time [ms]
	uint64_t t = get_Global_Time();
	while (get_Global_Time() - t < delay_time * 1000)
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

void update_motors(timeUs_t current_time)
{

#if defined(ESC_PROTOCOL_DSHOT)

	//	Dshot:

	fill_Dshot_buffer(prepare_Dshot_package(*motor_1_value_pointer),
					  prepare_Dshot_package(*motor_2_value_pointer),
					  prepare_Dshot_package(*motor_3_value_pointer),
					  prepare_Dshot_package(*motor_4_value_pointer));

	DMA1_Stream6->PAR = (uint32_t)(&(TIM2->CCR4));
	DMA1_Stream6->M0AR = (uint32_t)(dshot_buffer_1);
	DMA1_Stream6->NDTR = DSHOT_BUFFER_LENGTH;

	DMA1_Stream7->PAR = (uint32_t)(&(TIM3->CCR3));
	DMA1_Stream7->M0AR = (uint32_t)(dshot_buffer_2);
	DMA1_Stream7->NDTR = DSHOT_BUFFER_LENGTH;

	DMA1_Stream2->PAR = (uint32_t)(&(TIM3->CCR4));
	DMA1_Stream2->M0AR = (uint32_t)(dshot_buffer_3);
	DMA1_Stream2->NDTR = DSHOT_BUFFER_LENGTH;

	DMA1_Stream1->PAR = (uint32_t)(&(TIM2->CCR3));
	DMA1_Stream1->M0AR = (uint32_t)(dshot_buffer_4);
	DMA1_Stream1->NDTR = DSHOT_BUFFER_LENGTH;

	DMA1_Stream1->CR |= DMA_SxCR_EN;
	DMA1_Stream2->CR |= DMA_SxCR_EN;
	DMA1_Stream6->CR |= DMA_SxCR_EN;
	DMA1_Stream7->CR |= DMA_SxCR_EN;

#elif defined(ESC_PROTOCOL_DSHOT_BURST)

	//	Dshot with burst transfer:

	//	fill_Dshot_burst_buffer(prepare_Dshot_package(*motor_1_value_pointer),prepare_Dshot_package(*motor_2_value_pointer),prepare_Dshot_package(*motor_3_value_pointer),prepare_Dshot_package(*motor_4_value_pointer));
	//	DMA1_Stream2->CR |= DMA_SxCR_EN;
	//	DMA1_Stream6->CR |= DMA_SxCR_EN;

#elif defined(ESC_PROTOCOL_ONESHOT125) || defined(ESC_PROTOCOL_ONESHOT_V1)

	//	OneShot125 or OneShot125_v2:

	prepare_OneShot_PWM();

	TIM2->EGR |= TIM_EGR_UG;
	TIM3->EGR |= TIM_EGR_UG;
	TIM2->CR1 |= TIM_CR1_CEN;
	TIM3->CR1 |= TIM_CR1_CEN;

#elif defined(ESC_PROTOCOL_PWM)

	//	PWM:

	TIM2->CCR4 = *motor_1_value_pointer; // value motor 1
	TIM3->CCR3 = *motor_2_value_pointer; // value motor 2
	TIM3->CCR4 = *motor_3_value_pointer; // value motor 3
	TIM2->CCR3 = *motor_4_value_pointer; // value motor 4

#endif
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
		static uint16_t err_counter[10];
		EXTI->PR |= EXTI_PR_PR15; // clear(setting 1) this bit (and at the same time bit SWIER15)

		switch (FailSafe_status)
		{
		case NO_FAILSAFE:
			break;

		case INCORRECT_CHANNELS_VALUES: // BAD_CHANNELS_VALUES

			ARMING_STATUS = DISARMED;

			motor_1_value_pointer = &MOTOR_OFF;
			motor_2_value_pointer = &MOTOR_OFF;
			motor_3_value_pointer = &MOTOR_OFF;
			motor_4_value_pointer = &MOTOR_OFF;

			err_counter[1]++;
			FailSafe_status = NO_FAILSAFE;
			break;
		case RX_TIMEOUT: // RX_TIMEOUT

			ARMING_STATUS = DISARMED;

			motor_1_value_pointer = &MOTOR_OFF;
			motor_2_value_pointer = &MOTOR_OFF;
			motor_3_value_pointer = &MOTOR_OFF;
			motor_4_value_pointer = &MOTOR_OFF;

			err_counter[2]++;
			FailSafe_status = NO_FAILSAFE;
			break;
		case SETUP_ERROR:
			err_counter[3]++;
			FailSafe_status = NO_FAILSAFE;
			break;
		case I2C_ERROR:
			err_counter[4]++;
			FailSafe_status = NO_FAILSAFE;
			break;
		case SPI_IMU_ERROR:
			err_counter[5]++;
			FailSafe_status = NO_FAILSAFE;
			break;
		case SPI_FLASH_ERROR:
			err_counter[6]++;
			FailSafe_status = NO_FAILSAFE;
			break;
		case PID_LOOP_TIMEOUT:
			err_counter[7]++;
			FailSafe_status = NO_FAILSAFE;
			break;
		}
	}
}

//	motor 1
void DMA1_Stream6_IRQHandler(void)
{

	if (DMA1->HISR & DMA_HISR_TCIF6)
	{
		DMA1->HIFCR |= DMA_HIFCR_CTCIF6;
	}
	if (DMA1->HISR & DMA_HISR_HTIF6)
	{
		DMA1->HIFCR |= DMA_HIFCR_CHTIF6;
	}
	if (DMA1->HISR & DMA_HISR_DMEIF6)
	{
		DMA1->HIFCR |= DMA_HIFCR_CDMEIF6;
	}
	if (DMA1->HISR & DMA_HISR_TEIF6)
	{
		DMA1->HIFCR |= DMA_HIFCR_CTEIF6;
	}
}

//	motor 2
void DMA1_Stream7_IRQHandler(void)
{

	if (DMA1->HISR & DMA_HISR_TCIF7)
	{
		DMA1->HIFCR |= DMA_HIFCR_CTCIF7;
	}
	if (DMA1->HISR & DMA_HISR_HTIF7)
	{
		DMA1->HIFCR |= DMA_HIFCR_CHTIF7;
	}
	if (DMA1->HISR & DMA_HISR_DMEIF7)
	{
		DMA1->HIFCR |= DMA_HIFCR_CDMEIF7;
	}
	if (DMA1->HISR & DMA_HISR_TEIF7)
	{
		DMA1->HIFCR |= DMA_HIFCR_CTEIF7;
	}
}

//	motor 3
void DMA1_Stream2_IRQHandler(void)
{

	if (DMA1->LISR & DMA_LISR_TCIF2)
	{
		DMA1->LIFCR |= DMA_LIFCR_CTCIF2;
	}
	if (DMA1->LISR & DMA_LISR_HTIF2)
	{
		DMA1->LIFCR |= DMA_LIFCR_CHTIF2;
	}
	if (DMA1->LISR & DMA_LISR_DMEIF2)
	{
		DMA1->LIFCR |= DMA_LIFCR_CDMEIF2;
	}
	if (DMA1->LISR & DMA_LISR_TEIF2)
	{
		DMA1->LIFCR |= DMA_LIFCR_CTEIF2;
	}
}

//	motor 4
void DMA1_Stream1_IRQHandler(void)
{

	if (DMA1->LISR & DMA_LISR_TCIF1)
	{
		DMA1->LIFCR |= DMA_LIFCR_CTCIF1;
	}

	if (DMA1->LISR & DMA_LISR_HTIF1)
	{
		DMA1->LIFCR |= DMA_LIFCR_CHTIF1;
	}
	if (DMA1->LISR & DMA_LISR_DMEIF1)
	{
		DMA1->LIFCR |= DMA_LIFCR_CDMEIF1;
	}
	if (DMA1->LISR & DMA_LISR_TEIF1)
	{
		DMA1->LIFCR |= DMA_LIFCR_CTEIF1;
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

uint16_t get_Dshot_checksum(uint16_t value)
{
	value = value << 1;
	return (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
}

uint16_t prepare_Dshot_package(uint16_t value)
{
	// value is in range of 2000-4000 so I need to transform it into Dshot range (48-2047)
	value -= 1953;
	if (value > 0 && value < 48)
	{
		value = 48;
	}
	return ((value << 5) | get_Dshot_checksum(value));
}

void fill_Dshot_buffer(uint16_t m1_value, uint16_t m2_value, uint16_t m3_value,
					   uint16_t m4_value)
{

	for (uint8_t i = 2; i < DSHOT_BUFFER_LENGTH; i++)
	{
		if ((1 << (i - 2)) & m1_value)
		{
			dshot_buffer_1[DSHOT_BUFFER_LENGTH - 1 - i] = DSHOT_1_LENGTH;
		}
		else
		{
			dshot_buffer_1[DSHOT_BUFFER_LENGTH - 1 - i] = DSHOT_0_LENGTH;
		}
		if ((1 << (i - 2)) & m2_value)
		{
			dshot_buffer_2[DSHOT_BUFFER_LENGTH - 1 - i] = DSHOT_1_LENGTH;
		}
		else
		{
			dshot_buffer_2[DSHOT_BUFFER_LENGTH - 1 - i] = DSHOT_0_LENGTH;
		}
		if ((1 << (i - 2)) & m3_value)
		{
			dshot_buffer_3[DSHOT_BUFFER_LENGTH - 1 - i] = DSHOT_1_LENGTH;
		}
		else
		{
			dshot_buffer_3[DSHOT_BUFFER_LENGTH - 1 - i] = DSHOT_0_LENGTH;
		}
		if ((1 << (i - 2)) & m4_value)
		{
			dshot_buffer_4[DSHOT_BUFFER_LENGTH - 1 - i] = DSHOT_1_LENGTH;
		}
		else
		{
			dshot_buffer_4[DSHOT_BUFFER_LENGTH - 1 - i] = DSHOT_0_LENGTH;
		}
	}
	// make 0 pulse after Dshot frame:
	dshot_buffer_1[DSHOT_BUFFER_LENGTH - 1] = 0;
	dshot_buffer_1[DSHOT_BUFFER_LENGTH - 2] = 0;
	dshot_buffer_2[DSHOT_BUFFER_LENGTH - 1] = 0;
	dshot_buffer_2[DSHOT_BUFFER_LENGTH - 2] = 0;
	dshot_buffer_3[DSHOT_BUFFER_LENGTH - 1] = 0;
	dshot_buffer_3[DSHOT_BUFFER_LENGTH - 2] = 0;
	dshot_buffer_4[DSHOT_BUFFER_LENGTH - 1] = 0;
	dshot_buffer_4[DSHOT_BUFFER_LENGTH - 2] = 0;
}

void fill_Dshot_burst_buffer(uint16_t m1_value, uint16_t m2_value,
							 uint16_t m3_value, uint16_t m4_value)
{

	// buffer array looks like this [first bit motor1; first bit motor2 ;...; last bit motor1;last bit motor2]

	for (int i = 2; i < DSHOT_BUFFER_LENGTH; i++)
	{
		if ((1 << (i - 2)) & m4_value)
		{
			dshot_buffer_4_1[DSHOT_BUFFER_LENGTH * 2 - 1 - 2 * i - 1] =
				DSHOT_1_LENGTH;
		}
		else
		{
			dshot_buffer_4_1[DSHOT_BUFFER_LENGTH * 2 - 1 - 2 * i - 1] =
				DSHOT_0_LENGTH;
		}
		if ((1 << (i - 2)) & m1_value)
		{
			dshot_buffer_4_1[DSHOT_BUFFER_LENGTH * 2 - 1 - 2 * i] =
				DSHOT_1_LENGTH;
		}
		else
		{
			dshot_buffer_4_1[DSHOT_BUFFER_LENGTH * 2 - 1 - 2 * i] =
				DSHOT_0_LENGTH;
		}
		if ((1 << (i - 2)) & m2_value)
		{
			dshot_buffer_2_3[DSHOT_BUFFER_LENGTH * 2 - 1 - 2 * i - 1] =
				DSHOT_1_LENGTH;
		}
		else
		{
			dshot_buffer_2_3[DSHOT_BUFFER_LENGTH * 2 - 1 - 2 * i - 1] =
				DSHOT_0_LENGTH;
		}
		if ((1 << (i - 2)) & m3_value)
		{
			dshot_buffer_2_3[DSHOT_BUFFER_LENGTH * 2 - 1 - 2 * i] =
				DSHOT_1_LENGTH;
		}
		else
		{
			dshot_buffer_2_3[DSHOT_BUFFER_LENGTH * 2 - 1 - 2 * i] =
				DSHOT_0_LENGTH;
		}
	}
	dshot_buffer_4_1[2 * DSHOT_BUFFER_LENGTH - 1] = 0;
	dshot_buffer_4_1[2 * DSHOT_BUFFER_LENGTH - 2] = 0;
	dshot_buffer_2_3[2 * DSHOT_BUFFER_LENGTH - 1] = 0;
	dshot_buffer_2_3[2 * DSHOT_BUFFER_LENGTH - 2] = 0;
}

void prepare_OneShot_PWM()
{

	TIM2->CCR4 = 3500 - *motor_1_value_pointer * 0.875f + 1; // value motor 1
	TIM3->CCR3 = 3500 - *motor_2_value_pointer * 0.875f + 1; // value motor 2
	TIM3->CCR4 = 3500 - *motor_3_value_pointer * 0.875f + 1; // value motor 3
	TIM2->CCR3 = 3500 - *motor_4_value_pointer * 0.875f + 1; // value motor 4
}
