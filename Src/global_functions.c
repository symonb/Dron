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
#include "usb.h"

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
#if defined(USE_BLACKBOX) || defined(USE_I2C1)
	NVIC_DisableIRQ(DMA1_Stream0_IRQn);
#endif

	NVIC_DisableIRQ(DMA1_Stream1_IRQn);

	NVIC_DisableIRQ(DMA1_Stream2_IRQn);

#if defined(USE_FLASHFS)
	NVIC_DisableIRQ(DMA1_Stream5_IRQn);
#endif

	NVIC_DisableIRQ(DMA1_Stream6_IRQn);

	NVIC_DisableIRQ(DMA1_Stream7_IRQn);

	NVIC_DisableIRQ(DMA2_Stream5_IRQn);

	NVIC_DisableIRQ(DMA2_Stream6_IRQn);
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
#if defined(USE_BLACKBOX)|| defined(USE_I2C1)
	NVIC_EnableIRQ(DMA1_Stream0_IRQn);
#endif

	NVIC_EnableIRQ(DMA1_Stream1_IRQn);

	NVIC_EnableIRQ(DMA1_Stream2_IRQn);

#if defined(USE_FLASHFS)
	NVIC_EnableIRQ(DMA1_Stream5_IRQn);
#endif

	NVIC_EnableIRQ(DMA1_Stream6_IRQn);

	NVIC_EnableIRQ(DMA1_Stream7_IRQn);

	NVIC_EnableIRQ(DMA2_Stream5_IRQn);

	NVIC_EnableIRQ(DMA2_Stream6_IRQn);
}

void TIM5_IRQHandler()
{
	if (TIM_SR_UIF & TIM5->SR)
	{
		TIM5->SR &= ~TIM_SR_UIF;
		Global_Time += (TIM5->ARR + 1);
	}
}

timeUs_t get_Global_Time()
{
	// gives time in [us] with accuracy of 1 [us]
	return (Global_Time + TIM5->CNT);
}

void delay_mili(timeMs_t delay_time)
{
	// wait for time [ms]
	timeUs_t t = get_Global_Time();
	while ((get_Global_Time() - t) < (timeUs_t)delay_time * 1000)
	{
		;
	}
}

void delay_micro(uint16_t delay_time)
{
	timeUs_t t = get_Global_Time();
	while (get_Global_Time() - t < delay_time)
	{
		;
	}
}


void anti_windup()
{
	if (Arming_status == ARMED)
	{
		//	limit I_corrections:
		for (uint8_t i = 0;i < 3;i++) {
			if ((corr_PIDF[i].I) > MAX_I_CORRECTION)
			{
				corr_PIDF[i].I = MAX_I_CORRECTION;
			}
			else if ((corr_PIDF[i].I) < -MAX_I_CORRECTION)
			{
				corr_PIDF[i].I = -MAX_I_CORRECTION;
			}
		}
	}
	else
	{ // quad is disarmed so turn off I term of corrections
		corr_PIDF[0].I = 0;
		corr_PIDF[1].I = 0;
		corr_PIDF[2].I = 0;
	}
}

void set_motors(threef_t corr)
{
	//	Make corrections:
	//	right back:
	motor_value[0] = (receiver.Throttle - corr.roll + corr.pitch - corr.yaw) * 2;
	//	right front:
	motor_value[1] = (receiver.Throttle - corr.roll - corr.pitch + corr.yaw) * 2;
	//	left back:
	motor_value[2] = (receiver.Throttle + corr.roll + corr.pitch + corr.yaw) * 2;
	//	left front:
	motor_value[3] = (receiver.Throttle + corr.roll - corr.pitch - corr.yaw) * 2;

	for (uint8_t i = 0; i < MOTORS_COUNT;++i) {
		if (motor_value[i] < THROTTLE_MIN * 2)
		{
			motor_value[i] = THROTTLE_MIN * 2;
		}
		else if (motor_value[i] > THROTTLE_MAX * 2) {
			motor_value[i] = THROTTLE_MAX * 2;
		}
	}
}

void turn_ON_BLUE_LED()
{
	GPIOB->BSRR |= GPIO_BSRR_BR_5;
}
void turn_OFF_BLUE_LED()
{
	GPIOB->BSRR |= GPIO_BSRR_BS_5;
}
void turn_ON_RED_LED()
{
	GPIOB->BSRR |= GPIO_BSRR_BR_4;
}
void turn_OFF_RED_LED()
{
	GPIOB->BSRR |= GPIO_BSRR_BS_4;
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
		USB_check_connection();
		if (main_usb.class == USB_CLASS_MSC && !main_usb.connected) {
			NVIC_SystemReset();
		}
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

		EXTI->PR |= EXTI_PR_PR15; // clear(setting 1) this bit (and at the same time bit SWIER15)

		switch (FailSafe_status)
		{
		case FAILSAFE_NO_FAILSAFE:
			break;

		case FAILSAFE_INCORRECT_CHANNELS_VALUES: // BAD_CHANNELS_VALUES

			motor_1_value_pointer = &MOTOR_OFF;
			motor_2_value_pointer = &MOTOR_OFF;
			motor_3_value_pointer = &MOTOR_OFF;
			motor_4_value_pointer = &MOTOR_OFF;

			failsafe_counter[FAILSAFE_INCORRECT_CHANNELS_VALUES]++;
			FailSafe_status = FAILSAFE_NO_FAILSAFE;
			break;
		case FAILSAFE_RX_TIMEOUT: // FAILSAFE_RX_TIMEOUT

			Arming_status = DISARMED;

			motor_1_value_pointer = &MOTOR_OFF;
			motor_2_value_pointer = &MOTOR_OFF;
			motor_3_value_pointer = &MOTOR_OFF;
			motor_4_value_pointer = &MOTOR_OFF;

			failsafe_counter[FAILSAFE_RX_TIMEOUT]++;
			FailSafe_status = FAILSAFE_NO_FAILSAFE;
			break;
		case FAILSAFE_NO_PREARM:
			failsafe_counter[FAILSAFE_NO_PREARM]++;
			FailSafe_status = FAILSAFE_NO_FAILSAFE;
			break;
		case FAILSAFE_I2C_ERROR:
			failsafe_counter[FAILSAFE_I2C_ERROR]++;
			FailSafe_status = FAILSAFE_NO_FAILSAFE;
			break;
		case FAILSAFE_SPI_IMU_ERROR:
			failsafe_counter[FAILSAFE_SPI_IMU_ERROR]++;
			FailSafe_status = FAILSAFE_NO_FAILSAFE;
			break;
		case FAILSAFE_SPI_FLASH_ERROR:
			failsafe_counter[FAILSAFE_SPI_FLASH_ERROR]++;
			FailSafe_status = FAILSAFE_NO_FAILSAFE;
			break;
		case FAILSAFE_SPI_OSD_ERROR:
			failsafe_counter[FAILSAFE_SPI_OSD_ERROR]++;
			FailSafe_status = FAILSAFE_NO_FAILSAFE;
			break;
		case FAILSAFE_GYRO_CALIBRATION:
			failsafe_counter[FAILSAFE_GYRO_CALIBRATION]++;
			FailSafe_status = FAILSAFE_NO_FAILSAFE;
			break;
		default:
			break;
		}
	}
}

