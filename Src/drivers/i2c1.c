/*
 * i2c1.c
 *
 *  Created on: 04.08.2022
 *      Author: symon
 */

#include <math.h>
#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "I2C1.h"

static bool I2C1_failsafe();

#if defined(USE_I2C1)
void DMA1_Stream0_IRQHandler(void) {

	//if stream0 transfer is completed:
	if (DMA1->LISR & DMA_LISR_TCIF0) {
		DMA1->LIFCR |= DMA_LIFCR_CTCIF0;

		DMA1_Stream0->CR &= ~DMA_SxCR_EN;
		// generate STOP conditions:
		I2C1->CR1 |= I2C_CR1_STOP;
	}
}
#endif

/**
 *@brief manually unstack I2c lines by bitbanging
 *@note When I2C hang up it seems impossible to unstack it by I2C commands.
 *@note Set GPIO ports as outputs and manually clock STOP condition.
* @note Next set GPIO to alternate function for I2C operation and reconfigure I2C.
*/
void I2C1_reset() {

	I2C1->CR1 &= ~I2C_CR1_PE;
	// set output mode:
	GPIOB->MODER &= ~GPIO_MODER_MODER6;
	GPIOB->MODER |= GPIO_MODER_MODER6_0;

	GPIOB->MODER &= ~GPIO_MODER_MODER7;
	GPIOB->MODER |= GPIO_MODER_MODER7_0;

	// bitbanging stop conditions:
	GPIOB->BSRR |= GPIO_BSRR_BS_6;
	GPIOB->BSRR |= GPIO_BSRR_BS_7;

	GPIOB->BSRR |= GPIO_BSRR_BR7;
	delay_micro(10);
	GPIOB->BSRR |= GPIO_BSRR_BS7;


	// set alterante functions (back to I2C):
	GPIOB->MODER &= ~GPIO_MODER_MODER6;
	GPIOB->MODER |= GPIO_MODER_MODER6_1;

	GPIOB->MODER &= ~GPIO_MODER_MODER7;
	GPIOB->MODER |= GPIO_MODER_MODER7_1;

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

	/*400kHz setting (datasheet says that main clock should been multiply of 10 [MHz] but it should work)
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

void I2C1_Start() {

	I2C1->CR1 |= I2C_CR1_ACK;
	I2C1->CR1 |= I2C_CR1_START;
	// wait for flag:
	time_flag7_1 = get_Global_Time();
	while (!(I2C1->SR1 & I2C_SR1_SB)) {
		if (I2C1_failsafe())
		{
			break;
		}
	}
}

void I2C1_Stop() {
	I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C1_Write(uint8_t address, uint8_t command, uint8_t* data, uint16_t size) {

	I2C1_Start();
	uint8_t* ptr = data;
	I2C1->DR = address;
	//	wait for ADDR bit:
	time_flag7_1 = get_Global_Time();
	while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
		if (I2C1_failsafe())
		{
			break;
		}
	}
	// for clearing ADDR bit read SR1 and SR2:
	I2C1->SR2;
	//	send command:
	time_flag7_1 = get_Global_Time();
	while (!(I2C1->SR1 & I2C_SR1_TXE)) {
		if (I2C1_failsafe())
		{
			break;
		}
	}
	I2C1->DR = command;
	//	send data:
	while (size) {
		time_flag7_1 = get_Global_Time();
		while (!(I2C1->SR1 & I2C_SR1_TXE)) {
			if (I2C1_failsafe())
			{
				break;
			}
		}
		I2C1->DR = *ptr;

		ptr++;
		size--;
	}
	time_flag7_1 = get_Global_Time();
	while (!(I2C1->SR1 & I2C_SR1_BTF) && !(I2C1->SR1 & I2C_SR1_TXE)) {
		if (I2C1_failsafe())
		{
			break;
		}
	}
}

/**
 *@note STOP condition is generated during this function. No need for STOP generating after this function.
*/
void I2C1_Read(uint8_t address, uint8_t* data, uint16_t size) {

	uint8_t* ptr = data;
	I2C1_Start();
	I2C1->DR = address;
	// wait for ADDR bit:
	time_flag7_1 = get_Global_Time();
	while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
		if (I2C1_failsafe())
		{
			break;
		}
	}

	// different steps for 1/2/more bytes reception:
	if (size == 2) {
		I2C1->CR1 &= ~I2C_CR1_ACK;
		I2C1->CR1 |= I2C_CR1_POS;

		// for clearing ADDR bit read SR1 and SR2:
		I2C1->SR2;

		time_flag7_1 = get_Global_Time();
		while (!(I2C1->SR1 & I2C_SR1_BTF)) {
			if (I2C1_failsafe())
			{
				break;
			}
		}
		I2C1->CR1 |= I2C_CR1_STOP;
		*ptr++ = I2C1->DR;

		time_flag7_1 = get_Global_Time();
		while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
			if (I2C1_failsafe())
			{
				break;
			}
		}
		*ptr = I2C1->DR;
		I2C1->CR1 &= ~I2C_CR1_POS;
		return;
	}
	else if (size == 1) {
		I2C1->CR1 &= ~I2C_CR1_ACK;
		// for clearing ADDR bit read also SR2:
		I2C1->SR2;

		I2C1->CR1 |= I2C_CR1_STOP;
		time_flag7_1 = get_Global_Time();
		while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
			if (I2C1_failsafe())
			{
				break;
			}
		}
		*ptr = I2C1->DR;
		return;
	}
	else {
		// for clearing ADDR bit read SR1 and SR2:
		I2C1->SR2;

		while (size > 3) {
			time_flag7_1 = get_Global_Time();
			while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
				if (I2C1_failsafe())
				{
					break;
				}
			}
			*ptr++ = I2C1->DR;
			// ACK will be send automatically
			size--;
		}

		time_flag7_1 = get_Global_Time();
		while (!(I2C1->SR1 & I2C_SR1_BTF)) {
			if (I2C1_failsafe())
			{
				break;
			}
		}
		// set NACK after next byte
		I2C1->CR1 &= ~I2C_CR1_ACK;

		// read 3rd last byte:
		*ptr++ = I2C1->DR;

		time_flag7_1 = get_Global_Time();
		while (!(I2C1->SR1 & I2C_SR1_BTF)) {
			if (I2C1_failsafe())
			{
				break;
			}
		}

		// STOP condition:
		I2C1->CR1 |= I2C_CR1_STOP;

		// read 2nd last byte:
		*ptr++ = I2C1->DR;

		time_flag7_1 = get_Global_Time();
		while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
			if (I2C1_failsafe())
			{
				break;
			}
		}
		// read last byte:
		*ptr = I2C1->DR;
	}
}

static bool I2C1_failsafe()
{
	//	waiting as Data will be sent or failsafe if set time passed
	if ((get_Global_Time() - time_flag7_1) >= SEC_TO_US(I2C_TIMEOUT_VALUE))
	{
		if (FailSafe_status != FAILSAFE_I2C1_ERROR) {
			FailSafe_status = FAILSAFE_I2C1_ERROR;
			EXTI->SWIER |= EXTI_SWIER_SWIER15;
		}
		return true;
	}
	return false;
}

void I2C1_Read_DMA(uint8_t address, uint8_t* data, uint16_t size) {
	DMA1_Stream0->NDTR = size;
	DMA1_Stream0->M0AR = (uint32_t)(data);

	I2C1_Start();

	I2C1->DR = (address | 0x1);
	// wait for ADDR bit:
	while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
	}
	// activate DMA:
	DMA1_Stream0->CR |= DMA_SxCR_EN;

	// for clearing ADDR bit read also SR2:
	I2C1->SR2;

}



