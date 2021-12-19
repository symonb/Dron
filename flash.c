/*
 * flash.c

 *
 *  Created on: 04.09.2021
 *      Author: symon
 */
#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include <math.h>
#include "flash.h"

static void setup_SPI3();
static void CS_flash_disable();
static void CS_flash_enable();
static void SPI3_enable();
static void SPI3_disable();
static void SPI_transmit(uint8_t*data, uint8_t size);
static void SPI_transmit_DMA(uint8_t*data, uint8_t size);
static void SPI_receive( uint8_t *data,uint8_t size);
static void flash_write_enable();
static void failsafe_SPI();

float time_flag5_1;

//RX:
void DMA1_Stream0_IRQHandler(void) {

	//if stream0 transfer is completed:
	if (DMA1->LISR & DMA_LISR_TCIF0) {
		DMA1->LIFCR |= DMA_LIFCR_CTCIF0;
		DMA1_Stream0->CR &= ~DMA_SxCR_EN;

	}
}
//TX:
void DMA1_Stream5_IRQHandler(void) {

	//if stream5 transfer is completed:
	if (DMA1->HISR & DMA_HISR_TCIF5) {
		DMA1->HIFCR |= DMA_HIFCR_CTCIF5;

		time_flag5_1 = get_Global_Time();
		while (!((SPI3->SR) & SPI_SR_TXE)) {
			failsafe_SPI(); 			//wait
		}
		time_flag5_1 = get_Global_Time();
		while (((SPI3->SR) & SPI_SR_BSY)) {
			failsafe_SPI();				//wait
		}
		SPI3->DR;
		SPI3->SR;
DMA1_Stream5->CR &= ~DMA_SxCR_EN;

		CS_flash_disable();
		SPI3_disable();
	}
}

void setup_FLASH() {

	setup_SPI3();
	CS_flash_disable();

}

static void setup_SPI3() {
	RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;

	SPI3->CR1 |= SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2; //now it is 164 khz for debugging purpose then change to  &=~(SPI_CR1_BR); APB1 clock is 42 [MHz] so baudrate is 42/2=21 [MHz]
	SPI3->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | SPI_CR1_CPOL
			| SPI_CR1_CPHA; //NSS value of master is set by software (SSM) it has to be high so set  SSI; Master configuration; clock idle is high (CPOL); second edge data capture (CPHA)

	SPI3->CR2 = SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;

}

static void CS_flash_enable() {
	GPIOB->BSRR |= GPIO_BSRR_BR3;
}

static void CS_flash_disable() {
	GPIOB->BSRR |= GPIO_BSRR_BS3;
}

static void SPI3_enable() {
	SPI3->CR1 |= SPI_CR1_SPE; 			//	enabling SPI3

}

static void SPI3_disable() {
	SPI3->CR1 &= ~SPI_CR1_SPE; 			//	disabling SPI3

}

static void SPI_transmit(uint8_t *data, uint8_t size) {

	int i = 0;

	time_flag5_1 = get_Global_Time();
	while (!((SPI3->SR) & SPI_SR_TXE)) {
		failsafe_SPI(); 			//wait
	}
	SPI3->DR = data[i]; //first data
	i++;

	while (i < size) {
		time_flag5_1 = get_Global_Time();
		while (!((SPI3->SR) & SPI_SR_TXE)) {
			failsafe_SPI(); 			//wait
		}
		SPI3->DR = data[i]; //second and following data sending as soon as TX flag is set
		i++;
	}

	time_flag5_1 = get_Global_Time();
	while (!((SPI3->SR) & SPI_SR_TXE)) {
		failsafe_SPI(); 			//wait
	}
	time_flag5_1 = get_Global_Time();
	while (((SPI3->SR) & SPI_SR_BSY)) {
		failsafe_SPI();				//wait
	}
	SPI3->DR;
	SPI3->SR;

}

static void SPI_transmit_DMA(uint8_t *data, uint8_t size) {

	DMA1_Stream5->M0AR = (uint32_t) (data);
	DMA1_Stream5->NDTR = size;
	DMA1_Stream5->CR |= DMA_SxCR_EN;
}

static void SPI_receive( uint8_t *data,
		uint8_t size) {

	while (size > 0) {

		time_flag5_1 = get_Global_Time();
		while (!((SPI3->SR) & SPI_SR_TXE) && failsafe_type != 6) {
			failsafe_SPI(); 			//wait
		}
		SPI3->DR = 0xFF; 			//send anything IMPORTANT!
		time_flag5_1 = get_Global_Time();
		while (!((SPI3->SR) & SPI_SR_RXNE) && failsafe_type != 6) {
			failsafe_SPI(); 			//wait
		}
		*data++ = SPI3->DR;
		size--;

	}

	//wait for TXE flag
	time_flag5_1 = get_Global_Time();
	while (!((SPI3->SR) & SPI_SR_TXE)) {
		failsafe_SPI(); 			//wait
	}
	//wait for BSY flag
	time_flag5_1 = get_Global_Time();
	while (((SPI3->SR) & SPI_SR_BSY)) {
		failsafe_SPI();				//wait
	}

	SPI3->DR;
	SPI3->SR;

}

void flash_SPI_write(uint8_t instruction, uint8_t *data, uint8_t size) {

	//must be done before writing:
	flash_write_enable();

	SPI3_enable();
	CS_flash_enable();

	SPI_transmit(&instruction, 1);
	SPI_transmit(data, size);

	CS_flash_disable();
	SPI3_disable();

}

void flash_SPI_write_DMA(uint8_t instruction, uint8_t *data, uint8_t size) {

	//must be done before writing:
	flash_write_enable();

	SPI3_enable();
	CS_flash_enable();

	SPI_transmit(&instruction, 1);
	SPI_transmit_DMA(data, size);

}

void flash_SPI_read(uint8_t instruction, uint8_t *memory_address,
		uint8_t number_of_bytes) {

	SPI3_enable();
	CS_flash_enable();

	SPI_transmit(&instruction,1);
	SPI_transmit(memory_address,3);
	SPI_receive(memory_address, number_of_bytes);

	CS_flash_disable();
	SPI3_disable();
}

static void flash_write_enable() {

	uint8_t instruction = FLASH_WRITE_ENABLE;

	SPI3_enable();
	CS_flash_enable();

	SPI_transmit(&instruction, 1);

	CS_flash_disable();
	SPI3_disable();
}

void flash_erase(uint8_t instruction, uint8_t *address) {

	//must be done before erasing:
	flash_write_enable();

	SPI3_enable();
	CS_flash_enable();

	SPI_transmit(&instruction, 1);
	SPI_transmit(address, 3);

	CS_flash_disable();
	SPI3_disable();
}

void flash_full_chip_erase() {

	uint8_t instruction = FLASH_CHIP_ERASE;

	//must be done before some instruction:
	flash_write_enable();

	SPI3_enable();
	CS_flash_enable();

	SPI_transmit(&instruction, 1);

	CS_flash_disable();
	SPI3_disable();
}

uint8_t flash_read_status_register(uint8_t instruction) {
	uint8_t status;

	SPI3_enable();
	CS_flash_enable();

	//wait for TXE flag before sending anything
	time_flag5_1 = get_Global_Time();
	while (!((SPI3->SR) & SPI_SR_TXE) && failsafe_type != 6) {
		failsafe_SPI(); 			//wait
	}
	// send first data - instruction for reading
	SPI3->DR = instruction;
	//wait for TXE flag and send 2nd byte - anything
	while (!((SPI3->SR) & SPI_SR_TXE) && failsafe_type != 6) {
		failsafe_SPI(); 			//wait
	}
	// send 2nd byte:
	SPI3->DR = 0xFF;

	// now start receiving status value:
	time_flag5_1 = get_Global_Time();
	while (!((SPI3->SR) & SPI_SR_RXNE) && failsafe_type != 6) {
		failsafe_SPI(); 			//wait
	}
	//read first byte this is rubbish:
	SPI3->DR;

	//wait for TXE flag
	time_flag5_1 = get_Global_Time();
	while (!((SPI3->SR) & SPI_SR_TXE)) {
		failsafe_SPI(); 			//wait
	}
	//wait for BSY flag
	time_flag5_1 = get_Global_Time();
	while (((SPI3->SR) & SPI_SR_BSY)) {
		failsafe_SPI();				//wait
	}
	//read status:
	status = SPI3->DR;
	SPI3->SR;

	CS_flash_disable();
	SPI3_disable();

	return status;
}

static void failsafe_SPI() {

	//	waiting as Data will be sent or failsafe if set time passed
	if ((get_Global_Time() - time_flag5_1) >= TIMEOUT_VALUE) {
		failsafe_type = 7;
		EXTI->SWIER |= EXTI_SWIER_SWIER15;
	}
}

void flash_save_data(uint8_t instruction,uint32_t memory_address, uint8_t *data, uint8_t number_of_bytes){

	uint8_t memory_address_tab[3]={(memory_address>>16)&0xFF,(memory_address>>8)&0xFF,(memory_address)&0xFF};

	//must be done before writing:
		flash_write_enable();

		SPI3_enable();
		CS_flash_enable();

		SPI_transmit(&instruction, 1);
		SPI_transmit(memory_address_tab, 3);
		SPI_transmit_DMA(data, number_of_bytes);
}

void flash_read_data(uint8_t instruction,uint32_t memory_address, uint8_t *data, uint8_t number_of_bytes){

	uint8_t memory_address_tab[3]={(memory_address>>16)&0xFF,(memory_address>>8)&0xFF,(memory_address)&0xFF};

	SPI3_enable();
	CS_flash_enable();

	SPI_transmit(&instruction,1);
	SPI_transmit(memory_address_tab,3);
	SPI_receive(data, number_of_bytes);

	CS_flash_disable();
	SPI3_disable();
}




