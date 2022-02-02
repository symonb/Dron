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

static void CS_flash_disable();
static void CS_flash_enable();
static void SPI3_enable();
static void SPI3_disable();
static void SPI_transmit(uint8_t*data, uint8_t size);
static void SPI_transmit_DMA(uint8_t*data, int size);
static void SPI_receive(uint8_t *data, int size);
static void SPI_receive_DMA(uint8_t *data, int size);
static void flash_write_enable();
static void failsafe_SPI();



//RX:
void DMA1_Stream0_IRQHandler(void) {

	//if stream0 transfer is completed:
	if (DMA1->LISR & DMA_LISR_TCIF0) {


		time_flag5_1 = get_Global_Time();
		while (!((SPI3->SR) & SPI_SR_TXE)&& failsafe_type != 7) {
			failsafe_SPI(); 			//wait
		}
		time_flag5_1 = get_Global_Time();
		while (((SPI3->SR) & SPI_SR_BSY)&& failsafe_type != 7) {
			failsafe_SPI();				//wait
		}
		SPI3->DR;
		SPI3->SR;
		DMA1_Stream0->CR &= ~DMA_SxCR_EN;

		CS_flash_disable();
		SPI3_disable();
	}

}

//TX:
void DMA1_Stream5_IRQHandler(void) {

	//if stream5 transfer is completed:
	if (DMA1->HISR & DMA_HISR_TCIF5) {
		DMA1->HIFCR |= DMA_HIFCR_CTCIF5;

		time_flag5_1 = get_Global_Time();
		while (!((SPI3->SR) & SPI_SR_TXE)&& failsafe_type != 7) {
			failsafe_SPI(); 			//wait
		}
		time_flag5_1 = get_Global_Time();
		while (((SPI3->SR) & SPI_SR_BSY)&& failsafe_type != 7) {
			failsafe_SPI();				//wait
		}
		SPI3->DR;
		SPI3->SR;
		DMA1_Stream5->CR &= ~DMA_SxCR_EN;
		DMA1_Stream0->CR &= ~DMA_SxCR_EN;

		CS_flash_disable();
		SPI3_disable();
	}
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
	while (!((SPI3->SR) & SPI_SR_TXE)&& failsafe_type != 7) {
		failsafe_SPI(); 			//wait
	}
	SPI3->DR = data[i]; //first data
	i++;

	while (i < size) {
		time_flag5_1 = get_Global_Time();
		while (!((SPI3->SR) & SPI_SR_TXE)&& failsafe_type != 7) {
			failsafe_SPI(); 			//wait
		}
		SPI3->DR = data[i]; //second and following data sending as soon as TX flag is set
		i++;
	}

	time_flag5_1 = get_Global_Time();
	while (!((SPI3->SR) & SPI_SR_TXE)&& failsafe_type != 7) {
		failsafe_SPI(); 			//wait
	}
	time_flag5_1 = get_Global_Time();
	while (((SPI3->SR) & SPI_SR_BSY)&& failsafe_type != 7) {
		failsafe_SPI();				//wait
	}
	SPI3->DR;
	SPI3->SR;

}

static void SPI_transmit_DMA(uint8_t *data, int size) {

	DMA1_Stream5->M0AR = (uint32_t) (data);
	DMA1_Stream5->NDTR = size;
	DMA1_Stream5->CR |= DMA_SxCR_EN;
}

static void SPI_receive(uint8_t *data, int size) {

	while (size > 0) {

		time_flag5_1 = get_Global_Time();
		while (!((SPI3->SR) & SPI_SR_TXE) && failsafe_type != 7) {
			failsafe_SPI(); 			//wait
		}
		SPI3->DR = 0xFF; 			//send anything IMPORTANT!
		time_flag5_1 = get_Global_Time();
		while (!((SPI3->SR) & SPI_SR_RXNE) && failsafe_type != 7) {
			failsafe_SPI(); 			//wait
		}
		*data++ = SPI3->DR;
		size--;

	}

//wait for TXE flag
	time_flag5_1 = get_Global_Time();
	while (!((SPI3->SR) & SPI_SR_TXE)&& failsafe_type != 7) {
		failsafe_SPI(); 			//wait
	}
//wait for BSY flag
	time_flag5_1 = get_Global_Time();
	while (((SPI3->SR) & SPI_SR_BSY)&& failsafe_type != 7) {
		failsafe_SPI();				//wait
	}

	SPI3->DR;
	SPI3->SR;

}

static void SPI_receive_DMA(uint8_t *data, int size) {

	DMA1_Stream0->M0AR = (uint32_t) (data);
	DMA1_Stream0->NDTR = size;
	DMA1_Stream0->CR |= DMA_SxCR_EN;

	DMA1_Stream5->M0AR = (uint32_t) (data);
	DMA1_Stream5->NDTR = size;
	DMA1_Stream5->CR |= DMA_SxCR_EN;
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

void flash_SPI_write_DMA(uint8_t instruction, uint8_t *data, int size) {

//must be done before writing:
	flash_write_enable();

	SPI3_enable();
	CS_flash_enable();

	SPI_transmit(&instruction, 1);
	SPI_transmit_DMA(data, size);

}

void flash_SPI_read(uint8_t instruction, uint8_t *memory_address,
		int number_of_bytes) {

	SPI3_enable();
	CS_flash_enable();

	SPI_transmit(&instruction, 1);
	SPI_transmit(memory_address, 3);
	SPI_receive(memory_address, number_of_bytes);

	CS_flash_disable();
	SPI3_disable();
}
void flash_SPI_read_DMA(uint8_t instruction, uint8_t *data, int size) {

	SPI3_enable();
	CS_flash_enable();

	SPI_transmit(&instruction, 1);
	SPI_receive_DMA(data, size);

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
	while (!((SPI3->SR) & SPI_SR_TXE) && failsafe_type != 7) {
		failsafe_SPI(); 			//wait
	}
// send first data - instruction for reading
	SPI3->DR = instruction;
//wait for TXE flag and send 2nd byte - anything
	while (!((SPI3->SR) & SPI_SR_TXE) && failsafe_type != 7) {
		failsafe_SPI(); 			//wait
	}
// send 2nd byte:
	SPI3->DR = 0xFF;

// now start receiving status value:
	time_flag5_1 = get_Global_Time();
	while (!((SPI3->SR) & SPI_SR_RXNE) && failsafe_type != 7) {
		failsafe_SPI(); 			//wait
	}
//read first byte this is rubbish:
	SPI3->DR;

//wait for TXE flag
	time_flag5_1 = get_Global_Time();
	while (!((SPI3->SR) & SPI_SR_TXE)&& failsafe_type != 7) {
		failsafe_SPI(); 			//wait
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

void flash_save_data(uint8_t instruction, uint32_t memory_address,
		uint8_t *data, int number_of_bytes) {

	uint8_t memory_address_tab[3] = { (memory_address >> 16) & 0xFF,
			(memory_address >> 8) & 0xFF, (memory_address) & 0xFF };

//must be done before writing:
	flash_write_enable();

	SPI3_enable();
	CS_flash_enable();

	SPI_transmit(&instruction, 1);
	SPI_transmit(memory_address_tab, 3);
	SPI_transmit_DMA(data, number_of_bytes);
}

void flash_read_data(uint8_t instruction, uint32_t memory_address,
		uint8_t *data, int number_of_bytes) {

	uint8_t memory_address_tab[3] = { (memory_address >> 16) & 0xFF,
			(memory_address >> 8) & 0xFF, (memory_address) & 0xFF };

	SPI3_enable();
	CS_flash_enable();

	SPI_transmit(&instruction, 1);
	SPI_transmit(memory_address_tab, 3);
	SPI_receive(data,number_of_bytes);

	CS_flash_disable();
	SPI3_disable();
}

void flash_add_data_to_save(uint8_t data) {

	flash_write_buffer[flash_write_counter] = data;

	flash_write_counter++;

	if (flash_write_counter == 256) {
		flash_save_data(FLASH_PAGE_PROGRAM, flash_global_write_address,
				flash_write_buffer, 256);
		flash_global_write_address += 0x100;
	} else if (flash_write_counter == 512) {
		flash_save_data(FLASH_PAGE_PROGRAM, flash_global_write_address,
				&flash_write_buffer[256], 256);
		flash_global_write_address += 0x100;
		flash_write_counter = 0;
	}

}

