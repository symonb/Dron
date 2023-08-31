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
#include <string.h>
#include <math.h>
#include "drivers/SPI3.h"
#include "flash.h"

static volatile bool data_sending = false;

static void W25Q128_write_enable();

// RX:
#if !defined(USE_I2C1)
void DMA1_Stream0_IRQHandler(void)
{

	// if stream0 transfer is completed:
	if (DMA1->LISR & DMA_LISR_TCIF0)
	{
		DMA1->LIFCR |= DMA_LIFCR_CTCIF0;

		time_flag5_1 = get_Global_Time();
		while (!((SPI3->SR) & SPI_SR_TXE))
		{
			if (failsafe_SPI3())
			{
				break; // wait
			}
		}
		time_flag5_1 = get_Global_Time();
		while (((SPI3->SR) & SPI_SR_BSY))
		{
			if (failsafe_SPI3())
			{
				break; // wait
			}
		}
		SPI3->DR;
		SPI3->SR;
		DMA1_Stream0->CR &= ~DMA_SxCR_EN;

		CS_SPI3_disable();

	}
}
#endif

// TX:
void DMA1_Stream5_IRQHandler(void)
{
	// if stream5 transfer is completed:
	if (DMA1->HISR & DMA_HISR_TCIF5)
	{
		DMA1->HIFCR |= DMA_HIFCR_CTCIF5;

		time_flag5_1 = get_Global_Time();
		while (!((SPI3->SR) & SPI_SR_TXE))
		{
			if (failsafe_SPI3())
			{
				break; // wait
			}
		}
		time_flag5_1 = get_Global_Time();
		while (((SPI3->SR) & SPI_SR_BSY))
		{
			if (failsafe_SPI3())
			{
				break; // wait
			}
		}
		SPI3->DR;
		SPI3->SR;
		DMA1_Stream5->CR &= ~DMA_SxCR_EN;
		CS_SPI3_disable();
		data_sending = false;
	}
}

bool W25Q128_check_if_busy()
{
	if (data_sending) {
		return true;
	}
	if (W25Q128_read_status_register(FLASH_READ_STATUS_REGISTER_1) & FLASH_IS_BUSY_BIT)
	{
		return true;
	}
	return false;
}

static void W25Q128_write_enable()
{
	uint8_t instruction = FLASH_WRITE_ENABLE;

	CS_SPI3_enable();
	SPI3_transmit(&instruction, 1);
	CS_SPI3_disable();

}


void W25Q128_erase(uint8_t instruction, uint32_t address)
{
	uint8_t instruction_and_address[4] = { instruction, address >> 16,address >> 8,address };
	while (W25Q128_check_if_busy()) {
		;//wait
	}
	// must be done before erasing:
	W25Q128_write_enable();


	CS_SPI3_enable();
	SPI3_transmit(instruction_and_address, 4);
	CS_SPI3_disable();
}

void W25Q128_erase_full_chip()
{

	uint8_t instruction = FLASH_ERASE_CHIP;
	while (W25Q128_check_if_busy()) {
		;//wait
	}
	// must be done before some instruction:
	W25Q128_write_enable();


	CS_SPI3_enable();
	SPI3_transmit(&instruction, 1);
	CS_SPI3_disable();
}

uint8_t W25Q128_read_status_register(uint8_t instruction)
{
	uint8_t status;

	CS_SPI3_enable();

	// wait for TXE flag before sending anything
	time_flag5_1 = get_Global_Time();
	while (!((SPI3->SR) & SPI_SR_TXE))
	{
		if (failsafe_SPI3())
		{
			break; // wait
		}
	}
	// send first data - instruction for reading
	SPI3->DR = instruction;
	// wait for TXE flag 
	time_flag5_1 = get_Global_Time();
	while (!((SPI3->SR) & SPI_SR_TXE))
	{
		if (failsafe_SPI3())
		{
			break; // wait
		}
	}

	// now start receiving bytes:
	time_flag5_1 = get_Global_Time();
	while (!((SPI3->SR) & SPI_SR_RXNE))
	{ // wait
		if (failsafe_SPI3())
		{
			break;
		}
	}
	// read first byte (this should be a rubbish):
	SPI3->DR;

	// send 2nd byte (dummy byte):
	SPI3->DR = FLASH_DUMMY_BYTE;

	// wait for TXE flag
	time_flag5_1 = get_Global_Time();
	while (!((SPI3->SR) & SPI_SR_TXE))
	{
		if (failsafe_SPI3())
		{
			break; // wait
		}
	}
	// now start receiving status value
	// continue sending dummy byte to receive status again if RXNE not set after first one (happens)
	time_flag5_1 = get_Global_Time();
	while (!((SPI3->SR) & SPI_SR_RXNE))
	{
		if (failsafe_SPI3())
		{	//if failsafe occure return:
			return 0xFF;
			break;
		}
		// if (!((SPI3->SR) & SPI_SR_TXE)) {
		// 	// send 2nd byte (dummy byte):
		// 	SPI3->DR = FLASH_DUMMY_BYTE;
		// }


	}
	// read status:
	status = SPI3->DR;
	SPI3->SR;

	CS_SPI3_disable();

	return status;
}

uint32_t W25Q128_read_JEDEC_ID()
{
	uint8_t temp[3];
	uint8_t instr = FLASH_JEDEC_ID;

	CS_SPI3_enable();
	SPI3_transmit(&instr, 1);
	SPI3_receive(temp, 3);
	CS_SPI3_disable();

	return (temp[0] << 16 | temp[1] << 8 | temp[2]);
}

//	This function write any number of bytes (assumes that flash is erased):
/**
  * @brief  function to write data to W25Q128 Flash
  * @param memory_address memory address where data will be written
  *	@param data pointer for data to write
  * @param number_of_bytes number of bytes to write
  * @retval none
  * @note	This function check if number of data exceeds given page and if so write 2 or more pages. Function assumes that Flash pages which will be written are erased.
  */
void W25Q128_write_data(uint32_t memory_address, uint8_t* data, int number_of_bytes)
{
	uint8_t instruction_and_address[4] = { FLASH_PAGE_PROGRAM, (memory_address >> 16) & 0xFF,
									 (memory_address >> 8) & 0xFF, (memory_address) & 0xFF };

	uint32_t page_address = memory_address & 0xFFFFFF00;
	uint16_t offset = memory_address & 0xFF;
	//	if data exceed page range write a next page:
	if (number_of_bytes + offset > W25Q128_PAGE_SIZE) {
		W25Q128_write_data(page_address + W25Q128_PAGE_SIZE, &data[W25Q128_PAGE_SIZE - offset], number_of_bytes - (W25Q128_PAGE_SIZE - offset));
		number_of_bytes = W25Q128_PAGE_SIZE - offset;
	}
	while (W25Q128_check_if_busy()) {
		;
	}
	// must be done before writing:
	W25Q128_write_enable();

	CS_SPI3_enable();
	data_sending = true;
	SPI3_transmit(instruction_and_address, 4);
	SPI3_transmit_DMA(data, number_of_bytes);

}

/**
  * @brief  function to write data to W25Q128 Flash
  * @param memory_address memory address where data will be written
  *	@param data pointer for data to write
  * @param number_of_bytes number of bytes to write max. 256 (if memory_address == XXXXX00)
  * @retval none
  * @note	This function doesn't provide any check of size or address. It is up to you to provide coreect inputs. Function assumes that Flash pages which will be written are erased.
  */
void W25Q128_fast_write_data(uint32_t memory_address, uint8_t* data, int number_of_bytes)
{
	uint8_t instruction_and_address[4] = { FLASH_PAGE_PROGRAM, (memory_address >> 16) & 0xFF,
									 (memory_address >> 8) & 0xFF, (memory_address) & 0xFF };

	while (W25Q128_check_if_busy()) {
		;//wait
	}
	// must be done before writing:
	W25Q128_write_enable();

	CS_SPI3_enable();
	data_sending = true;
	SPI3_transmit(instruction_and_address, 4);
	SPI3_transmit_DMA(data, number_of_bytes);
}

/**
  * @brief  function to write data to W25Q128 Flash
  * @param memory_address memory address where data will be written
  *	@param data pointer for data to write
  * @param number_of_bytes number of bytes to write max. 256 (if memory_address == XXXXX00)
  * @retval none
  * @note	This function doesn't provide any check of size or address or busy flag. Function assumes that Flash page which will be written is erased.
  */
void W25Q128_unsafe_write_data(uint32_t memory_address, uint8_t* data, int number_of_bytes)
{
	uint8_t instruction_and_address[4] = { FLASH_PAGE_PROGRAM, (memory_address >> 16) & 0xFF,
									 (memory_address >> 8) & 0xFF, (memory_address) & 0xFF };


	// must be done before writing:
	W25Q128_write_enable();

	CS_SPI3_enable();
	data_sending = true;
	SPI3_transmit(instruction_and_address, 4);
	SPI3_transmit_DMA(data, number_of_bytes);
}

/**
  * @brief  function to modify data storaged in W25Q128 Flash
  * @param memory_address memory address where data will be written
  *	@param new_data npointer for data to write
  * @param number_of_bytes number of bytes to write
  * @retval none
  * @note	Function will copy current data from flash, erase needed space and overwrite specified number of bytes. Then write it to flash module.
  */
void W25Q128_modify_data(uint32_t memory_address, uint8_t* new_data, int number_of_bytes)
{
	uint32_t sector_address = memory_address & 0xFFFFF000;
	uint16_t offset = memory_address & 0xFFF;
	static uint8_t buffer[W25Q128_SECTOR_SIZE];

	while (number_of_bytes > 0) {
		//	copy old data:
		W25Q128_read_data(sector_address, buffer, W25Q128_SECTOR_SIZE);
		//	erase sector:
		W25Q128_erase(FLASH_ERASE_SECTOR_4KB, sector_address);

		//	modify data:
		if (number_of_bytes + offset > W25Q128_SECTOR_SIZE) {
			memmove(&buffer[offset], new_data, W25Q128_SECTOR_SIZE - offset);
			number_of_bytes -= (W25Q128_SECTOR_SIZE - offset);
			new_data += (W25Q128_SECTOR_SIZE - offset);
		}
		else {
			memmove(&buffer[offset], new_data, number_of_bytes);
			number_of_bytes = 0;
		}

		offset = 0;
		//	write modified data:
		for (int i = 0;i < (W25Q128_SECTOR_SIZE / W25Q128_PAGE_SIZE); ++i) {
			W25Q128_fast_write_data(sector_address + i * W25Q128_PAGE_SIZE, &buffer[i * W25Q128_PAGE_SIZE], W25Q128_PAGE_SIZE);
		}

		sector_address += W25Q128_SECTOR_SIZE;

	}
}

/**
  * @brief  function for reading data from W25Q128 Flash
  * @param memory_address memory address from data will be read
  *	@param pointer_for_data tab for read data
  * @param number_of_bytes number of bytes to read
  * @retval none
  * @note	works up to 50 [MHz] for higher frequencies use W25Q128_fast_read_data()
  */
void W25Q128_read_data(uint32_t memory_address, uint8_t* pointer_for_data, int number_of_bytes)
{
	uint8_t instruction_and_address[4] = { FLASH_READ_DATA, (memory_address >> 16) & 0xFF,
										   (memory_address >> 8) & 0xFF, (memory_address) & 0xFF };
	while (W25Q128_check_if_busy()) {
		;//wait
	}

	CS_SPI3_enable();
	SPI3_transmit(instruction_and_address, 4);
	SPI3_receive(pointer_for_data, number_of_bytes);
	CS_SPI3_disable();

}

/**
  * @brief  function for reading data from W25Q128 Flash
  * @param memory_address memory address from data will be read
  *	@param pointer_for_data tab for read data
  * @param number_of_bytes number of bytes to read
  * @retval none
  * @note	works up to 104 [MHz] but before reading it sends one more byte compering to W25Q128_read_data()
  */
void W25Q128_fast_read_data(uint32_t memory_address, uint8_t* pointer_for_data, int number_of_bytes)
{
	uint8_t instruction_and_address[5] = { FLASH_FAST_READ, (memory_address >> 16) & 0xFF,
										   (memory_address >> 8) & 0xFF, (memory_address) & 0xFF, FLASH_DUMMY_BYTE };
	while (W25Q128_check_if_busy()) {
		;//wait
	}

	CS_SPI3_enable();
	SPI3_transmit(instruction_and_address, 5);
	SPI3_receive(pointer_for_data, number_of_bytes);
	CS_SPI3_disable();

}

void flash_save(uint8_t data)
{
	static uint8_t data_ready_to_write = 0;
	flash_write_buffer[flash_write_counter++] = data;

	if (flash_write_counter == W25Q128_PAGE_SIZE)
	{
		data_ready_to_write = 1;
	}
	else if (flash_write_counter == 512)
	{
		data_ready_to_write = 2;
		flash_write_counter = 0;
	}
	// try to save data to flash (if buffer ready and flash not busy):
	if (data_ready_to_write > 0 && !W25Q128_check_if_busy()) {
		W25Q128_unsafe_write_data(flash_global_write_address, &flash_write_buffer[W25Q128_PAGE_SIZE * (data_ready_to_write - 1)], W25Q128_PAGE_SIZE);
		flash_global_write_address += 0x100;
		data_ready_to_write = 0;
	}
}

/**
  * @brief  Function for flushing data to W25Q128 Flash.
  * @retval The remaining bytes to fully fill the page.
  * @note	It will flush data from buffer into memory even is page will not be fully written. Next writing have to take care of remaining space to fill page.
  */
uint16_t flash_flush() {

	if (flash_write_counter != 0) {
		uint16_t counter = flash_write_counter > W25Q128_PAGE_SIZE ? flash_write_counter - W25Q128_PAGE_SIZE : flash_write_counter;
		while (W25Q128_check_if_busy()) {}
		W25Q128_unsafe_write_data(flash_global_write_address, &flash_write_buffer[(flash_write_counter / W25Q128_PAGE_SIZE) * W25Q128_PAGE_SIZE], counter);
		flash_global_write_address += counter;
		flash_write_counter = 0;
		return W25Q128_PAGE_SIZE - counter;
	}
	return 0;
}
uint16_t flash_get_write_buffer_free_space() {
	return sizeof(flash_write_buffer) / sizeof(flash_write_buffer[0]) - flash_write_counter;

}
uint16_t flash_get_write_buffer_size() {
	return sizeof(flash_write_buffer) / sizeof(flash_write_buffer[0]);

}
