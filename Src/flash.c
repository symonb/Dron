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
		data_sending = false;

		CS_SPI3_disable();
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
	// wait for TXE flag and send 2nd byte - anything
	time_flag5_1 = get_Global_Time();
	while (!((SPI3->SR) & SPI_SR_TXE))
	{
		if (failsafe_SPI3())
		{
			break; // wait
		}
	}
	// send 2nd byte (dummy):
	SPI3->DR = FLASH_DUMMY_BYTE;

	// now start receiving bytes:
	time_flag5_1 = get_Global_Time();
	while (!((SPI3->SR) & SPI_SR_RXNE))
	{
		if (failsafe_SPI3())
		{
			break; // wait
		}
	}
	// read first byte this is rubbish:
	SPI3->DR;

	// wait for TXE flag
	time_flag5_1 = get_Global_Time();
	while (!((SPI3->SR) & SPI_SR_TXE))
	{
		if (failsafe_SPI3())
		{
			break; // wait
		}
	}
	// now start receiving status value:
	time_flag5_1 = get_Global_Time();
	while (!((SPI3->SR) & SPI_SR_RXNE))
	{
		if (failsafe_SPI3())
		{
			break; // wait
		}
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

//	This function doesn't provide any check of size or address - it is up to you:
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

void W25Q128_modify_data(uint32_t memory_address, uint8_t* new_data, int number_of_bytes)
{
	uint32_t sector_address = memory_address & 0xFFFFF000;
	uint16_t offset = memory_address & 0xFFF;
	uint8_t buffer[W25Q128_SECTOR_SIZE];

	while (number_of_bytes > 0) {
		//	copy old data:
		W25Q128_read_data(sector_address, buffer, W25Q128_SECTOR_SIZE);
		//	erase sector:
		W25Q128_erase(FLASH_ERASE_SECTOR_4KB, sector_address);
		//	modify data:
		if (number_of_bytes + offset > W25Q128_SECTOR_SIZE) {
			memcpy(&buffer[offset], new_data, W25Q128_SECTOR_SIZE - offset);
			number_of_bytes -= (W25Q128_SECTOR_SIZE - offset);
			new_data += (W25Q128_SECTOR_SIZE - offset);
		}
		else {
			memcpy(&buffer[offset], new_data, number_of_bytes);
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

//	works up to 50 [MHz] for higher use W25Q128_fast_read_data()
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

//	works up to 104 [MHz]
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

void flash_add_data_to_save(uint8_t data)
{

	flash_write_buffer[flash_write_counter] = data;

	flash_write_counter++;

	if (flash_write_counter == W25Q128_PAGE_SIZE)
	{
		W25Q128_fast_write_data(flash_global_write_address,
			flash_write_buffer, W25Q128_PAGE_SIZE);
		flash_global_write_address += 0x100;
	}
	else if (flash_write_counter == 512)
	{
		W25Q128_fast_write_data(flash_global_write_address,
			&flash_write_buffer[W25Q128_PAGE_SIZE], W25Q128_PAGE_SIZE);
		flash_global_write_address += 0x100;
		flash_write_counter = 0;
	}
}

void Gyro_Acc_save_to_flash(float* not_filtered)
{
#if defined(USE_FLASH_BLACKBOX)

	if (BLACKBOX_STATUS == BLACKBOX_COLLECT_DATA)
	{

#if defined(BLACKBOX_SAVE_EULER_ANGLES)

		flash_add_data_to_save(
			((int16_t)(global_euler_angles.roll * 150) >> 8) & 0xFF);
		flash_add_data_to_save((int16_t)(global_euler_angles.roll * 150) & 0xFF);

		flash_add_data_to_save(
			((int16_t)(global_euler_angles.pitch * 150) >> 8) & 0xFF);
		flash_add_data_to_save((int16_t)(global_euler_angles.pitch * 150) & 0xFF);

		flash_add_data_to_save(
			((int16_t)(global_euler_angles.yaw * 150) >> 8) & 0xFF);
		flash_add_data_to_save((int16_t)(global_euler_angles.yaw * 150) & 0xFF);
#endif

#if defined(BLACKBOX_SAVE_SET_ANGLES)
		for (uint8_t i = 0; i < 3; i++)
		{
			flash_add_data_to_save(((int16_t)(global_variable_monitor[i] * 150) >> 8) & 0xFF);
			flash_add_data_to_save((int16_t)(global_variable_monitor[i] * 150) & 0xFF);
		}

#endif

#if defined(BLACKBOX_SAVE_FILTERED_GYRO_AND_ACC)
		for (uint8_t i = 0; i < 6; i++)
		{
			flash_add_data_to_save((Gyro_Acc[i] >> 8) & 0xFF);
			flash_add_data_to_save(Gyro_Acc[i] & 0xFF);
		}
#elif defined(BLACKBOX_SAVE_FILTERED_GYRO)
		for (uint8_t i = 0; i < 3; i++)
		{
			flash_add_data_to_save((Gyro_Acc[i] >> 8) & 0xFF);
			flash_add_data_to_save(Gyro_Acc[i] & 0xFF);
		}
#elif defined(BLACKBOX_SAVE_FILTERED_ACC)
		for (uint8_t i = 3; i < 6; i++)
		{
			flash_add_data_to_save((Gyro_Acc[i] >> 8) & 0xFF);
			flash_add_data_to_save(Gyro_Acc[i] & 0xFF);
		}
#endif

#if defined(BLACKBOX_SAVE_RAW_GYRO_AND_ACC)

		for (uint8_t i = 0; i < 6; i++)
		{
			flash_add_data_to_save(((int16_t)not_filtered[i] >> 8) & 0xFF);
			flash_add_data_to_save((int16_t)not_filtered[i] & 0xFF);
		}
#elif defined(BLACKBOX_SAVE_RAW_GYRO)
		for (uint8_t i = 0; i < 3; i++)
		{
			flash_add_data_to_save(((int16_t)not_filtered[i] >> 8) & 0xFF);
			flash_add_data_to_save((int16_t)not_filtered[i] & 0xFF);
		}
#elif defined(BLACKBOX_SAVE_RAW_ACC)
		for (uint8_t i = 3; i < 6; i++)
		{
			flash_add_data_to_save(((int16_t)not_filtered[i] >> 8) & 0xFF);
			flash_add_data_to_save((int16_t)not_filtered[i] & 0xFF);
		}
#endif

#if defined(BLACKBOX_SAVE_STICKS)
		for (uint8_t i = 0; i < 2; i++)
		{
			flash_add_data_to_save(((int16_t)channels[i] >> 8) & 0xFF);
			flash_add_data_to_save((int16_t)channels[i] & 0xFF);
		}
		flash_add_data_to_save(((int16_t)channels[3] >> 8) & 0xFF);
		flash_add_data_to_save((int16_t)channels[3] & 0xFF);
#endif
	}

#endif
}
