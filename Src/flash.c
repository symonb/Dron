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


static void flash_write_enable();


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
		SPI3_disable();
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
		DMA1_Stream0->CR &= ~DMA_SxCR_EN;

		CS_SPI3_disable();
		SPI3_disable();
	}
}

bool is_flash_busy()
{
	if (flash_read_status_register(FLASH_READ_STATUS_REGISTER_1) & FLASH_IS_BUSY_BIT)
	{
		return true;
	}
	return false;
}


void flash_SPI_write(uint8_t instruction, uint8_t* data, uint8_t size)
{

	// must be done before writing:
	flash_write_enable();

	SPI3_enable();
	CS_SPI3_enable();

	SPI3_transmit(&instruction, 1);
	SPI3_transmit(data, size);

	CS_SPI3_disable();
	SPI3_disable();
}

void flash_SPI_write_DMA(uint8_t instruction, uint8_t* data, int size)
{

	// must be done before writing:
	flash_write_enable();

	SPI3_enable();
	CS_SPI3_enable();

	SPI3_transmit(&instruction, 1);
	SPI3_transmit_DMA(data, size);
}

void flash_SPI_read(uint8_t instruction, uint8_t* memory_address,
	int number_of_bytes)
{
	SPI3_enable();
	CS_SPI3_enable();

	SPI3_transmit(&instruction, 1);
	SPI3_transmit(memory_address, 3);
	SPI3_receive(memory_address, number_of_bytes);

	CS_SPI3_disable();
	SPI3_disable();
}

#if !defined(USE_I2C1)
void flash_SPI_read_DMA(uint8_t instruction, uint8_t* data, int size)
{

	SPI3_enable();
	CS_SPI3_enable();

	SPI3_transmit(&instruction, 1);
	SPI3_receive_DMA(data, size);
}
#endif

static void flash_write_enable()
{

	uint8_t instruction = FLASH_WRITE_ENABLE;

	SPI3_enable();
	CS_SPI3_enable();

	SPI3_transmit(&instruction, 1);

	CS_SPI3_disable();
	SPI3_disable();
}

void flash_erase(uint8_t instruction, uint8_t* address)
{

	// must be done before erasing:
	flash_write_enable();

	SPI3_enable();
	CS_SPI3_enable();

	SPI3_transmit(&instruction, 1);
	SPI3_transmit(address, 3);

	CS_SPI3_disable();
	SPI3_disable();
}

void flash_full_chip_erase()
{

	uint8_t instruction = FLASH_CHIP_ERASE;

	// must be done before some instruction:
	flash_write_enable();

	SPI3_enable();
	CS_SPI3_enable();

	SPI3_transmit(&instruction, 1);

	CS_SPI3_disable();
	SPI3_disable();
}

uint8_t flash_read_status_register(uint8_t instruction)
{
	uint8_t status;

	SPI3_enable();
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
	SPI3_disable();

	return status;
}

void flash_read_unique_ID(uint8_t* memory_address)
{
	// save in memory_address 64-bit unique ID as 8 bytes
	uint8_t temp[9];
	flash_read_data(FLASH_READ_UNIQUE_ID, 0x000000, temp, 8);
	memmove(memory_address, &temp[1], sizeof(temp[1]) * 8);
}



void flash_save_data(uint8_t instruction, uint32_t memory_address,
	uint8_t* data, int number_of_bytes)
{

	uint8_t memory_address_tab[3] = { (memory_address >> 16) & 0xFF,
									 (memory_address >> 8) & 0xFF, (memory_address) & 0xFF };

	// must be done before writing:
	flash_write_enable();

	SPI3_enable();
	CS_SPI3_enable();

	SPI3_transmit(&instruction, 1);
	SPI3_transmit(memory_address_tab, 3);
	SPI3_transmit_DMA(data, number_of_bytes);
}

void flash_read_data(uint8_t instruction, uint32_t memory_address,
	uint8_t* data, int number_of_bytes)
{

	uint8_t memory_address_tab[3] = { (memory_address >> 16) & 0xFF,
									 (memory_address >> 8) & 0xFF, (memory_address) & 0xFF };

	SPI3_enable();
	CS_SPI3_enable();

	SPI3_transmit(&instruction, 1);
	SPI3_transmit(memory_address_tab, 3);
	SPI3_receive(data, number_of_bytes);

	CS_SPI3_disable();
	SPI3_disable();
}

void flash_add_data_to_save(uint8_t data)
{

	flash_write_buffer[flash_write_counter] = data;

	flash_write_counter++;

	if (flash_write_counter == 256)
	{
		flash_save_data(FLASH_PAGE_PROGRAM, flash_global_write_address,
			flash_write_buffer, 256);
		flash_global_write_address += 0x100;
	}
	else if (flash_write_counter == 512)
	{
		flash_save_data(FLASH_PAGE_PROGRAM, flash_global_write_address,
			&flash_write_buffer[256], 256);
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
