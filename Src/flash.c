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
#include "flash.h"

static void CS_flash_disable();
static void CS_flash_enable();
static void SPI3_enable();
static void SPI3_disable();
static void SPI_transmit(uint8_t *data, uint8_t size);
static void SPI_transmit_DMA(uint8_t *data, int size);
static void SPI_receive(uint8_t *data, int size);
static void SPI_receive_DMA(uint8_t *data, int size);
static void flash_write_enable();
static bool failsafe_SPI();

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
			if (failsafe_SPI())
			{
				break; // wait
			}
		}
		time_flag5_1 = get_Global_Time();
		while (((SPI3->SR) & SPI_SR_BSY))
		{
			if (failsafe_SPI())
			{
				break; // wait
			}
		}
		SPI3->DR;
		SPI3->SR;
		DMA1_Stream0->CR &= ~DMA_SxCR_EN;

		CS_flash_disable();
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
			if (failsafe_SPI())
			{
				break; // wait
			}
		}
		time_flag5_1 = get_Global_Time();
		while (((SPI3->SR) & SPI_SR_BSY))
		{
			if (failsafe_SPI())
			{
				break; // wait
			}
		}
		SPI3->DR;
		SPI3->SR;
		DMA1_Stream5->CR &= ~DMA_SxCR_EN;
		DMA1_Stream0->CR &= ~DMA_SxCR_EN;

		CS_flash_disable();
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

static void CS_flash_enable()
{
	GPIOB->BSRRH |= GPIO_BSRR_BS_3;
}

static void CS_flash_disable()
{
	GPIOB->BSRRL |= GPIO_BSRR_BS_3;
}

static void SPI3_enable()
{
	SPI3->CR1 |= SPI_CR1_SPE; //	enabling SPI3
}

static void SPI3_disable()
{
	SPI3->CR1 &= ~SPI_CR1_SPE; //	disabling SPI3
}

static void SPI_transmit(uint8_t *data, uint8_t size)
{

	int i = 0;

	time_flag5_1 = get_Global_Time();
	while (!((SPI3->SR) & SPI_SR_TXE))
	{
		if (failsafe_SPI())
		{
			break; // wait
		}
	}
	SPI3->DR = data[i]; // first data
	i++;

	while (i < size)
	{
		time_flag5_1 = get_Global_Time();
		while (!((SPI3->SR) & SPI_SR_TXE))
		{
			if (failsafe_SPI())
			{
				break; // wait
			}
		}
		SPI3->DR = data[i]; // second and following data sending as soon as TX flag is set
		i++;
	}

	time_flag5_1 = get_Global_Time();
	while (!((SPI3->SR) & SPI_SR_TXE))
	{
		if (failsafe_SPI())
		{
			break; // wait
		}
	}
	time_flag5_1 = get_Global_Time();
	while (((SPI3->SR) & SPI_SR_BSY))
	{
		if (failsafe_SPI())
		{
			break; // wait
		}
	}
	SPI3->DR;
	SPI3->SR;
}

static void SPI_transmit_DMA(uint8_t *data, int size)
{

	DMA1_Stream5->M0AR = (uint32_t)(data);
	DMA1_Stream5->NDTR = size;
	DMA1_Stream5->CR |= DMA_SxCR_EN;
}

static void SPI_receive(uint8_t *data, int size)
{

	while (size > 0)
	{

		time_flag5_1 = get_Global_Time();
		while (!((SPI3->SR) & SPI_SR_TXE))
		{
			if (failsafe_SPI())
			{
				break; // wait
			}
		}
		SPI3->DR = FLASH_DUMMY_BYTE; // send anything IMPORTANT!
		time_flag5_1 = get_Global_Time();
		while (!((SPI3->SR) & SPI_SR_RXNE))
		{
			if (failsafe_SPI())
			{
				break; // wait
			}
		}
		*data++ = SPI3->DR;
		size--;
	}

	// wait for TXE flag
	time_flag5_1 = get_Global_Time();
	while (!((SPI3->SR) & SPI_SR_TXE))
	{
		if (failsafe_SPI())
		{
			break; // wait
		}
	}
	// wait for BSY flag
	time_flag5_1 = get_Global_Time();
	while (((SPI3->SR) & SPI_SR_BSY))
	{
		if (failsafe_SPI())
		{
			break; // wait
		}
	}

	SPI3->DR;
	SPI3->SR;
}

#if !defined(USE_I2C1)
static void SPI_receive_DMA(uint8_t *data, int size)
{

	DMA1_Stream0->M0AR = (uint32_t)(data);
	DMA1_Stream0->NDTR = size;
	DMA1_Stream0->CR |= DMA_SxCR_EN;

	DMA1_Stream5->M0AR = (uint32_t)(data);
	DMA1_Stream5->NDTR = size;
	DMA1_Stream5->CR |= DMA_SxCR_EN;
}
#endif

void flash_SPI_write(uint8_t instruction, uint8_t *data, uint8_t size)
{

	// must be done before writing:
	flash_write_enable();

	SPI3_enable();
	CS_flash_enable();

	SPI_transmit(&instruction, 1);
	SPI_transmit(data, size);

	CS_flash_disable();
	SPI3_disable();
}

void flash_SPI_write_DMA(uint8_t instruction, uint8_t *data, int size)
{

	// must be done before writing:
	flash_write_enable();

	SPI3_enable();
	CS_flash_enable();

	SPI_transmit(&instruction, 1);
	SPI_transmit_DMA(data, size);
}

void flash_SPI_read(uint8_t instruction, uint8_t *memory_address,
					int number_of_bytes)
{
	SPI3_enable();
	CS_flash_enable();

	SPI_transmit(&instruction, 1);
	SPI_transmit(memory_address, 3);
	SPI_receive(memory_address, number_of_bytes);

	CS_flash_disable();
	SPI3_disable();
}

#if !defined(USE_I2C1)
void flash_SPI_read_DMA(uint8_t instruction, uint8_t *data, int size)
{

	SPI3_enable();
	CS_flash_enable();

	SPI_transmit(&instruction, 1);
	SPI_receive_DMA(data, size);
}
#endif

static void flash_write_enable()
{

	uint8_t instruction = FLASH_WRITE_ENABLE;

	SPI3_enable();
	CS_flash_enable();

	SPI_transmit(&instruction, 1);

	CS_flash_disable();
	SPI3_disable();
}

void flash_erase(uint8_t instruction, uint8_t *address)
{

	// must be done before erasing:
	flash_write_enable();

	SPI3_enable();
	CS_flash_enable();

	SPI_transmit(&instruction, 1);
	SPI_transmit(address, 3);

	CS_flash_disable();
	SPI3_disable();
}

void flash_full_chip_erase()
{

	uint8_t instruction = FLASH_CHIP_ERASE;

	// must be done before some instruction:
	flash_write_enable();

	SPI3_enable();
	CS_flash_enable();

	SPI_transmit(&instruction, 1);

	CS_flash_disable();
	SPI3_disable();
}

uint8_t flash_read_status_register(uint8_t instruction)
{
	uint8_t status;

	SPI3_enable();
	CS_flash_enable();

	// wait for TXE flag before sending anything
	time_flag5_1 = get_Global_Time();
	while (!((SPI3->SR) & SPI_SR_TXE))
	{
		if (failsafe_SPI())
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
		if (failsafe_SPI())
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
		if (failsafe_SPI())
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
		if (failsafe_SPI())
		{
			break; // wait
		}
	}
	// now start receiving status value:
	time_flag5_1 = get_Global_Time();
	while (!((SPI3->SR) & SPI_SR_RXNE))
	{
		if (failsafe_SPI())
		{
			break; // wait
		}
	}
	// read status:
	status = SPI3->DR;
	SPI3->SR;

	CS_flash_disable();
	SPI3_disable();

	return status;
}

void flash_read_unique_ID(uint8_t *memory_address)
{
	// save in memory_address 64-bit unique ID as 8 bytes
	uint8_t temp[9];
	flash_read_data(FLASH_READ_UNIQUE_ID, 0x000000, temp, 8);
	memmove(memory_address, &temp[1], sizeof(temp[1]) * 8);
}

static bool failsafe_SPI()
{

	//	waiting as Data will be sent or failsafe (if set time passed)
	if ((get_Global_Time() - time_flag5_1) >= SEC_TO_US(TIMEOUT_VALUE))
	{
		FailSafe_status = SPI_FLASH_ERROR;
		EXTI->SWIER |= EXTI_SWIER_SWIER15;
		return true;
	}
	return false;
}

void flash_save_data(uint8_t instruction, uint32_t memory_address,
					 uint8_t *data, int number_of_bytes)
{

	uint8_t memory_address_tab[3] = {(memory_address >> 16) & 0xFF,
									 (memory_address >> 8) & 0xFF, (memory_address)&0xFF};

	// must be done before writing:
	flash_write_enable();

	SPI3_enable();
	CS_flash_enable();

	SPI_transmit(&instruction, 1);
	SPI_transmit(memory_address_tab, 3);
	SPI_transmit_DMA(data, number_of_bytes);
}

void flash_read_data(uint8_t instruction, uint32_t memory_address,
					 uint8_t *data, int number_of_bytes)
{

	uint8_t memory_address_tab[3] = {(memory_address >> 16) & 0xFF,
									 (memory_address >> 8) & 0xFF, (memory_address)&0xFF};

	SPI3_enable();
	CS_flash_enable();

	SPI_transmit(&instruction, 1);
	SPI_transmit(memory_address_tab, 3);
	SPI_receive(data, number_of_bytes);

	CS_flash_disable();
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

void Gyro_Acc_save_to_flash(float *not_filtered)
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
