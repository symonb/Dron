/*
 * telemetry.c
 *
 *  Created on: 17.03.2021
 *      Author: symon
 */
#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "telemetry.h"
#include "flash.h"

uint8_t table_of_bytes_to_sent[2 * ALL_ELEMENTS_TO_SEND + 4];
static int32_t txTransmitted;
static int32_t txSize;
static uint8_t bufor[50];

void DMA2_Stream7_IRQHandler(void)
{
	// if channel6 transfer USART6 is completed:
	if (DMA2->HISR & DMA_HISR_TCIF7)
	{
		DMA2->HIFCR |= DMA_HIFCR_CTCIF7;
		DMA2_Stream7->CR &= ~DMA_SxCR_EN;
		transmitting_is_Done = 1;
	}
	//	//if channel3 transfer I2C is completed:
	//	if (DMA1->ISR & DMA_ISR_TCIF3) {
	//		DMA1->IFCR |= DMA_IFCR_CTCIF3;
	//		I2C1_read_write_flag = 1;
	//
	//		DMA1_Channel3->CCR &= ~DMA_CCR_EN;
	//		if (ibus_received == 0) {
	//			USART2->CR1 |= USART_CR1_RXNEIE;
	//			//USART2->CR1 |= USART_CR1_IDLEIE;
	//		}
	//	}
}

void USART6_IRQHandler(void)
{

	if (USART_SR_RXNE & USART6->SR)
	{
		//	read actual value (flag will be automatically removed):
		uint8_t i = 0;

		bufor[i] = USART6->DR;
		switch (bufor[i])
		{
		case 0:
			break;
		case 1:
			break;
		case 2:
			break;
		case 3:
			break;
		case 9:
			break;
		default:
			break;
		}

		i++;
		if (i >= 50)
		{
			i = 0;
		}
	}
	if (USART6->SR & USART_SR_IDLE)
	{
		USART6->DR;						  // If not read usart will crush
		DMA2_Stream7->CR &= ~DMA_SxCR_EN; /* Disable DMA on stream 6 - trigers dma TC */
	}

	// TRANSMISJA:

	//	check if interrupt was generated by right flag:
	if ((USART_CR1_TXEIE & USART6->CR1) && (USART_SR_TXE & USART6->SR))
	{
		//	transmit data:
		if ((0 < txSize) && (txTransmitted < txSize))
		{
			USART6->DR = table_of_bytes_to_sent[txTransmitted];
			txTransmitted += 1;
		}
		// if everything is transmitted, unable transmission interrupts:
		if (txTransmitted == txSize)
		{
			USART6->CR1 &= ~USART_CR1_TXEIE;
			transmitting_is_Done = 1;
		}
	}
}

void USART3_IRQHandler(void)
{
	if (USART3->SR & USART_SR_IDLE)
	{
		USART3->DR; // If not read usart will crush
		// DMA1_StreamX->CR &= ~DMA_SxCR_EN;     // Disable DMA on stream X
	}
}

void print(const uint16_t x[], uint8_t data_to_send)
{
	uint16_t sum = 0;

	table_of_bytes_to_sent[0] = 0x20;
	table_of_bytes_to_sent[1] = 0x40;
	for (int i = 0; i < data_to_send; i++)
	{
		table_of_bytes_to_sent[2 * i + 2] = x[i] >> 8;
		table_of_bytes_to_sent[2 * i + 3] = x[i];
		sum += x[i];
	}
	table_of_bytes_to_sent[2 * data_to_send + 2] = sum >> 8;
	table_of_bytes_to_sent[2 * data_to_send + 3] = sum;

	txSize = 2 * data_to_send + 4;

	txTransmitted = 0;
	transmitting_is_Done = 0;

	DMA2_Stream7->M0AR = (uint32_t)(table_of_bytes_to_sent);
	DMA2_Stream7->NDTR = 2 * ALL_ELEMENTS_TO_SEND + 4;

	DMA2_Stream7->CR |= DMA_SxCR_EN; // DMA option
	// USART6->CR1 |= USART_CR1_TXEIE;			//Interrupt option
}

void print_flash(uint8_t data_pack_size)
{

	uint32_t read_address;
	uint8_t temporary_array[512];
	int16_t checksum;
	int16_t i, j, k;

	read_address = FLASH_READ_BLOCK_0;
	checksum = 0;
	i = 0;
	j = 0;
	k = 0;

	if (0 != transmitting_is_Done && read_address < flash_global_write_address)
	{
		toggle_RED_LED();
		toggle_BLUE_LED();
		// read data from flash:
		W25Q128_read_data(read_address, flash_read_buffer,
			256);
		read_address += 0x100;
		transmitting_is_Done = 0;

		if (data_pack_size == 1)
		{
			// if you sending only 1 value (2 bytes) it is not worth to send frame with 6 byte

			DMA2_Stream7->M0AR = (uint32_t)(flash_read_buffer);
			DMA2_Stream7->NDTR = 256;
			DMA2_Stream7->CR |= DMA_SxCR_EN;
		}

		else
		{

			while (j < 256)
			{
				if (k == 0)
				{
					// header=32767 it is unlikely to have this value to transmit
					temporary_array[i] = 0x7F;
					temporary_array[i + 1] = 0xFF;
					i += 2;
				}
				while (k < data_pack_size * 2 && j < 256)
				{
					checksum += flash_read_buffer[j];
					temporary_array[i] = flash_read_buffer[j];
					i++;
					j++;
					k++;
				}
				if (k >= data_pack_size * 2)
				{
					k = 0;
					temporary_array[i] = (checksum >> 8) & 0xFF;
					temporary_array[i + 1] = checksum & 0xFF;
					checksum = 0;
					i += 2;
				}
			}

			DMA2_Stream7->M0AR = (uint32_t)(temporary_array);
			DMA2_Stream7->NDTR = i;
			DMA2_Stream7->CR |= DMA_SxCR_EN;

			j = 0;
			i = 0;
		}
	}
	else if (read_address >= flash_global_write_address)
	{
		delay_mili(1000);
		toggle_BLUE_LED();
		toggle_RED_LED();
	}
	else
	{
		delay_mili(100);
	}
}

void print_blackbox()
{
	turn_OFF_RED_LED();
	turn_OFF_BLUE_LED();
	delay_mili(3000);
	turn_ON_RED_LED();
	delay_mili(1000);
	turn_ON_BLUE_LED();
	delay_mili(1000);
	turn_OFF_RED_LED();
	turn_OFF_BLUE_LED();
	delay_mili(1000);
	turn_ON_RED_LED();
	turn_ON_BLUE_LED();
	delay_mili(1000);
	turn_OFF_RED_LED();
	turn_OFF_BLUE_LED();
	delay_mili(1000);
	turn_ON_RED_LED();
	turn_ON_BLUE_LED();
	delay_mili(2000);

	print_flash(9);
}