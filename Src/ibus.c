/*
 * ibus.c
 *
 *  Created on: 25.01.2021
 */
#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "flash.h"
#include "rx.h"
#include "ibus.h"

volatile uint8_t rxBuf[32];
static uint8_t rxindex = 0;

void DMA2_Stream5_IRQHandler(void)
{
	// if channel4 transfer is completed:

	// if channel5 transfer is completed:
	if (DMA2->HISR & DMA_HISR_TCIF5)
	{
		DMA2->HIFCR |= DMA_HIFCR_CTCIF5;
		DMA2_Stream5->CR &= ~DMA_SxCR_EN;

		receiver.new_data_flag = true;
	}
}
// for DMA:
void USART1_IRQHandler(void)
{
	// RDR not empty flag:
	if (USART_SR_RXNE & USART1->SR)
	{
		//	read actual value of I-BUS (Interrupt flag will be automatically removed):
		rxBuf[rxindex] = USART1->DR;
		if (rxindex == 1 && rxBuf[rxindex] == 0x40)
		{
			// block USART1 interrupt until DMA reading finish and data are processed:
			USART1->CR1 &= ~USART_CR1_RXNEIE;
			DMA2_Stream5->CR |= DMA_SxCR_EN;
		}
		else if (rxindex == 0 && rxBuf[rxindex] != 0x20)
		{
			rxindex = 0;
		}
		else if (rxindex == 1 && rxBuf[rxindex] != 0x40)
		{
			rxindex = 0;
		}
		else
		{
			// if header is right increase rxindex
			rxindex++;
		}
	}
	// idle detection flag:
	if (0 != (USART_SR_IDLE & USART1->SR))
	{
		USART1->DR;

		if (!receiver.new_data_flag)
		{
			USART1->CR1 |= USART_CR1_RXNEIE;
			DMA2_Stream5->CR &= ~DMA_SxCR_EN;
		}
	}
}

/**
 *@return true if frame was read succesfully and new data are written to channels, false otherwise
*/
bool Ibus_save(rx_t* receiver)
{
	if (receiver->new_data_flag) {
		// checking checksum: 
		uint16_t checksum = 0xFFFF;
		for (int8_t i = 0; i < 30; i++)
		{
			checksum -= rxBuf[i];
		}
		// if checksum correct rewrite data:
		if (checksum == ((rxBuf[31] << 8) + rxBuf[30]))
		{
			// save roll, pitch, yaw, throttle (these channels will be interpolated)
			for (uint8_t i = 0; i < 4; i++)
			{
				receiver->channels_raw[i] = (rxBuf[2 * (i + 1) + 1] << 8) + rxBuf[2 * (i + 1)];
			}
			// save rest of the channels:
			for (int8_t i = 4; i < CHANNELS; i++)
			{
				receiver->channels[i] = (rxBuf[2 * (i + 1) + 1] << 8) + rxBuf[2 * (i + 1)];
			}
			//	unlock receiving new RX data:
			rxindex = 0;
			USART1->CR1 |= USART_CR1_RXNEIE;
			return true;
		}
	}
	//	unlock receiving new RX data:
	rxindex = 0;
	USART1->CR1 |= USART_CR1_RXNEIE;
	return false;

}
