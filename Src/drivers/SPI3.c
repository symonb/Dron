#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include <math.h>
#include "SPI3.h"



void CS_SPI3_enable()
{
    GPIOB->BSRR |= GPIO_BSRR_BR_3;
}

void CS_SPI3_disable()
{
    GPIOB->BSRR |= GPIO_BSRR_BS_3;
}

void SPI3_enable()
{
    SPI3->CR1 |= SPI_CR1_SPE; //	enabling SPI3
}

void SPI3_disable()
{
    SPI3->CR1 &= ~SPI_CR1_SPE; //	disabling SPI3
}

void SPI3_transmit(const uint8_t* data, uint8_t size)
{

    uint8_t  i = 0;

    while (i < size)
    {
        time_flag5_1 = get_Global_Time();
        while (!((SPI3->SR) & SPI_SR_TXE))
        {
            if (failsafe_SPI3())
            {
                break; // wait
            }
        }
        SPI3->DR = data[i];  // data sending as soon as TX flag is set
        i++;
    }

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
}

void SPI3_transmit_DMA(uint8_t* data, int size)
{
    DMA1_Stream5->M0AR = (uint32_t)(data);
    DMA1_Stream5->NDTR = size;
    DMA1_Stream5->CR |= DMA_SxCR_EN;
}

void SPI3_receive(uint8_t* data, int size)
{
    while (size > 0)
    {
        time_flag5_1 = get_Global_Time();
        while (!((SPI3->SR) & SPI_SR_TXE))
        {
            if (failsafe_SPI3())
            {
                break; // wait
            }
        }
        SPI3->DR = 0xFF; // send anything
        time_flag5_1 = get_Global_Time();
        while (!((SPI3->SR) & SPI_SR_RXNE))
        {
            if (failsafe_SPI3())
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
        if (failsafe_SPI3())
        {
            break; // wait
        }
    }
    // wait for BSY flag
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
}


void SPI3_receive_DMA(uint8_t* data, int size)
{
    // set parameters of reception:
    DMA1_Stream0->M0AR = (uint32_t)(data);
    DMA1_Stream0->NDTR = size;
    DMA1_Stream0->CR |= DMA_SxCR_EN;
    // set anything for transmission:
    DMA1_Stream5->M0AR = (uint32_t)(data);
    DMA1_Stream5->NDTR = size;
    DMA1_Stream5->CR |= DMA_SxCR_EN;
}

bool failsafe_SPI3()
{
    //	waiting as Data will be sent or failsafe (if set time passed)
    if ((get_Global_Time() - time_flag5_1) >= SEC_TO_US(TIMEOUT_VALUE))
    {
        FailSafe_status = FS_SPI_FLASH_ERROR;
        EXTI->SWIER |= EXTI_SWIER_SWIER15;
        return true;
    }
    return false;
}