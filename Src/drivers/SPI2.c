#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include <math.h>
#include "SPI2.h"


void SPI2_enable()
{
    SPI2->CR1 |= SPI_CR1_SPE;
}

void SPI2_disable()
{
    SPI2->CR1 &= ~SPI_CR1_SPE;
}

void SPI2_CS_enable()
{
    GPIOB->BSRR |= GPIO_BSRR_BR_12;
}

void SPI2_CS_disable()
{
    GPIOB->BSRR |= GPIO_BSRR_BS_12;
}

void SPI2_transmit(const uint8_t* data, uint16_t size)
{
    uint16_t i = 0;

    while (i < size)
    {
        time_flag6_1 = get_Global_Time();
        while (!((SPI2->SR) & SPI_SR_TXE))
        {
            if (SPI2_failsafe())
            {
                break; // wait
            }
        }
        SPI2->DR = data[i]; //  data sending as soon as TX flag is set
        i++;
    }

    time_flag6_1 = get_Global_Time();
    while (!((SPI2->SR) & SPI_SR_TXE))
    {
        if (SPI2_failsafe())
        {
            break; // wait
        }
    }

    time_flag6_1 = get_Global_Time();
    while (((SPI2->SR) & SPI_SR_BSY))
    {
        if (SPI2_failsafe())
        {
            break; // wait
        }
    }
    SPI2->DR;
    SPI2->SR;
}

void SPI2_transmit_one(uint8_t data)
{
    time_flag6_1 = get_Global_Time();
    while (!((SPI2->SR) & SPI_SR_TXE))
    {
        if (SPI2_failsafe())
        {
            break; // wait
        }
    }

    // transmit data:
    SPI2->DR = data;

    time_flag6_1 = get_Global_Time();
    while (!((SPI2->SR) & SPI_SR_TXE))
    {
        if (SPI2_failsafe())
        {
            break; // wait
        }
    }
    time_flag6_1 = get_Global_Time();
    while (((SPI2->SR) & SPI_SR_BSY))
    {
        if (SPI2_failsafe())
        {
            break; // wait
        }
    }
    SPI2->DR;
    SPI2->SR;
}

void SPI2_transmit_DMA(uint8_t* data, uint16_t size)
{
    DMA1_Stream4->M0AR = (uint32_t)(data);
    DMA1_Stream4->NDTR = size;
    DMA1_Stream4->CR |= DMA_SxCR_EN;
}

void SPI2_receive(uint8_t* data, uint16_t size)
{
    while (size > 0)
    {
        time_flag6_1 = get_Global_Time();
        while (!((SPI2->SR) & SPI_SR_TXE))
        {
            if (SPI2_failsafe())
            {
                break; // wait
            }
        }
        SPI2->DR = 0xFF; // send anything
        time_flag6_1 = get_Global_Time();
        while (!((SPI2->SR) & SPI_SR_RXNE))
        {
            if (SPI2_failsafe())
            {
                break;
            }
        }
        *data++ = SPI2->DR;
        size--;
    }

    // wait for TXE flag
    time_flag6_1 = get_Global_Time();
    while (!((SPI2->SR) & SPI_SR_TXE))
    {
        if (SPI2_failsafe())
        {
            break;
        }
    }
    // wait for BSY flag
    time_flag6_1 = get_Global_Time();
    while (((SPI2->SR) & SPI_SR_BSY))
    {// wait
        if (SPI2_failsafe())
        {
            break;
        }
    }
    SPI2->DR;
    SPI2->SR;
}

void SPI2_receive_one(uint8_t* data)
{
    time_flag6_1 = get_Global_Time();
    while (!((SPI2->SR) & SPI_SR_TXE))
    { // wait
        if (SPI2_failsafe())
        {
            break;
        }
    }
    SPI2->DR = 0xFF; // send anything

    time_flag6_1 = get_Global_Time();
    while (!((SPI2->SR) & SPI_SR_RXNE))
    { // wait
        if (SPI2_failsafe())
        {
            break;
        }
    }
    *data = SPI2->DR;
}

bool SPI2_failsafe()
{
    //	waiting as Data will be sent or failsafe if set time passed
    if ((get_Global_Time() - time_flag6_1) >= SEC_TO_US(SPI_TIMEOUT_VALUE))
    {
        FailSafe_status = FAILSAFE_SPI_OSD_ERROR;
        EXTI->SWIER |= EXTI_SWIER_SWIER15;
        return true;
    }
    return false;
}