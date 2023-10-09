
#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include <math.h>
#include "SPI1.h"


void SPI1_enable()
{
    SPI1->CR1 |= SPI_CR1_SPE; //	enabling SPI1
}

void SPI1_disable()
{
    SPI1->CR1 &= ~SPI_CR1_SPE; //	disabling SPI1
}

void SPI1_CS_enable()
{
    GPIOA->BSRR |= GPIO_BSRR_BR_4;
}

void SPI1_CS_disable()
{
    GPIOA->BSRR |= GPIO_BSRR_BS_4;
}

void SPI1_transmit(const uint8_t* data, int size)
{
    //----------------STEPS--------------------
    /* 1 Wait for TXE bit to set in the Status Register
     * 2 Write the Register Adress (&~x80 for writing) to the Data register
     * 3 Write first byte that you want send to slave
     * 4 Receive 1 data - it is anything (slave started sending it when received first bit of first byte so it is not interesting byte for sure) ignore it
     * 5 Send second data and repeat until you receive all data you wanted
     * 6 After last transmission you need to clear DR and wait until everything stop (more inf. in datasheet)
     * */
    int i = 0;

    while (i < size)
    {
        time_flag4_1 = get_Global_Time();
        while (!((SPI1->SR) & SPI_SR_TXE))
        {
            if (SPI1_failsafe())
            {
                break;
            }
        }
        SPI1->DR = data[i]; // data sending as soon as TX flag is set
        i++;
    }

    time_flag4_1 = get_Global_Time();
    while (!((SPI1->SR) & SPI_SR_TXE))
    {
        if (SPI1_failsafe())
        {
            break;
        }
    }
    time_flag4_1 = get_Global_Time();
    while (((SPI1->SR) & SPI_SR_BSY))
    {
        if (SPI1_failsafe())
        {
            break;
        }
    }
    SPI1->DR;
    SPI1->SR;
}

void SPI1_receive(uint8_t* data, int size)
{
    /*
     * 1 Wait for TXE and send dummy data, wait for RXNE and receive your data. Repeat until you receive all data
     * 2 Wait for TXE and BSY flag
     * */
    while (size > 0)
    {
        time_flag4_1 = get_Global_Time();
        while (!((SPI1->SR) & SPI_SR_TXE))
        {
            if (SPI1_failsafe())
            {
                break;
            }
        }
        SPI1->DR = 0xAA; // send anything 
        time_flag4_1 = get_Global_Time();
        while (!((SPI1->SR) & SPI_SR_RXNE))
        {
            if (SPI1_failsafe())
            {
                break;
            }
        }
        *data++ = SPI1->DR;
        size--;
    }

    // wait for TXE flag
    time_flag4_1 = get_Global_Time();
    while (!((SPI1->SR) & SPI_SR_TXE))
    {
        if (SPI1_failsafe())
        {
            break;
        }
    }
    // wait for BSY flag
    time_flag4_1 = get_Global_Time();
    while (((SPI1->SR) & SPI_SR_BSY))
    {
        if (SPI1_failsafe())
        {
            break;
        }
    }
    SPI1->DR;
    SPI1->SR;
}

void SPI1_receive_DMA(uint8_t* data, uint16_t size)
{
    // disable DMA:
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    DMA2_Stream3->CR &= ~DMA_SxCR_EN;
    // set parameters of reception:
    DMA2_Stream0->M0AR = (uint32_t)(data);
    DMA2_Stream0->NDTR = size;
    // set anything for transmission:
    DMA2_Stream3->M0AR = (uint32_t)(data);
    DMA2_Stream3->NDTR = size;

    SPI1_enable();

    // enable DMA:
    DMA2_Stream0->CR |= DMA_SxCR_EN;
    DMA2_Stream3->CR |= DMA_SxCR_EN;
}

bool SPI1_failsafe()
{
    //	waiting as Data will be sent or failsafe if set time passed
    if ((get_Global_Time() - time_flag4_1) >= SEC_TO_US(SPI_TIMEOUT_VALUE))
    {
        FailSafe_status = FAILSAFE_SPI_IMU_ERROR;
        EXTI->SWIER |= EXTI_SWIER_SWIER15;
        return true;
    }
    return false;
}

