
#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "adc1.h"

const float V_25 = 0.76f;    // [V]
const float AVG_SLOPE = 400; // [C/V]
uint32_t ADC1_buffer[2] = {0, 0};

void DMA2_Stream4_IRQHandler(void)
{
    if (DMA2->HISR & DMA_HISR_TCIF4)
    {
        DMA2->HIFCR |= DMA_HIFCR_CTCIF4;
        DMA2_Stream4->CR &= ~DMA_SxCR_EN;
    }
}

void ADC1_start(timeUs_t current_time)
{
    //  convert previous measurements:
    battery_voltage = battery_voltage * 0.7f + (ADC1_buffer[0] * ADC_REFERENCE_VOLTAGE / 0xFFF * BATTERY_SCALE) * 0.3f;
    MCU_temperature = MCU_temperature * 0.7f + ((ADC1_buffer[1] * ADC_REFERENCE_VOLTAGE / 0xFFF - V_25) * AVG_SLOPE + 25) * 0.3f;

    //  turn on DMA:
    DMA2_Stream4->CR |= DMA_SxCR_EN;
    //  start conversion:
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

void ADC_IRQHandler(void)
{
    //  ADC1 overrun handler:
    if (ADC1->SR & ADC_SR_OVR)
    {
        DMA2_Stream4->CR &= ~DMA_SxCR_EN;
        DMA2_Stream4->M0AR = (uint32_t)(ADC1_buffer);
        DMA2_Stream4->NDTR = 2;
        ADC1->SR &= ~ADC_SR_OVR;
    }
}