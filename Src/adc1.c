
#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "battery.h"
#include "adc1.h"




void DMA2_Stream4_IRQHandler(void)
{
    if (DMA2->HISR & DMA_HISR_TCIF4)
    {
        DMA2->HIFCR |= DMA_HIFCR_CTCIF4;
        DMA2_Stream4->CR &= ~DMA_SxCR_EN;
    }
}



void ADC_IRQHandler(void)
{
    //  ADC1 overrun handler:
    if (ADC1->SR & ADC_SR_OVR)
    {
        ADC1->SR &= ~ADC_SR_OVR;
        DMA2_Stream4->CR &= ~DMA_SxCR_EN;
        DMA2_Stream4->M0AR = (uint32_t)(main_battery.ADC_value);
        DMA2_Stream4->NDTR = 2;
    }
}