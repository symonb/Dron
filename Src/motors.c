#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "motors.h"

static uint16_t get_Dshot_checksum(uint16_t value);
static uint16_t prepare_Dshot_package(uint16_t value);
static void fill_Dshot_buffer(uint16_t m1_value, uint16_t m2_value, uint16_t m3_value, uint16_t m4_value);
static void fill_Dshot_burst_buffer(uint16_t m1_value, uint16_t m2_value, uint16_t m3_value, uint16_t m4_value);
static void prepare_OneShot_PWM();

//	motor 1
void DMA1_Stream6_IRQHandler(void)
{

    if (DMA1->HISR & DMA_HISR_TCIF6)
    {
        DMA1->HIFCR |= DMA_HIFCR_CTCIF6;
    }
    if (DMA1->HISR & DMA_HISR_HTIF6)
    {
        DMA1->HIFCR |= DMA_HIFCR_CHTIF6;
    }
    if (DMA1->HISR & DMA_HISR_DMEIF6)
    {
        DMA1->HIFCR |= DMA_HIFCR_CDMEIF6;
    }
    if (DMA1->HISR & DMA_HISR_TEIF6)
    {
        DMA1->HIFCR |= DMA_HIFCR_CTEIF6;
    }
}

//	motor 2
void DMA1_Stream7_IRQHandler(void)
{

    if (DMA1->HISR & DMA_HISR_TCIF7)
    {
        DMA1->HIFCR |= DMA_HIFCR_CTCIF7;
    }
    if (DMA1->HISR & DMA_HISR_HTIF7)
    {
        DMA1->HIFCR |= DMA_HIFCR_CHTIF7;
    }
    if (DMA1->HISR & DMA_HISR_DMEIF7)
    {
        DMA1->HIFCR |= DMA_HIFCR_CDMEIF7;
    }
    if (DMA1->HISR & DMA_HISR_TEIF7)
    {
        DMA1->HIFCR |= DMA_HIFCR_CTEIF7;
    }
}

//	motor 3
void DMA1_Stream2_IRQHandler(void)
{

    if (DMA1->LISR & DMA_LISR_TCIF2)
    {
        DMA1->LIFCR |= DMA_LIFCR_CTCIF2;
    }
    if (DMA1->LISR & DMA_LISR_HTIF2)
    {
        DMA1->LIFCR |= DMA_LIFCR_CHTIF2;
    }
    if (DMA1->LISR & DMA_LISR_DMEIF2)
    {
        DMA1->LIFCR |= DMA_LIFCR_CDMEIF2;
    }
    if (DMA1->LISR & DMA_LISR_TEIF2)
    {
        DMA1->LIFCR |= DMA_LIFCR_CTEIF2;
    }
}

//	motor 4
void DMA1_Stream1_IRQHandler(void)
{

    if (DMA1->LISR & DMA_LISR_TCIF1)
    {
        DMA1->LIFCR |= DMA_LIFCR_CTCIF1;
    }

    if (DMA1->LISR & DMA_LISR_HTIF1)
    {
        DMA1->LIFCR |= DMA_LIFCR_CHTIF1;
    }
    if (DMA1->LISR & DMA_LISR_DMEIF1)
    {
        DMA1->LIFCR |= DMA_LIFCR_CDMEIF1;
    }
    if (DMA1->LISR & DMA_LISR_TEIF1)
    {
        DMA1->LIFCR |= DMA_LIFCR_CTEIF1;
    }
}

void update_motors(timeUs_t current_time)
{

#if defined(ESC_PROTOCOL_DSHOT)

    //	Dshot:

    fill_Dshot_buffer(prepare_Dshot_package(*motor_1_value_pointer),
                      prepare_Dshot_package(*motor_2_value_pointer),
                      prepare_Dshot_package(*motor_3_value_pointer),
                      prepare_Dshot_package(*motor_4_value_pointer));

    DMA1_Stream6->PAR = (uint32_t)(&(TIM2->CCR4));
    DMA1_Stream6->M0AR = (uint32_t)(dshot_buffer_1);
    DMA1_Stream6->NDTR = DSHOT_BUFFER_LENGTH;

    DMA1_Stream7->PAR = (uint32_t)(&(TIM3->CCR3));
    DMA1_Stream7->M0AR = (uint32_t)(dshot_buffer_2);
    DMA1_Stream7->NDTR = DSHOT_BUFFER_LENGTH;

    DMA1_Stream2->PAR = (uint32_t)(&(TIM3->CCR4));
    DMA1_Stream2->M0AR = (uint32_t)(dshot_buffer_3);
    DMA1_Stream2->NDTR = DSHOT_BUFFER_LENGTH;

    DMA1_Stream1->PAR = (uint32_t)(&(TIM2->CCR3));
    DMA1_Stream1->M0AR = (uint32_t)(dshot_buffer_4);
    DMA1_Stream1->NDTR = DSHOT_BUFFER_LENGTH;

    DMA1_Stream1->CR |= DMA_SxCR_EN;
    DMA1_Stream2->CR |= DMA_SxCR_EN;
    DMA1_Stream6->CR |= DMA_SxCR_EN;
    DMA1_Stream7->CR |= DMA_SxCR_EN;

#elif defined(ESC_PROTOCOL_DSHOT_BURST)

    //	Dshot with burst transfer:

    //	fill_Dshot_burst_buffer(prepare_Dshot_package(*motor_1_value_pointer),prepare_Dshot_package(*motor_2_value_pointer),prepare_Dshot_package(*motor_3_value_pointer),prepare_Dshot_package(*motor_4_value_pointer));
    //	DMA1_Stream2->CR |= DMA_SxCR_EN;
    //	DMA1_Stream6->CR |= DMA_SxCR_EN;

#elif defined(ESC_PROTOCOL_ONESHOT125) || defined(ESC_PROTOCOL_ONESHOT_V1)

    //	OneShot125 or OneShot125_v2:

    prepare_OneShot_PWM();

    TIM2->EGR |= TIM_EGR_UG;
    TIM3->EGR |= TIM_EGR_UG;
    TIM2->CR1 |= TIM_CR1_CEN;
    TIM3->CR1 |= TIM_CR1_CEN;

#elif defined(ESC_PROTOCOL_PWM)

    //	PWM:

    TIM2->CCR4 = *motor_1_value_pointer; // value motor 1
    TIM3->CCR3 = *motor_2_value_pointer; // value motor 2
    TIM3->CCR4 = *motor_3_value_pointer; // value motor 3
    TIM2->CCR3 = *motor_4_value_pointer; // value motor 4

#endif
}

uint16_t get_Dshot_checksum(uint16_t value)
{
    value = value << 1;
    return (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
}

uint16_t prepare_Dshot_package(uint16_t value)
{
    // value is in range of 2000-4000 so I need to transform it into Dshot range (48-2047)
    value -= 1953;
    if (value > 0 && value < 48)
    {
        value = 48;
    }
    return ((value << 5) | get_Dshot_checksum(value));
}

static void fill_Dshot_buffer(uint16_t m1_value, uint16_t m2_value, uint16_t m3_value,
                              uint16_t m4_value)
{

    for (uint8_t i = 2; i < DSHOT_BUFFER_LENGTH; i++)
    {
        if ((1 << (i - 2)) & m1_value)
        {
            dshot_buffer_1[DSHOT_BUFFER_LENGTH - 1 - i] = DSHOT_1_LENGTH;
        }
        else
        {
            dshot_buffer_1[DSHOT_BUFFER_LENGTH - 1 - i] = DSHOT_0_LENGTH;
        }
        if ((1 << (i - 2)) & m2_value)
        {
            dshot_buffer_2[DSHOT_BUFFER_LENGTH - 1 - i] = DSHOT_1_LENGTH;
        }
        else
        {
            dshot_buffer_2[DSHOT_BUFFER_LENGTH - 1 - i] = DSHOT_0_LENGTH;
        }
        if ((1 << (i - 2)) & m3_value)
        {
            dshot_buffer_3[DSHOT_BUFFER_LENGTH - 1 - i] = DSHOT_1_LENGTH;
        }
        else
        {
            dshot_buffer_3[DSHOT_BUFFER_LENGTH - 1 - i] = DSHOT_0_LENGTH;
        }
        if ((1 << (i - 2)) & m4_value)
        {
            dshot_buffer_4[DSHOT_BUFFER_LENGTH - 1 - i] = DSHOT_1_LENGTH;
        }
        else
        {
            dshot_buffer_4[DSHOT_BUFFER_LENGTH - 1 - i] = DSHOT_0_LENGTH;
        }
    }
    // make 0 pulse after Dshot frame:
    dshot_buffer_1[DSHOT_BUFFER_LENGTH - 1] = 0;
    dshot_buffer_1[DSHOT_BUFFER_LENGTH - 2] = 0;
    dshot_buffer_2[DSHOT_BUFFER_LENGTH - 1] = 0;
    dshot_buffer_2[DSHOT_BUFFER_LENGTH - 2] = 0;
    dshot_buffer_3[DSHOT_BUFFER_LENGTH - 1] = 0;
    dshot_buffer_3[DSHOT_BUFFER_LENGTH - 2] = 0;
    dshot_buffer_4[DSHOT_BUFFER_LENGTH - 1] = 0;
    dshot_buffer_4[DSHOT_BUFFER_LENGTH - 2] = 0;
}

static void fill_Dshot_burst_buffer(uint16_t m1_value, uint16_t m2_value,
                                    uint16_t m3_value, uint16_t m4_value)
{

    // buffer array looks like this [first bit motor1; first bit motor2 ;...; last bit motor1;last bit motor2]

    for (int i = 2; i < DSHOT_BUFFER_LENGTH; i++)
    {
        if ((1 << (i - 2)) & m4_value)
        {
            dshot_buffer_4_1[DSHOT_BUFFER_LENGTH * 2 - 1 - 2 * i - 1] =
                DSHOT_1_LENGTH;
        }
        else
        {
            dshot_buffer_4_1[DSHOT_BUFFER_LENGTH * 2 - 1 - 2 * i - 1] =
                DSHOT_0_LENGTH;
        }
        if ((1 << (i - 2)) & m1_value)
        {
            dshot_buffer_4_1[DSHOT_BUFFER_LENGTH * 2 - 1 - 2 * i] =
                DSHOT_1_LENGTH;
        }
        else
        {
            dshot_buffer_4_1[DSHOT_BUFFER_LENGTH * 2 - 1 - 2 * i] =
                DSHOT_0_LENGTH;
        }
        if ((1 << (i - 2)) & m2_value)
        {
            dshot_buffer_2_3[DSHOT_BUFFER_LENGTH * 2 - 1 - 2 * i - 1] =
                DSHOT_1_LENGTH;
        }
        else
        {
            dshot_buffer_2_3[DSHOT_BUFFER_LENGTH * 2 - 1 - 2 * i - 1] =
                DSHOT_0_LENGTH;
        }
        if ((1 << (i - 2)) & m3_value)
        {
            dshot_buffer_2_3[DSHOT_BUFFER_LENGTH * 2 - 1 - 2 * i] =
                DSHOT_1_LENGTH;
        }
        else
        {
            dshot_buffer_2_3[DSHOT_BUFFER_LENGTH * 2 - 1 - 2 * i] =
                DSHOT_0_LENGTH;
        }
    }
    dshot_buffer_4_1[2 * DSHOT_BUFFER_LENGTH - 1] = 0;
    dshot_buffer_4_1[2 * DSHOT_BUFFER_LENGTH - 2] = 0;
    dshot_buffer_2_3[2 * DSHOT_BUFFER_LENGTH - 1] = 0;
    dshot_buffer_2_3[2 * DSHOT_BUFFER_LENGTH - 2] = 0;
}

static void prepare_OneShot_PWM()
{

    TIM2->CCR4 = 3500 - *motor_1_value_pointer * 0.875f + 1; // value motor 1
    TIM3->CCR3 = 3500 - *motor_2_value_pointer * 0.875f + 1; // value motor 2
    TIM3->CCR4 = 3500 - *motor_3_value_pointer * 0.875f + 1; // value motor 3
    TIM2->CCR3 = 3500 - *motor_4_value_pointer * 0.875f + 1; // value motor 4
}