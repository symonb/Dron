#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "motors.h"
#if defined(ESC_PROTOCOL_BDSHOT)||defined(ESC_PROTOCOL_DSHOT) || defined(ESC_PROTOCOL_DSHOT_BURST)
static uint16_t calculate_Dshot_checksum(uint16_t value);
static uint16_t prepare_Dshot_package(uint16_t value);
#endif
#if defined(ESC_PROTOCOL_DSHOT)
static void fill_Dshot_buffer(uint16_t m1_value, uint16_t m2_value, uint16_t m3_value, uint16_t m4_value);
#elif defined(ESC_PROTOCOL_DSHOT_BURST)
static void fill_Dshot_burst_buffer(uint16_t m1_value, uint16_t m2_value, uint16_t m3_value, uint16_t m4_value);
#elif defined(ESC_PROTOCOL_PWM) || defined(ESC_PROTOCOL_ONESHOT125)
static void prepare_OneShot_PWM();
#elif defined(ESC_PROTOCOL_BDSHOT)
static void fill_bb_Dshot_buffer(uint16_t m1_value, uint16_t m2_value, uint16_t m3_value, uint16_t m4_value);
static bool BDshot_check_checksum(uint16_t value);
static uint32_t get_BDshot_response(uint32_t raw_buffer[], const uint8_t motor_shift);
static void read_BDshot_response(uint32_t value, uint8_t motor);
#endif

#if defined(ESC_PROTOCOL_DSHOT)
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
#elif defined(ESC_PROTOCOL_BDSHOT)

bool bdshot_reception_1 = true;
bool bdshot_reception_2 = true;

void DMA2_Stream6_IRQHandler(void)
{

    if (DMA2->HISR & DMA_HISR_TCIF6)
    {
        DMA2->HIFCR |= DMA_HIFCR_CTCIF6;

        if (bdshot_reception_1)
        {
            // set GPIOs as inputs:
            GPIOA->MODER &= ~GPIO_MODER_MODER2;
            GPIOA->MODER &= ~GPIO_MODER_MODER3;
            // set pull up for those pins:
            GPIOA->PUPDR |= GPIO_PUPDR_PUPDR2_0 | GPIO_PUPDR_PUPDR3_0;

            // set timer:

            TIM1->ARR = DSHOT_BB_FRAME_LENGTH * DSHOT_MODE / BDSHOT_RESPONSE_BITRATE / BDSHOT_RESPONSE_OVERSAMPLING - 1;
            TIM1->CCR1 = DSHOT_BB_FRAME_LENGTH * DSHOT_MODE / BDSHOT_RESPONSE_BITRATE / BDSHOT_RESPONSE_OVERSAMPLING;

            DMA2_Stream6->CR &= ~(DMA_SxCR_DIR);
            DMA2_Stream6->PAR = (uint32_t)(&(GPIOA->IDR));
            DMA2_Stream6->M0AR = (uint32_t)(dshot_bb_buffer_1_4_r);
            // there is ~33 [us] gap before the response so it is necessary to add more samples:
            DMA2_Stream6->NDTR = ((int)(33 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING);

            DMA2_Stream6->CR |= DMA_SxCR_EN;
            bdshot_reception_1 = false;
        }
    }

    if (DMA2->HISR & DMA_HISR_HTIF6)
    {
        DMA2->HIFCR |= DMA_HIFCR_CHTIF6;
    }
    if (DMA2->HISR & DMA_HISR_DMEIF6)
    {
        DMA2->HIFCR |= DMA_HIFCR_CDMEIF6;
    }
    if (DMA2->HISR & DMA_HISR_TEIF6)
    {
        DMA2->HIFCR |= DMA_HIFCR_CTEIF6;
    }
}

void DMA2_Stream2_IRQHandler(void)
{
    if (DMA2->LISR & DMA_LISR_TCIF2)
    {
        DMA2->LIFCR |= DMA_LIFCR_CTCIF2;

        if (bdshot_reception_2)
        {
            // set GPIOs as inputs:
            GPIOB->MODER &= ~GPIO_MODER_MODER0;
            GPIOB->MODER &= ~GPIO_MODER_MODER1;
            // set pull up for these pins:
            GPIOB->PUPDR |= GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0;

            // set timer:

            TIM8->ARR = DSHOT_BB_FRAME_LENGTH * DSHOT_MODE / BDSHOT_RESPONSE_BITRATE / BDSHOT_RESPONSE_OVERSAMPLING - 1;
            TIM8->CCR1 = DSHOT_BB_FRAME_LENGTH * DSHOT_MODE / BDSHOT_RESPONSE_BITRATE / BDSHOT_RESPONSE_OVERSAMPLING;

            DMA2_Stream2->CR &= ~(DMA_SxCR_DIR);
            DMA2_Stream2->PAR = (uint32_t)(&(GPIOB->IDR));
            DMA2_Stream2->M0AR = (uint32_t)(dshot_bb_buffer_2_3_r);
            // there is ~33 [us] gap before the response so it is necessary to add more samples:
            DMA2_Stream2->NDTR = ((int)(33 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING);

            DMA2_Stream2->CR |= DMA_SxCR_EN;
            bdshot_reception_2 = false;
        }
    }

    if (DMA2->LISR & DMA_LISR_HTIF2)
    {
        DMA2->LIFCR |= DMA_LIFCR_CHTIF2;
    }
    if (DMA2->LISR & DMA_LISR_DMEIF2)
    {
        DMA2->LIFCR |= DMA_LIFCR_CDMEIF2;
    }
    if (DMA2->LISR & DMA_LISR_TEIF2)
    {
        DMA2->LIFCR |= DMA_LIFCR_CTEIF2;
    }
}

#endif

void update_motors(timeUs_t current_time)
{
#if defined(ESC_PROTOCOL_BDSHOT)
    // prepare for sending

    update_motors_rpm();

    fill_bb_Dshot_buffer(prepare_Dshot_package(*motor_1_value_pointer),
        prepare_Dshot_package(*motor_2_value_pointer),
        prepare_Dshot_package(*motor_3_value_pointer),
        prepare_Dshot_package(*motor_4_value_pointer));

    bdshot_reception_1 = true;
    bdshot_reception_2 = true;

    // set GPIOs as output:
    GPIOB->MODER |= GPIO_MODER_MODER0_0;
    GPIOB->MODER |= GPIO_MODER_MODER1_0;
    GPIOA->MODER |= GPIO_MODER_MODER2_0;
    GPIOA->MODER |= GPIO_MODER_MODER3_0;

#if defined(BIT_BANGING_V1)
    DMA2_Stream6->CR |= DMA_SxCR_DIR_0;
    DMA2_Stream6->PAR = (uint32_t)(&(GPIOA->BSRRL));
    DMA2_Stream6->M0AR = (uint32_t)(dshot_bb_buffer_1_4);
    DMA2_Stream6->NDTR = DSHOT_BB_BUFFER_LENGTH * DSHOT_BB_FRAME_SECTIONS;

    DMA2_Stream2->CR |= DMA_SxCR_DIR_0;
    DMA2_Stream2->PAR = (uint32_t)(&(GPIOB->BSRRL));
    DMA2_Stream2->M0AR = (uint32_t)(dshot_bb_buffer_2_3);
    DMA2_Stream2->NDTR = DSHOT_BB_BUFFER_LENGTH * DSHOT_BB_FRAME_SECTIONS;

    //	TIM1 setup:
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM1->ARR = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS - 1;
    TIM1->CCR1 = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS;

    // TIM8 setup:
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM8->CCR1 = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS;
    TIM8->ARR = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS - 1;

    //  send:
    DMA2_Stream6->CR |= DMA_SxCR_EN;
    DMA2_Stream2->CR |= DMA_SxCR_EN;

    TIM1->EGR |= TIM_EGR_UG;
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM8->EGR |= TIM_EGR_UG;
    TIM8->CR1 |= TIM_CR1_CEN;

#elif defined(BIT_BANGING_V2)
    DMA2_Stream6->CR |= DMA_SxCR_DIR_0;
    DMA2_Stream6->PAR = (uint32_t)(&(GPIOA->BSRRL));
    DMA2_Stream6->M0AR = (uint32_t)(dshot_bb_buffer_1_4);
    DMA2_Stream6->NDTR = DSHOT_BB_BUFFER_LENGTH * DSHOT_BB_FRAME_SECTIONS;

    DMA2_Stream2->CR |= DMA_SxCR_DIR_0;
    DMA2_Stream2->PAR = (uint32_t)(&(GPIOB->BSRRL));
    DMA2_Stream2->M0AR = (uint32_t)(dshot_bb_buffer_2_3);
    DMA2_Stream2->NDTR = DSHOT_BB_BUFFER_LENGTH * DSHOT_BB_FRAME_SECTIONS;

    //	TIM1 setup:
    TIM1->CCR1 = 0;
    TIM1->CCR2 = DSHOT_BB_0_LENGTH;
    TIM1->CCR3 = DSHOT_BB_1_LENGTH;
    TIM1->ARR = DSHOT_BB_FRAME_LENGTH - 1;

    // TIM8 setup:
    TIM8->CCR1 = 0;
    TIM8->CCR2 = DSHOT_BB_0_LENGTH;
    TIM8->CCR3 = DSHOT_BB_1_LENGTH;
    TIM8->ARR = DSHOT_BB_FRAME_LENGTH - 1;

    //  send:
    TIM1->EGR |= TIM_EGR_UG;
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM8->EGR |= TIM_EGR_UG;
    TIM8->CR1 |= TIM_CR1_CEN;

    DMA2_Stream6->CR |= DMA_SxCR_EN;
    DMA2_Stream2->CR |= DMA_SxCR_EN;
#endif

#elif defined(ESC_PROTOCOL_DSHOT)

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

#elif defined(ESC_PROTOCOL_ONESHOT125)

    //	OneShot125:

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

uint16_t calculate_Dshot_checksum(uint16_t value)
{
    // 12th bit for telemetry on/off (1/0):
    value = value << 1;
#if defined(ESC_PROTOCOL_BDSHOT)
    return (~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F;
#else
    return (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
#endif
}

uint16_t prepare_Dshot_package(uint16_t value)
{
    // value is in range of 2000-4000 so I need to transform it into Dshot range (48-2047)
    value -= 1953;
    if (value > 0 && value < 48)
    {
        value = 48;
    }
    return ((value << 5) | calculate_Dshot_checksum(value));
}
#if defined(ESC_PROTOCOL_DSHOT)
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
#endif

#if defined(BIT_BANGING_V1)

void preset_bb_Dshot_buffers()
{
    // these values are constant so they can be set once in the setup rutine:
    for (uint16_t i = 0; i < DSHOT_BB_BUFFER_LENGTH * DSHOT_BB_FRAME_SECTIONS; i++)
    {
        // set all bits to 0x00. after that state of GPIOs outputs will stay the same:
        dshot_bb_buffer_1_4[i] = 0x00;
        dshot_bb_buffer_2_3[i] = 0x00;
    }
    for (uint8_t i = 0; i < DSHOT_BB_BUFFER_LENGTH - 2; i++)
    { //  2 last bit will stay always high for ESC to capture dshot frame to the end
        // each bit is starting with lowering edge and after DSHOT_BB_1_LENGTH is rising (for 0-bit it rises earlier but always is high after 1-bit time)
        // set low edge at the begining of each bit:
        dshot_bb_buffer_1_4[i * DSHOT_BB_FRAME_SECTIONS] = GPIO_BSRR_BR_0 << MOTOR_1 | GPIO_BSRR_BR_0 << MOTOR_4;
        dshot_bb_buffer_2_3[i * DSHOT_BB_FRAME_SECTIONS] = GPIO_BSRR_BR_0 << MOTOR_2 | GPIO_BSRR_BR_0 << MOTOR_3;
        // set high after 1-bit length:
        dshot_bb_buffer_1_4[i * DSHOT_BB_FRAME_SECTIONS + DSHOT_BB_1_LENGTH - 1] = GPIO_BSRR_BS_0 << MOTOR_1 | GPIO_BSRR_BS_0 << MOTOR_4;
        dshot_bb_buffer_2_3[i * DSHOT_BB_FRAME_SECTIONS + DSHOT_BB_1_LENGTH - 1] = GPIO_BSRR_BS_0 << MOTOR_2 | GPIO_BSRR_BS_0 << MOTOR_3;
    }
}

static void fill_bb_Dshot_buffer(uint16_t m1_value, uint16_t m2_value, uint16_t m3_value,
    uint16_t m4_value)
{
    // Each bite is preset (lowering edge at first and rising edge after DSHOT_1_length rest values are 0 so there will be no changes in GPIO uotput registers).
    // Now it is needed to only decide about rising edge after DSHOT_0_length (if bit is 0) or seting 0 so LOW state will stay until DSHOT_1_length.
    // In addition 2 last bites are set always high (ESC needs time for proper signal detection and those high values define end off the transmission).
    for (uint8_t i = 0; i < DSHOT_BB_BUFFER_LENGTH - 2; i++) // last 2 bits are always high (logic 0)
    {
        if (1 << (DSHOT_BB_BUFFER_LENGTH - 3 - i) & m1_value)
        {
            // if bit is one send 0x00 so that GPIOs output will not change (will stay low):
            dshot_bb_buffer_1_4[i * DSHOT_BB_FRAME_SECTIONS + DSHOT_BB_0_LENGTH - 1] = 0x00;
        }
        else
        {
            // if bit is zero set rising edge after DSHOT_0_length:
            dshot_bb_buffer_1_4[i * DSHOT_BB_FRAME_SECTIONS + DSHOT_BB_0_LENGTH - 1] = GPIO_BSRR_BS_0 << MOTOR_1;
        }
        if (1 << (DSHOT_BB_BUFFER_LENGTH - 3 - i) & m2_value)
        {
            // if bit is one send 0x00 so that GPIOs output will not change (will stay low):
            dshot_bb_buffer_2_3[i * DSHOT_BB_FRAME_SECTIONS + DSHOT_BB_0_LENGTH - 1] = 0x00;
        }
        else
        {
            // if bit is zero set rising edge after DSHOT_0_length:
            dshot_bb_buffer_2_3[i * DSHOT_BB_FRAME_SECTIONS + DSHOT_BB_0_LENGTH - 1] = GPIO_BSRR_BS_0 << MOTOR_2;
        }
        if (1 << (DSHOT_BB_BUFFER_LENGTH - 3 - i) & m3_value)
        {
            // if bit is one send 0x00 so that GPIOs output will not change (will stay low):
            dshot_bb_buffer_2_3[i * DSHOT_BB_FRAME_SECTIONS + DSHOT_BB_0_LENGTH - 1] |= 0x00;
        }
        else
        {
            // if bit is zero set rising edge after DSHOT_0_length:
            dshot_bb_buffer_2_3[i * DSHOT_BB_FRAME_SECTIONS + DSHOT_BB_0_LENGTH - 1] |= GPIO_BSRR_BS_0 << MOTOR_3;
        }
        if (1 << (DSHOT_BB_BUFFER_LENGTH - 3 - i) & m4_value)
        {
            // if bit is one send 0x00 so that GPIOs output will not change (will stay low):
            dshot_bb_buffer_1_4[i * DSHOT_BB_FRAME_SECTIONS + DSHOT_BB_0_LENGTH - 1] |= 0x00;
        }
        else
        {
            // if bit is zero set rising edge after DSHOT_0_length:
            dshot_bb_buffer_1_4[i * DSHOT_BB_FRAME_SECTIONS + DSHOT_BB_0_LENGTH - 1] |= GPIO_BSRR_BS_0 << MOTOR_4;
        }
    }
}
#elif defined(BIT_BANGING_V2)

void preset_bb_Dshot_buffers()
{

    // this values are constant so they can be set once in the setup rutine:

    // make 2 high frames after Dshot frame (it is always this way, so should be set in rhe setup, not here):
    for (uint8_t i = 0; i < DSHOT_BB_FRAME_SECTIONS * 2; i++)
    {
        dshot_bb_buffer_1_4[(DSHOT_BUFFER_LENGTH)*DSHOT_BB_FRAME_SECTIONS - i - 1] = GPIO_BSRR_BS_0 << MOTOR_1 | GPIO_BSRR_BS_0 << MOTOR_4;
        dshot_bb_buffer_2_3[(DSHOT_BUFFER_LENGTH)*DSHOT_BB_FRAME_SECTIONS - i - 1] = GPIO_BSRR_BS_0 << MOTOR_2 | GPIO_BSRR_BS_0 << MOTOR_3;
    }
}

static void fill_bb_Dshot_buffer(uint16_t m1_value, uint16_t m2_value, uint16_t m3_value,
    uint16_t m4_value)
{
    // each bite frame is divided in sections where slope can vary:
    for (uint8_t i = 0; i < (DSHOT_BB_BUFFER_LENGTH - 2); i++) // last 2 bits are always high (logic 0)
    {
        //  first section always lower edge:
        dshot_bb_buffer_1_4[i * DSHOT_BB_FRAME_SECTIONS] = GPIO_BSRR_BR_0 << MOTOR_1 | GPIO_BSRR_BR_0 << MOTOR_4;

        dshot_bb_buffer_2_3[i * DSHOT_BB_FRAME_SECTIONS] = GPIO_BSRR_BR_0 << MOTOR_2 | GPIO_BSRR_BR_0 << MOTOR_3;

        // last section always rise edge:
        dshot_bb_buffer_1_4[i * DSHOT_BB_FRAME_SECTIONS + 2] = GPIO_BSRR_BS_0 << MOTOR_1 | GPIO_BSRR_BS_0 << MOTOR_4;
        dshot_bb_buffer_2_3[i * DSHOT_BB_FRAME_SECTIONS + 2] = GPIO_BSRR_BS_0 << MOTOR_2 | GPIO_BSRR_BS_0 << MOTOR_3;

        if ((1 << (DSHOT_BB_BUFFER_LENGTH - 3 - i)) & m1_value)
        {
            // if bit is one set middle section low:
            dshot_bb_buffer_1_4[i * DSHOT_BB_FRAME_SECTIONS + 1] = GPIO_BSRR_BR_0 << MOTOR_1;
        }
        else
        {
            // if bit is zero set high:
            dshot_bb_buffer_1_4[i * DSHOT_BB_FRAME_SECTIONS + 1] = GPIO_BSRR_BS_0 << MOTOR_1;
        }
        if ((1 << (DSHOT_BB_BUFFER_LENGTH - 3 - i)) & m2_value)
        {
            // if bit is one set 3rd and 4th section low:
            dshot_bb_buffer_2_3[i * DSHOT_BB_FRAME_SECTIONS + 1] = GPIO_BSRR_BR_0 << MOTOR_2;
        }
        else
        {
            // if bit is zero set high 3rd and 4th section:
            dshot_bb_buffer_2_3[i * DSHOT_BB_FRAME_SECTIONS + 1] = GPIO_BSRR_BS_0 << MOTOR_2;
        }
        if ((1 << (DSHOT_BB_BUFFER_LENGTH - 3 - i)) & m3_value)
        {
            // if bit is one set 3rd and 4th section low:
            dshot_bb_buffer_2_3[i * DSHOT_BB_FRAME_SECTIONS + 1] |= GPIO_BSRR_BR_0 << MOTOR_3;
        }
        else
        {
            // if bit is zero set high 3rd and 4th section:
            dshot_bb_buffer_2_3[i * DSHOT_BB_FRAME_SECTIONS + 1] |= GPIO_BSRR_BS_0 << MOTOR_3;
        }
        if ((1 << (DSHOT_BB_BUFFER_LENGTH - 3 - i)) & m4_value)
        {
            // if bit is one set 3rd and 4th section low:
            dshot_bb_buffer_1_4[i * DSHOT_BB_FRAME_SECTIONS + 1] |= GPIO_BSRR_BR_0 << MOTOR_4;
        }
        else
        {
            // if bit is zero set high 3rd and 4th section:
            dshot_bb_buffer_1_4[i * DSHOT_BB_FRAME_SECTIONS + 1] |= GPIO_BSRR_BS_0 << MOTOR_4;
        }
    }
}
#endif

bool BDshot_check_checksum(uint16_t value)
{
    // BDshot frame has 4 last bits CRC:
    if (((value ^ (value >> 4) ^ (value >> 8) ^ (value >> 12)) & 0x0F) == 0x0F)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void update_motors_rpm()
{
    // BDshot bit banging reads whole GPIO register.
    // Now it's time to create BDshot responses from all motors (made of individual bits).
    uint32_t motor_1_response = get_BDshot_response(dshot_bb_buffer_1_4_r, MOTOR_1);
    uint32_t motor_2_response = get_BDshot_response(dshot_bb_buffer_2_3_r, MOTOR_2);
    uint32_t motor_3_response = get_BDshot_response(dshot_bb_buffer_2_3_r, MOTOR_3);
    uint32_t motor_4_response = get_BDshot_response(dshot_bb_buffer_1_4_r, MOTOR_4);

    read_BDshot_response(motor_1_response, 1);
    read_BDshot_response(motor_2_response, 2);
    read_BDshot_response(motor_3_response, 3);
    read_BDshot_response(motor_4_response, 4);
}

static uint32_t get_BDshot_response(uint32_t raw_buffer[], const uint8_t motor_shift)
{
    // reception starts just after transmission, so there is a lot of HIGH samples. Find first LOW bit:

    uint16_t i = 0;
    uint16_t previous_i = 0;
    uint16_t end_i = 0;
    uint32_t previous_value = 1;
    uint32_t motor_response = 0;
    uint8_t bits = 0;

    while (i < (int)(33 * BDSHOT_RESPONSE_BITRATE / 1000 * BDSHOT_RESPONSE_OVERSAMPLING))
    {
        if (!(raw_buffer[i] & (1 << motor_shift)))
        {
            previous_value = 0;
            previous_i = i;
            end_i = i + BDSHOT_RESPONSE_LENGTH * BDSHOT_RESPONSE_OVERSAMPLING;
            break;
        }
        i++;
    }
    // if LOW edge was detected:
    if (previous_value == 0)
    {
        while (i < end_i)
        {
            // then look for changes in bits values and compute BDSHOT bits:
            if ((raw_buffer[i] & (1 << motor_shift)) != previous_value)
            {
                const uint8_t len = MAX((i - previous_i) / BDSHOT_RESPONSE_OVERSAMPLING, 1); // how many bits had the same value
                bits += len;
                motor_response <<= len;
                if (previous_value != 0)
                {
                    motor_response |= (0x1FFFFF >> (21 - len)); // 21 ones right-shifted by 20 or less
                }
                previous_value = raw_buffer[i] & (1 << motor_shift);
                previous_i = i;
            }
            i++;
        }
        // if last bits were 1 they were not added so far
        motor_response <<= (BDSHOT_RESPONSE_LENGTH - bits);
        motor_response |= 0x1FFFFF >> bits; // 21 ones right-shifted

        return motor_response;
    }
    else
    { // if LOW edge was not found return incorrect motor response:
        return 0xFFFFFFFF;
    }
}

static void read_BDshot_response(uint32_t value, uint8_t motor)
{
    // BDshot frame contain 21 bytes but first is always 0 (used only for detection).
    // Next 20 bits are 4 sets of 5-bits which are mapped with 4-bits real value.
    // After all, value is 16-bit long with 12-bit eRPM value (actually it is a period of eRPM) and 4-bit CRC.
    // 12-bit eRPM value has 3 first bits od left shifting and 9-bit mantisa.

    // put nibbles in the array in places of mapped values (to reduce empty elements smallest mapped value will always be subtracted)
    // now it is easy to create real value - mapped value indicate array element which contain nibble value:
#define iv 0xFFFFFFFF
    static const uint32_t GCR_table[32] = {
        iv, iv, iv, iv, iv, iv, iv, iv, iv, 9, 10, 11, iv, 13, 14, 15,
        iv, iv, 2, 3, iv, 5, 6, 7, iv, 0, 8, 1, iv, 4, 12, iv };

    value = (value ^ (value >> 1)); // now we have GCR value

    uint32_t decoded_value = GCR_table[(value & 0x1F)];
    decoded_value |= GCR_table[((value >> 5) & 0x1F)] << 4;
    decoded_value |= GCR_table[((value >> 10) & 0x1F)] << 8;
    decoded_value |= GCR_table[((value >> 15) & 0x1F)] << 12;

    // if wrongly decoded decoded_value will be bigger than uint16_t:
    if (decoded_value < 0xFFFF && BDshot_check_checksum(decoded_value))
    {
        // if checksum is correct real save real RPM.
        // value sent by ESC is a period between each pole changes [us].
        // to achive eRPM we need to find out how many of these changes are in one minute.
        // eRPM = (60*1000 000)/T_us next RPM can be achived -> RPM = eRPM/(poles/2):

        motors_rpm[motor - 1] = ((decoded_value & 0x1FF0) >> 4) << (decoded_value >> 13);      // cut off CRC and add shifting - this is period in [us]
        motors_rpm[motor - 1] = 60 * 1000000 / motors_rpm[motor - 1] * 2 / MOTOR_POLES_NUMBER; // convert to RPM
        motors_error[motor - 1] = 0.9 * motors_error[motor - 1];                               // reduce motor error
    }
    else
    {
        motors_error[motor - 1] = 0.9 * motors_error[motor - 1] + 10; // increase motor error
    }
}

#if defined(ESC_PROTOCOL_BDSHOT_BURST)
static void fill_Dshot_burst_buffer(uint16_t m1_value, uint16_t m2_value,
    uint16_t m3_value, uint16_t m4_value)
{

    // buffer array looks like this [first bit motor1; first bit motor2 ;...; last bit motor1;last bit motor2]

    for (int i = 2; i < DSHOT_BUFFER_LENGTH; i++)
    {
        if ((1 << (i - 2)) & m4_value)
        {
            dshot_burst_buffer_4_1[DSHOT_BUFFER_LENGTH * 2 - 1 - 2 * i - 1] =
                DSHOT_1_LENGTH;
        }
        else
        {
            dshot_burst_buffer_4_1[DSHOT_BUFFER_LENGTH * 2 - 1 - 2 * i - 1] =
                DSHOT_0_LENGTH;
        }
        if ((1 << (i - 2)) & m1_value)
        {
            dshot_burst_buffer_4_1[DSHOT_BUFFER_LENGTH * 2 - 1 - 2 * i] =
                DSHOT_1_LENGTH;
        }
        else
        {
            dshot_burst_buffer_4_1[DSHOT_BUFFER_LENGTH * 2 - 1 - 2 * i] =
                DSHOT_0_LENGTH;
        }
        if ((1 << (i - 2)) & m2_value)
        {
            dshot_burst_buffer_2_3[DSHOT_BUFFER_LENGTH * 2 - 1 - 2 * i - 1] =
                DSHOT_1_LENGTH;
        }
        else
        {
            dshot_burst_buffer_2_3[DSHOT_BUFFER_LENGTH * 2 - 1 - 2 * i - 1] =
                DSHOT_0_LENGTH;
        }
        if ((1 << (i - 2)) & m3_value)
        {
            dshot_burst_buffer_2_3[DSHOT_BUFFER_LENGTH * 2 - 1 - 2 * i] =
                DSHOT_1_LENGTH;
        }
        else
        {
            dshot_burst_buffer_2_3[DSHOT_BUFFER_LENGTH * 2 - 1 - 2 * i] =
                DSHOT_0_LENGTH;
        }
    }
    dshot_burst_buffer_4_1[2 * DSHOT_BUFFER_LENGTH - 1] = 0;
    dshot_burst_buffer_4_1[2 * DSHOT_BUFFER_LENGTH - 2] = 0;
    dshot_burst_buffer_2_3[2 * DSHOT_BUFFER_LENGTH - 1] = 0;
    dshot_burst_buffer_2_3[2 * DSHOT_BUFFER_LENGTH - 2] = 0;
}
#endif

#if defined(ESC_PROTOCOL_ONESHOT125)
static void prepare_OneShot_PWM()
{
    TIM2->CCR4 = 3500 - *motor_1_value_pointer * 0.875f + 1; // value motor 1
    TIM3->CCR3 = 3500 - *motor_2_value_pointer * 0.875f + 1; // value motor 2
    TIM3->CCR4 = 3500 - *motor_3_value_pointer * 0.875f + 1; // value motor 3
    TIM2->CCR3 = 3500 - *motor_4_value_pointer * 0.875f + 1; // value motor 4
}
#endif