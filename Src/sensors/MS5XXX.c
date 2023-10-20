#include <math.h>
#include <stdint.h>
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "drivers/I2C1.h"
#include "MS5XXX.h"

static bool MS5XXX_get_coefficients(uint8_t* data, baro_t* baro);

bool MS5XXX_init(baro_t* baro) {
    delay_mili(10);
    uint8_t temp[16];
    MS5XXX_reset();
    delay_mili(10);
    MS5XXX_PROM_read(temp);
    return MS5XXX_get_coefficients(temp, baro);
}
/**
 *@brief function to reset sensor after hang up
 *@note According to datasheet the only way to get back to function is to send a few SCLKs and reset sequence
 * */
void MS5XXX_unstack() {
    I2C1_reset();
    for (uint8_t i = 0; i < 5; i++)
    {
        MS5XXX_reset();
    }
}

/**
 *@brief Calculate preasure and temperature values with corrections
 *@param baro pointer for baro_t object where preasure and temperature can be saved.
*@attention Some sensors can have really similar data processing. be aware of possible micro difference (e.g. 2^13 not 2^15).
 *@note Temperature value should be used only for preasure correction (not reliable for real  temperature).
 * */
void MS5XXX_calculate_preasure(baro_t* baro) {

    uint32_t D1;
    uint32_t D2;
    int64_t dT;
    int64_t TEMP;
    int64_t OFF;
    int64_t SENS;
    int64_t Ti;
    int64_t OFFi;
    int64_t SENSi;



    D1 = baro->ADC_value[0];
    D2 = baro->ADC_value[1];

    // TEST
    // D1 = 9085466;
    // D2 = 8569150;

    // baro->C1_SENS = 40127;
    // baro->C2_OFF = 36924;
    // baro->C3_TCS = 23317;
    // baro->C4_TCO = 23282;
    // baro->C5_T = 33464;
    // baro->C6_TEMPSENS = 28312;

    // first order temperature compensation:
    dT = (int64_t)D2 - baro->C5_T * 256;
    TEMP = (int64_t)(2000 + dT * baro->C6_TEMPSENS / 8388608);
    OFF = (int64_t)baro->C2_OFF * 65536 + baro->C4_TCO * dT / 128;
    SENS = (int64_t)baro->C1_SENS * 32768 + baro->C3_TCS * dT / 256;

    // second order of compensation:
#if MS5XXX_SENSOR == MS5611
    if (TEMP / 100 < 20)
    {
        Ti = (int64_t)dT * dT / 2147483648.;
        OFFi = (int64_t)5 * (TEMP - 2000) * (TEMP - 2000) / 2;
        SENSi = (int64_t)5 * (TEMP - 2000) * (TEMP - 2000) / 4;
        if (TEMP / 100 < -15)
        {
            OFFi = OFFi + 7 * (TEMP + 1500) * (TEMP + 1500);
            SENSi = SENSi + 11 * (TEMP + 1500) * (TEMP + 1500) / 2;
        }
    }
    else
    {
        Ti = 0;
        OFFi = 0;
        SENSi = 0;
    }

    baro->temperature = (TEMP - Ti) / 100.f;                                           // in [C]
    baro->raw_preasure = (((D1 * (SENS - SENSi)) / 2097152 - (OFF - OFFi)) / 32768) / 100.f; // in [mbar];
#elif MS5XXX_SENSOR == MS5837

    if (TEMP / 100 < 20)
    {
        Ti = 3 * dT * dT / 8589934592;
        OFFi = 3 * (TEMP - 2000) * (TEMP - 2000) / 2;
        SENSi = 5 * (TEMP - 2000) * (TEMP - 2000) / 8;
        if (TEMP / 100 < -15)
        {
            OFFi = OFFi + 7 * (TEMP + 1500) * (TEMP + 1500);
            SENSi = SENSi + 4 * (TEMP + 1500) * (TEMP + 1500);
        }
    }
    else
    {
        Ti = 2 * dT * dT / 137438953472.;
        OFFi = (TEMP - 2000) * (TEMP - 2000) / 16;
        SENSi = 0;
    }

    baro->temperature = (TEMP - Ti) / 100.f;                                           // in [C]
    baro->raw_preasure = (((D1 * (SENS - SENSi)) / 2097152 - (OFF - OFFi)) / 8192) / 10.f; // in [mbar];
#endif
}

/**
 *@brief read ADC value and save as raw preasure
 *@param baro pointer for baro_t object where preasure can be saved.
* @note Before this function conversion has to be performed and conversion time needs to be waited.
 * */
void MS5XXX_read_preasure(baro_t* baro) {
    uint8_t data[3];
    I2C1_Write(MS5XXX_ADDR_W, MS5XXX_CMD_ADC_READ, NULL, 0);
    I2C1_Stop();
    I2C1_Read(MS5XXX_ADDR_R, data, 3);

    baro->ADC_value[0] = data[0] << 16 | data[1] << 8 | data[2];
}

/**
 *@brief read ADC value and save as raw temperature
 *@param baro pointer for baro_t object where temperature can be saved.
* @note Before this function conversion has to be performed and conversion time needs to be waited.
 * */
void MS5XXX_read_temp(baro_t* baro) {
    uint8_t data[3];
    I2C1_Write(MS5XXX_ADDR_W, MS5XXX_CMD_ADC_READ, NULL, 0);
    I2C1_Stop();
    I2C1_Read(MS5XXX_ADDR_R, data, 3);

    baro->ADC_value[1] = data[0] << 16 | data[1] << 8 | data[2];
}

/**
 *@brief start conversion for preasure
* @param CONVERSION_OSR oversampling value - use defined values in MS5XXX.h
 * */
void MS5XXX_start_conversion_preasure(uint8_t CONVERSION_OSR) {
    I2C1_Write(MS5XXX_ADDR_W, MS5XXX_CMD_ADC_CONV + MS5XXX_CMD_ADC_D1 + CONVERSION_OSR, NULL, 0);
    I2C1_Stop();
}

/**
 *@brief start conversion for tmeperature
* @param CONVERSION_OSR oversampling ratio value - use defined values in MS5XXX.h
 * */
void MS5XXX_start_conversion_temp(uint8_t CONVERSION_OSR) {
    I2C1_Write(MS5XXX_ADDR_W, MS5XXX_CMD_ADC_CONV + MS5XXX_CMD_ADC_D2 + CONVERSION_OSR, NULL, 0);
    I2C1_Stop();
}

/**
 *@brief simplest version of reading data from sensor with usage of delay functions
 *@param baro pointer for baro_t object where temperature and preasure can be saved.
* @param CONVERSION_OSR oversampling ratio value - use defined values in MS5XXX.h
* @note This function will block program for 2 conversions. Recomended only for initialization.
 * */
void MS5XXX_ADC_read_wait(baro_t* baro, uint8_t CONVERSION_OSR)
{
    uint8_t data[6];
    uint16_t delay_time = CONVERSION_OSR > MS5XXX_CMD_ADC_256 ? MS5XXX_CONVERSION_TIME * (1 << (CONVERSION_OSR / 0x02)) : MS5XXX_CONVERSION_TIME;

    // read pressure:
    I2C1_Write(MS5XXX_ADDR_W, MS5XXX_CMD_ADC_CONV + MS5XXX_CMD_ADC_D1 + CONVERSION_OSR, NULL, 0);
    I2C1_Stop();
    delay_micro(delay_time);
    I2C1_Write(MS5XXX_ADDR_W, MS5XXX_CMD_ADC_READ, NULL, 0);
    I2C1_Stop();
    I2C1_Read(MS5XXX_ADDR_R, data, 3);
    delay_micro(100);
    // read temp:
    I2C1_Write(MS5XXX_ADDR_W, MS5XXX_CMD_ADC_CONV + MS5XXX_CMD_ADC_D2 + CONVERSION_OSR, NULL, 0);
    I2C1_Stop();
    delay_micro(delay_time);
    I2C1_Write(MS5XXX_ADDR_W, MS5XXX_CMD_ADC_READ, NULL, 0);
    I2C1_Stop();
    I2C1_Read(MS5XXX_ADDR_R, &data[3], 3);

    baro->ADC_value[0] = data[0] << 16 | data[1] << 8 | data[2];
    baro->ADC_value[1] = data[3] << 16 | data[4] << 8 | data[5];
}

/**
 *@brief reset MS5XXX sensor
*/
void MS5XXX_reset()
{
    I2C1_Write(MS5XXX_ADDR_W, MS5XXX_CMD_RESET, NULL, 0);
    I2C1_Stop();
}

/**
 *@brief function read PROM registers
* @param data pointer to store read values (16 bytes)
 *@note Read data are saved in address which pointer is poiting on. Data are not modified -> coefficients need to be combined from these bytes.
*/
void MS5XXX_PROM_read(uint8_t* data)
{
    uint8_t address = MS5XXX_CMD_PROM_RD;
    for (uint8_t coef = 0; coef < 8;coef++) {
        I2C1_Write(MS5XXX_ADDR_W, address, NULL, 0);
        I2C1_Stop();
        I2C1_Read(MS5XXX_ADDR_R, &data[2 * coef], 2);
        address += 0x02;
    }
}

/**
* @brief calculate calibration coefficients with CRC check for MS5XXX sensor
* @param data pointer for bytes read from PROM (16 values)
* @param baro pointer for baro_t object where coefficients can be saved
* @retval true if coefficients read correctly, otherwise return false
* @attention For some sensors PROM data are differenty storaged (CRC can be in the first or last byte).
*/
static bool MS5XXX_get_coefficients(uint8_t* data, baro_t* baro) {
    uint16_t coeff[8]; // coeff defined as 8x uint16_t (for coefficients)
    uint16_t n_rem = 0; // crc remainder
    uint16_t crc_value = 0x00;

    // create coefficients from bytes:
    for (uint8_t i = 0;i < 8;i++) {

        coeff[i] = (*data) << 8 | *(data + 1);
        data += 2;
    }

#if MS5XXX_SENSOR == MS5611
    // CRC checking:
    crc_value = coeff[7] & 0x0FF; // save read CRC value
    coeff[7] = ((coeff[0]) & 0xFF00); // CRC byte is replaced by 0

#elif MS5XXX_SENSOR == MS5837
    crc_value = coeff[7] >> 12; // save read CRC value
    coeff[0] = ((coeff[0]) & 0x0FFF); // CRC byte is replaced by 0
    coeff[7] = 0; // Subsidiary value, set to 0

#endif

    for (uint8_t cnt = 0; cnt < 16; cnt++) { // operation is performed on bytes
        // choose LSB or MSB
        if (cnt % 2 == 1) {
            n_rem ^= (uint8_t)((coeff[cnt >> 1]) & 0x00FF);
        }
        else {
            n_rem ^= (uint8_t)(coeff[cnt >> 1] >> 8);
        }
        for (uint8_t n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & (0x8000)) {
                n_rem = (n_rem << 1) ^ 0x3000;
            }
            else {
                n_rem = (n_rem << 1);
            }
        }
    }
    n_rem = ((n_rem >> 12) & 0x000F); // final 4-bit remainder is CRC code
    if (crc_value == (n_rem ^ 0x00)) {
        // if CRC correct return true:
        baro->C1_SENS = coeff[1];
        baro->C2_OFF = coeff[2];
        baro->C3_TCS = coeff[3];
        baro->C4_TCO = coeff[4];
        baro->C5_T = coeff[5];
        baro->C6_TEMPSENS = coeff[6];
        return true;
    }
    else {
        // if CRC not correct return false:
        return false;
    }
}



