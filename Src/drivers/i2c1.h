/*
 * i2c1.h
 *
 *  Created on: 04.08.2022
 *      Author: symon
 */

#ifndef I2C1_H_
#define I2C1_H_

#include "stdbool.h"
 // Macros for sensor commands

#define PRE_SENS_RESET 0x1E
#define PRE_SENS_ADC_READ 0x00
#define PRE_SENS_PROM_READ_0 0xA0
#define PRE_SENS_PROM_READ_1 0xA2
#define PRE_SENS_PROM_READ_2 0xA4
#define PRE_SENS_PROM_READ_3 0xA6
#define PRE_SENS_PROM_READ_4 0xA8
#define PRE_SENS_PROM_READ_5 0xAA
#define PRE_SENS_PROM_READ_6 0xAC
#define PRE_SENS_D1_256 0x40
#define PRE_SENS_D1_512 0x42
#define PRE_SENS_D1_1024 0x44
#define PRE_SENS_D1_2048 0x46
#define PRE_SENS_D1_4096 0x48
#define PRE_SENS_D2_256 0x50
#define PRE_SENS_D2_512 0x52
#define PRE_SENS_D2_1024 0x54
#define PRE_SENS_D2_2048 0x56
#define PRE_SENS_D2_4096 0x58

#define SENSOR_ADRRESS 0x76<<1
//#define I2C1_TESTS

// coefficients which you've once read from PROM and keep for ever:
#define C1_SENS 30082
#define C2_OFF 28401
#define C3_TCS 17985
#define C4_TCO 18647
#define C5_T 26233
#define C6_TEMPSENS 26798


void pressure_sensor_reset();
void pressure_sensor_PROM_read(uint8_t* data);
void pressure_sensor_ADC_read(uint8_t* data);
void temp_sensor_ADC_read(uint8_t* data);
void pressure_and_temp_calculation(uint8_t* raw_data, float* pressure, float* temperature);
bool get_coefficients(uint8_t* data);
void calibrate_sensor();

extern float depth_ses_temperature;
extern float depth_sens_pressure;

#endif /* I2C1_H_ */
