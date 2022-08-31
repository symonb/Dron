/*
 * i2c1.c
 *
 *  Created on: 04.08.2022
 *      Author: symon
 */


/*
 * 1 reset command
 * 2 read PROM
 * 3 read data
 *
 */


#include <math.h>
#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "i2c1.h"

// variable for DMA purpose (you can use any other):
uint8_t DMA_receiving_table[20];
uint8_t DMA_transmitting_table[10];
float depth_sens_temperature = 0;
float depth_sens_pressure = 0;

static void I2C1_Start();
static void I2C1_Stop();
static void I2C1_Write(uint8_t address, uint8_t command, uint8_t *data, uint16_t size);
static void I2C1_Read(uint8_t address, uint8_t *data, uint16_t size);
static void I2C1_Read_DMA(uint8_t address, uint8_t *data, uint16_t size);


//RX:
#if defined(USE_I2C1)
void DMA1_Stream0_IRQHandler(void) {

	//if stream0 transfer is completed:
	if (DMA1->LISR & DMA_LISR_TCIF0) {
		DMA1->LIFCR |= DMA_LIFCR_CTCIF0;

		DMA1_Stream0->CR &= ~DMA_SxCR_EN;
		// generate STOP conditions:
		I2C1->CR1 |= I2C_CR1_STOP;
	}
}
#endif

// follow this code to get coefficients (only once, then use them as constants):
void calibrate_sensor(){

	// tests of I2C on MPU6050:

#if defined(I2C1_TESTS)

	uint8_t MPU6050_ADRRESS = 0x68<<1;

	DMA_transmitting_table[0] = 0x00; //reset SLEEP bit

	I2C1_Write(MPU6050_ADRRESS,0x6B,DMA_transmitting_table,1);

 	I2C1_Write(MPU6050_ADRRESS,0x41,DMA_transmitting_table,0);

	I2C1_Read_DMA(MPU6050_ADRRESS,DMA_receiving_table,2);

	// end of tests
#else

	pressure_sensor_reset();

	#if defined(DEPTH_SENS_GET_COEFFICIENTS)
		pressure_sensor_PROM_read(DMA_receiving_table);

		get_coefficients(DMA_receiving_table);

	#else
	pressure_sensor_ADC_read(DMA_receiving_table);
	pressure_and_temp_calculation(DMA_receiving_table,&depth_sens_pressure,&depth_sens_temperature);
	depth_sens_temperature=0;
	depth_sens_pressure=0;
	#endif

#endif
}


// functions for I2C usage:

static void I2C1_Start() {

	I2C1->CR1 |= I2C_CR1_ACK;
	I2C1->CR1 |= I2C_CR1_START;
	while (!(I2C1->SR1 & I2C_SR1_SB)) {
	}
}

static void I2C1_Stop() {
	I2C1->CR1 |= I2C_CR1_STOP;
}

static void I2C1_Write(uint8_t address, uint8_t command, uint8_t *data, uint16_t size) {

	I2C1_Start();
	I2C1->DR = address;
	//	wait for ADDR bit:
	while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
	}
	// for clearing ADDR bit read also SR2:
	I2C1->SR2;
	//	send command:
	while (!(I2C1->SR1 & I2C_SR1_TXE)) {
		;
	}
	I2C1->DR = command;
	//	send data:
	while (size) {
		while (!(I2C1->SR1 & I2C_SR1_TXE)) {
		}
		I2C1->DR = (uint32_t) *data;

		data++;
		size--;
	}
	while (!(I2C1->SR1 & I2C_SR1_BTF) && !(I2C1->SR1 & I2C_SR1_TXE)) {
	}
}

static void I2C1_Read(uint8_t address, uint8_t *data, uint16_t size) {

	I2C1_Start();

	I2C1->DR = (address| 0x1);
	// wait for ADDR bit:
	while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
	}

	// different steps for 1, 2, more bytes reception:
	if (size == 2) {
		I2C1->CR1 &= ~I2C_CR1_ACK;
		I2C1->CR1 |= I2C_CR1_POS;

		// for clearing ADDR bit read also SR2:
		I2C1->SR2;
		while (!(I2C1->SR1 & I2C_SR1_BTF)) {
		}
		I2C1->CR1 |= I2C_CR1_STOP;
		// now receive 2 bytes:
		while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
		}
		*data = I2C1->DR;
		data++;
		while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
		}
		*data = I2C1->DR;
		return;
	}
	if (size == 1) {
		I2C1->CR1 &= ~I2C_CR1_ACK;
		// for clearing ADDR bit read also SR2:
		I2C1->SR2;
		I2C1->CR1 |= I2C_CR1_STOP;
		while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
			//wait for byte to read
		}
		*data = I2C1->DR;
		return;
	}

	// for clearing ADDR bit read also SR2:
	I2C1->SR2;

	while (size > 3) {

		while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
		}
		*data = I2C1->DR;
		// ACK will be send automatically
		data++;
		size--;
	}

	while (!(I2C1->SR1 & I2C_SR1_BTF)) {
	}
	I2C1->CR1 &= ~I2C_CR1_ACK;
	*data = I2C1->DR;
	data++;
	while (!(I2C1->SR1 & I2C_SR1_BTF)) {
	}
	I2C1->CR1 |= I2C_CR1_STOP;
	// now receive 2 last bytes:
	while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
	}
	*data = I2C1->DR;
	data++;
	while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
	}
	*data = I2C1->DR;
}

static void I2C1_Read_DMA(uint8_t address, uint8_t *data, uint16_t size){
	DMA1_Stream0->NDTR = size;
	DMA1_Stream0->M0AR = (uint32_t) (data);

	I2C1_Start();

	I2C1->DR = (address| 0x1);
	// wait for ADDR bit:
	while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
	}
	// activate DMA:
	DMA1_Stream0->CR |= DMA_SxCR_EN;

	// for clearing ADDR bit read also SR2:
	I2C1->SR2;

}


void pressure_sensor_reset() {
	I2C1_Write(SENSOR_ADRRESS, PRE_SENS_RESET, NULL, 0);
	I2C1_Stop();
}

void pressure_sensor_PROM_read(uint8_t *data) {
	I2C1_Write(SENSOR_ADRRESS, PRE_SENS_PROM_READ_0, NULL, 0);
	I2C1_Stop();
	I2C1_Read(SENSOR_ADRRESS, data, 2);
	data += 2;
	I2C1_Write(SENSOR_ADRRESS, PRE_SENS_PROM_READ_1, NULL, 0);
	I2C1_Stop();
	I2C1_Read(SENSOR_ADRRESS, data, 2);
	data += 2;
	I2C1_Write(SENSOR_ADRRESS, PRE_SENS_PROM_READ_2, NULL, 0);
	I2C1_Stop();
	I2C1_Read(SENSOR_ADRRESS, data, 2);
	data += 2;
	I2C1_Write(SENSOR_ADRRESS, PRE_SENS_PROM_READ_3, NULL, 0);
	I2C1_Stop();
	I2C1_Read(SENSOR_ADRRESS, data, 2);
	data += 2;
	I2C1_Write(SENSOR_ADRRESS, PRE_SENS_PROM_READ_4, NULL, 0);
	I2C1_Stop();
	I2C1_Read(SENSOR_ADRRESS, data, 2);
	data += 2;
	I2C1_Write(SENSOR_ADRRESS, PRE_SENS_PROM_READ_5, NULL, 0);
	I2C1_Stop();
	I2C1_Read(SENSOR_ADRRESS, data, 2);
	data += 2;
	I2C1_Write(SENSOR_ADRRESS, PRE_SENS_PROM_READ_6, NULL, 0);
	I2C1_Stop();
	I2C1_Read(SENSOR_ADRRESS, data, 2);
}

void pressure_sensor_ADC_read(uint8_t *data) {
	// read pressure:
	I2C1_Write(SENSOR_ADRRESS, PRE_SENS_D1_1024, NULL, 0);
	I2C1_Stop();
	delay_micro(2500);
	I2C1_Write(SENSOR_ADRRESS, PRE_SENS_ADC_READ, NULL, 0);
	I2C1_Stop();
	I2C1_Read(SENSOR_ADRRESS, data, 3);

	// read temp:
	data+=3;
	I2C1_Write(SENSOR_ADRRESS, PRE_SENS_D2_1024, NULL, 0);
	I2C1_Stop();
	delay_micro(2500);
	I2C1_Write(SENSOR_ADRRESS, PRE_SENS_ADC_READ, NULL, 0);
	I2C1_Stop();
	I2C1_Read(SENSOR_ADRRESS, data, 3);
}


void pressure_and_temp_calculation(uint8_t *raw_data, float *pressure, float *temperature){
	/* implemented according to page 10, 11, 12:
	 *	https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS5837-30BA%7FB1%7Fpdf%7FEnglish%7FENG_DS_MS5837-30BA_B1.pdf%7FCAT-BLPS0017
	 */

	uint32_t D1;
	uint32_t D2;
	int64_t dT;
	int64_t TEMP;
	int64_t OFF;
	int64_t SENS;
	int64_t P;
	int64_t Ti;
	int64_t OFFi;
	int64_t SENSi;

	// create raw data from read bytes:
	D1 = *raw_data<<16 | *(raw_data+1)<<8 | *(raw_data+2);
	raw_data += 3;
	D2 = *raw_data<<16 | *(raw_data+1)<<8 | *(raw_data+2);

	// first order temperature compensation:
	dT = D2 - C5_T * 256;
	TEMP =(2000 + dT * C6_TEMPSENS / 8388608);
	OFF = C2_OFF * 65536 + C4_TCO * dT / 128;
	SENS = C1_SENS * 32768 + C3_TCS * dT / 256;
	P = (D1 * SENS/2097152 - OFF) / 8192;

	// second order of compensation:
	if(TEMP/100 < 20){
		Ti = 3 * dT*dT / 8589934592;
		OFFi = 3 * (TEMP - 2000)*(TEMP - 2000) / 2;
		SENSi = 5 * (TEMP - 2000)*(TEMP - 2000) / 8;
		if(TEMP/100 < -15){
			OFFi = OFFi + 7 * (TEMP + 1500)*(TEMP + 1500);
			SENSi = SENSi + 4 * (TEMP + 1500)*(TEMP + 1500);
		}
	}
	else{
		Ti = 2 * dT*dT / 137438953472.;
		OFFi = (TEMP - 2000)*(TEMP - 2000)/ 16;
		SENSi = 0;
	}
	*temperature = (TEMP - Ti) / 100.f;	// in [C]
	*pressure = ((( D1 * (SENS - SENSi) ) / 2097152 - (OFF - OFFi)) / 8192) / 10.f ; // in [mbar]
}

bool get_coefficients(uint8_t *data){
	uint16_t n_prom[8]; // n_prom defined as 8x uint16_t (for coefficients)
	uint16_t n_rem = 0; // crc remainder
	uint16_t crc_value = 0x00;

	// create coefficients from bytes:
	for(uint8_t i=0;i<8;i++){
		n_prom[i]=*data<<8 |*(data+1);
		data+=2;
	}

	// CRC checking:
	crc_value = n_prom[0]>>12; // save read CRC value
	n_prom[0] = ((n_prom[0]) & 0x0FFF); // CRC byte is replaced by 0
	n_prom[7] = 0; // Subsidiary value, set to 0

	for (uint8_t cnt = 0; cnt < 16; cnt++) { // operation is performed on bytes
		// choose LSB or MSB
		if (cnt % 2 == 1)
			n_rem ^= (uint8_t) ((n_prom[cnt >> 1]) & 0x00FF);
		else
			n_rem ^= (uint8_t) (n_prom[cnt >> 1] >> 8);
		for (uint8_t n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & (0x8000)) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}
	n_rem = ((n_rem >> 12) & 0x000F); // final 4-bit remainder is CRC code
	if (crc_value == (n_rem ^ 0x00)){
		return true;
		}
	else{
		return false;
	}
}

