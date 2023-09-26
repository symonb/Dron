/*
 * I2C1.h
 *
 *  Created on: 04.08.2022
 *      Author: symon
 */

#ifndef I2C1_H_
#define I2C1_H_

void I2C1_Start();
void I2C1_Stop();
void I2C1_reset(); // do not use reconfiguration
void I2C1_Write(uint8_t address, uint8_t command, uint8_t* data, uint16_t size);
void I2C1_Read(uint8_t address, uint8_t* data, uint16_t size);
// void I2C1_Read_DMA(uint8_t address, uint8_t* data, uint16_t size);

#endif /* I2C1_H_ */
