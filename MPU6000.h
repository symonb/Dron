/*
 * MPU6050.h
 *
 *  Created on: 03.01.2021
 *      Author: filip
 */

#ifndef MPU6000_H_
#define MPU6000_H_

void setup_MPU6000();
void SPI_enable();
void SPI_disable();
void CS_enable();
void CS_disable();
void MPU6000_SPI_write(uint8_t adress_of_register, uint8_t value);
void MPU6000_SPI_read(uint8_t adress_of_register,uint8_t* memory_adress,int number_of_bytes);
void rewrite_data();
void read_all();

#endif /* MPU6000_H_ */
