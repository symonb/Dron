/*
 * MPU6000.c
 *
 *  Created on: 03.01.2021
 *
 */

#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include <math.h>
#include "MPU6000.h"

static void setup_conf();
static void setup_gyro();
static void setup_acc();
void SPI_transmit(uint8_t*data, int size);
void SPI_receive(uint8_t adress_of_register,uint8_t*data, int size);
void failsafe_CONF();
void failsafe_I2C();

double M_rotacji[3][3] = { { (ACC_CALIBRATION_X_X - ACC_PITCH_OFFSET)
		/ sqrt(
				pow(ACC_CALIBRATION_X_Z - ACC_YAW_OFFSET, 2)
						+ pow(ACC_CALIBRATION_X_Y - ACC_ROLL_OFFSET, 2)
						+ pow(ACC_CALIBRATION_X_X - ACC_PITCH_OFFSET, 2)),
		(ACC_CALIBRATION_X_Y - ACC_ROLL_OFFSET)
				/ sqrt(
						pow(ACC_CALIBRATION_X_Z - ACC_YAW_OFFSET, 2)
								+ pow(ACC_CALIBRATION_X_Y - ACC_ROLL_OFFSET, 2)
								+ pow(ACC_CALIBRATION_X_X - ACC_PITCH_OFFSET,
										2)), (ACC_CALIBRATION_X_Z
				- ACC_YAW_OFFSET)
				/ sqrt(
						pow(ACC_CALIBRATION_X_Z - ACC_YAW_OFFSET, 2)
								+ pow(ACC_CALIBRATION_X_Y - ACC_ROLL_OFFSET, 2)
								+ pow(ACC_CALIBRATION_X_X - ACC_PITCH_OFFSET,
										2)) }, { (ACC_CALIBRATION_Y_X
		- ACC_PITCH_OFFSET)
		/ sqrt(
				pow(ACC_CALIBRATION_Y_Z - ACC_YAW_OFFSET, 2)
						+ pow(ACC_CALIBRATION_Y_Y - ACC_ROLL_OFFSET, 2)
						+ pow(ACC_CALIBRATION_Y_X - ACC_PITCH_OFFSET, 2)),
		(ACC_CALIBRATION_Y_Y - ACC_ROLL_OFFSET)
				/ sqrt(
						pow(ACC_CALIBRATION_Y_Z - ACC_YAW_OFFSET, 2)
								+ pow(ACC_CALIBRATION_Y_Y - ACC_ROLL_OFFSET, 2)
								+ pow(ACC_CALIBRATION_Y_X - ACC_PITCH_OFFSET,
										2)), (ACC_CALIBRATION_Y_Z
				- ACC_YAW_OFFSET)
				/ sqrt(
						pow(ACC_CALIBRATION_Y_Z - ACC_YAW_OFFSET, 2)
								+ pow(ACC_CALIBRATION_Y_Y - ACC_ROLL_OFFSET, 2)
								+ pow(ACC_CALIBRATION_Y_X - ACC_PITCH_OFFSET,
										2)) }, { (ACC_CALIBRATION_Z_X
		- ACC_PITCH_OFFSET)
		/ sqrt(
				pow(ACC_CALIBRATION_Z_Z - ACC_YAW_OFFSET, 2)
						+ pow(ACC_CALIBRATION_Z_Y - ACC_ROLL_OFFSET, 2)
						+ pow(ACC_CALIBRATION_Z_X - ACC_PITCH_OFFSET, 2)),
		(ACC_CALIBRATION_Z_Y - ACC_ROLL_OFFSET)
				/ sqrt(
						pow(ACC_CALIBRATION_Z_Z - ACC_YAW_OFFSET, 2)
								+ pow(ACC_CALIBRATION_Z_Y - ACC_ROLL_OFFSET, 2)
								+ pow(ACC_CALIBRATION_Z_X - ACC_PITCH_OFFSET,
										2)), (ACC_CALIBRATION_Z_Z
				- ACC_YAW_OFFSET)
				/ sqrt(
						pow(ACC_CALIBRATION_Z_Z - ACC_YAW_OFFSET, 2)
								+ pow(ACC_CALIBRATION_Z_Y - ACC_ROLL_OFFSET, 2)
								+ pow(ACC_CALIBRATION_Z_X - ACC_PITCH_OFFSET,
										2)) } };

uint8_t read_write_tab[14];
static volatile uint8_t read_write_quantity;
static float time_flag4_1;

//for debugging only:
static uint32_t pak1 = 0;

void I2C1_IRQHandler() {
	static uint8_t i = 0;
	if (I2C1->SR1 |= I2C_SR1_RXNE) {
		read_write_tab[i] = I2C1->DR;
		i++;
		if (i > read_write_quantity - 1) {
			i = 0;

			I2C1_read_write_flag = 1;

		}
	}
}

void EXTI4_IRQHandler() {
	if ((EXTI->PR & EXTI_PR_PR4)) {
		EXTI->PR |= EXTI_PR_PR4; // clear this bit setting it high
		pak1++;
		imu_received = 0;
		EXTI->IMR &= ~EXTI_IMR_IM4;
		read_all();
		//INSTRUCTIONS FOR READING IMU SENSORS

	}
}

// DO PRZEROBIENIA NA I2C NA F4 plus SPI na F4 dla MPU6000

void setup_MPU6000() {
	setup_conf();
	setup_gyro();
	setup_acc();
	//change speed of SPI to 10.5/2 [MHz]
	SPI1->CR1 &= ~SPI_CR1_BR;
	SPI1->CR1 |=  SPI_CR1_BR_2;

}

void SPI_enable() {
	SPI1->CR1 |= SPI_CR1_SPE; 			//	enabling SPI1

}

void SPI_disable() {
	SPI1->CR1 &= ~SPI_CR1_SPE; 			//	disabling SPI1

}

void CS_enable() {
	GPIOA->BSRR |= GPIO_BSRR_BR4;
}

void CS_disable() {
	GPIOA->BSRR |= GPIO_BSRR_BS4;
}

void SPI_transmit(uint8_t *data, int size) {
	//----------------STEPS--------------------
	/* 1 Wait for TXE bit to set in the Status Register
	 * 2 Write the data to the Data register
	 * 3 After the data has been transmitted, wait for the BSY bit to reset in Status Register
	 * 4 Clear the Overrun flag by reading DR and SR
	 * */
	int i = 0;
	int rx_data;
	while (!((SPI1->SR) & SPI_SR_TXE)) {
		; 			//wait
	}
	SPI1->DR = data[i];
	i++;

	while (i < size) {
		while (!((SPI1->SR) & SPI_SR_TXE)) {
			; 			//wait
		}
		SPI1->DR = data[i];
		i++;
		while (!((SPI1->SR) & SPI_SR_RXNE)) {
			; 			//wait
		}
		rx_data=SPI1->DR;
	}
	while (!((SPI1->SR) & SPI_SR_RXNE)) {
				; 			//wait
			}
	rx_data=SPI1->DR;
	while (!((SPI1->SR) & SPI_SR_TXE)) {
		; 			//wait
	}
	while (((SPI1->SR) & SPI_SR_BSY)) {
		;
	}

}

void SPI_receive(uint8_t adress_of_register,uint8_t *data, int size) {
	//----------------STEPS--------------------
	/* 1 Wait for BSY bit to reset in the Status Register
	 * 2 Send some data (anything) before reading from the Data register
	 * 3 Wait for the RXNE bit to set in Status Register
	 * 4 Read data from DR
	 * */

	int i = 0;
		while (!((SPI1->SR) & SPI_SR_TXE)) {
			; 			//wait
		}
		SPI1->DR = adress_of_register;


		while (i < size-1) {
			while (!((SPI1->SR) & SPI_SR_TXE)) {
				; 			//wait
			}
			SPI1->DR = 0x0;//send anything IMPORTANT!

			while (!((SPI1->SR) & SPI_SR_RXNE)) {
				; 			//wait
			}
			data[i]=SPI1->DR;
			i++;
		}
		while (!((SPI1->SR) & SPI_SR_RXNE)) {
					; 			//wait
				}
		data[i]=SPI1->DR;
		while (!((SPI1->SR) & SPI_SR_TXE)) {
			; 			//wait
		}
		while (((SPI1->SR) & SPI_SR_BSY)) {
			;
		}


//	while (size) {
//		while (((SPI1->SR) & SPI_SR_BSY)) {
//			; 			//wait
//		}
//		SPI1->DR = 0x0; //send anything IMPORTANT!
//		while (!((SPI1->SR) & SPI_SR_RXNE)) {
//			; //wait
//		}
//		*data = SPI1->DR;
//		data++;
//		size--;
//	}

}

void MPU6000_SPI_write(uint8_t adress_of_register, uint8_t value) {
	uint8_t data[2];
	data[0] = adress_of_register & ~0x80; //first bit of 1 byte has to be write (0) or read(1)
	data[1] = value;
	CS_enable();
	SPI_transmit(data, 2);
	CS_disable();
}

void MPU6000_SPI_read(uint8_t adress_of_register, uint8_t *memory_adress,
		int number_of_bytes) {
	adress_of_register |= 0x80; //first bit of 1 byte has to be write (0) or read(1)
	CS_enable();
	SPI_receive(adress_of_register,memory_adress, number_of_bytes);
	CS_disable();
}

//-------main MPU6000 setting-----------
void setup_conf() {

	// enable SPI1
	SPI_enable();

	// 0x6B - address of Power Management 1 register:
	// set 0x80 in this register (RESET)
	MPU6000_SPI_write(0x6B, 0x80);
	//delay of min. 0.1[s] according the MPU6000 datasheet
	delay_mili(150);

	// 0x68 - address of Signal Path Reset register:
	// set 0x7 in this register (RESET GYRO, ACC, TEMP)
	MPU6000_SPI_write(0x68, 0x7);
	//delay of min. 0.1[s] according the MPU6000 datasheet
	delay_mili(150);

	// 0x6A - address of (106)  User Control register:
	// Reset FIFO
//	MPU6000_SPI_write(0x6A, 0x4);
//	delay_mili(150);

	// 0x6B - address of (107) Power Management 1 register:
	// set 0x3 in this register (SLEEP -> 0 and PLL with Z axis gyroscope reference)
	MPU6000_SPI_write(0x6B, 0x3);
	delay_micro(15);

	// 0x23 - address of (35) FIFO Enable register:
	// setting all sensors to FIFO
	//MPU6000_SPI_write(0x23, 0xFF);

	// 0x6A - address of (106)  User Control register:
	//  and disable I2C
	MPU6000_SPI_write(0x6A, 0x10);
	delay_micro(15);

	// 0x1A - address of (26) Configuration register:
	// setting low pass filter in this register
	MPU6000_SPI_write(0x1A, 0x01);
	delay_micro(15);

	//0x38 - address of (56) Interrupt Enable register:
	// setting interrupt source as Data Register
	MPU6000_SPI_write(0x38, 0x1);
	delay_micro(15);

	SPI_disable();
}
static void setup_gyro() {
	//---------setting Gyro--------
	//start communication:

	// enable SPI1
	SPI_enable();

	// 0x1B- address of Gyroscope Configuration register:
	// set +/-1000[deg/s]
	MPU6000_SPI_write(0x1B, 0x10);
	delay_micro(15);

	SPI_disable();
}
static void setup_acc() {
	//---------setting Accelerometer-----------
	//	start communication:

	// enable SPI1
	SPI_enable();

	//	0x1C - address of Accelerometer Configuration register:
	// set +/-8[g]

	MPU6000_SPI_write(0x1C, 0x10);
	delay_micro(15);

	SPI_disable();
}

void read_all() {
	I2C1_read_write_flag = 0;
	SPI_enable();
	MPU6000_SPI_read(0x3B, &read_write_tab[0], 1);
	MPU6000_SPI_read(0x3C, &read_write_tab[1], 1);
	MPU6000_SPI_read(0x3D, &read_write_tab[2], 1);
	MPU6000_SPI_read(0x3E, &read_write_tab[3], 1);
	MPU6000_SPI_read(0x3F, &read_write_tab[4], 1);
	MPU6000_SPI_read(0x40, &read_write_tab[5], 1);
	MPU6000_SPI_read(0x41, &read_write_tab[6], 1);
	MPU6000_SPI_read(0x42, &read_write_tab[7], 1);
	MPU6000_SPI_read(0x43, &read_write_tab[8], 1);
	MPU6000_SPI_read(0x44, &read_write_tab[9], 1);
	MPU6000_SPI_read(0x45, &read_write_tab[10], 1);
	MPU6000_SPI_read(0x46, &read_write_tab[11], 1);
	MPU6000_SPI_read(0x47, &read_write_tab[12], 1);
	MPU6000_SPI_read(0x48, &read_write_tab[13], 1);
	SPI_disable();
	imu_received = 1;
}

void rewrite_data() {
	if (imu_received) {
		for (int i = 0; i < 3; i++) {
			//gyro:
			Gyro_Acc[i] = read_write_tab[2 * i + 8] << 8
					| read_write_tab[2 * i + 9];
			//acc:
			Gyro_Acc[i + 3] = read_write_tab[2 * i] << 8
					| read_write_tab[2 * i + 1];

		}

		double temporary[6] = { 0 };
		for (int j = 0; j < 3; j++) {

			for (int i = 0; i < 3; i++) {
				temporary[j] += Gyro_Acc[i] * M_rotacji[i][j];
				temporary[j + 3] += Gyro_Acc[i + 3] * M_rotacji[i][j];
			}
		}

		Gyro_Acc[0] = temporary[0] - GYRO_ROLL_OFFSET;
		Gyro_Acc[1] = temporary[1] - GYRO_PITCH_OFFSET;
		Gyro_Acc[2] = temporary[2] - GYRO_YAW_OFFSET;
		Gyro_Acc[3] = temporary[3] - ACC_ROLL_OFFSET;
		Gyro_Acc[4] = temporary[4] - ACC_PITCH_OFFSET;
		Gyro_Acc[5] = temporary[5] - ACC_YAW_OFFSET;

		//temperature:
		Gyro_Acc[6] = read_write_tab[6] << 8 | read_write_tab[7];
		EXTI->IMR |= EXTI_IMR_IM4;
	}
}

void failsafe_CONF() {
	//	waiting as Data will be sent or failsafe if set time passed
	if ((get_Global_Time() - time_flag4_1) >= MAX_I2C_TIME) {
		failsafe_type = 4;
		EXTI->SWIER |= EXTI_SWIER_SWIER15;
	}
}
void failsafe_I2C() {
	//	waiting as Data will be sent or failsafe if set time passed
	if ((get_Global_Time() - time_flag4_1) >= MAX_I2C_TIME) {
		failsafe_type = 5;
		EXTI->SWIER |= EXTI_SWIER_SWIER15;
	}
}
