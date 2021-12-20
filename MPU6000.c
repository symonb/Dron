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
#include "filters.h"
#include <math.h>
#include "MPU6000.h"

static void setup_conf();
static void setup_gyro();
static void setup_acc();
static void SPI1_enable();
static void SPI1_disable();
static void CS_IMU_enable();
static void CS_IMU_disable();
static void SPI_transmit(uint8_t*data, int size);
static void SPI_receive(uint8_t address_of_register, uint8_t*data, int size);
static void failsafe_CONF();
static void failsafe_I2C();
static void failsafe_SPI();
static void average_filters_setup();
static void MPU6000_self_test_configuration();
static void MPU6000_self_test_measurements();

//float M_rotacji[3][3] = { { (ACC_CALIBRATION_X_X - ACC_PITCH_OFFSET)
//		/ sqrtf(
//				powf(ACC_CALIBRATION_X_Z - ACC_YAW_OFFSET, 2)
//						+ powf(ACC_CALIBRATION_X_Y - ACC_ROLL_OFFSET, 2)
//						+ powf(ACC_CALIBRATION_X_X - ACC_PITCH_OFFSET, 2)),
//		(ACC_CALIBRATION_X_Y - ACC_ROLL_OFFSET)
//				/ sqrtf(
//						powf(ACC_CALIBRATION_X_Z - ACC_YAW_OFFSET, 2)
//								+ powf(ACC_CALIBRATION_X_Y - ACC_ROLL_OFFSET, 2)
//								+ powf(ACC_CALIBRATION_X_X - ACC_PITCH_OFFSET,
//										2)), (ACC_CALIBRATION_X_Z
//				- ACC_YAW_OFFSET)
//				/ sqrtf(
//						powf(ACC_CALIBRATION_X_Z - ACC_YAW_OFFSET, 2)
//								+ powf(ACC_CALIBRATION_X_Y - ACC_ROLL_OFFSET, 2)
//								+ powf(ACC_CALIBRATION_X_X - ACC_PITCH_OFFSET,
//										2)) }, { (ACC_CALIBRATION_Y_X
//		- ACC_PITCH_OFFSET)
//		/ sqrtf(
//				powf(ACC_CALIBRATION_Y_Z - ACC_YAW_OFFSET, 2)
//						+ powf(ACC_CALIBRATION_Y_Y - ACC_ROLL_OFFSET, 2)
//						+ powf(ACC_CALIBRATION_Y_X - ACC_PITCH_OFFSET, 2)),
//		(ACC_CALIBRATION_Y_Y - ACC_ROLL_OFFSET)
//				/ sqrtf(
//						powf(ACC_CALIBRATION_Y_Z - ACC_YAW_OFFSET, 2)
//								+ powf(ACC_CALIBRATION_Y_Y - ACC_ROLL_OFFSET, 2)
//								+ powf(ACC_CALIBRATION_Y_X - ACC_PITCH_OFFSET,
//										2)), (ACC_CALIBRATION_Y_Z
//				- ACC_YAW_OFFSET)
//				/ sqrtf(
//						powf(ACC_CALIBRATION_Y_Z - ACC_YAW_OFFSET, 2)
//								+ powf(ACC_CALIBRATION_Y_Y - ACC_ROLL_OFFSET, 2)
//								+ powf(ACC_CALIBRATION_Y_X - ACC_PITCH_OFFSET,
//										2)) }, { (ACC_CALIBRATION_Z_X
//		- ACC_PITCH_OFFSET)
//		/ sqrtf(
//				powf(ACC_CALIBRATION_Z_Z - ACC_YAW_OFFSET, 2)
//						+ powf(ACC_CALIBRATION_Z_Y - ACC_ROLL_OFFSET, 2)
//						+ powf(ACC_CALIBRATION_Z_X - ACC_PITCH_OFFSET, 2)),
//		(ACC_CALIBRATION_Z_Y - ACC_ROLL_OFFSET)
//				/ sqrtf(
//						powf(ACC_CALIBRATION_Z_Z - ACC_YAW_OFFSET, 2)
//								+ powf(ACC_CALIBRATION_Z_Y - ACC_ROLL_OFFSET, 2)
//								+ powf(ACC_CALIBRATION_Z_X - ACC_PITCH_OFFSET,
//										2)), (ACC_CALIBRATION_Z_Z
//				- ACC_YAW_OFFSET)
//				/ sqrtf(
//						powf(ACC_CALIBRATION_Z_Z - ACC_YAW_OFFSET, 2)
//								+ powf(ACC_CALIBRATION_Z_Y - ACC_ROLL_OFFSET, 2)
//								+ powf(ACC_CALIBRATION_Z_X - ACC_PITCH_OFFSET,
//										2)) } };

float M_rotacji[3][3] = { { -1, 0, 0 }, { 0, -1, 0 }, { 0, 0, 1 } };

uint8_t read_write_tab[14];
static volatile uint8_t read_write_quantity;
static float time_flag4_1; //for SPI timeout detection

static FIR_Filter average_gyro_X;
static FIR_Filter average_gyro_Y;
static FIR_Filter average_gyro_Z;
static FIR_Filter average_acc_X;
static FIR_Filter average_acc_Y;
static FIR_Filter average_acc_Z;
//for debugging only:

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
		static uint32_t counter;
		//INSTRUCTIONS FOR READING IMU SENSORS:
		imu_received = 0;
		EXTI->IMR &= ~EXTI_IMR_IM4;
		//block RX reading while IMU reading
		USART1->CR1 &= ~USART_CR1_RXNEIE;
		read_all();
		counter++;
		//unblock RX reading while IMU reading
		USART1->CR1 |= USART_CR1_RXNEIE;
	}
}

void setup_MPU6000() {
	delay_mili(30); // MPU datasheet specifies 30ms
	setup_conf();
	setup_gyro();
	setup_acc();
	average_filters_setup();
	//change speed of SPI up to 10.5 [MHz] (only for reading sensors) IT DOESN T WORK
//	SPI1->CR1 &= ~SPI_CR1_BR;
//	SPI1->CR1 |=  SPI_CR1_BR_1|SPI_CR1_BR_0;

}

static void SPI1_enable() {
	SPI1->CR1 |= SPI_CR1_SPE; 			//	enabling SPI1

}

static void SPI1_disable() {
	SPI1->CR1 &= ~SPI_CR1_SPE; 			//	disabling SPI1

}

static void CS_IMU_enable() {
	GPIOA->BSRR |= GPIO_BSRR_BR4;
}

static void CS_IMU_disable() {
	GPIOA->BSRR |= GPIO_BSRR_BS4;
}

static void SPI_transmit(uint8_t *data, int size) {
	//----------------STEPS--------------------
	/* 1 Wait for TXE bit to set in the Status Register
	 * 2 Write the Register Adress (&~x80 for writing) to the Data register
	 * 3 Write first byte that you want send to slave
	 * 4 Receive 1 data - it is anything (slave started sending it when received first bit of first byte so it is not interesting byte for sure) ignore it
	 * 5 Send second data and repeat until you receive all data you wanted
	 * 6 After last transmission you need to clear DR and wait until everything stop (more inf. in datasheet)
	 * */
	int i = 0;

	time_flag4_1 = get_Global_Time();
	while (!((SPI1->SR) & SPI_SR_TXE)) {
		failsafe_SPI(); 			//wait
	}
	SPI1->DR = data[i]; //first data - usually  slave's Register which you're interested in
	i++;

	while (i < size) {
		time_flag4_1 = get_Global_Time();
		while (!((SPI1->SR) & SPI_SR_TXE)) {
			failsafe_SPI(); 			//wait
		}
		SPI1->DR = data[i]; //second and following data sending as soon as TX flag is set
		i++;
	}

	time_flag4_1 = get_Global_Time();
	while (!((SPI1->SR) & SPI_SR_TXE)) {
		failsafe_SPI(); 			//wait
	}
	time_flag4_1 = get_Global_Time();
	while (((SPI1->SR) & SPI_SR_BSY)) {
		failsafe_SPI();				//wait
	}
	SPI1->DR;
	SPI1->SR;

}

static void SPI_receive(uint8_t address_of_register, uint8_t *data, int size) {
	//----------------STEPS--------------------
	/* 1 Wait for TXE flag and write address_of_register |0x80 (first bit = 1 - reading)
	 * 2 Wait for TXE bit and write anything since you want reading and slave will send your data only after receiving sth.
	 * 3 Wait for RXNE and receive first data - it is rubbish (started as soon as you started sending register address so it is not data from this register)
	 * 4 Wait for TXE send dummy data, wait for RXNE and receive your data. Repeat until you receive size-1 data
	 * 5 Wait for RXNE and receive last byte
	 * */

	time_flag4_1 = get_Global_Time();
	while (!((SPI1->SR) & SPI_SR_TXE) && failsafe_type != 6) {
		failsafe_SPI(); 			//wait
	}
	SPI1->DR = address_of_register; //first data - usually  slave's Register which you're interested in

	time_flag4_1 = get_Global_Time();
	while (!((SPI1->SR) & SPI_SR_TXE) && failsafe_type != 6) {
		failsafe_SPI(); 			//wait
	}
	SPI1->DR = 0xFF;	//send something - IMPORTANT!

	time_flag4_1 = get_Global_Time();
	while (!((SPI1->SR) & SPI_SR_RXNE) && failsafe_type != 6) {
		failsafe_SPI(); 			//wait
	}
	SPI1->DR;	//first byte - rubbish

	while (size > 1) {

		time_flag4_1 = get_Global_Time();
		while (!((SPI1->SR) & SPI_SR_TXE) && failsafe_type != 6) {
			failsafe_SPI(); 			//wait
		}
		SPI1->DR = 0xFF; 			//send anything IMPORTANT!
		time_flag4_1 = get_Global_Time();
		while (!((SPI1->SR) & SPI_SR_RXNE) && failsafe_type != 6) {
			failsafe_SPI(); 			//wait
		}
		*data++ = SPI1->DR;
		size--;
	}

	time_flag4_1 = get_Global_Time();
	while (!((SPI1->SR) & SPI_SR_RXNE) && failsafe_type != 6) {
		failsafe_SPI(); 			//wait
	}
	*data = SPI1->DR;
}

void MPU6000_SPI_write(uint8_t adress_of_register, uint8_t value) {
	uint8_t data[2];
	data[0] = adress_of_register & 0x7F; //first bit of 1 byte has to be write (0) or read(1)
	data[1] = value;
	CS_IMU_enable();
	SPI_transmit(data, 2);
	CS_IMU_disable();
}

void MPU6000_SPI_read(uint8_t adress_of_register, uint8_t *memory_adress,
		int number_of_bytes) {
	adress_of_register |= 0x80; //first bit of 1 byte has to be write (0) or read(1)
	CS_IMU_enable();
	SPI_receive(adress_of_register, memory_adress, number_of_bytes);
	CS_IMU_disable();
}

//-------main MPU6000 setting-----------
static void setup_conf() {

	// enable SPI1
	SPI1_enable();

	// 0x6A - address of (106)  User Control register:
	//  and disable I2C
	MPU6000_SPI_write(0x6A, 0x10);
	delay_micro(15);

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

	// 0x6B - address of (107) Power Management 1 register:
	// set 0x3 in this register (SLEEP -> 0 and PLL with Z axis gyroscope reference)
	MPU6000_SPI_write(0x6B, 0x3);
	delay_micro(15);

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

	SPI1_disable();
}
static void setup_gyro() {
	//---------setting Gyro--------
	//start communication:

	// enable SPI1
	SPI1_enable();

	// 0x1B- address of Gyroscope Configuration register:
	// set +/-1000[deg/s]
	MPU6000_SPI_write(0x1B, 0x10);
	delay_micro(15);

	SPI1_disable();
}
static void setup_acc() {
	//---------setting Accelerometer-----------
	//	start communication:

	// enable SPI1
	SPI1_enable();

	//	0x1C - address of Accelerometer Configuration register:
	// set +/-8[g]

	MPU6000_SPI_write(0x1C, 0x10);
	delay_micro(15);

	SPI1_disable();
}

void average_filters_setup() {

	average_gyro_X.length = 5;
	float value = 1. / average_gyro_X.length;
	FIR_Filter_Init(&average_gyro_X);

	for (uint8_t i = 0; i < average_gyro_X.length; i++) {
		average_gyro_X.impulse_responce[i] = value;
	}

	average_gyro_Y.length = 5;
	value = 1. / average_gyro_Y.length;
	FIR_Filter_Init(&average_gyro_Y);
	for (uint8_t i = 0; i < average_gyro_Y.length; i++) {
		average_gyro_Y.impulse_responce[i] = value;
	}

	average_gyro_Z.length = 5;
	value = 1. / average_gyro_Z.length;
	FIR_Filter_Init(&average_gyro_Z);
	for (uint8_t i = 0; i < average_gyro_Z.length; i++) {
		average_gyro_Z.impulse_responce[i] = value;
	}

	average_acc_X.length = 5;
	value = 1. / average_acc_X.length;
	FIR_Filter_Init(&average_acc_X);
	for (uint8_t i = 0; i < average_acc_X.length; i++) {
		average_acc_X.impulse_responce[i] = value;
	}

	average_acc_Y.length = 5;
	value = 1. / average_acc_Y.length;
	FIR_Filter_Init(&average_acc_Y);
	for (uint8_t i = 0; i < average_acc_Y.length; i++) {
		average_acc_Y.impulse_responce[i] = value;
	}

	average_acc_Z.length = 5;
	value = 1. / average_acc_Z.length;
	FIR_Filter_Init(&average_acc_Z);
	for (uint8_t i = 0; i < average_acc_Z.length; i++) {
		average_acc_Z.impulse_responce[i] = value;
	}

}

void read_all() {
//	uint8_t temp[117] = { 0 };
//		//X:
//		MPU6000_SPI_read(0x0D, temp, 117);
//		read_write_tab[0] = temp[0];
//		read_write_tab[1] = temp[1];

	read_acc();
	read_temp();
	read_gyro();
	imu_received = 1;
}

void read_acc() {

	SPI1_enable();
	static uint8_t temp[6];

	//X:
	MPU6000_SPI_read(0x3B, temp, 6);
	read_write_tab[0] = temp[0];
	read_write_tab[1] = temp[1];

	//Y:
	read_write_tab[2] = temp[2];
	read_write_tab[3] = temp[3];

	//Z:
	read_write_tab[4] = temp[4];
	read_write_tab[5] = temp[5];

	SPI1_disable();
}

void read_temp() {
	SPI1_enable();
	static uint8_t temp[2];
	MPU6000_SPI_read(0x3F, temp, 2);
	read_write_tab[6] = temp[0];
	read_write_tab[7] = temp[1];

	SPI1_disable();
}

void read_gyro() {
	SPI1_enable();
	static uint8_t temp[6];

	//X:
	MPU6000_SPI_read(0x43, temp, 6);
	read_write_tab[8] = temp[0];
	read_write_tab[9] = temp[1];

	//Y:
	read_write_tab[10] = temp[2];
	read_write_tab[11] = temp[3];

	//Z:
	read_write_tab[12] = temp[4];
	read_write_tab[13] = temp[5];

	SPI1_disable();
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

		float temporary[6] = { 0 };
		for (int j = 0; j < 3; j++) {

			for (int i = 0; i < 3; i++) {
				temporary[j] += Gyro_Acc[i] * M_rotacji[i][j];
				temporary[j + 3] += Gyro_Acc[i + 3] * M_rotacji[i][j];
			}
		}

		Gyro_Acc[0] = FIR_Filter_filtering(&average_gyro_X,
				temporary[0] - GYRO_ROLL_OFFSET);
		Gyro_Acc[1] = FIR_Filter_filtering(&average_gyro_Y,
				temporary[1] - GYRO_PITCH_OFFSET);
		Gyro_Acc[2] = FIR_Filter_filtering(&average_gyro_Z,
				temporary[2] - GYRO_YAW_OFFSET);
		Gyro_Acc[3] = FIR_Filter_filtering(&average_acc_X,
				temporary[3] - ACC_ROLL_OFFSET);
		Gyro_Acc[4] = FIR_Filter_filtering(&average_acc_Y,
				temporary[4] - ACC_PITCH_OFFSET);
		Gyro_Acc[5] = FIR_Filter_filtering(&average_acc_Z,
				temporary[5] - ACC_YAW_OFFSET);
		Gyro_Acc[6] = read_write_tab[6] << 8 | read_write_tab[7];

		//Save data to flash:
#if defined(USE_FLASH_BLACKBOX)

		if (blackbox_command == 1) {

//			for (uint8_t i = 0; i < 6; i++) {
//				flash_add_data_to_save(((uint16_t) temporary[i] >> 8) & 0xFF);
//				flash_add_data_to_save((uint16_t) temporary[i] & 0xFF);
//			}
			for (uint8_t i = 0; i < 1; i++) {
				flash_add_data_to_save((Gyro_Acc[i] >> 8) & 0xFF);
				flash_add_data_to_save(Gyro_Acc[i] & 0xFF);
			}
		}

#endif
		EXTI->IMR |= EXTI_IMR_IM4;

	}
}

static void failsafe_CONF() {
	//	waiting as Data will be sent or failsafe if set time passed
	if ((get_Global_Time() - time_flag4_1) >= TIMEOUT_VALUE) {
		failsafe_type = 4;
		EXTI->SWIER |= EXTI_SWIER_SWIER15;

	}
}
static void failsafe_I2C() {
	//	waiting as Data will be sent or failsafe if set time passed
	if ((get_Global_Time() - time_flag4_1) >= TIMEOUT_VALUE) {
		failsafe_type = 5;
		EXTI->SWIER |= EXTI_SWIER_SWIER15;
	}
}
static void failsafe_SPI() {
	//	waiting as Data will be sent or failsafe if set time passed
	if ((get_Global_Time() - time_flag4_1) >= TIMEOUT_VALUE) {
		failsafe_type = 6;
		EXTI->SWIER |= EXTI_SWIER_SWIER15;
	}
}

static void MPU6000_self_test_configuration() {
	SPI1_enable();
	// 0x1B- address of Gyroscope Configuration register:
	// set +/-250[deg/s] and Self_test activate
	MPU6000_SPI_write(0x1B, 0xE0);
	delay_micro(15);

	//	0x1C - address of Accelerometer Configuration register:
	// set +/-8[g]	and Self_test activate

	MPU6000_SPI_write(0x1C, 0xF0);
	delay_micro(15);

	SPI1_disable();

}
static void MPU6000_self_test_measurements(float temporary[]) {
	static int i;
	static float averagegyroX;
	static float averagegyroY;
	static float averagegyroZ;
	static float averageaccX;
	static float averageaccY;
	static float averageaccZ;

	static float averagegyroX_ST;
	static float averagegyroY_ST;
	static float averagegyroZ_ST;
	static float averageaccX_ST;
	static float averageaccY_ST;
	static float averageaccZ_ST;

	if (i < 1000) {
		averagegyroX_ST += temporary[0] / 1000;
		averagegyroY_ST += temporary[1] / 1000;
		averagegyroZ_ST += temporary[2] / 1000;
		averageaccX_ST += temporary[3] / 1000;
		averageaccY_ST += temporary[4] / 1000;
		averageaccZ_ST += temporary[5] / 1000;
		i++;
	} else if (i == 1000) {
		SPI1_enable();
		// 0x1B- address of Gyroscope Configuration register:
		// set +/-250[deg/s] and Self_test deactivate
		MPU6000_SPI_write(0x1B, 0x00);
		delay_micro(15);

		//	0x1C - address of Accelerometer Configuration register:
		// set +/-8[g]	and Self_test deactivate

		MPU6000_SPI_write(0x1C, 0x10);
		delay_micro(15);

		SPI1_disable();
		i++;
	} else if (i < 2001) {
		averagegyroX += temporary[0] / 1000;
		averagegyroY += temporary[1] / 1000;
		averagegyroZ += temporary[2] / 1000;
		averageaccX += temporary[3] / 1000;
		averageaccY += temporary[4] / 1000;
		averageaccZ += temporary[5] / 1000;
		i++;

	} else if (i == 2001) {
		i = 12345;	// end of measurements
	}
}
