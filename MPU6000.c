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
void read_acc();
void read_temp();
void read_gyro();
void SPI_transmit(uint8_t*data, int size);
void SPI_receive(uint8_t adress_of_register, uint8_t*data, int size);
void failsafe_CONF();
void failsafe_I2C();
void failsafe_SPI();
void MPU6000_self_test_configuration();
void MPU6000_self_test_measurements();

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

float M_rotacji[3][3]={{0,1,0},{-1,0,0},{0,0,1}};


uint8_t read_write_tab[14];
static volatile uint8_t read_write_quantity;
static float time_flag4_1; //for SPI timeout detection

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

		//INSTRUCTIONS FOR READING IMU SENSORS:
		imu_received = 0;
		EXTI->IMR &= ~EXTI_IMR_IM4;
		read_all();

	}
}

void setup_MPU6000() {
	delay_mili(30);// MPU datasheet specifies 30ms
	setup_conf();
	setup_gyro();
	setup_acc();
	//change speed of SPI up to 10.5 [MHz] (only for reading sensors) IT DOESN T WORK
//	SPI1->CR1 &= ~SPI_CR1_BR;
//	SPI1->CR1 |=  SPI_CR1_BR_1|SPI_CR1_BR_0;
////
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
	 * 2 Write the Register Adress (&~x80 for writing) to the Data register
	 * 3 Write first byte that you want send to slave
	 * 4 Receive 1 data - it is anything (slave started sending it when received first bit of first byte so it is not interesting byte for sure) ignore it
	 * 5 Send second data and repeat until you receive all data you wanted
	 * 6 After last transmission you need to clear DR and wait until everything stop (more inf. in datasheet)
	 * */
	int i = 0;
	int rx_data;
	while (!((SPI1->SR) & SPI_SR_TXE)) {
	failsafe_SPI(); 			//wait
	}
	SPI1->DR = data[i]; //first data usually  slave's Register which you're interested in
	i++;

	while (i < size) {
		time_flag4_1=get_Global_Time();
		while (!((SPI1->SR) & SPI_SR_TXE)) {
			failsafe_SPI(); 			//wait
		}
		SPI1->DR = data[i]; //second and following data sending as soon as TX flag is set
		i++;
		time_flag4_1=get_Global_Time();
		while (!((SPI1->SR) & SPI_SR_RXNE)) {
			failsafe_SPI(); 			//wait
		}
		rx_data = SPI1->DR;
	}
	time_flag4_1=get_Global_Time();
	while (!((SPI1->SR) & SPI_SR_RXNE)) {
		failsafe_SPI(); 			//wait
	}
	rx_data = SPI1->DR;
	time_flag4_1=get_Global_Time();
	while (!((SPI1->SR) & SPI_SR_TXE)) {
		failsafe_SPI(); 			//wait
	}
	time_flag4_1=get_Global_Time();
	while (((SPI1->SR) & SPI_SR_BSY)) {
		failsafe_SPI();
	}

}

void SPI_receive(uint8_t adress_of_register, uint8_t *data, int size) {
	//----------------STEPS--------------------
	/* 1 Wait for TXE bit to set in the Status Register
	 * 2 Write the Register Adress (|0x80 for reading) to the Data register
	 * 3 Write 0x00 (slave will send data only if receive sth.)
	 * 4 Receive 1 data it is anything (slave started sending it when received first bit of first byte so it is not interesting byte for sure) ignore it
	 * 5 Repeat step 3 and then receive data from slave do it until you receive all data you wanted
	 * 6 After last transmission you need to clear DR and wait until everything stop (more inf. in datasheet)
	 * */

	int i = 0;
	time_flag4_1=get_Global_Time();
	while (!((SPI1->SR) & SPI_SR_TXE)&&failsafe_type!=6) {
		failsafe_SPI(); 			//wait
	}
	SPI1->DR = adress_of_register;

	while (i < size) {
		time_flag4_1=get_Global_Time();
		while (!((SPI1->SR) & SPI_SR_TXE)&&failsafe_type!=6) {
			failsafe_SPI(); 			//wait
		}
		SPI1->DR = 0xFF; 			//send anything IMPORTANT!
		time_flag4_1=get_Global_Time();
		while (!((SPI1->SR) & SPI_SR_RXNE)&&failsafe_type!=6) {
			failsafe_SPI(); 			//wait
		}
		if (i == 0) {
			*data=SPI1->DR;	//ignore first received data because it is some dummy data from slave

		} else {
			*data = SPI1->DR;
			data++;
		}

		i++;
	}
	time_flag4_1=get_Global_Time();
	while (!((SPI1->SR) & SPI_SR_RXNE)&&failsafe_type!=6) {
		failsafe_SPI(); 			//wait
	}
	*data = SPI1->DR;
	time_flag4_1=get_Global_Time();
	while (!((SPI1->SR) & SPI_SR_TXE)&&failsafe_type!=6) {
		failsafe_SPI(); 			//wait
	}
	time_flag4_1=get_Global_Time();
	while (((SPI1->SR) & SPI_SR_BSY)&&failsafe_type!=6) {
		failsafe_SPI();
	}

}

void MPU6000_SPI_write(uint8_t adress_of_register, uint8_t value) {
	uint8_t data[2];
	data[0] = adress_of_register & 0x7F; //first bit of 1 byte has to be write (0) or read(1)
	data[1] = value;
	CS_enable();
	SPI_transmit(data, 2);
	CS_disable();
}

void MPU6000_SPI_read(uint8_t adress_of_register, uint8_t *memory_adress,
		int number_of_bytes) {
	adress_of_register |= 0x80; //first bit of 1 byte has to be write (0) or read(1)
	CS_enable();
	SPI_receive(adress_of_register, memory_adress, number_of_bytes);
	CS_disable();
}

//-------main MPU6000 setting-----------
void setup_conf() {

	// enable SPI1
	SPI_enable();

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

	SPI_enable();

//	uint8_t temp[117] = { 0 };
//		//X:
//		MPU6000_SPI_read(0x0D, temp, 117);
//		read_write_tab[0] = temp[0];
//		read_write_tab[1] = temp[1];

	read_acc();
	read_temp();
	read_gyro();

	SPI_disable();
	imu_received = 1;
}

void read_acc() {
	SPI_enable();
	uint8_t temp[6] = { 0 };

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

	SPI_disable();
}

void read_temp() {
	SPI_enable();
	uint8_t temp[2] = { 0 };
	MPU6000_SPI_read(0x3F, temp, 2);
	read_write_tab[6] = temp[0];
	read_write_tab[7] = temp[1];

	SPI_disable();
}

void read_gyro() {
	SPI_enable();
	uint8_t temp[6] = { 0 };

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

	SPI_disable();
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

		Gyro_Acc[0] = temporary[0] - GYRO_ROLL_OFFSET;
		Gyro_Acc[1] = temporary[1] - GYRO_PITCH_OFFSET;
		Gyro_Acc[2] = temporary[2] - GYRO_YAW_OFFSET;
		Gyro_Acc[3] = temporary[3] - ACC_ROLL_OFFSET;
		Gyro_Acc[4] = temporary[4] - ACC_PITCH_OFFSET;
		Gyro_Acc[5] = temporary[5] - ACC_YAW_OFFSET;

		//temperature:
		float temperature;
		float gyroX;
		float gyroY;
		float gyroZ;
		float accX;
		float accY;
		float accZ;
		Gyro_Acc[6] = read_write_tab[6] << 8 | read_write_tab[7];




		temperature = Gyro_Acc[6] / 340.f + 36.53f;
		gyroX = temporary[0] /32.768;
		gyroY = temporary[1] / 32.768;
		gyroZ = temporary[2] / 32.768;
		accX = temporary[3] / 4096.;
		accY = temporary[4] / 4096.;
		accZ = temporary[5] / 4096.;
 		EXTI->IMR |= EXTI_IMR_IM4;
	}
}

void failsafe_CONF() {
	//	waiting as Data will be sent or failsafe if set time passed
	if ((get_Global_Time() - time_flag4_1) >= TIMEOUT_VALUE) {
		failsafe_type = 4;
		EXTI->SWIER |= EXTI_SWIER_SWIER15;

	}
}
void failsafe_I2C() {
	//	waiting as Data will be sent or failsafe if set time passed
	if ((get_Global_Time() - time_flag4_1) >= TIMEOUT_VALUE) {
		failsafe_type = 5;
		EXTI->SWIER |= EXTI_SWIER_SWIER15;
	}
}
void failsafe_SPI() {
	//	waiting as Data will be sent or failsafe if set time passed
	if ((get_Global_Time() - time_flag4_1) >= TIMEOUT_VALUE) {
		failsafe_type = 6;
		EXTI->SWIER |= EXTI_SWIER_SWIER15;
	}
}

void MPU6000_self_test_configuration(){
	SPI_enable();
	// 0x1B- address of Gyroscope Configuration register:
	// set +/-250[deg/s] and Self_test activate
	MPU6000_SPI_write(0x1B, 0xE0);
	delay_micro(15);

	//	0x1C - address of Accelerometer Configuration register:
	// set +/-8[g]	and Self_test activate

	MPU6000_SPI_write(0x1C, 0xF0);
	delay_micro(15);

	SPI_disable();

}
void MPU6000_self_test_measurements(float temporary[]){
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

if (i<1000){
	averagegyroX_ST += temporary[0] /1000;
	averagegyroY_ST += temporary[1] /1000;
	averagegyroZ_ST += temporary[2] /1000;
	averageaccX_ST += temporary[3] /1000;
	averageaccY_ST += temporary[4] /1000;
	averageaccZ_ST += temporary[5] /1000;
	i++;
}
else if(i==1000){
	SPI_enable();
	// 0x1B- address of Gyroscope Configuration register:
	// set +/-250[deg/s] and Self_test deactivate
	MPU6000_SPI_write(0x1B, 0x00);
	delay_micro(15);

	//	0x1C - address of Accelerometer Configuration register:
	// set +/-8[g]	and Self_test deactivate

	MPU6000_SPI_write(0x1C, 0x10);
	delay_micro(15);

	SPI_disable();
	i++;
}
else if(i<2001){
	averagegyroX += temporary[0] /1000;
	averagegyroY += temporary[1] /1000;
	averagegyroZ += temporary[2] /1000;
	averageaccX += temporary[3] /1000;
	averageaccY += temporary[4] /1000;
	averageaccZ += temporary[5] /1000;
i++;

}
else if(i==2001){
	i=12345;// end of measurements
}
}
