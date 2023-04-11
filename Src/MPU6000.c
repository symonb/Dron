/*
 * MPU6000.c
 *
 *  Created on: 03.01.2021
 *
 */

#include <math.h>
#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "filters.h"
#include "flash.h"
#include "math/statistics.h"
#include "drivers/SPI1.h"
#include "MPU6000.h"



static void	check_gyro_version(gyro_t* gyro);
static void setup_conf();

#if defined(IMU_TEST)
static void MPU6000_self_test_configuration();
static void MPU6000_self_test_measurements();
#endif

// float transform_matrix[3][3] = { { (ACC_CALIBRATION_X_X - ACC_PITCH_OFFSET)
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

static float transform_matrix[3][3] = { {-1, 0, 0}, {0, -1, 0}, {0, 0, 1} };
static uint8_t rx_buffer[14];


// ------SPI1 with DMA usage-----
// reception:
void DMA2_Stream0_IRQHandler(void)
{
	// if stream0 transfer is completed:
	if (DMA2->LISR & DMA_LISR_TCIF0)
	{
		DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
		DMA2_Stream0->CR &= ~DMA_SxCR_EN;

		// X acc axis:
		acc_1.raw_data.roll = (int16_t)(rx_buffer[0] << 8 | rx_buffer[1]);
		// Y acc axis:
		acc_1.raw_data.pitch = (int16_t)(rx_buffer[2] << 8 | rx_buffer[3]);
		// Z acc axis:
		acc_1.raw_data.yaw = (int16_t)(rx_buffer[4] << 8 | rx_buffer[5]);
		acc_1.new_raw_data_flag = true;

		// X gyro axis:
		gyro_1.raw_data.roll = (int16_t)(rx_buffer[8] << 8 | rx_buffer[9]);
		// Y gyro axis:
		gyro_1.raw_data.pitch = (int16_t)(rx_buffer[10] << 8 | rx_buffer[11]);
		// Z gyro axis:
		gyro_1.raw_data.yaw = (int16_t)(rx_buffer[12] << 8 | rx_buffer[13]);
		gyro_1.new_raw_data_flag = true;

		CS_SPI1_disable();
		SPI1_disable();
	}
}

// transmission:
void DMA2_Stream3_IRQHandler(void)
{
	// if stream3 transfer is completed:
	if (DMA2->LISR & DMA_LISR_TCIF3)
	{
		DMA2->LIFCR |= DMA_LIFCR_CTCIF3;
		DMA2_Stream3->CR &= ~DMA_SxCR_EN;
	}
}

//--------END--------


void EXTI4_IRQHandler()
{
	if ((EXTI->PR & EXTI_PR_PR4))
	{
		EXTI->PR |= EXTI_PR_PR4;	// clear this bit by setting it high
		// start reading with DMA:
		read_all_DMA();
	}
}

void setup_MPU6000()
{
	delay_mili(30); // MPU datasheet specifies 30ms

	setup_conf();
	// 	check device id and update variable:
	check_gyro_version(&gyro_1);

	Gyro_Acc_filters_setup();
	//change SPI speed up to 20 [MHz] for reading gyro and acc:
	SPI1_disable();
	SPI1->CR1 &= ~SPI_CR1_BR;
	SPI1->CR1 |= SPI_CR1_BR_1;
}

void MPU6000_SPI_write(uint8_t adress_of_register, uint8_t value)
{
	uint8_t data[2];
	data[0] = adress_of_register & 0x7F; // first bit of 1 byte has to be write (0) or read(1)
	data[1] = value;
	CS_SPI1_enable();
	SPI1_transmit(data, 2);
	CS_SPI1_disable();
}

void MPU6000_SPI_read(uint8_t adress_of_register, uint8_t* data,
	uint16_t size)
{
	CS_SPI1_enable();
	SPI1_transmit(&adress_of_register, 1);

	// SPI1->CR1 |= SPI_CR1_RXONLY;

	SPI1_receive(data, size);

	CS_SPI1_disable();
}

void MPU6000_SPI_read_DMA(const uint8_t instruction, uint8_t* data, uint16_t size)
{
	// RXONLY mode doesn't work so full duplex DMA:
	SPI1_enable();
	CS_SPI1_enable();
	SPI1_transmit(&instruction, 1);
	SPI1_disable();
	SPI1_receive_DMA(data, size);
}

//-------main MPU6000 setting-----------
static void	check_gyro_version(gyro_t* gyro) {
	SPI1_enable();
	MPU6000_SPI_read(MPU6000_WHO_I_AM_READ, &(gyro->address), 1);
	delay_micro(15);
	MPU6000_SPI_read(MPU6000_PRODUCT_ID_READ, &(gyro->rev_id), 1);
	SPI1_disable();
}

static void setup_conf()
{
	// enable SPI1
	SPI1_enable();
	uint8_t data;
	uint8_t register_address;

	// 0x6B - address of (107) Power Management 1 register:
	// set 0x80 in this register (RESET) 
	register_address = 0x6B;
	data = 0x80;
	MPU6000_SPI_write(register_address, data);
	// delay of min. 0.1[s] according the MPU6000 datasheet
	delay_mili(100);

	// 0x68 - address of (104)  Signal Path Reset:
	// reset sensors paths and registers 
	register_address = 0x68;
	data = 0x07;
	MPU6000_SPI_write(register_address, data);
	// delay of min. 0.1[s] according the MPU6000 datasheet
	delay_mili(100);

	// 0x6B - address of Power Management 1 register:
	//  set clock and turn off sleep mode
	register_address = 0x6B;
	data = 0x03;
	MPU6000_SPI_write(register_address, data);
	delay_micro(15);

	// 0x6A - address of (106)  User Control register:
	//  disable I2C
	register_address = 0x6A;
	data = 0x10;
	MPU6000_SPI_write(register_address, data);
	delay_micro(15);

	// 0x6C - address of (108) Power Menagment 2:
	// theoretically it should be set by default but...
	register_address = 0x6C;
	data = 0x00;
	MPU6000_SPI_write(register_address, data);
	delay_micro(15);

	// Acc sample rate = (gyro_sample rate)/(div_val + 1):
	register_address = 0x19;
	data = 0x00;
	MPU6000_SPI_write(register_address, data);
	delay_micro(15);

	// // 0x6A - address of (106)  User Control register:
	// //  reset FIFO
	// register_address = 0x6A;
	// data = 0x14;
	// MPU6000_SPI_write(register_address, data);
	// delay_micro(15);

	// // 0x6A - address of (106)  User Control register:
	// //  enable FIFO 
	// register_address = 0x6A;
	// data = 0x50;
	// MPU6000_SPI_write(register_address,  data);
	// delay_micro(15);

	// 0x1A - address of (26) Configuration register:
	// setting low pass filter in this register
	register_address = 0x1A;
	data = 0x00;
	MPU6000_SPI_write(register_address, data);
	delay_micro(15);

	// 0x1B- address of Gyroscope Configuration register:
	// set +/-2000[deg/s]
	register_address = 0x1B;
	data = 0x18;
	MPU6000_SPI_write(register_address, data);
	delay_micro(15);

	register_address = 0x1C;
	data = 0x10;
	MPU6000_SPI_write(register_address, data);
	delay_micro(15);

	// 0x37 - address of (55) Interrupt Pin Configuraation register:
	//  int status cleared with any read command:
	register_address = 0x37;
	data = 0x10;
	MPU6000_SPI_write(register_address, data);
	delay_micro(15);

	// 0x38 - address of (56) Interrupt Enable register:
	//  setting interrupt source as Data Register
	register_address = 0x38;
	data = 0x01;
	MPU6000_SPI_write(register_address, data);
	delay_micro(15);

	// // 0x23 - address of (35) FIFO Enable register:
	// //  set data loaded to FIFO (gyro and acc)
	// register_address = 0x23;
	// data = 0x78;
	// MPU6000_SPI_write(register_address, data);
	// delay_micro(15);

	SPI1_disable();
}

void read_acc()
{
	SPI1_enable();
	uint8_t temp[6];
	MPU6000_SPI_read(MPU6000_ACCEL_READ, temp, 6);

	// X acc axis:
	acc_1.raw_data.roll = (int16_t)(temp[0] << 8 | temp[1]);
	// Y acc axis:
	acc_1.raw_data.pitch = (int16_t)(temp[2] << 8 | temp[3]);
	// Z acc axis:
	acc_1.raw_data.yaw = (int16_t)(temp[4] << 8 | temp[5]);

	acc_1.new_raw_data_flag = true;

	SPI1_disable();
}

void read_temp()
{
	SPI1_enable();
	static uint8_t temp[2];
	MPU6000_SPI_read(MPU6000_TEMP_READ, temp, 2);
	Gyro_Acc[6] = temp[0] << 8 | temp[1];
	SPI1_disable();

}

void read_gyro()
{
	SPI1_enable();
	uint8_t temp[6];
	MPU6000_SPI_read(MPU6000_GYRO_READ, temp, 6);
	// X gyro axis:
	gyro_1.raw_data.roll = (int16_t)(temp[0] << 8 | temp[1]);
	// Y gyro axis:
	gyro_1.raw_data.pitch = (int16_t)(temp[2] << 8 | temp[3]);
	// Z gyro axis:
	gyro_1.raw_data.yaw = (int16_t)(temp[4] << 8 | temp[5]);

	gyro_1.new_raw_data_flag = true;
	SPI1_disable();

}

void read_all()
{

	SPI1_enable();
	//	burst reading sensors:
	MPU6000_SPI_read(MPU6000_ACCEL_READ, &rx_buffer[0], 14);

	// X acc axis:
	acc_1.raw_data.roll = (int16_t)(rx_buffer[0] << 8 | rx_buffer[1]);
	// Y acc axis:
	acc_1.raw_data.pitch = (int16_t)(rx_buffer[2] << 8 | rx_buffer[3]);
	// Z acc axis:
	acc_1.raw_data.yaw = (int16_t)(rx_buffer[4] << 8 | rx_buffer[5]);
	acc_1.new_raw_data_flag = true;

	// X gyro axis:
	gyro_1.raw_data.roll = (int16_t)(rx_buffer[8] << 8 | rx_buffer[9]);
	// Y gyro axis:
	gyro_1.raw_data.pitch = (int16_t)(rx_buffer[10] << 8 | rx_buffer[11]);
	// Z gyro axis:
	gyro_1.raw_data.yaw = (int16_t)(rx_buffer[12] << 8 | rx_buffer[13]);
	gyro_1.new_raw_data_flag = true;

	SPI1_disable();

}

void read_all_DMA() {
	MPU6000_SPI_read_DMA(MPU6000_ACCEL_READ, rx_buffer, 14);
}

void gyro_update(timeUs_t time)
{
	static float temporary[3];
	for (int j = 0; j < 3; j++)
	{
		temporary[j] = gyro_1.raw_data.roll * transform_matrix[j][0];
		temporary[j] += gyro_1.raw_data.pitch * transform_matrix[j][1];
		temporary[j] += gyro_1.raw_data.yaw * transform_matrix[j][2];
	}
	gyro_filtering(temporary);

	gyro_1.new_raw_data_flag = false;
	gyro_1.new_filtered_data = true;
}

void acc_update(timeUs_t time)
{
	float temporary[3] = { 0,0,0 };
	for (int j = 0; j < 3; j++)
	{
		temporary[j] = acc_1.raw_data.roll * transform_matrix[j][0];
		temporary[j] += acc_1.raw_data.pitch * transform_matrix[j][1];
		temporary[j] += acc_1.raw_data.yaw * transform_matrix[j][2];
	}
	acc_filtering(temporary);

	acc_1.new_raw_data_flag = false;
}

void gyro_calibration(gyro_t* gyro_to_calibrate, timeUs_t time)
{
	static timeUs_t calibration_start;
	static stdev_t gyro_roll_dev;
	static stdev_t gyro_pitch_dev;
	static stdev_t gyro_yaw_dev;

	// start calibration in first loop:
	if (calibration_start == 0) {
		calibration_start = time;
	}

	dev_push(&gyro_roll_dev, Gyro_Acc[0]);
	dev_push(&gyro_pitch_dev, Gyro_Acc[1]);
	dev_push(&gyro_yaw_dev, Gyro_Acc[2]);

	//check deviation end update variable or start over:
	if (dev_standard_deviation(&gyro_roll_dev) <= GYRO_STARTUP_CALIB_MAX_DEV
		&& dev_standard_deviation(&gyro_pitch_dev) <= GYRO_STARTUP_CALIB_MAX_DEV
		&& dev_standard_deviation(&gyro_yaw_dev) <= GYRO_STARTUP_CALIB_MAX_DEV) {

		// if calibration is long enough end whole calibration:
		if (time - calibration_start >= SEC_TO_US(GYRO_STARTUP_CALIB_DURATION)) {

			gyro_to_calibrate->calibrated = true;

			gyro_to_calibrate->offset.roll = gyro_roll_dev.m_newM;
			gyro_to_calibrate->offset.pitch = gyro_pitch_dev.m_newM;
			gyro_to_calibrate->offset.yaw = gyro_yaw_dev.m_newM;

		}
	}
	else {
		//start calibration again
		calibration_start = 0;
		dev_clear(&gyro_roll_dev);
		dev_clear(&gyro_pitch_dev);
		dev_clear(&gyro_yaw_dev);
	}

}


#if defined(IMU_TEST)

static void MPU6000_self_test_configuration()
{
	SPI1_enable();
	// 0x1B- address of Gyroscope Configuration register:
	// set +/-250[deg/s] and Self_test activate
	MPU6000_SPI_write(0x1B, 0xE0);
	delay_micro(15);

	//	0x1C - address of Accelerometer Configuration register:
	// set +/-8[g]	and Self_test activate

	MPU6000_SPI_write(0x1C, 0xF0);
	delay_micro(15);

}

static void MPU6000_self_test_measurements(const float temporary[])
{
	static int i;
	static float averagegyroX;
	static float averagegyroY;
	static float averagegyroZ;
	static float averageaccX;
	static float averageaccY;
	static float averageaccZ;



	if (i < 1000)
	{
		static float averagegyroX_ST;
		static float averagegyroY_ST;
		static float averagegyroZ_ST;
		static float averageaccX_ST;
		static float averageaccY_ST;
		static float averageaccZ_ST;
		averagegyroX_ST += temporary[0] / 1000;
		averagegyroY_ST += temporary[1] / 1000;
		averagegyroZ_ST += temporary[2] / 1000;
		averageaccX_ST += temporary[3] / 1000;
		averageaccY_ST += temporary[4] / 1000;
		averageaccZ_ST += temporary[5] / 1000;
		i++;
	}
	else if (i == 1000)
	{
		SPI1_enable();
		// 0x1B- address of Gyroscope Configuration register:
		// set +/-250[deg/s] and Self_test deactivate
		MPU6000_SPI_write(0x1B, 0x00);
		delay_micro(15);

		//	0x1C - address of Accelerometer Configuration register:
		// set +/-8[g]	and Self_test deactivate

		MPU6000_SPI_write(0x1C, 0x10);
		delay_micro(15);

		i++;
	}
	else if (i < 2001)
	{
		averagegyroX += temporary[0] / 1000;
		averagegyroY += temporary[1] / 1000;
		averagegyroZ += temporary[2] / 1000;
		averageaccX += temporary[3] / 1000;
		averageaccY += temporary[4] / 1000;
		averageaccZ += temporary[5] / 1000;
		i++;
	}
	else if (i == 2001)
	{
		i = 12345; // end of measurements
	}
}
#endif


