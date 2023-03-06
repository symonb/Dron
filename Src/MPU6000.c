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
#include "MPU6000.h"

static void	check_gyro_version(gyro_t* gyro);
static void setup_conf();
static void setup_gyro();
static void setup_acc();
static void SPI1_enable();
static void SPI1_disable();
static void CS_IMU_enable();
static void CS_IMU_disable();
static void SPI1_transmit(uint8_t* data, int size);
static void SPI1_receive(uint8_t* data, int size);
static void SPI1_receive_DMA(uint8_t* data, uint16_t size);
static bool failsafe_CONF();
static bool failsafe_SPI();
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

// RX:
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
		acc_1.new_data_flag = true;

		// X gyro axis:
		gyro_1.raw_data.roll = (int16_t)(rx_buffer[8] << 8 | rx_buffer[9]);
		// Y gyro axis:
		gyro_1.raw_data.pitch = (int16_t)(rx_buffer[10] << 8 | rx_buffer[11]);
		// Z gyro axis:
		gyro_1.raw_data.yaw = (int16_t)(rx_buffer[12] << 8 | rx_buffer[13]);
		gyro_1.new_data_flag = true;
	}
}

// TX:
void DMA2_Stream3_IRQHandler(void)
{
	// if stream3 transfer is completed:
	if (DMA2->LISR & DMA_LISR_TCIF3)
	{
		DMA2->LIFCR |= DMA_LIFCR_CTCIF3;
		DMA2_Stream3->CR &= ~DMA_SxCR_EN;

		CS_IMU_disable();
		SPI1_disable();
	}
}


void EXTI4_IRQHandler()
{
	if ((EXTI->PR & EXTI_PR_PR4))
	{
		EXTI->PR |= EXTI_PR_PR4;	// clear this bit by setting it high

		// test czy to blokowanie wzajemne jest w ogole konieczne
		// sprawdzac ilosc wywolanych failsafeow od tego spi lub uarta dla ibusa 
		// jak komunikacja bedzie bez problemu to nie bedzie failsafow

		// lock RX reading while IMU reading:
		USART1->CR1 &= ~USART_CR1_RXNEIE;
		static timeUs_t flag;
		if (get_Global_Time() - flag >= SEC_TO_US(10)) {
			flag = get_Global_Time();
			debug_variable_1 = 0;
		}
		debug_variable_1++;
		// // lock interrupts from imu since they will be processed
		// EXTI->IMR &= ~EXTI_IMR_MR4;
		read_all_DMA();

		// unlock RX reading while IMU reading
		USART1->CR1 |= USART_CR1_RXNEIE;
	}
}

void setup_MPU6000()
{
	delay_mili(30); // MPU datasheet specifies 30ms


	setup_conf();
	check_gyro_version(&gyro_1);
	setup_gyro();
	setup_acc();
	Gyro_Acc_filters_setup();
	//change SPI speed up to 20 [MHz] for reading gyro and acc:
	SPI1_disable();
	SPI1->CR1 &= ~SPI_CR1_BR;
	SPI1->CR1 |= SPI_CR1_BR_1;


}

static void SPI1_enable()
{
	SPI1->CR1 |= SPI_CR1_SPE; //	enabling SPI1
}

static void SPI1_disable()
{
	SPI1->CR1 &= ~SPI_CR1_SPE; //	disabling SPI1
}

static void CS_IMU_enable()
{
	GPIOA->BSRRH |= GPIO_BSRR_BS_4;
}

static void CS_IMU_disable()
{
	GPIOA->BSRRL |= GPIO_BSRR_BS_4;
}

static void SPI1_transmit(uint8_t* data, int size)
{
	//----------------STEPS--------------------
	/* 1 Wait for TXE bit to set in the Status Register
	 * 2 Write the Register Adress (&~x80 for writing) to the Data register
	 * 3 Write first byte that you want send to slave
	 * 4 Receive 1 data - it is anything (slave started sending it when received first bit of first byte so it is not interesting byte for sure) ignore it
	 * 5 Send second data and repeat until you receive all data you wanted
	 * 6 After last transmission you need to clear DR and wait until everything stop (more inf. in datasheet)
	 * */
	int i = 0;

	while (i < size)
	{
		time_flag4_1 = get_Global_Time();
		while (!((SPI1->SR) & SPI_SR_TXE))
		{
			if (failsafe_SPI())
			{
				break; // wait
			}
		}
		SPI1->DR = data[i]; // data sending as soon as TX flag is set
		i++;
	}

	time_flag4_1 = get_Global_Time();
	while (!((SPI1->SR) & SPI_SR_TXE))
	{
		if (failsafe_SPI())
		{
			break; // wait
		}
	}
	time_flag4_1 = get_Global_Time();
	while (((SPI1->SR) & SPI_SR_BSY))
	{
		if (failsafe_SPI())
		{
			break; // wait
		}
	}
	SPI1->DR;
	SPI1->SR;
}

static void SPI1_receive(uint8_t* data, int size)
{
	/*
	 * 1 Wait for TXE and send dummy data, wait for RXNE and receive your data. Repeat until you receive all data
	 * 2 Wait for TXE and BSY flag
	 * */
	while (size > 0)
	{
		time_flag4_1 = get_Global_Time();
		while (!((SPI1->SR) & SPI_SR_TXE))
		{
			if (failsafe_SPI())
			{
				break; // wait
			}
		}
		SPI1->DR = 0xAA; // send anything IMPORTANT!
		time_flag4_1 = get_Global_Time();
		while (!((SPI1->SR) & SPI_SR_RXNE))
		{
			if (failsafe_SPI())
			{
				break; // wait
			}
		}
		*data++ = SPI1->DR;
		size--;
		// if (size == 1) {
		// 	SPI1_disable();
		// }
	}


	// wait for TXE flag
	time_flag4_1 = get_Global_Time();
	while (!((SPI1->SR) & SPI_SR_TXE))
	{
		if (failsafe_SPI())
		{
			break; // wait
		}
	}
	// wait for BSY flag
	time_flag4_1 = get_Global_Time();
	while (((SPI1->SR) & SPI_SR_BSY))
	{
		if (failsafe_SPI())
		{
			break; // wait
		}
	}
	SPI1->DR;
	SPI1->SR;

}

static void SPI1_receive_DMA(uint8_t* data, uint16_t size)
{
	// set parameters of reception:
	DMA2_Stream0->M0AR = (uint32_t)(data);
	DMA2_Stream0->NDTR = size;

	DMA2_Stream3->M0AR = (uint32_t)(rx_buffer);
	DMA2_Stream3->NDTR = size;

	SPI1_enable();
	// enable DMA:

	DMA2_Stream0->CR |= DMA_SxCR_EN;
	DMA2_Stream3->CR |= DMA_SxCR_EN;
}

void MPU6000_SPI_write(uint8_t adress_of_register, uint8_t value)
{
	static uint8_t data[2];
	data[0] = adress_of_register & 0x7F; // first bit of 1 byte has to be write (0) or read(1)
	data[1] = value;
	CS_IMU_enable();
	SPI1_transmit(data, 2);
	CS_IMU_disable();
}

void MPU6000_SPI_read(uint8_t adress_of_register, uint8_t* data,
	uint16_t size)
{
	CS_IMU_enable();
	SPI1_transmit(&adress_of_register, 1);

	// SPI1->CR1 |= SPI_CR1_RXONLY;

	SPI1_receive(data, size);

	CS_IMU_disable();


}

void MPU6000_SPI_read_DMA(uint8_t instruction, uint8_t* data, uint16_t size)
{
	// RXONLY mode doesn't work so full duplex DMA:
	SPI1_enable();
	CS_IMU_enable();
	SPI1_transmit(&instruction, 1);
	SPI1_disable();
	SPI1_receive_DMA(data, size);
}

//-------main MPU6000 setting-----------
static void	check_gyro_version(gyro_t* gyro) {
	SPI1_enable();
	MPU6000_SPI_read(MPU6000_WHO_I_AM_READ, &(gyro->id), 1);
	SPI1_disable();
}

static void setup_conf()
{
	// enable SPI1
	SPI1_enable();
	uint8_t test;
	uint8_t data;
	uint8_t register_address;
	// 0x6A - address of (106)  User Control register:
	//  disable I2C
	register_address = 0x6A;
	data = 0x10;
	MPU6000_SPI_write(register_address, data);

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

	// // 0x6A - address of (106)  User Control register:
	// //  reset FIFO
	// register_address = 0x6A;
	// data = 0x14;
	// MPU6000_SPI_write(register_address, data);

	// // 0x6A - address of (106)  User Control register:
	// //  enable FIFO 
	// register_address = 0x6A;
	// data = 0x50;
	// MPU6000_SPI_write(register_address,  data);

	// 0x1A - address of (26) Configuration register:
	// setting low pass filter in this register
	register_address = 0x1A;
	data = 0x00;
	MPU6000_SPI_write(register_address, data);

	// 0x38 - address of (56) Interrupt Enable register:
	//  setting interrupt source as Data Register
	register_address = 0x38;
	data = 0x01;
	MPU6000_SPI_write(register_address, data);

	// // 0x23 - address of (35) FIFO Enable register:
	// //  set data loaded to FIFO (gyro and acc)
	// register_address = 0x23;
	// data = 0x78;
	// MPU6000_SPI_write(register_address, data);

	SPI1_disable();
}

static void setup_gyro()
{
	//---------setting Gyro--------
	SPI1_enable();
	// 0x1B- address of Gyroscope Configuration register:
	// set +/-2000[deg/s]
	MPU6000_SPI_write(0x1B, 0x18);
	delay_micro(15);
	SPI1_disable();

}

static void setup_acc()
{
	//---------setting Accelerometer-----------

	//	0x1C - address of Accelerometer Configuration register:
	// set +/-8[g]
	SPI1_enable();
	MPU6000_SPI_write(0x1C, 0x10);
	delay_micro(15);
	SPI1_disable();

}

void read_acc()
{
	SPI1_enable();
	static uint8_t temp[6];
	MPU6000_SPI_read(MPU6000_ACCEL_READ, temp, 6);

	// X acc axis:
	acc_1.raw_data.roll = (int16_t)(temp[0] << 8 | temp[1]);
	// Y acc axis:
	acc_1.raw_data.pitch = (int16_t)(temp[2] << 8 | temp[3]);
	// Z acc axis:
	acc_1.raw_data.yaw = (int16_t)(temp[4] << 8 | temp[5]);

	acc_1.new_data_flag = true;

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
	static uint8_t temp[6];
	MPU6000_SPI_read(MPU6000_GYRO_READ, temp, 6);
	// X gyro axis:
	gyro_1.raw_data.roll = (int16_t)(temp[0] << 8 | temp[1]);
	// Y gyro axis:
	gyro_1.raw_data.pitch = (int16_t)(temp[2] << 8 | temp[3]);
	// Z gyro axis:
	gyro_1.raw_data.yaw = (int16_t)(temp[4] << 8 | temp[5]);

	gyro_1.new_data_flag = true;
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
	acc_1.new_data_flag = true;

	// X gyro axis:
	gyro_1.raw_data.roll = (int16_t)(rx_buffer[8] << 8 | rx_buffer[9]);
	// Y gyro axis:
	gyro_1.raw_data.pitch = (int16_t)(rx_buffer[10] << 8 | rx_buffer[11]);
	// Z gyro axis:
	gyro_1.raw_data.yaw = (int16_t)(rx_buffer[12] << 8 | rx_buffer[13]);
	gyro_1.new_data_flag = true;

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

	gyro_1.new_data_flag = false;
	EXTI->IMR |= EXTI_IMR_MR4;
}

void acc_update(timeUs_t time)
{
	static float temporary[3];
	for (int j = 0; j < 3; j++)
	{
		temporary[j] = acc_1.raw_data.roll * transform_matrix[j][0];
		temporary[j] += acc_1.raw_data.pitch * transform_matrix[j][1];
		temporary[j] += acc_1.raw_data.yaw * transform_matrix[j][2];
	}

	acc_filtering(temporary);

	acc_1.new_data_flag = false;
	EXTI->IMR |= EXTI_IMR_MR4;
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

static bool failsafe_CONF()
{
	//	waiting as Data will be sent or failsafe if set time passed
	if ((get_Global_Time() - time_flag4_1) >= SEC_TO_US(TIMEOUT_VALUE))
	{
		FailSafe_status = FS_SETUP_ERROR;
		EXTI->SWIER |= EXTI_SWIER_SWIER15;
		return true;
	}
	return false;
}

static bool failsafe_SPI()
{
	//	waiting as Data will be sent or failsafe if set time passed
	if ((get_Global_Time() - time_flag4_1) >= SEC_TO_US(TIMEOUT_VALUE))
	{
		FailSafe_status = FS_SPI_IMU_ERROR;
		EXTI->SWIER |= EXTI_SWIER_SWIER15;
		return true;
	}
	return false;
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

static void MPU6000_self_test_measurements(float temporary[])
{
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

	if (i < 1000)
	{
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
