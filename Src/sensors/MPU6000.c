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
#include"math/quaternions.h"
#include "drivers/SPI1.h"
#include "sensors/MPU6000.h"

static void	check_gyro_version(gyro_t* gyro);
static void setup_conf();

#if defined(IMU_TEST)
static void MPU6000_self_test_configuration();
static void MPU6000_self_test_measurements();
#endif



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


		for (int i = 0;i < 3;i++) {
			acc_1.raw_data[i] = (int16_t)(rx_buffer[i * 2] << 8 | rx_buffer[i * 2 + 1]);
			gyro_1.raw_data[i] = (int16_t)(rx_buffer[i * 2 + 8] << 8 | rx_buffer[i * 2 + 9]);
		}
		acc_1.new_raw_data_flag = true;
		gyro_1.new_raw_data_flag = true;
		SPI1_CS_disable();
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
	SPI1_CS_enable();
	SPI1_transmit(data, 2);
	SPI1_CS_disable();
}

void MPU6000_SPI_read(uint8_t adress_of_register, uint8_t* data,
	uint16_t size)
{
	SPI1_CS_enable();
	SPI1_transmit(&adress_of_register, 1);

	// SPI1->CR1 |= SPI_CR1_RXONLY;

	SPI1_receive(data, size);

	SPI1_CS_disable();
}

void MPU6000_SPI_read_DMA(const uint8_t instruction, uint8_t* data, uint16_t size)
{
	// RXONLY mode doesn't work so full duplex DMA:
	SPI1_enable();
	SPI1_CS_enable();
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

	for (uint8_t i = 0; i < 3;++i) {
		acc_1.raw_data[i] = (int16_t)(rx_buffer[2 * i] << 8 | rx_buffer[2 * i + 1]);
	}

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
	for (uint8_t i = 0; i < 3;++i) {
		gyro_1.raw_data[i] = (int16_t)(rx_buffer[2 * i] << 8 | rx_buffer[2 * i + 1]);
	}
	gyro_1.new_raw_data_flag = true;
	SPI1_disable();

}

void read_all()
{
	SPI1_enable();
	//	burst reading sensors:
	MPU6000_SPI_read(MPU6000_ACCEL_READ, &rx_buffer[0], 14);

	for (uint8_t i = 0; i < 3;++i) {
		acc_1.raw_data[i] = (int16_t)(rx_buffer[2 * i] << 8 | rx_buffer[2 * i + 1]);
		gyro_1.raw_data[i] = (int16_t)(rx_buffer[2 * i + 8] << 8 | rx_buffer[2 * i + 9]);
	}
	acc_1.new_raw_data_flag = true;
	gyro_1.new_raw_data_flag = true;

	SPI1_disable();

}

void read_all_DMA() {
	MPU6000_SPI_read_DMA(MPU6000_ACCEL_READ, rx_buffer, 14);
}

/**
*@brief Function process and filter raw data from gyro
* @param gyro pointer to the struct to update
*/
void gyro_update(gyro_t* gyro)
{
	// convert raw input to [degree/s]:
	threef_t temp = { GYRO_TO_DPS * gyro->raw_data[0] - gyro->offset[0],
	GYRO_TO_DPS * gyro->raw_data[1] - gyro->offset[1],
	GYRO_TO_DPS * gyro->raw_data[2] - gyro->offset[2] };
	// transform raw data from sensor frame into drone frame:
	temp = quaternion_rotate_vector(temp, q_trans_sensor_to_body_frame);
	float temporary[3] = { temp.roll, temp.pitch, temp.yaw };
	// filter raw data:
	gyro_filtering(temporary);
	// set flags:
	gyro->new_raw_data_flag = false;
	gyro->new_filtered_data = true;
}

/**
 *Function process and filter raw data from accelerometer
 *@param acc pointer to the struct to update
*/
void acc_update(acc_t* acc)
{
	// convert raw data to [g] unit:
	threef_t temp = {
	acc->scale[0] * (ACC_TO_GRAVITY * acc->raw_data[0] - acc->offset[0]),
	acc->scale[1] * (ACC_TO_GRAVITY * acc->raw_data[1] - acc->offset[1]),
	acc->scale[2] * (ACC_TO_GRAVITY * acc->raw_data[2] - acc->offset[2]) };
	// trnansform raw data from sensor frame into drone frame:
	temp = quaternion_rotate_vector(temp, q_trans_sensor_to_body_frame);
	float temporary[3] = { temp.roll, temp.pitch, temp.yaw };
	// filter raw data:
	acc_filtering(temporary);
	// set flags:
	acc->new_raw_data_flag = false;
	acc->new_filtered_data = true;
}

void gyro_calibration(gyro_t* gyro_to_calibrate, timeUs_t time)
{
	static timeUs_t calibration_start;
	static stdev_t gyro_dev[3];
	// start calibration in first loop:
	if (calibration_start == 0) {
		calibration_start = time;
	}

	for (uint8_t i = 0;i < 3;i++)
	{
		dev_push(&gyro_dev[i], gyro_1.raw_data[i] * GYRO_TO_DPS);
	}

	//check deviation end update variable or start over:
	if (dev_standard_deviation(&gyro_dev[0]) <= GYRO_STARTUP_CALIB_MAX_DEV
		&& dev_standard_deviation(&gyro_dev[1]) <= GYRO_STARTUP_CALIB_MAX_DEV
		&& dev_standard_deviation(&gyro_dev[2]) <= GYRO_STARTUP_CALIB_MAX_DEV) {

		// if calibration is long enough end whole calibration:
		if (time - calibration_start >= SEC_TO_US(GYRO_STARTUP_CALIB_DURATION)) {

			gyro_to_calibrate->calibrated = true;
			for (uint8_t i = 0;i < 3;i++) {
				gyro_to_calibrate->offset[i] = gyro_dev[i].m_newM;
			}
		}
	}
	else {
		//start calibration again
		calibration_start = 0;
		for (uint8_t i = 0;i < 3;++i) {
			dev_clear(&gyro_dev[i]);
		}
	}

}

void acc_level_calibration(acc_t* acc_to_calibrate) {
	timeUs_t calibration_start = get_Global_Time();
	stdev_t acc_x_dev = { .m_n = 0 };
	stdev_t acc_y_dev = { .m_n = 0 };
	stdev_t acc_z_dev = { .m_n = 0 };
	const float MAX_DEV = 0.001f;	// maximal deviation for succesful calibration [g] units

	acc_to_calibrate->calibrated = false;
	q_trans_sensor_to_body_frame = (quaternion_t){ 0, 0, 0, 1 };


	while (!acc_to_calibrate->calibrated) {

		acc_update(0);

		dev_push(&acc_x_dev, Gyro_Acc[3]);
		dev_push(&acc_y_dev, Gyro_Acc[4]);
		dev_push(&acc_z_dev, Gyro_Acc[5]);

		//check deviation end update variable or start over:
		if (dev_standard_deviation(&acc_x_dev) <= MAX_DEV
			&& dev_standard_deviation(&acc_y_dev) <= MAX_DEV
			&& dev_standard_deviation(&acc_z_dev) <= MAX_DEV) {

			// if calibration is long enough end whole calibration:
			if (get_Global_Time() - calibration_start >= SEC_TO_US(1)) {
				// convert acc reading to quaternion transforming from sensor frame to body frame and set it as 
				// q_trans_sensor_to_body_frame used for acc and gyro data transforamtion
				threef_t euler_angles = quaternion_to_euler_angles(q_trans_sensor_to_body_frame);
				euler_angles.roll += -atan2f(acc_y_dev.m_newM, acc_z_dev.m_newM) * RAD_TO_DEG;
				euler_angles.pitch += atan2f(acc_x_dev.m_newM, acc_z_dev.m_newM) * RAD_TO_DEG;
				q_trans_sensor_to_body_frame = quaternion_conjugate(euler_angles_to_quaternion(euler_angles));
				acc_to_calibrate->calibrated = true;
			}
		}
		else {
			//start calibration again
			calibration_start = get_Global_Time();
			dev_clear(&acc_x_dev);
			dev_clear(&acc_y_dev);
			dev_clear(&acc_z_dev);
		}
		// acc is updated with frequency = 1000 [Hz]
		delay_micro(TASK_PERIOD_HZ(1000));
	}

}

/**
 *@brief Full accelerometer calibration.
 *@note Require seting accelerometer on each side: nose up, nose down, side up, side down, bottom down, bottom up
 *@return After end of each step returns number of remaining steps, 0 if calibration completed
*/
uint8_t acc_calibration(acc_t* acc_to_calibrate) {
	timeUs_t calibration_start = get_Global_Time();
	stdev_t acc_x_dev = { .m_n = 0 };
	stdev_t acc_y_dev = { .m_n = 0 };
	stdev_t acc_z_dev = { .m_n = 0 };
	float MAX_DEV = 5; // maximal deviation for succesful calibration 
	static uint8_t step;
	static float calibration_tab[6][3];

	acc_to_calibrate->calibrated = false;
	for (uint8_t i = 0;i < 3;++i) {
		acc_to_calibrate->offset[i] = 0;
		acc_to_calibrate->scale[i] = 1;
	}

	if (step == 6)
	{
		// process collected data:
		newton_gauss_method(calibration_tab, 6);

		acc_to_calibrate->calibrated = true;
		step = 0;
		return 0;
	}

	while (1) {
		acc_update(0);

		dev_push(&acc_x_dev, acc_1.raw_data[0] * ACC_TO_GRAVITY);
		dev_push(&acc_y_dev, acc_1.raw_data[1] * ACC_TO_GRAVITY);
		dev_push(&acc_z_dev, acc_1.raw_data[2] * ACC_TO_GRAVITY);

		//check deviation end update variable or start over:
		if (dev_standard_deviation(&acc_x_dev) <= MAX_DEV
			&& dev_standard_deviation(&acc_y_dev) <= MAX_DEV
			&& dev_standard_deviation(&acc_z_dev) <= MAX_DEV) {

			// if calibration is long enough end whole calibration:
			if (get_Global_Time() - calibration_start >= SEC_TO_US(1)) {

				calibration_tab[step][0] = acc_x_dev.m_newM;
				calibration_tab[step][1] = acc_y_dev.m_newM;
				calibration_tab[step][2] = acc_z_dev.m_newM;
				step++;
				break;
			}
		}
		else {
			//start calibration again
			calibration_start = get_Global_Time();
			dev_clear(&acc_x_dev);
			dev_clear(&acc_y_dev);
			dev_clear(&acc_z_dev);
		}
		// acc is updated with frequency = 1000 [Hz]
		delay_micro(TASK_PERIOD_HZ(1000));
	}

	return 7 - step;
}



