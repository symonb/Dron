/*
 * global_constants.h
 *
 *  Created on: 18.04.2021
 *      Author: symon
 */

#ifndef GLOBAL_CONSTANTS_H_
#define GLOBAL_CONSTANTS_H_
#include <stdbool.h>

//------DIFFERENT_CONSTANTS------------
#define MAX(a, b) ((a > b) ? a : b) // choose greater value
#define GYRO_ACC_SIZE 7				// 3 for gyro, 3 acc and 1 for temperature
#define ALL_ELEMENTS_TO_SEND 14		// telemetry information different values
#define GYRO_TO_DPS 2000 / 32767.f	// convert gyro register into degrees per second unit
#define RAD_TO_DEG 180 / M_PI
#define DEG_TO_RAD M_PI / 180
#define GYRO_TO_RAD (1.f / 32.767f * DEG_TO_RAD)  // convert gyro register into rad per second unit
#define ACC_TO_GRAVITY 1.f / 4096				  // convert acc register into gravity unit
#define TIMEOUT_VALUE 0.5f						  // time for SPI_failsafe activation [s]
#define TASK_PERIOD_KHZ(kHz) (1000000000 / (kHz)) // converts frequency in [kHz] into period in [us]
#define TASK_PERIOD_HZ(Hz) (1000000 / (Hz))		  // converts frequency in [Hz] into [us]
#define SEC_TO_US(s) ((s)*1000000)				  // converts [s] into [us]
#define MS_TO_US(s) ((s)*1000)					  // converts [ms] into [us]
#define US_TO_SEC(s) ((s) / 1000000.f)			  // converts [us] into [s]

//-------------BATTERY------------
#define BATTERY_SCALE 11
#define ADC_REFERENCE_VOLTAGE 3.3f	  //	[V]
#define BATTERY_CELL_MIN_VOLTAGE 3.5f //	[V]
#define BATTERY_CELL_MAX_VOLTAGE 4.2f //	[V]

//-------------BUZZER-------------
#define BUZZER_TIME_ON 0.2	//	[s]
#define BUZZER_TIME_OFF 0.8 //	[s]

//-----------RX_SETTINGS----------
#define CHANNELS 10 // how many channels you want use (4 for steering; 14 is max)
#define MAX_RX_SIGNAL 2050
#define MIN_RX_SIGNAL 950
#define MAX_NO_SIGNAL_TIME 1	  // time after RX_failsafe occurs[s]
#define ARM_VALUE 1600			  //	value of arming switch to arm drone
#define PREARM_VALUE 1600		  //	value of prearming switch to prearm drone
#define MAX_ARM_THROTTLE_VAL 1100 //	maximum value of throttle that allow for arming
#define USE_PREARM				  //	if you want use PREARM define USE_PREARM
#define PREARM_CHANNEL 8		  //	and PREARM_CHANNEL
#define ARM_CHANNEL 4
#define FLIGHT_MODE_CHANNEL 6
#define BLACKBOX_CHANNEL 7 //	switch to turn on blackbox data collection

//------------ESC_PROTOCOLS----------
#define ESC_PROTOCOL_BDSHOT // ESC_PROTOCOL_PWM, ESC_PROTOCOL_ONESHOT125, ESC_PROTOCOL_BDSHOT, ESC_PROTOCOL_DSHOT, ESC_PROTOCOL_DSHOT_BURST - nieskonczone
#define BIT_BANGING_V1
#define DSHOT_MODE 300 // 150 300 600 1200

#define DSHOT_BUFFER_LENGTH 18 // 16 bits of Dshot and 2 for clearing
#define DSHOT_PWM_FRAME_LENGTH 35
#define DSHOT_1_LENGTH 26
#define DSHOT_0_LENGTH 13

#if defined(BIT_BANGING_V1)
#define DSHOT_BB_BUFFER_LENGTH 18  // 16 bits of Dshot and 2 for clearing - used when bit-banging dshot used
#define DSHOT_BB_FRAME_LENGTH 140  //	how many counts of timer gives one bit frame
#define DSHOT_BB_FRAME_SECTIONS 35 // in how many sections is bit frame divided (must be factor of DSHOT_BB_FRAME_LENGTH)
#define DSHOT_BB_1_LENGTH 25
#define DSHOT_BB_0_LENGTH 10
#define BDSHOT_RESPONSE_LENGTH 21
#define BDSHOT_RESPONSE_BITRATE (DSHOT_MODE * 4 / 3) // in my tests this value was not 5/4 * DSHOT_MODE as documentation suggests
#define BDSHOT_RESPONSE_OVERSAMPLING 3				 // it has to be a factor of DSHOT_BB_FRAME_LENGTH * DSHOT_MODE / BDSHOT_RESPONSE_BITRATE

#elif defined(BIT_BANGING_V2)
#define DSHOT_BB_BUFFER_LENGTH 18 // 16 bits of Dshot and 2 for clearing - used when bit-banging dshot used
#define DSHOT_BB_FRAME_LENGTH 35
#define DSHOT_BB_1_LENGTH 26
#define DSHOT_BB_0_LENGTH 13
#define DSHOT_BB_FRAME_SECTIONS 3
#define BDSHOT_RESPONSE_BUFFER_LENGTH 21
#endif

//----------MOTORS_AND_CORRECTIONS-------
#define MAX_I_CORRECTION 300 // maximal I_corr for PIDs betwenn <0;4000>
#define IDLE_VALUE 1050
#define MOTOR_1 3			  // PA3
#define MOTOR_2 0			  // PB0
#define MOTOR_3 1			  // PB1
#define MOTOR_4 2			  // PA2
#define MOTOR_POLES_NUMBER 14 // how many poles have your motors (usually 14 or 12)

//----------STABILIZE_SETINGS------
#define MAX_ROLL_ANGLE 45
#define MAX_PITCH_ANGLE 45

#define STABILIZE_FILTER_MAGDWICK // STABILIZE_FILTER_MAGDWICK, STABILIZE_FILTER_MAHONY, STABILIZE_FILTER_COMPLEMENTARY
#define MAGDWICK_NEW			  // MAGDWICK_ORIGINAL, MAGDWICK_NEW

#define GYRO_PART 0.995f // complementary filter
#define ACC_PART 0.005f	 // complementary filter

//---------FREQUENCY_SETTINGS--------
#define FREQUENCY_MAIN_LOOP 700			 //[Hz]   IF YOU' RE USING PWM MAX. IS 500[Hz] (little less), IF DSHOT you can go up to 1[kHz]
#define FREQUENCY_STABILIZATION_LOOP 200 //[Hz]
#define FREQUENCY_ESC_UPDATE 700		 //[Hz]
#define FREQUENCY_IMU_READING 700		 //[Hz]
#define FREQUENCY_RX_READING 200		 //[Hz]	set more than it is possible (>150)
#define FREQUENCY_TELEMETRY_UPDATE 1	 //[Hz]
#define FREQUENCY_SYSTEM_CHECK 5		 //[Hz]
#define FREQUENCY_OSD_UPDATE 5			 //[Hz]

//------------FAILSAFE--------------------

enum failsafe_t
{
	NO_FAILSAFE,
	INCORRECT_CHANNELS_VALUES,
	RX_TIMEOUT,
	NO_PREARM,
	SETUP_ERROR,
	I2C_ERROR,
	SPI_IMU_ERROR,
	SPI_FLASH_ERROR,
	SPI_OSD_ERROR,
	PID_LOOP_TIMEOUT,
	FAILSAFES_COUNTER
};

enum arming_t
{
	DISARMED,
	ARMED,
	PREARMED
};

//--------------BLACKBOX-------------------
#define USE_FLASH_BLACKBOX // USE_FLASH_BLACKBOX if you want BLACKBOX or NOT_USE_FLASH_BLACKBOX
/*define which parameters you would like to save:
 *
 * BLACKBOX_SAVE_FILTERED_GYRO_AND_ACC 	- 6 values
 * BLACKBOX_SAVE_FILTERED_GYRO 			- 3 values
 * BLACKBOX_SAVE_FILTERED_ACC			- 3 values
 * BLACKBOX_SAVE_RAW_GYRO_AND_ACC		- 6 values
 * BLACKBOX_SAVE_RAW_GYRO				- 3 values
 * BLACKBOX_SAVE_RAW_ACC				- 3 values
 * BLACKBOX_SAVE_EULER_ANGLES			- 3 values
 * BLACKBOX_SAVE_SET_ANGLES				- 3 values
 * BLACKBOX_SAVE_STICKS					- 3 values
 *
 */
#define BLACKBOX_SAVE_EULER_ANGLES
#define BLACKBOX_SAVE_SET_ANGLES
#define BLACKBOX_SAVE_FILTERED_GYRO

enum blackbox_t
{
	BLACKBOX_IDLE,
	BLACKBOX_COLLECT_DATA,
	BLACKBOX_SEND_DATA,
	BLACKBOX_ERASE
};

//---------------------OSD--------------------

#define OSD_BATTERY_VOLTAGE_PLACEMENT 361
#define OSD_BATTERY_CELL_VOLTAGE_PLACEMENT 370
#define OSD_TIMER_PLACEMENT 380
#define OSD_FLIGHT_MODE_PLACEMENT 23
#define OSD_LOGO_PLACEMENT 123
#define OSD_WARNING_PLACEMENT 315
#define OSD_AUTO_NTSC_PAL //	options:	OSD_AUTO_NTSC_PAL/OSD_CAMERA_PAL/OSD_CAMERA_NTSC

//-----------------I2C DEVICES----------------
#define USE_I2C1
#define I2C1_BUFFER_SIZE 10

//-------------------FILTERS------------------
#define USE_IIR_FILTERS // USE_FIR_FILTERS or USE_IIR_FILTERS
#define GYRO_FILTERS_ORDER 2
#define ACC_FILTERS_ORDER 2

//	remember to define coefficients in global_variables.c

//-----------OFFSETS and CALIBRATIONS VALUE----------
#define PITCH_OFFSET -7
#define ROLL_OFFSET 0

#define GYRO_ROLL_OFFSET 99
#define GYRO_PITCH_OFFSET 8
#define GYRO_YAW_OFFSET -9

#define ACC_PITCH_OFFSET 125.6525f
#define ACC_ROLL_OFFSET 2.862f
#define ACC_YAW_OFFSET 391.325f

#define ACC_CALIBRATION_X_X 4249.908f
#define ACC_CALIBRATION_X_Y -43.456f
#define ACC_CALIBRATION_X_Z 108.501f

#define ACC_CALIBRATION_Y_X 184.890f
#define ACC_CALIBRATION_Y_Y 4107.778f
#define ACC_CALIBRATION_Y_Z 755.494f

#define ACC_CALIBRATION_Z_X -114.671f
#define ACC_CALIBRATION_Z_Y -279.031f
#define ACC_CALIBRATION_Z_Z 4521.060f

//-------------DEBUGGING--------------

//#define IMU_TEST // for IMU SELFTEST functions

#endif /* GLOBAL_CONSTANTS_H_ */
