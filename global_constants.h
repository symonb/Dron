/*
 * global_constants.h
 *
 *  Created on: 18.04.2021
 *      Author: symon
 */

#ifndef GLOBAL_CONSTANTS_H_
#define GLOBAL_CONSTANTS_H_

#define CHANNELS  10					//ilosc kanalĂłw (4 potrzebne do sterownaia)
#define GYRO_ACC_SIZE 7				//3 for gyro, 3 acc and 1 for temperature
#define ALL_ELEMENTS_TO_SEND 14			//telemetry informations
#define MAX_RX_SIGNAL 2050
#define MIN_RX_SIGNAL 950
#define MAX_NO_SIGNAL_TIME 0.5 			//[s]
#define DISARM_VALUE 1600
#define TIMEOUT_VALUE 0.005				//[s]
#define MAX_ROLL_ANGLE 30
#define MAX_PITCH_ANGLE 30

#define DSHOT_MODE 600 		// 150 300 600 1200
#define DSHOT_BUFFER_LENGTH 18 // 16 bits of Dshot and 2 for clearing
#define DSHOT_PWM_FRAME_LENGTH 35
#define DSHOT_1_LENGTH 26
#define DSHOT_0_LENGTH 13

#define ESC_PROTOCOL_DSHOT // ESC_PROTOCOL_PWM, ESC_PROTOCOL_ONESHOT125, ESC_PROTOCOL_ONESHOT_V1 - bledy , ESC_PROTOCOL_DSHOT, ESC_PROTOCOL_DSHOT_BURST - nieskonczone

#define STABILIZE_FILTER_MAGDWICK	//STABILIZE_FILTER_MAGDWICK, STABILIZE_FILTER_MAHONY, STABILIZE_FILTER_COMPLEMENTARY
#define MAGDWICK_NEW  //MAGDWICK_ORIGINAL, MAGDWICK_NEW

#define FREQUENCY_PID_LOOP 600 //[Hz]   IF YOU' RE USING PWM MAX. IS 500[Hz] (little less), IF DSHOT you can go up to 1[kHz]
#define FREQUENCY_ESC_UPDATE 1000 //[Hz]
#define FREQUENCY_IMU_READING 600 //[Hz]
#define FREQUENCY_TELEMETRY_UPDATE 1 //[Hz]

#define USE_FLASH_BLACKBOX // USE_FLASH_BLACKBOX if you want BLACKBOX or NOT_USE_FLASH_BLACKBOX

// OFFSETS and CALIBRATIONS VALUE

#define PITCH_OFFSET -7
#define ROLL_OFFSET 0

#define GYRO_ROLL_OFFSET -33
#define GYRO_PITCH_OFFSET 32
#define GYRO_YAW_OFFSET -7

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

#define GYRO_PART 0.995f
#define ACC_PART 0.005f
#define GYRO_TO_DPS 1000/32767.f // convert gyro register into degrees per second unit

#define RAD_TO_DEG  180 / M_PI
#define DEG_TO_RAD  M_PI/180
#define GYRO_TO_RAD  (1.f / 32.767f * DEG_TO_RAD)// convert gyro register into rad per second unit

#define ACC_TO_GRAVITY  1.f / 4096 //convert acc register into gravity unit

#endif /* GLOBAL_CONSTANTS_H_ */
