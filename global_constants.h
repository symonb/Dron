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

#define DSHOT_MODE 600 		// 150 300 600 are available
#define DSHOT_BUFFER_LENGTH 16
#define DSHOT_PWM_FRAME_LENGTH 8
#define DSHOT_1_LENGTH 6
#define DSHOT_0_LENGTH 3

#define FREQUENCY_PID_LOOP 400 //[Hz]   IF YOU' RE USING PWM MAX. IS 500[Hz] (little less), IF DSHOT you can go up to 1[kHz]
#define FREQUENCY_ESC_UPDATE 400 //[Hz]
#define FREQUENCY_IMU_READING 400 //[Hz]
#define FREQUENCY_TELEMETRY_UPDATE 50 //[Hz]

// OFFSETS and CALIBRATIONS VALUE

#define PITCH_OFFSET -3
#define ROLL_OFFSET 12

#define GYRO_ROLL_OFFSET -33
#define GYRO_PITCH_OFFSET 32
#define GYRO_YAW_OFFSET -55

#define ACC_PITCH_OFFSET 125.6525
#define ACC_ROLL_OFFSET 2.862
#define ACC_YAW_OFFSET 391.325

#define ACC_CALIBRATION_X_X 4249.908
#define ACC_CALIBRATION_X_Y -43.456
#define ACC_CALIBRATION_X_Z 108.501

#define ACC_CALIBRATION_Y_X 184.890
#define ACC_CALIBRATION_Y_Y 4107.778
#define ACC_CALIBRATION_Y_Z 755.494

#define ACC_CALIBRATION_Z_X -114.671
#define ACC_CALIBRATION_Z_Y -279.031
#define ACC_CALIBRATION_Z_Z 4521.060

#define GYRO_PART 0.99
#define ACC_PART 0.01
#define GYRO_TO_DPS 32768/1000. // convert gyro register into degrees per second unit

#define RAD_TO_DEG  180 / M_PI
#define DEG_TO_RAD  M_PI/180


#endif /* GLOBAL_CONSTANTS_H_ */
