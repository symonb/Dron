/*
 * global_variables.c
 *
 *  Created on: 05.05.2021
 *      Author: symon
 */

#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"

// FOR DEBUGING ONLY

int64_t debug_variable_1;

flight_mode_e flight_mode;

//----------TIME VARIABLES--------
volatile timeUs_t Global_Time = 0;

timeUs_t dt_global = 0;

// main:
timeUs_t time_flag0_2 = 0;
timeUs_t time_flag0_3 = 0;
// stabilize:
timeUs_t time_flag1_2 = 0;
// acro:
timeUs_t time_flag2_2 = 0;
// ibus:
timeUs_t time_flag3_1 = 0;
// MPU:
timeUs_t time_flag4_1 = 0;
// flash:
timeUs_t time_flag5_1 = 0;

uint16_t channels[14] = {
	1500,
	1500,
	1000,
	1500,
	1000,
	1000,
	1000,
	1000,
	1000,
	1000,
	1000,
	1000,
	1000,
	1000,
};
uint16_t channels_previous_values[14] = {
	1500,
	1500,
	1000,
	1500,
	1000,
	1000,
	1000,
	1000,
	1000,
	1000,
	1000,
	1000,
	1000,
	1000,
};

int8_t arming_status = 0; // 1 - armed, 0 - prearmed, -1  disarmed, -2 - prearm error

int16_t Throttle = 1000;

ThreeF global_euler_angles = {0, 0, 0};

ThreeF global_angles = {0, 0, 0};

Quaternion q_global_position = {1, 0, 0, 0};

// motor's values set by PID's:
uint16_t motor_1_value;
uint16_t motor_2_value;
uint16_t motor_3_value;
uint16_t motor_4_value;

// pointers for motor's values:
uint16_t *motor_1_value_pointer;
uint16_t *motor_2_value_pointer;
uint16_t *motor_3_value_pointer;
uint16_t *motor_4_value_pointer;

int16_t Gyro_Acc[GYRO_ACC_SIZE]; // table for measurements 3 gyro, 3 accelerometer, 1 temperature

uint16_t table_to_send[ALL_ELEMENTS_TO_SEND];

bool New_data_to_send = false;

bool ibus_received = false;

//---------------I2C1---------------
uint8_t I2C1_read_write_flag = 1;
uint8_t I2C1_read_buffer[I2C1_BUFFER_SIZE];

bool imu_received = false;

bool transmitting_is_Done = true;

uint32_t dshot_buffer_1[DSHOT_BUFFER_LENGTH];
uint16_t dshot_buffer_2[DSHOT_BUFFER_LENGTH];
uint16_t dshot_buffer_3[DSHOT_BUFFER_LENGTH];
uint32_t dshot_buffer_4[DSHOT_BUFFER_LENGTH];
uint16_t dshot_buffer_4_1[DSHOT_PWM_FRAME_LENGTH * 2];
uint16_t dshot_buffer_2_3[DSHOT_PWM_FRAME_LENGTH * 2];

enum failsafe_type FailSafe_type;
/* 1-disarmed 2-incorrect channels values 3-RX timeout
 * 4-setup_error 5-I2C communication error 6-SPI communication error
 * 7-SPI flash error 8-PID loop time too long
 */

// value of PWM to power off motors (range is 2000-4000 which is in standard PWM 1000-2000):
#if defined(ESC_PROTOCOL_DSHOT)

uint16_t MOTOR_OFF = 1953;

#elif defined(ESC_PROTOCOL_PWM) || defined(ESC_PROTOCOL_ONESHOT125)

uint16_t MOTOR_OFF = 2000;

#endif

//---------FLASH-----------

bool USB_detected = false;

uint8_t flash_write_buffer[512];
uint16_t flash_write_counter = 0;

uint8_t flash_read_buffer[512];
uint16_t flash_read_counter = 0;

uint8_t blackbox_command = 0; // 0 no saving, no reading; 1 saving to flash; 2 reading from flash and sending via uart
uint32_t flash_global_write_address = 0x0;

float global_variable_monitor[3];
