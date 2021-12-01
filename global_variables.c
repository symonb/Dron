/*
 * global_variables.c
 *
 *  Created on: 05.05.2021
 *      Author: symon
 */

#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"

volatile double Global_Time = 0;
volatile uint8_t Tim_7_flag=0;

uint16_t channels[14] = { 1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000,
		1000, 1000, 1000, 1000, 1000, };

int16_t Throttle = 1000;

//motor's values set by PID's:
uint16_t motor_1_value;
uint16_t motor_2_value;
uint16_t motor_3_value;
uint16_t motor_4_value;

//pointers for motor's values:
uint16_t *motor_1_value_pointer;
uint16_t *motor_2_value_pointer;
uint16_t *motor_3_value_pointer;
uint16_t *motor_4_value_pointer;

int16_t Gyro_Acc[GYRO_ACC_SIZE]; //table for measurements 3 gyro, 3 accelerometer, 1 temperature

uint16_t table_to_send[ALL_ELEMENTS_TO_SEND];

uint8_t New_data_to_send = 0;

uint8_t ibus_received = 0;

uint8_t I2C1_read_write_flag = 1;

uint8_t imu_received = 0;

uint8_t transmitting_is_Done = 1;

uint32_t dshot_buffer_1[DSHOT_BUFFER_LENGTH];
uint16_t dshot_buffer_2[DSHOT_BUFFER_LENGTH];
uint16_t dshot_buffer_3[DSHOT_BUFFER_LENGTH];
uint32_t dshot_buffer_4[DSHOT_BUFFER_LENGTH];
uint16_t dshot_buffer_4_1[DSHOT_PWM_FRAME_LENGTH*2];
uint16_t dshot_buffer_2_3[DSHOT_PWM_FRAME_LENGTH*2];

uint8_t failsafe_type;
/* 1-disarmed 2-incorrect channels values 3-RX timeout
 * 4-setup_error 5-I2C communication error 6-SPI communication error
 */
//uint16_t MOTOR_OFF= 1953;	//value of PWM to power off motors (range is 2000-4000 which is in standard PWM 1000-2000)

#if defined(ESC_PROTOCOL_DSHOT)

	uint16_t MOTOR_OFF= 1953;

#elif defined(ESC_PROTOCOL_PWM) || defined(ESC_PROTOCOL_ONESHOT125)

uint16_t MOTOR_OFF= 2000;

#endif

ThreeF global_euler_angles={ 0, 0, 0 };

ThreeF global_angles={ 0, 0, 0 };
