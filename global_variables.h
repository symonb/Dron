/*
 * global_variables.h
 *
 *  Created on: 05.05.2021
 *      Author: symon
 */

#ifndef GLOBAL_VARIABLES_H_
#define GLOBAL_VARIABLES_H_


typedef struct {
	double roll; // X
	double pitch; // Y
	double yaw; // Z
} ThreeD;
typedef struct {
	float roll; // X
	float pitch; // Y
	float yaw; // Z
} ThreeF;

typedef struct {
	float P;
	float I;
	float D;
} PID;

typedef struct {
	float P;
	float I;
	float D;
	float F;
} PIDF;

typedef struct {
	int32_t roll; // X
	int32_t pitch; // Y
	int32_t yaw; // Z
} Three;

extern volatile double Global_Time;

extern volatile uint8_t Tim_7_flag;

extern uint16_t channels[];

extern int16_t Throttle;

extern ThreeF global_euler_angles;

extern ThreeF global_angles;

extern uint16_t motor_1_value;
extern uint16_t motor_2_value;
extern uint16_t motor_3_value;
extern uint16_t motor_4_value;

extern uint16_t *motor_1_value_pointer;
extern uint16_t *motor_2_value_pointer;
extern uint16_t *motor_3_value_pointer;
extern uint16_t *motor_4_value_pointer;

extern int16_t Gyro_Acc[];

extern uint16_t table_to_send[];

extern uint8_t New_data_to_send;

extern uint8_t ibus_received;

extern uint8_t imu_received;

extern uint8_t I2C1_read_write_flag;

extern uint8_t transmitting_is_Done;

extern uint32_t dshot_buffer_1[];
extern uint16_t dshot_buffer_2[];
extern uint16_t dshot_buffer_3[];
extern uint32_t dshot_buffer_4[];
extern uint16_t dshot_buffer_4_1[];
extern uint16_t dshot_buffer_2_3[];

extern uint8_t failsafe_type;

extern uint16_t MOTOR_OFF;

//----------FLASH----------
extern uint8_t USB_detected;

extern uint8_t flash_write_buffer[];
extern uint16_t flash_write_counter;
extern uint8_t flash_read_buffer[];
extern uint16_t flash_read_counter;

extern uint8_t blackbox_command;
extern uint32_t flash_global_write_address;

#endif /* GLOBAL_VARIABLES_H_ */
