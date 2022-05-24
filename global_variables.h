/*
 * global_variables.h
 *
 *  Created on: 05.05.2021
 *      Author: symon
 */

#ifndef GLOBAL_VARIABLES_H_
#define GLOBAL_VARIABLES_H_

//---------GLOBAL NEW TYPES-------------

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


typedef struct {
	float w;
	float x;
	float y;
	float z;
} Quaternion;


typedef enum
{
	FLIGHT_MODE_ACRO,
    FLIGHT_MODE_STABLE,
    FLIGHT_MODE_COUNT
}flight_mode_e;

typedef uint64_t timeUs_t;
typedef uint32_t timeMs_t;

extern flight_mode_e flight_mode;

//---------TIME VARIABLES-------------

extern volatile timeUs_t Global_Time;

extern timeUs_t dt_global;

//main:
extern  timeUs_t time_flag0_1;
extern  timeUs_t time_flag0_2;
extern  timeUs_t time_flag0_3;
//stabilize:
extern timeUs_t time_flag1_1;
extern timeUs_t time_flag1_2;
//acro:
extern timeUs_t time_flag2_1;
extern timeUs_t time_flag2_2;
//ibus:
extern timeUs_t time_flag3_1;
//MPU:
extern timeUs_t time_flag4_1;
//flash:
extern timeUs_t time_flag5_1;



//---------VARIABLES-----------

extern uint16_t channels[];

extern uint16_t channels_previous_values[];

extern int8_t arming_status;

extern int16_t Throttle;

extern ThreeF global_euler_angles;

extern ThreeF global_angles;

extern Quaternion q_global_position;

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

extern enum failsafe_type FailSafe_type;

extern uint16_t MOTOR_OFF;


//----------FLASH----------
extern uint8_t USB_detected;

extern uint8_t flash_write_buffer[];
extern uint16_t flash_write_counter;
extern uint8_t flash_read_buffer[];
extern uint16_t flash_read_counter;

extern uint8_t blackbox_command;
extern uint32_t flash_global_write_address;

extern float global_variable_monitor[];


#endif /* GLOBAL_VARIABLES_H_ */
