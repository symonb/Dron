/*
 * global_variables.h
 *
 *  Created on: 05.05.2021
 *      Author: symon
 */

#ifndef GLOBAL_VARIABLES_H_
#define GLOBAL_VARIABLES_H_

#include "stdint.h"
#include "stdbool.h"

 //---------GLOBAL NEW TYPES-------------

// roll pitch yaw struct for double
typedef struct
{
	double roll;  // X
	double pitch; // Y
	double yaw;	  // Z
} threed_t;

// roll pitch yaw struct for float
typedef struct
{
	float roll;	 // X
	float pitch; // Y
	float yaw;	 // Z
} threef_t;

// roll pitch yaw struct for uint32_t
typedef struct
{
	int32_t roll;  // X
	int32_t pitch; // Y
	int32_t yaw;   // Z
} three_t;

typedef struct
{
	float P;
	float I;
	float D;
} PID_t;

typedef struct
{
	float P;
	float I;
	float D;
	float F;
} PIDF_t;

typedef struct
{
	float w;
	float x;
	float y;
	float z;
} quaternion_t;

typedef enum
{
	FLIGHT_MODE_ACRO,
	FLIGHT_MODE_STABLE,
	FLIGHT_MODE_COUNT
} flight_mode_e;

typedef struct {
	char* name;
	uint8_t	address;
	uint8_t rev_id;
	int16_t raw_data[3];
	threef_t offset;
	bool calibrated;
	bool new_raw_data_flag;
	bool new_filtered_data;
} gyro_t;

typedef struct {
	char* name;
	uint8_t id;
	int16_t raw_data[3];
	threef_t offset;
	bool calibrated;
	bool new_raw_data_flag;
} acc_t;

typedef enum
{
	RX_IBUS
} rx_protocol_e;

typedef struct
{
	rx_protocol_e type;
	const uint8_t number_of_channels;
	uint8_t number_of_used_channels;
	uint16_t* channels;
	uint16_t* channels_previous_values;
	int16_t Throttle;

	bool new_data_flag;

} rx_t;

extern rx_t receiver;

extern volatile flight_mode_e flight_mode;

extern volatile arming_e Arming_status;

extern gyro_t gyro_1;
extern acc_t acc_1;

typedef uint64_t timeUs_t;
typedef uint32_t timeMs_t;

// FOR DEBUG ONLY:
extern double debug_variable_1;

//---------TIME VARIABLES-------------

extern volatile timeUs_t Global_Time;

extern timeUs_t dt_global;

// main:
extern timeUs_t time_flag0_1;
// stabilize:
extern timeUs_t time_flag1_1;
// acro:
extern timeUs_t time_flag2_1;
// ibus:
extern timeUs_t time_flag3_1;
// MPU:
extern timeUs_t time_flag4_1;
// flash:
extern timeUs_t time_flag5_1;
//	OSD:
extern timeUs_t time_flag6_1;

//---------VARIABLES-----------

//------------PIDF-------------

extern PIDF_t R_PIDF;
extern PIDF_t P_PIDF;
extern PIDF_t Y_PIDF;

extern PIDF_t corr_PIDF[];
extern threef_t corr_sum;

extern float MCU_temperature;

extern bool buzzer_active;

extern threef_t global_euler_angles;

extern threef_t global_angles;

extern quaternion_t q_global_position;

extern threef_t desired_rotation_speed;
extern threef_t desired_angles;

extern uint16_t motor_value[];

extern uint32_t motors_rpm[];

extern float motors_error[];
extern float BDshot_invalid_response[];
extern float BDshot_no_response[];

extern uint16_t* motor_1_value_pointer;
extern uint16_t* motor_2_value_pointer;
extern uint16_t* motor_3_value_pointer;
extern uint16_t* motor_4_value_pointer;

extern int16_t Gyro_Acc[];
extern int16_t Gyro_Acc_raw[];

extern uint16_t table_to_send[];

//----------------FAILSAFE-------------------

extern uint16_t failsafe_counter[FAILSAFE_COUNTER];

//-------------------FILTERS--------------------
#if defined(USE_FIR_FILTERS)
extern const float GYRO_FILTER_X_COEF[];
extern const float GYRO_FILTER_Y_COEF[];
extern const float GYRO_FILTER_Z_COEF[];

extern const float ACC_FILTER_X_COEF[];
extern const float ACC_FILTER_Y_COEF[];
extern const float ACC_FILTER_Z_COEF[];

extern const float D_TERM_FILTER_COEF[];

#elif defined(USE_IIR_FILTERS)

extern const float GYRO_FILTER_X_FORW_COEF[];
extern const float GYRO_FILTER_Y_FORW_COEF[];
extern const float GYRO_FILTER_Z_FORW_COEF[];

extern const float GYRO_FILTER_X_BACK_COEF[];
extern const float GYRO_FILTER_Y_BACK_COEF[];
extern const float GYRO_FILTER_Z_BACK_COEF[];

extern const float ACC_FILTER_X_FORW_COEF[];
extern const float ACC_FILTER_Y_FORW_COEF[];
extern const float ACC_FILTER_Z_FORW_COEF[];

extern const float ACC_FILTER_X_BACK_COEF[];
extern const float ACC_FILTER_Y_BACK_COEF[];
extern const float ACC_FILTER_Z_BACK_COEF[];

extern const float D_TERM_FILTER_FORW_COEF[];
extern const float D_TERM_FILTER_BACK_COEF[];

#endif

//--------------------I2C1----------------------
extern uint8_t I2C1_read_write_flag;
extern uint8_t I2C1_read_buffer[];

extern bool transmitting_is_Done;

extern uint32_t dshot_buffer_1[];
extern uint16_t dshot_buffer_2[];
extern uint16_t dshot_buffer_3[];
extern uint32_t dshot_buffer_4[];
extern uint16_t dshot_burst_buffer_4_1[];
extern uint16_t dshot_burst_buffer_2_3[];
extern uint32_t dshot_bb_buffer_1_4[];
extern uint32_t dshot_bb_buffer_2_3[];
extern uint32_t dshot_bb_buffer_1_4_r[];
extern uint32_t dshot_bb_buffer_2_3_r[];

extern failsafe_e FailSafe_status;

extern uint16_t MOTOR_OFF;

//----------FLASH----------

extern uint8_t flash_write_buffer[512];
extern uint16_t flash_write_counter;
extern uint8_t flash_read_buffer[];
extern uint16_t flash_read_counter;

extern volatile blackbox_e Blackbox_status;
extern uint32_t flash_global_write_address;

extern float global_variable_monitor[];

#endif /* GLOBAL_VARIABLES_H_ */
