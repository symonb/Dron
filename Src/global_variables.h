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
#include "sensors/MS5XXX.h"

 //---------GLOBAL NEW TYPES-------------


typedef uint64_t timeUs_t;
typedef uint32_t timeMs_t;

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
	FLIGHT_MODE_ALT_HOLD,
	FLIGHT_MODE_COUNT
} flight_mode_e;

//------------FAILSAFE--------------------

typedef enum
{
	FAILSAFE_NO_FAILSAFE,
	FAILSAFE_INCORRECT_CHANNELS_VALUES,
	FAILSAFE_RX_TIMEOUT,
	FAILSAFE_NO_PREARM,
	FAILSAFE_THROTTLE_PREARM,
	FAILSAFE_SPI_IMU_ERROR,
	FAILSAFE_SPI_FLASH_ERROR,
	FAILSAFE_SPI_OSD_ERROR,
	FAILSAFE_GYRO_CALIBRATION,
	FAILSAFE_I2C1_ERROR,
	FAILSAFE_BLACKBOX_FULL,
	FAILSAFE_BATTERY_LOW,
	FAILSAFE_COUNTER
} failsafe_e;

typedef enum {
	WARNING_NONE,
	WARNING_NO_RX,
	WARNING_RX_ERROR,
	WARNING_NO_PREARM,
	WARNING_THROTTLE_PREARM,
	WARNING_IMU_ERROR,
	WARNING_FLASH_ERROR,
	WARNING_GYRO_CALIBRATION,
	WARNING_I2C_ERROR,
	WARNING_BLACKBOX_FULL,
	WARNING_BATTERY_LOW,
	WARNING_COUNTER
} warning_e;

typedef enum
{
	DISARMED,
	ARMED,
	PREARMED
}arming_e;


typedef struct {
	char* name;
	uint8_t	address;
	uint8_t rev_id;
	int16_t raw_data[3];
	float offset[3];
	bool calibrated;
	bool new_raw_data_flag;
	bool new_filtered_data;
} gyro_t;

typedef struct {
	char* name;
	uint8_t id;
	int16_t raw_data[3];
	float offset[3];
	float scale[3];
	bool calibrated;
	bool new_raw_data_flag;
	bool new_filtered_data;
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
	float* channels;
	float channels_raw[4];	//only for roll, pitch, yaw, throttle
	float channels_previous[4]; //only for roll, pitch, yaw, throttle
	int16_t Throttle;
	timeUs_t last_time;
	bool new_data_flag;

} rx_t;

extern rx_t receiver;

extern volatile flight_mode_e flight_mode;

extern volatile arming_e Arming_status;

extern gyro_t gyro_1;
extern acc_t acc_1;
#if defined(USE_BARO)
extern baro_t baro_1;
#endif

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
// I2C1:
extern timeUs_t time_flag7_1;

//---------VARIABLES-----------

//------------PIDF-------------

extern PIDF_t R_PIDF;
extern PIDF_t P_PIDF;
extern PIDF_t Y_PIDF;

extern float tpa_coef;

extern PIDF_t att_PIDF;

extern PIDF_t rate_throttle_PIDF;
extern PIDF_t acc_PIDF;
extern PIDF_t corr_rate_throttle;
extern PIDF_t acc_throttle_PIDF;
extern PIDF_t corr_acc_throttle;

extern PIDF_t corr_att[];
extern threef_t corr_sum;
extern PIDF_t corr_baro;
extern float throttle;

extern float MCU_temperature;

extern bool buzzer_active;

extern threef_t global_euler_angles;

extern threef_t global_angles;

extern quaternion_t q_global_attitude;

extern quaternion_t q_trans_sensor_to_body_frame;

//------------SETPOINTS---------------
extern threef_t desired_rotation_speed;
extern threef_t desired_angles;
extern float desired_altitude;

extern uint16_t motor_value[];

extern uint32_t motors_rpm[];

extern float motors_error[];
extern float BDshot_invalid_response[];
extern float BDshot_no_response[];

extern uint16_t* motor_1_value_pointer;
extern uint16_t* motor_2_value_pointer;
extern uint16_t* motor_3_value_pointer;
extern uint16_t* motor_4_value_pointer;

extern float Gyro_Acc[];

extern uint16_t table_to_send[];

//----------------FAILSAFE-------------------
extern failsafe_e FailSafe_status;
extern uint16_t failsafe_counter[FAILSAFE_COUNTER];
extern warning_e OSD_warning;

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

//----------TELEMETRY----------
extern bool transmitting_is_Done;




//------------ESC PROTOCOL VARIABLES-----------
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
