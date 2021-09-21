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
	int32_t roll; // X
	int32_t pitch; // Y
	int32_t yaw; // Z
} Three;

extern volatile double Global_Time;

extern volatile uint8_t Tim_7_flag;

extern uint16_t channels[];

extern int16_t Throttle;

extern uint16_t pwm_m1;
extern uint16_t pwm_m2;
extern uint16_t pwm_m3;
extern uint16_t pwm_m4;

extern uint16_t *PWM_M1;
extern uint16_t *PWM_M2;
extern uint16_t *PWM_M3;
extern uint16_t *PWM_M4;

extern int16_t Gyro_Acc[];

extern uint16_t table_to_send[];

extern uint8_t New_data_to_send;

extern uint8_t ibus_received;

extern uint8_t imu_received;

extern uint8_t I2C1_read_write_flag;

extern uint8_t transmitting_is_Done;

extern uint8_t failsafe_type;

extern uint16_t motor_off;

extern ThreeF global_euler_angles;

extern ThreeF global_angles;

#endif /* GLOBAL_VARIABLES_H_ */
