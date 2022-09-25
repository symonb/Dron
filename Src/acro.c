/*
 * acro.c
 *
 *  Created on: 06.02.2021
 *      Author: symon
 */

#include <math.h>
#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "MPU6000.h"
#include "acro.h"
#include "tasks.h"

static ThreeF corrections(float);

//---User defines maximum speed of spinning [deg/s]:
static Three Rates = {500, 500, 400};

// 4s
static PIDF R_PIDF = {200, 300, 1.25, 0.5};
static PIDF P_PIDF = {200, 300, 2, 0.5};
static PIDF Y_PIDF = {1000, 50, 0.5, 0.5};

// 3s
// static PIDF R_PIDF = { 350, 400, 5, 5};
// static PIDF P_PIDF = { 350, 400, 5, 5};
// static PIDF Y_PIDF = { 1000, 50, 0.5, 0.5 };

static ThreeF err = {0, 0, 0};
static ThreeF sum_err = {0, 0, 0};
static ThreeF D_corr = {0, 0, 0};
static ThreeF last_D_corr = {0, 0, 0};
static ThreeF F_corr = {0, 0, 0};

void acro(timeUs_t dt_us)
{
	static float dt;

	dt = US_TO_SEC(dt_us);

	set_motors(corrections(dt));
}

static ThreeF corrections(float dt)
{

	static ThreeF corr = {0, 0, 0};
	static Three last_measurement = {0, 0, 0};

	if (flight_mode == FLIGHT_MODE_ACRO)
	{
		//	use user sticks values as inputs:
		err.roll = (channels[0] - 1500) / 500.f - Gyro_Acc[0] * GYRO_TO_DPS / Rates.roll;
		err.pitch = (channels[1] - 1500) / 500.f - Gyro_Acc[1] * GYRO_TO_DPS / Rates.pitch;
		err.yaw = (channels[3] - 1500) / 500.f - Gyro_Acc[2] * GYRO_TO_DPS / Rates.yaw;
	}
	else if (flight_mode == FLIGHT_MODE_STABLE)
	{
		// use rotation speed computed from desired angles:
		err.roll = desired_rotation_speed.roll - Gyro_Acc[0] * GYRO_TO_DPS / Rates.roll;
		err.pitch = desired_rotation_speed.pitch - Gyro_Acc[1] * GYRO_TO_DPS / Rates.pitch;
		err.yaw = desired_rotation_speed.yaw - Gyro_Acc[2] * GYRO_TO_DPS / Rates.yaw;
	}

	//	estimate Integral by sum (I term):
	sum_err.roll += err.roll * dt;
	sum_err.pitch += err.pitch * dt;
	sum_err.yaw += err.yaw * dt;

	// D correction will be divide for measurements and set-point corrections:

	D_corr.roll = (last_measurement.roll - Gyro_Acc[0]) * 0.0305185f / Rates.roll / dt;
	D_corr.pitch = (last_measurement.pitch - Gyro_Acc[1]) * 0.0305185f / Rates.pitch / dt;
	D_corr.yaw = (last_measurement.yaw - Gyro_Acc[2]) * 0.0305185f / Rates.yaw / dt;

	F_corr.roll = (channels[0] - channels_previous_values[0]) * 0.002f / dt;
	F_corr.pitch = (channels[1] - channels_previous_values[1]) * 0.002f / dt;
	F_corr.yaw = (channels[3] - channels_previous_values[3]) * 0.002f / dt;

	anti_windup(&sum_err, &R_PIDF, &P_PIDF, &Y_PIDF);

	//	calculate corrections:
	corr.roll = (R_PIDF.P * err.roll + R_PIDF.I * sum_err.roll + R_PIDF.D * D_corr.roll + R_PIDF.F * F_corr.roll) * 2;
	corr.pitch = (P_PIDF.P * err.pitch + P_PIDF.I * sum_err.pitch + P_PIDF.D * D_corr.pitch + P_PIDF.F * F_corr.pitch) * 2;
	corr.yaw = (Y_PIDF.P * err.yaw + Y_PIDF.I * sum_err.yaw + Y_PIDF.D * D_corr.yaw + Y_PIDF.F * F_corr.yaw) * 2;

	//	set current measurements as last measurements:

	last_measurement.roll = Gyro_Acc[0];
	last_measurement.pitch = Gyro_Acc[1];
	last_measurement.yaw = Gyro_Acc[2];

	last_D_corr.roll = D_corr.roll;
	last_D_corr.pitch = D_corr.pitch;
	last_D_corr.yaw = D_corr.yaw;

	return corr;
}
void send_telemetry_acro(timeUs_t time)
{
	// send corrections pitch P I D i roll P I D; angle_ratio; set_values
	table_to_send[0] = P_PIDF.P * err.pitch + 1000;
	table_to_send[1] = P_PIDF.I * sum_err.pitch + 1000;
	table_to_send[2] = P_PIDF.D * D_corr.pitch + 1000;
	table_to_send[3] = R_PIDF.P * err.roll + 1000;
	table_to_send[4] = R_PIDF.I * sum_err.roll + 1000;
	table_to_send[5] = R_PIDF.D * D_corr.roll + 1000;
	table_to_send[6] = (Gyro_Acc[0] / 32.768f / Rates.roll * 50) + 1000;
	table_to_send[7] = (Gyro_Acc[1] / 32.768f / Rates.pitch * 50) + 1000;
	table_to_send[8] = Y_PIDF.P * err.yaw + 1000;
	table_to_send[9] = Y_PIDF.I * sum_err.yaw + 1000;
	table_to_send[10] = Y_PIDF.D * D_corr.yaw + 1000;
	table_to_send[11] = (Gyro_Acc[2] / 32.768f / Rates.yaw * 50) + 1000;
	table_to_send[12] = channels[1] - 500;
	table_to_send[13] = channels[0] - 500;

	print(table_to_send, ALL_ELEMENTS_TO_SEND);
}
