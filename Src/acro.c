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
#include "filters.h"
#include "telemetry.h"
#include "acro.h"

static threef_t corrections(float);

void acro(timeUs_t dt_us)
{
	float dt = US_TO_SEC(dt_us);
	set_motors(corrections(dt));
}

static threef_t corrections(float dt)
{
	static three_t last_measurement = { 0, 0, 0 };

	threef_t err = { 0, 0, 0 };

	if (flight_mode == FLIGHT_MODE_ACRO)
	{
		// use rotation speed computed sticks position:
		err.roll = (desired_rotation_speed.roll - Gyro_Acc[0] * GYRO_TO_DPS) / RATES_MAX_RATE_R;
		err.pitch = (desired_rotation_speed.pitch - Gyro_Acc[1] * GYRO_TO_DPS) / RATES_MAX_RATE_P;
		err.yaw = (desired_rotation_speed.yaw - Gyro_Acc[2] * GYRO_TO_DPS) / RATES_MAX_RATE_Y;
	}
	else if (flight_mode == FLIGHT_MODE_STABLE)
	{
		// use rotation speed computed from desired angles:
		err.roll = 1.5 * desired_rotation_speed.roll - Gyro_Acc[0] * GYRO_TO_DPS / RATES_MAX_RATE_R;
		err.pitch = 1.5 * desired_rotation_speed.pitch - Gyro_Acc[1] * GYRO_TO_DPS / RATES_MAX_RATE_P;
		err.yaw = (desired_rotation_speed.yaw - Gyro_Acc[2] * GYRO_TO_DPS) / RATES_MAX_RATE_Y;
	}

	corr_PIDF[0].P = R_PIDF.P * err.roll;
	corr_PIDF[1].P = P_PIDF.P * err.pitch;
	corr_PIDF[2].P = Y_PIDF.P * err.yaw;

	//	estimate Integral by sum (I term):
	corr_PIDF[0].I += err.roll * dt * R_PIDF.I;
	corr_PIDF[1].I += err.pitch * dt * P_PIDF.I;
	corr_PIDF[2].I += err.yaw * dt * Y_PIDF.I;

	// D correction will be divide for measurements and set-point corrections:
	corr_PIDF[0].D = (last_measurement.roll - Gyro_Acc[0]) * 0.0305185f / RATES_MAX_RATE_R / dt * R_PIDF.D;
	corr_PIDF[1].D = (last_measurement.pitch - Gyro_Acc[1]) * 0.0305185f / RATES_MAX_RATE_P / dt * P_PIDF.D;
	corr_PIDF[2].D = (last_measurement.yaw - Gyro_Acc[2]) * 0.0305185f / RATES_MAX_RATE_Y / dt * Y_PIDF.D;

	corr_PIDF[0].F = (receiver.channels[0] - receiver.channels_previous_values[0]) * 0.002f / dt * R_PIDF.F;
	corr_PIDF[1].F = (receiver.channels[1] - receiver.channels_previous_values[1]) * 0.002f / dt * P_PIDF.F;
	corr_PIDF[2].F = (receiver.channels[3] - receiver.channels_previous_values[3]) * 0.002f / dt * Y_PIDF.F;

	anti_windup();
	D_term_filtering();

	//	calculate corrections:
	corr_sum.roll = corr_PIDF[0].P + corr_PIDF[0].I + corr_PIDF[0].D + corr_PIDF[0].F;
	corr_sum.pitch = corr_PIDF[1].P + corr_PIDF[1].I + corr_PIDF[1].D + corr_PIDF[1].F;
	corr_sum.yaw = corr_PIDF[2].P + corr_PIDF[2].I + corr_PIDF[2].D + corr_PIDF[2].F;

	//	set current measurements as last measurements:

	last_measurement.roll = Gyro_Acc[0];
	last_measurement.pitch = Gyro_Acc[1];
	last_measurement.yaw = Gyro_Acc[2];

	return corr_sum;
}
void send_telemetry_acro(timeUs_t time)
{
	// // send corrections pitch P I D i roll P I D; angle_ratio; set_values
	// table_to_send[0] = P_PIDF.P * err.pitch + 1000;
	// table_to_send[1] = P_PIDF.I * sum_err.pitch + 1000;
	// table_to_send[2] = P_PIDF.D * D_corr.pitch + 1000;
	// table_to_send[3] = R_PIDF.P * err.roll + 1000;
	// table_to_send[4] = R_PIDF.I * sum_err.roll + 1000;
	// table_to_send[5] = R_PIDF.D * D_corr.roll + 1000;
	// table_to_send[6] = (Gyro_Acc[0] / 32.768f / RATES_MAX_RATE_R * 50) + 1000;
	// table_to_send[7] = (Gyro_Acc[1] / 32.768f / RATES_MAX_RATE_P * 50) + 1000;
	// table_to_send[8] = Y_PIDF.P * err.yaw + 1000;
	// table_to_send[9] = Y_PIDF.I * sum_err.yaw + 1000;
	// table_to_send[10] = Y_PIDF.D * D_corr.yaw + 1000;
	// table_to_send[11] = (Gyro_Acc[2] / 32.768f / RATES_MAX_RATE_Y * 50) + 1000;
	// table_to_send[12] = receiver.channels[1] - 500;
	// table_to_send[13] = receiver.channels[0] - 500;

	print(table_to_send, ALL_ELEMENTS_TO_SEND);
}
