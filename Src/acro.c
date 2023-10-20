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
#include "sensors/MPU6000.h"
#include "filters.h"
#include "telemetry.h"
#include "mixer.h"
#include "acro.h"

static threef_t corrections(float);

void acro(timeUs_t dt_us)
{
	float dt = US_TO_SEC(dt_us);
	set_motors(corrections(dt));
}

static threef_t corrections(float dt)
{
	static threef_t last_measurement;

	threef_t err = { 0, 0, 0 };

	// compute error:
	err.roll = (desired_rotation_speed.roll - Gyro_Acc[0]) / RATES_MAX_RATE_R;
	err.pitch = (desired_rotation_speed.pitch - Gyro_Acc[1]) / RATES_MAX_RATE_P;
	err.yaw = (desired_rotation_speed.yaw - Gyro_Acc[2]) / RATES_MAX_RATE_Y;

	corr_att[0].P = R_PIDF.P * err.roll * tpa_coef;
	corr_att[1].P = P_PIDF.P * err.pitch * tpa_coef;
	corr_att[2].P = Y_PIDF.P * err.yaw * tpa_coef;

	//	estimate Integral by sum (I term):
	corr_att[0].I += err.roll * dt * R_PIDF.I * tpa_coef;
	corr_att[1].I += err.pitch * dt * P_PIDF.I * tpa_coef;
	corr_att[2].I += err.yaw * dt * Y_PIDF.I * tpa_coef;

	// D correction will be divide for measurements and set-point corrections:
	corr_att[0].D = (last_measurement.roll - Gyro_Acc[0]) / RATES_MAX_RATE_R / dt * R_PIDF.D * tpa_coef;
	corr_att[1].D = (last_measurement.pitch - Gyro_Acc[1]) / RATES_MAX_RATE_P / dt * P_PIDF.D * tpa_coef;
	corr_att[2].D = (last_measurement.yaw - Gyro_Acc[2]) / RATES_MAX_RATE_Y / dt * Y_PIDF.D * tpa_coef;

	corr_att[0].F = (receiver.channels[0] - receiver.channels_previous[0]) * 0.0005f / dt * R_PIDF.F * tpa_coef;
	corr_att[1].F = (receiver.channels[1] - receiver.channels_previous[1]) * 0.0005f / dt * P_PIDF.F * tpa_coef;
	corr_att[2].F = (receiver.channels[3] - receiver.channels_previous[3]) * 0.0005f / dt * Y_PIDF.F * tpa_coef;

	anti_windup();
	D_term_filtering();
	ff_filtering();

	//	calculate corrections:
	corr_sum.roll = corr_att[0].P + corr_att[0].I + corr_att[0].D + corr_att[0].F;
	corr_sum.pitch = corr_att[1].P + corr_att[1].I + corr_att[1].D + corr_att[1].F;
	corr_sum.yaw = corr_att[2].P + corr_att[2].I + corr_att[2].D + corr_att[2].F;

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
	// table_to_send[6] = Gyro_Acc[0] ;
	// table_to_send[7] = Gyro_Acc[1];
	// table_to_send[8] = Y_PIDF.P * err.yaw + 1000;
	// table_to_send[9] = Y_PIDF.I * sum_err.yaw + 1000;
	// table_to_send[10] = Y_PIDF.D * D_corr.yaw + 1000;
	// table_to_send[11] = Gyro_Acc[2];
	// table_to_send[12] = receiver.channels[1] - 500;
	// table_to_send[13] = receiver.channels[0] - 500;

	print(table_to_send, ALL_ELEMENTS_TO_SEND);
}
