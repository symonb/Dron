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
#include "acro.h"
#include "MPU6000.h"

static ThreeF corrections();

//---User defines maximum speed of spinning [deg/s]:
static Three Rates = { 500, 500, 400 };

//4s
static PIDF R_PIDF = { 0.4, 0.6, 0.0025, 0.001 };
static PIDF P_PIDF = { 0.4, 0.6, 0.004, 0.001 };
static PIDF Y_PIDF = { 2, 0.1, 0.001, 0.001 };
// 3s
//static PIDF R_PIDF = { 0.7, 0.8, 0.01, 0.01};
//static PIDF P_PIDF = { 0.7, 0.8, 0.01, 0.01};
//static PIDF Y_PIDF = { 2, 0.1, 0.001, 0.001 };

static ThreeF err = { 0, 0, 0 };
static ThreeF sum_err = { 0, 0, 0 };
static ThreeF D_corr = { 0, 0, 0 };
static ThreeF last_D_corr = { 0, 0, 0 };
static ThreeF F_corr = { 0, 0, 0 };

static float dt;

//for debugging only:
static int puk2 = 0;
static float czas_trwania_petli = 0;

void acro() {
	czas_trwania_petli = get_Global_Time();
	static float time_flag2_1;
	static float time_flag2_2;

	dt = (get_Global_Time() - time_flag2_1);
	time_flag2_1 = get_Global_Time();

	set_motors(corrections());

	if ((get_Global_Time() - time_flag2_2) >= 1. / FREQUENCY_TELEMETRY_UPDATE) {
		time_flag2_2 = get_Global_Time();
		puk2++;
		//wypisywanie korekcji pitch P I D i roll P I D; k¹tów; zadanych wartosci
		table_to_send[0] = P_PIDF.P * err.pitch * 500.  + 1000;
		table_to_send[1] = P_PIDF.I * sum_err.pitch * 500. + 1000;
		table_to_send[2] = P_PIDF.D * D_corr.pitch * 500.  + 1000;
		table_to_send[3] = R_PIDF.P * err.roll * 500. + 1000;
		table_to_send[4] = R_PIDF.I * sum_err.roll * 500. + 1000;
		table_to_send[5] = R_PIDF.D * D_corr.roll * 500. + 1000;
		table_to_send[6] = (Gyro_Acc[0] / 32.768 / Rates.roll * 50) + 1000;
		table_to_send[7] = (Gyro_Acc[1] / 32.768 / Rates.pitch * 50) + 1000;
		table_to_send[8] = Y_PIDF.P * err.yaw * 500.  + 1000;
		table_to_send[9] = Y_PIDF.I * sum_err.yaw * 500. + 1000;
		table_to_send[10] = Y_PIDF.D * D_corr.yaw * 500. + 1000;
		table_to_send[11] = (Gyro_Acc[2] / 32.768 / Rates.yaw * 50) + 1000;
		table_to_send[12] = channels[1] - 500;
		table_to_send[13] = channels[0] - 500;

		czas_trwania_petli = get_Global_Time() - czas_trwania_petli;
		New_data_to_send = 1;
	}

}

static ThreeF corrections() {

	static ThreeF corr = { 0, 0, 0 };
	static Three last_channels = { 0, 0, 0 };
	static Three last_measurement = { 0, 0, 0 };

	err.roll = (channels[0] - 1500) / 500. - Gyro_Acc[0] *0.0305185f/ Rates.roll;
	err.pitch = (channels[1] - 1500) / 500. - Gyro_Acc[1] *0.0305185f/ Rates.pitch;
	err.yaw = (channels[3] - 1500) / 500. - Gyro_Acc[2] *0.0305185f/ Rates.yaw;

	//	estimate Integral by sum (I term):
	sum_err.roll += err.roll * dt;
	sum_err.pitch += err.pitch * dt;
	sum_err.yaw += err.yaw * dt;

	//D correction will be divide for measurements and set-point corrections:

	D_corr.roll = (last_measurement.roll - Gyro_Acc[0]) * 0.0305185f /Rates.roll/ dt;
	D_corr.pitch = (last_measurement.pitch - Gyro_Acc[1]) * 0.0305185f/Rates.pitch / dt;
	D_corr.yaw = (last_measurement.yaw - Gyro_Acc[2]) * 0.0305185f /Rates.yaw/ dt;

	F_corr.roll = (channels[0] - last_channels.roll)*0.002f / dt;
	F_corr.pitch = (channels[1] - last_channels.pitch) *0.002f/ dt;
	F_corr.yaw = (channels[3] - last_channels.yaw) *0.002f/ dt;

	anti_windup(&sum_err,&R_PIDF,&P_PIDF,&Y_PIDF);

	//	calculate corrections:
	corr.roll = (R_PIDF.P * err.roll + R_PIDF.I * sum_err.roll
			+ R_PIDF.D * D_corr.roll + R_PIDF.F * F_corr.roll) * 500;
	corr.pitch = (P_PIDF.P * err.pitch + P_PIDF.I * sum_err.pitch
			+ P_PIDF.D * D_corr.pitch + P_PIDF.F * F_corr.pitch) * 500;
	corr.yaw = (Y_PIDF.P * err.yaw + Y_PIDF.I * sum_err.yaw
			+ Y_PIDF.D * D_corr.yaw + Y_PIDF.F * F_corr.yaw) * 500;

	//	set current measurements as last measurements:

	last_measurement.roll = Gyro_Acc[0];
	last_measurement.pitch = Gyro_Acc[1];
	last_measurement.yaw = Gyro_Acc[2];

	last_D_corr.roll = D_corr.roll;
	last_D_corr.pitch = D_corr.pitch;
	last_D_corr.yaw = D_corr.yaw;

	last_channels.roll = channels[0];
	last_channels.pitch = channels[1];
	last_channels.yaw = channels[3];

	return corr;
}

