///*
// * tasks.c
// *
// *  Created on: 04.03.2022
// *      Author: symon
// */

#include <math.h>
#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"

#include "setup.h"
#include "stabilize.h"
#include "acro.h"
#include "ibus.h"
#include "scheduler.h"
#include "MPU6000.h"
#include "tasks.h"

bool gyro_acc_filtering_check_fun(timeUs_t current_time, timeUs_t delta_time);
bool Ibus_saving_check_fun(timeUs_t current_time, timeUs_t delta_time);
void (*mode_fun_tab[FLIGHT_MODE_COUNT])(timeUs_t dt);
void (*telemetry_fun_tab[FLIGHT_MODE_COUNT])(timeUs_t dt);

// ------DEFINE ALL TASKS--------
task_t all_tasks[TASKS_COUNT] =
	{[TASK_SYSTEM] = DEFINE_TASK("SYSTEM CHECK", task_system_fun, NULL, TASK_PRIORITY_LOW, TASK_PERIOD_HZ(FREQUENCY_SYSTEM_CHECK)),
	 [TASK_IBUS_SAVE] = DEFINE_TASK("IBUS SAVING", Ibus_save, Ibus_saving_check_fun, TASK_PRIORITY_REALTIME, TASK_PERIOD_HZ(FREQUENCY_RX_READING)),
	 [TASK_GYRO_ACC_FILTER] = DEFINE_TASK("GYRO ACC FILTERING", rewrite_Gyro_Acc_data, gyro_acc_filtering_check_fun, TASK_PRIORITY_REALTIME, TASK_PERIOD_HZ(FREQUENCY_IMU_READING)),
	 [TASK_PID] = DEFINE_TASK("PID", PID_fun, NULL, TASK_PRIORITY_REALTIME, TASK_PERIOD_HZ(FREQUENCY_PID_LOOP)),
	 [TASK_UPDATE_MOTORS] = DEFINE_TASK("UPDATE MOTORS", update_motors, NULL, TASK_PRIORITY_REALTIME, TASK_PERIOD_HZ(FREQUENCY_ESC_UPDATE)),
	 [TASK_TELEMETRY] = DEFINE_TASK("TELEMETRY", telemetry_fun, NULL, TASK_PRIORITY_LOW, TASK_PERIOD_HZ(FREQUENCY_TELEMETRY_UPDATE))};

void PID_fun(timeUs_t current_time)
{
	static timeUs_t last_time;
	static timeUs_t dt;
	dt = current_time - last_time;
	last_time = current_time;
	mode_fun_tab[get_Flight_Mode()](dt);
}

bool gyro_acc_filtering_check_fun(timeUs_t current_time, timeUs_t delta_time)
{
	return imu_received;
}

bool Ibus_saving_check_fun(timeUs_t current_time, timeUs_t delta_time)
{
	return ibus_received;
}

void (*mode_fun_tab[FLIGHT_MODE_COUNT])() =
	{
		[FLIGHT_MODE_STABLE] = stabilize,
		[FLIGHT_MODE_ACRO] = acro};

void telemetry_fun(timeUs_t time)
{
	telemetry_fun_tab[get_Flight_Mode()](time);
}

void (*telemetry_fun_tab[FLIGHT_MODE_COUNT])(timeUs_t dt) =
	{
		[FLIGHT_MODE_STABLE] = send_telemetry_stabilize,
		[FLIGHT_MODE_ACRO] = send_telemetry_acro};
