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

static void main_PID_fun(timeUs_t current_time);
static void stabilization_fun(timeUs_t current_time);
static bool stabilization_check_fun(timeUs_t current_time, timeUs_t delta_time);
static bool gyro_acc_filtering_check_fun(timeUs_t current_time, timeUs_t delta_time);
static void ibus_save_enable(timeUs_t current_time);
static bool ibus_saving_fun(timeUs_t current_time, timeUs_t delta_time);
static void telemetry_fun(timeUs_t time);
static void (*telemetry_fun_tab[FLIGHT_MODE_COUNT])(timeUs_t dt);

// ------DEFINE ALL TASKS--------
task_t all_tasks[TASKS_COUNT] =
	{[TASK_SYSTEM] = DEFINE_TASK("SYSTEM CHECK", task_system_fun, NULL, TASK_PRIORITY_LOW, TASK_PERIOD_HZ(FREQUENCY_SYSTEM_CHECK)),
	 [TASK_IBUS_SAVE] = DEFINE_TASK("IBUS SAVING", ibus_save_enable, NULL, TASK_PRIORITY_REALTIME, TASK_PERIOD_HZ(FREQUENCY_RX_READING)),
	 [TASK_GYRO_ACC_FILTER] = DEFINE_TASK("GYRO ACC FILTERING", rewrite_Gyro_Acc_data, gyro_acc_filtering_check_fun, TASK_PRIORITY_REALTIME, TASK_PERIOD_HZ(FREQUENCY_IMU_READING)),
	 [TASK_MAIN_LOOP] = DEFINE_TASK("MAIN LOOP", main_PID_fun, NULL, TASK_PRIORITY_REALTIME, TASK_PERIOD_HZ(FREQUENCY_MAIN_LOOP)),
	 [TASK_STABILIZATION_LOOP] = DEFINE_TASK("STABILIZATION LOOP", stabilization_fun, stabilization_check_fun, TASK_PRIORITY_REALTIME, TASK_PERIOD_HZ(FREQUENCY_STABILIZATION_LOOP)),
	 [TASK_UPDATE_MOTORS] = DEFINE_TASK("UPDATE MOTORS", update_motors, NULL, TASK_PRIORITY_REALTIME, TASK_PERIOD_HZ(FREQUENCY_ESC_UPDATE)),
	 [TASK_TELEMETRY] = DEFINE_TASK("TELEMETRY", telemetry_fun, NULL, TASK_PRIORITY_LOW, TASK_PERIOD_HZ(FREQUENCY_TELEMETRY_UPDATE))};

static void main_PID_fun(timeUs_t current_time)
{
	static timeUs_t last_time;
	static timeUs_t dt;
	dt = current_time - last_time;
	last_time = current_time;
	acro(dt);
}

static void stabilization_fun(timeUs_t current_time)
{
	static timeUs_t last_time;
	static timeUs_t dt;
	dt = current_time - last_time;
	last_time = current_time;
	stabilize(dt);
}

static bool stabilization_check_fun(timeUs_t current_time, timeUs_t delta_time)
{
	if (flight_mode == FLIGHT_MODE_STABLE)
	{
		return true;
	}
	return false;
}

static bool gyro_acc_filtering_check_fun(timeUs_t current_time, timeUs_t delta_time)
{
	return imu_received;
}

static void ibus_save_enable(timeUs_t current_time)
{
	//	When time comes (specified in task frequency) this function will allow for receiving RX.
	//	Until succeed .check_fun() will be executed on each scheduler iteration:
	all_tasks[TASK_IBUS_SAVE].check_fun = ibus_saving_fun;
}

static bool ibus_saving_fun(timeUs_t current_time, timeUs_t delta_time)
{
	//	This is the main function for ibus saving and will be executed until succeed:
	if (Ibus_save(current_time))
	{ // if succeed (ibus is received) turn off this function and let scheduler decide about next execution:
		all_tasks[TASK_IBUS_SAVE].check_fun = NULL;
	}
	//	always return false:
	return false;
}

static void telemetry_fun(timeUs_t time)
{
	telemetry_fun_tab[get_Flight_Mode()](time);
}

static void (*telemetry_fun_tab[FLIGHT_MODE_COUNT])(timeUs_t dt) =
	{
		[FLIGHT_MODE_STABLE] = send_telemetry_stabilize,
		[FLIGHT_MODE_ACRO] = send_telemetry_acro};
