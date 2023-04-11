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
#include "battery.h"
#include "OSD.h"
#include "motors.h"
#include "tasks.h"

static void main_PID_fun(timeUs_t current_time);
static bool main_PID_check_fun(timeUs_t current_time, timeUs_t delta_time);
static void stabilization_fun(timeUs_t current_time);
static bool stabilization_check_fun(timeUs_t current_time, timeUs_t delta_time);
static bool gyro_update_check_fun(timeUs_t current_time, timeUs_t delta_time);
static bool acc_update_check_fun(timeUs_t current_time, timeUs_t delta_time);
static void rx_save_fun(timeUs_t current_time);
static bool rx_save_check_fun(timeUs_t current_time, timeUs_t delta_time);
static void telemetry_fun(timeUs_t time);
static void (*telemetry_fun_tab[FLIGHT_MODE_COUNT])(timeUs_t dt);
static void buzzer_fun(timeUs_t time);
static void OSD_update_fun(timeUs_t time);
static void gyro_calibration_fun(timeUs_t time);
static bool gyro_calibration_check_fun(timeUs_t current_time, timeUs_t delta_time);

// ------DEFINE ALL TASKS--------
task_t all_tasks[TASKS_COUNT] =
{ [TASK_SYSTEM] = DEFINE_TASK("SYSTEM CHECK", task_system_fun, NULL, TASK_PRIORITY_LOW, TASK_PERIOD_HZ(FREQUENCY_SYSTEM_CHECK)),
 [TASK_IBUS_SAVE] = DEFINE_TASK("IBUS SAVING", rx_save_fun, rx_save_check_fun, TASK_PRIORITY_REALTIME, TASK_PERIOD_HZ(FREQUENCY_RX_READING)),
 [TASK_GYRO_UPDATE] = DEFINE_TASK("GYRO READ&FILTER", gyro_update, gyro_update_check_fun, TASK_PRIORITY_REALTIME, TASK_PERIOD_HZ(FREQUENCY_MAIN_LOOP)),
 [TASK_ACC_UPDATE] = DEFINE_TASK("ACC READ&FILTER", acc_update, acc_update_check_fun, TASK_PRIORITY_REALTIME, TASK_PERIOD_HZ(FREQUENCY_STABILIZATION_LOOP)),
 [TASK_MAIN_LOOP] = DEFINE_TASK("MAIN LOOP", main_PID_fun, main_PID_check_fun, TASK_PRIORITY_REALTIME, TASK_PERIOD_HZ(FREQUENCY_MAIN_LOOP)),
 [TASK_STABILIZATION_LOOP] = DEFINE_TASK("STABILIZATION LOOP", stabilization_fun, stabilization_check_fun, TASK_PRIORITY_REALTIME, TASK_PERIOD_HZ(FREQUENCY_STABILIZATION_LOOP)),
 [TASK_UPDATE_MOTORS] = DEFINE_TASK("UPDATE MOTORS", update_motors, NULL, TASK_PRIORITY_REALTIME, TASK_PERIOD_HZ(FREQUENCY_ESC_UPDATE)),
 [TASK_TELEMETRY] = DEFINE_TASK("TELEMETRY", telemetry_fun, NULL, TASK_PRIORITY_LOW, TASK_PERIOD_HZ(FREQUENCY_TELEMETRY_UPDATE)),
 [TASK_BUZZER] = DEFINE_TASK("BUZZER", buzzer_fun, NULL, TASK_PRIORITY_IDLE, TASK_PERIOD_HZ(10)),
 [TASK_OSD] = DEFINE_TASK("OSD UPDATE", OSD_update_fun, NULL, TASK_PRIORITY_LOW, TASK_PERIOD_HZ(FREQUENCY_OSD_UPDATE)),
 [TASK_GYRO_CALIBRATION] = DEFINE_TASK("GYRO CALIBRATION", gyro_calibration_fun, gyro_calibration_check_fun, TASK_PRIORITY_REALTIME, TASK_PERIOD_HZ(FREQUENCY_MAIN_LOOP)) };

static void main_PID_fun(timeUs_t current_time)
{
	static timeUs_t last_time;
	timeUs_t dt = current_time - last_time;
	last_time = current_time;
	acro(dt);
	gyro_1.new_filtered_data = false;
}

static bool main_PID_check_fun(timeUs_t current_time, timeUs_t delta_time)
{
	return gyro_1.new_filtered_data;
}

static void stabilization_fun(timeUs_t current_time)
{
	static timeUs_t last_time;
	timeUs_t dt = current_time - last_time;
	//	prevent too big dt (e.g first iteration after being inactive):
	if (dt > 1.5f * TASK_PERIOD_HZ(FREQUENCY_STABILIZATION_LOOP))
	{
		dt = TASK_PERIOD_HZ(FREQUENCY_STABILIZATION_LOOP);
	}
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

static bool gyro_update_check_fun(timeUs_t current_time, timeUs_t delta_time)
{

	return gyro_1.new_raw_data_flag;
}

static bool acc_update_check_fun(timeUs_t current_time, timeUs_t delta_time)
{
	return acc_1.new_raw_data_flag;
}

static void rx_save_fun(timeUs_t current_time)
{
	Ibus_save(current_time);
}

static bool rx_save_check_fun(timeUs_t current_time, timeUs_t delta_time)
{
	return receiver.new_data_flag;
}

static void telemetry_fun(timeUs_t time)
{
	telemetry_fun_tab[get_Flight_Mode()](time);
}

static void (*telemetry_fun_tab[FLIGHT_MODE_COUNT])(timeUs_t dt) =
{
	[FLIGHT_MODE_STABLE] = send_telemetry_stabilize,
	[FLIGHT_MODE_ACRO] = send_telemetry_acro };

static void buzzer_fun(timeUs_t time)
{
	if (main_battery.BATTERY_STATUS == BATTERY_LOW)
	{
		static timeUs_t previous_time;
		if (time - previous_time >= BUZZER_TIME_ON)
		{
			turn_off_BUZZER();

			if (time - previous_time >= (BUZZER_TIME_OFF + BUZZER_TIME_ON))
			{
				turn_on_BUZZER();
				previous_time = time;
			}
		}
	}
	else
	{
		turn_off_BUZZER();
	}
}

static void OSD_update_fun(timeUs_t time)
{ //	OSD chip works only when main power is connected:
	if (main_battery.BATTERY_STATUS != BATTERY_NOT_CONNECTED)
	{
		OSD_print_battery_voltage();
		OSD_print_battery_cell_voltage();
		OSD_print_time();
		OSD_print_flight_mode();
		OSD_print_warnings();
	}
}

static void gyro_calibration_fun(timeUs_t time) {
	gyro_calibration(&gyro_1, time);
}

static bool gyro_calibration_check_fun(timeUs_t current_time, timeUs_t delta_time) {
	// if gyro is calibrated remove this task from scheduler:
	if (gyro_1.calibrated) {
		remove_from_queue(&all_tasks[TASK_GYRO_CALIBRATION], &main_scheduler);
		// reset tasks stats:
		scheduler_reset_tasks_statistics(&main_scheduler);
		return false;
	}

	return true;
}