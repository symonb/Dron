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
#include "rx.h"
#include "scheduler.h"
#include "sensors/MPU6000.h"
#include "sensors/MS5XXX.h"
#include "sensors/barometer.h"
#include "battery.h"
#include "OSD.h"
#include "motors.h"
#include "usb.h"
#include "blackbox/blackbox.h"
#include "tasks.h"

void task_system_fun(timeUs_t current_time);
static void main_PID_fun(timeUs_t current_time);
static bool main_PID_check_fun(timeUs_t current_time, timeUs_t delta_time);
static void stabilization_fun(timeUs_t current_time);
static bool gyro_update_check_fun(timeUs_t current_time, timeUs_t delta_time);
static bool acc_update_check_fun(timeUs_t current_time, timeUs_t delta_time);
static void rx_save_fun(timeUs_t current_time);
static void telemetry_fun(timeUs_t current_time);
static void (*telemetry_fun_tab[FLIGHT_MODE_COUNT])(timeUs_t dt);
static void buzzer_fun(timeUs_t current_time);
static void OSD_update_fun(timeUs_t current_time);
static bool OSD_init_check_fun(timeUs_t current_time, timeUs_t delta_time);
static void gyro_calibration_fun(timeUs_t current_time);
static bool gyro_calibration_check_fun(timeUs_t current_time, timeUs_t delta_time);
static bool usb_check_fun(timeUs_t current_time, timeUs_t delta_time);
static bool blackbox_init_check_fun(timeUs_t current_time, timeUs_t delta_time);
#if defined(USE_BARO)
static bool baro_init_check_fun(timeUs_t current_time, timeUs_t delta_time);
static void baro_update(timeUs_t current_time);
static bool baro_update_check_fun(timeUs_t current_time, timeUs_t delta_time);
static bool alt_hold_check_fun(timeUs_t current_time, timeUs_t delta_time);
#endif



// ------DEFINE ALL TASKS--------
task_t all_tasks[TASKS_COUNT] =
{ [TASK_SYSTEM] = DEFINE_TASK("SYSTEM CHECK", task_system_fun, NULL, TASK_PRIORITY_LOW, TASK_PERIOD_HZ(FREQUENCY_SYSTEM_CHECK)),
 [TASK_IBUS_SAVE] = DEFINE_TASK("RX SAVING", rx_save_fun, NULL, TASK_PRIORITY_HIGH, TASK_PERIOD_HZ(FREQUENCY_MAIN_LOOP)),
 [TASK_GYRO_CALIBRATION] = DEFINE_TASK("GYRO CALIBRATION", gyro_calibration_fun, gyro_calibration_check_fun, TASK_PRIORITY_REALTIME, TASK_PERIOD_HZ(FREQUENCY_MAIN_LOOP)),
 [TASK_GYRO_UPDATE] = DEFINE_TASK("GYRO READ&FILTER", gyro_update, gyro_update_check_fun, TASK_PRIORITY_REALTIME, TASK_PERIOD_HZ(FREQUENCY_MAIN_LOOP)),
 [TASK_ACC_UPDATE] = DEFINE_TASK("ACC READ&FILTER", acc_update, acc_update_check_fun, TASK_PRIORITY_MEDIUM, TASK_PERIOD_HZ(FREQUENCY_ACC_SAMPLING)),
 [TASK_MAIN_LOOP] = DEFINE_TASK("MAIN LOOP", main_PID_fun, main_PID_check_fun, TASK_PRIORITY_REALTIME, TASK_PERIOD_HZ(FREQUENCY_MAIN_LOOP)),
 [TASK_STABILIZATION_LOOP] = DEFINE_TASK("STABILIZATION LOOP", stabilization_fun, NULL, TASK_PRIORITY_MEDIUM, TASK_PERIOD_HZ(FREQUENCY_STABILIZATION_LOOP)),
 [TASK_UPDATE_MOTORS] = DEFINE_TASK("UPDATE MOTORS", update_motors, NULL, TASK_PRIORITY_REALTIME, TASK_PERIOD_HZ(FREQUENCY_ESC_UPDATE)),
 [TASK_TELEMETRY] = DEFINE_TASK("TELEMETRY", telemetry_fun, NULL, TASK_PRIORITY_LOW, TASK_PERIOD_HZ(FREQUENCY_TELEMETRY_UPDATE)),
 [TASK_BUZZER] = DEFINE_TASK("BUZZER", buzzer_fun, NULL, TASK_PRIORITY_IDLE, TASK_PERIOD_HZ(10)),
 [TASK_OSD] = DEFINE_TASK("OSD UPDATE", OSD_update_fun, NULL, TASK_PRIORITY_LOW, TASK_PERIOD_HZ(FREQUENCY_OSD_UPDATE)),
 [TASK_OSD_INIT] = DEFINE_TASK("OSD INITIALIZATION", NULL, OSD_init_check_fun, TASK_PRIORITY_LOW, TASK_PERIOD_HZ(FREQUENCY_OSD_UPDATE)),
 [TASK_USB_HANDLING] = DEFINE_TASK("USB CONNECTION", usb_communication, usb_check_fun, TASK_PRIORITY_LOW, TASK_PERIOD_HZ(FREQUENCY_USB_CHECK)),
 [TASK_BLACKBOX] = DEFINE_TASK("BLACKBOX SAVE",blackbox_update ,NULL , TASK_PRIORITY_LOW, TASK_PERIOD_HZ(FREQUENCY_MAIN_LOOP)),
[TASK_BLACKBOX_INIT] = DEFINE_TASK("BLACKBOX INITIALIZATION", NULL, blackbox_init_check_fun, TASK_PRIORITY_LOW, TASK_PERIOD_HZ(FREQUENCY_MAIN_LOOP)),
#if defined(USE_BARO)
[TASK_BARO_INIT] = DEFINE_TASK("BARO INITIALIZATION", NULL, baro_init_check_fun, TASK_PRIORITY_LOW, TASK_PERIOD_HZ(FREQUENCY_BARO)),
[TASK_BARO] = DEFINE_TASK("BARO READ&FILTER", baro_update, NULL, TASK_PRIORITY_LOW, TASK_PERIOD_HZ(FREQUENCY_BARO)),
[TASK_ALT_HOLD] = DEFINE_TASK("ALTITUDE HOLD", alt_hold, alt_hold_check_fun,TASK_PRIORITY_LOW,TASK_PERIOD_HZ(FREQUENCY_ALT_HOLD)),
#endif
};


void task_system_fun(timeUs_t current_time)
{
	static timeUs_t last_time;
	main_scheduler.system_load = main_scheduler.system_load * 0.9f + 10 * (1 - (float)(main_scheduler.idle_time_counter) / (current_time - last_time));
	main_scheduler.idle_time_counter = 0;
	last_time = current_time;

	battery_manage();

	if (main_battery.status == BATTERY_NOT_CONNECTED) {
		remove_from_queue(&all_tasks[TASK_OSD], &main_scheduler);
		add_to_queue(&all_tasks[TASK_OSD_INIT], &main_scheduler);
#if defined(USE_BARO)		
		remove_from_queue(&all_tasks[TASK_BARO], &main_scheduler);
		add_to_queue(&all_tasks[TASK_BARO_INIT], &main_scheduler);
#endif
	}

}

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
	// update attitiude:
	att_update(dt);

	if (flight_mode == FLIGHT_MODE_STABLE || flight_mode == FLIGHT_MODE_ALT_HOLD) {
		// calculate corretions:
		stabilize(dt);
	}
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
	rx_handling(current_time);
}

static void telemetry_fun(timeUs_t time)
{
	telemetry_fun_tab[get_Flight_Mode()](time);
}

static void (*telemetry_fun_tab[FLIGHT_MODE_COUNT])(timeUs_t dt) =
{ [FLIGHT_MODE_ALT_HOLD] = send_telemetry_stabilize,
	[FLIGHT_MODE_STABLE] = send_telemetry_stabilize,
	[FLIGHT_MODE_ACRO] = send_telemetry_acro };

static void buzzer_fun(timeUs_t time)
{
	if (main_battery.status == BATTERY_LOW)
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
	if (main_OSD.calibrated)
	{
		OSD_print_battery_voltage();
		OSD_print_battery_cell_voltage();
		OSD_print_time();
		OSD_print_flight_mode();
		OSD_print_warnings();
	}
}
static bool OSD_init_check_fun(timeUs_t current_time, timeUs_t delta_time) {
	if (main_battery.status != BATTERY_NOT_CONNECTED) {
		if (OSD_init()) {
			remove_from_queue(&all_tasks[TASK_OSD_INIT], &main_scheduler);
			add_to_queue(&all_tasks[TASK_OSD], &main_scheduler);
		}
	}
	return false;
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

static bool usb_check_fun(timeUs_t current_time, timeUs_t delta_time) {
	// check if USB is connected:
	return main_usb.connected;
}

static bool blackbox_init_check_fun(timeUs_t current_time, timeUs_t delta_time) {
	// if blackbox is initialized remove this task from scheduler:
	if (blackbox_init()) {
		remove_from_queue(&all_tasks[TASK_BLACKBOX_INIT], &main_scheduler);
		// reset tasks stats:
		scheduler_reset_tasks_statistics(&main_scheduler);

	}
	else {

		FailSafe_status = FAILSAFE_BLACKBOX_FULL;
		EXTI->SWIER |= EXTI_SWIER_SWIER15;
		if (get_Global_Time() > 10000000) {
			remove_from_queue(&all_tasks[TASK_BLACKBOX_INIT], &main_scheduler);
		}
	}

	return false;
}

#if defined(USE_BARO)
static bool baro_init_check_fun(timeUs_t current_time, timeUs_t delta_time) {
	if (main_battery.status != BATTERY_NOT_CONNECTED) {
		if (MS5XXX_init(&baro_1)) {
			remove_from_queue(&all_tasks[TASK_BARO_INIT], &main_scheduler);
			add_to_queue(&all_tasks[TASK_BARO], &main_scheduler);
		}
	}
	return false;
}

static void baro_update(timeUs_t current_time)
{
	// process the last data read from the sensor (filtering):
	baro_calculate_altitude(&baro_1, current_time);
	// this function only starts process of reading barometer sensor main part is in check_fun():
	// initialize pressure conversion:
	MS5XXX_start_conversion_preasure(MS5XXX_CMD_ADC_4096);
	// set .checkFun (check baro_update_check_fun() for further explanation):
	all_tasks[TASK_BARO].check_fun = baro_update_check_fun;
}

/*  This function is the main function of reading new data from depth sensor.
	 It is triggered according to the set frequency (TASK_READ_DEPTH_SENSOR_HZ).
	 Next, after CONVERSION_TIME raw pressure is read and temp conversion started.
	 Similarly after CONVERSION_TIME raw temperature is read.
	 At the end new values of temperature and pressure are calculated and updated.
	 Also at the end .checkFun is set to NULL to disabling this function
	 (it will be enable in .taskFun according to the scheduler).
 */
static bool baro_update_check_fun(timeUs_t current_time, timeUs_t delta_time)
{
	static timeUs_t time_flag;
	static uint16_t counter;
	enum state
	{
		PRESSURE_CONVERSION,
		TEMP_CONVERSION_SEND,
		TEMP_CONVERSION_WAIT,
		UPDATE_MEASUREMENTS
	};

	static enum state current_state = PRESSURE_CONVERSION;

	switch (current_state)
	{
	case PRESSURE_CONVERSION:
		// after initialization in .taskFun wait for end of conversion:
		if (delta_time > MS5XXX_CONVERSION_TIME * 16)
		{
			// read pressure after conversion:
			MS5XXX_read_preasure(&baro_1);

			time_flag = current_time;
			// there is no need for reading temperature each time (it doesn't chage so much)
			if (counter++ < 1) {
				current_state = UPDATE_MEASUREMENTS;
			}
			else {
				current_state = TEMP_CONVERSION_SEND;
				counter = 0;
			}
		}
		break;

	case TEMP_CONVERSION_SEND:
		// without quick break before sending TEMP conversion command sensor likes to freeze 
		if (current_time - time_flag > 100) {
			// start TEMP conversion:
			MS5XXX_start_conversion_temp(MS5XXX_CMD_ADC_4096);
			current_state = TEMP_CONVERSION_WAIT;
			time_flag = current_time;
		}
		break;
	case TEMP_CONVERSION_WAIT:
		//  wait for end of conversion:
		if (current_time - time_flag > MS5XXX_CONVERSION_TIME * 16)
		{
			// read temperature after conversion:
			MS5XXX_read_temp(&baro_1);
			current_state = UPDATE_MEASUREMENTS;

		}
		break;
	case UPDATE_MEASUREMENTS:

		// update variables:
		MS5XXX_calculate_preasure(&baro_1);
		current_state = PRESSURE_CONVERSION;
		// disable this function (now, scheduler will decide about next measurements reading):
		all_tasks[TASK_BARO].check_fun = NULL;
		break;
	}
	// always return false:
	return false;
}

static bool alt_hold_check_fun(timeUs_t current_time, timeUs_t delta_time) {
	return (flight_mode == FLIGHT_MODE_ALT_HOLD);
}

#endif

