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

static void task_system_fun(timeUs_t current_time, timeUs_t dt_us);
static void task_main_loop_fun(timeUs_t current_time, timeUs_t dt_us);
static bool task_main_loop_check_fun(timeUs_t current_time, timeUs_t dt_us);
static void task_gyro_calibration_fun(timeUs_t current_time, timeUs_t dt_us);
static bool task_gyro_calibration_check_fun(timeUs_t current_time, timeUs_t dt_us);
static void task_gyro_update_fun(timeUs_t current_time, timeUs_t dt_us);
static bool task_gyro_update_check_fun(timeUs_t current_time, timeUs_t dt_us);
static void task_acc_update_fun(timeUs_t current_time, timeUs_t dt_us);
static bool task_acc_update_check_fun(timeUs_t current_time, timeUs_t dt_us);
static void task_rx_save_fun(timeUs_t current_time, timeUs_t dt_us);
static void task_stabilization_fun(timeUs_t current_time, timeUs_t dt_us);
static void task_update_motors_fun(timeUs_t current_time, timeUs_t dt_us);
static void task_telemetry_fun(timeUs_t current_time, timeUs_t dt_us);
static void (*telemetry_fun_tab[FLIGHT_MODE_COUNT])(timeUs_t time);
static void task_buzzer_fun(timeUs_t current_time, timeUs_t dt_us);
static void task_OSD_update_fun(timeUs_t current_time, timeUs_t dt_us);
static bool task_OSD_update_check_fun(timeUs_t current_time, timeUs_t dt_us);
static bool task_OSD_init_check_fun(timeUs_t current_time, timeUs_t dt_us);
static void task_usb_communication_fun(timeUs_t current_time, timeUs_t dt_us);
static void task_blackbox_update_fun(timeUs_t current_time, timeUs_t dt_us);
static bool task_blackbox_init_check_fun(timeUs_t current_time, timeUs_t dt_us);
#if defined(USE_BARO)
static bool task_baro_init_check_fun(timeUs_t current_time, timeUs_t dt_us);
static void task_baro_update_fun(timeUs_t current_time, timeUs_t dt_us);
static bool task_baro_update_check_fun(timeUs_t current_time, timeUs_t dt_us);
static void task_alt_hold_fun(timeUs_t current_time, timeUs_t dt_us);
static bool task_alt_hold_check_fun(timeUs_t current_time, timeUs_t dt_us);
#endif



// ------DEFINE ALL TASKS--------
task_t all_tasks[TASKS_COUNT] =
{ [TASK_SYSTEM] = DEFINE_TASK("SYSTEM CHECK", task_system_fun, NULL, TASK_PRIORITY_LOW, TASK_PERIOD_HZ(FREQUENCY_SYSTEM_CHECK)),
  [TASK_MAIN_LOOP] = DEFINE_TASK("MAIN LOOP", task_main_loop_fun, task_main_loop_check_fun, TASK_PRIORITY_REALTIME, TASK_PERIOD_HZ(FREQUENCY_MAIN_LOOP)),
  [TASK_RX_HANDLING] = DEFINE_TASK("RX SAVING", task_rx_save_fun, NULL, TASK_PRIORITY_HIGH, TASK_PERIOD_HZ(FREQUENCY_MAIN_LOOP)),
  [TASK_GYRO_CALIBRATION] = DEFINE_TASK("GYRO CALIBRATION", task_gyro_calibration_fun, task_gyro_calibration_check_fun, TASK_PRIORITY_REALTIME, TASK_PERIOD_HZ(FREQUENCY_MAIN_LOOP)),
  [TASK_GYRO_UPDATE] = DEFINE_TASK("GYRO READ&FILTER", task_gyro_update_fun, task_gyro_update_check_fun, TASK_PRIORITY_REALTIME, TASK_PERIOD_HZ(FREQUENCY_MAIN_LOOP)),
  [TASK_ACC_UPDATE] = DEFINE_TASK("ACC READ&FILTER",task_acc_update_fun, task_acc_update_check_fun, TASK_PRIORITY_MEDIUM, TASK_PERIOD_HZ(FREQUENCY_ACC_SAMPLING)),
  [TASK_STABILIZATION_LOOP] = DEFINE_TASK("STABILIZATION LOOP", task_stabilization_fun, NULL, TASK_PRIORITY_MEDIUM, TASK_PERIOD_HZ(FREQUENCY_STABILIZATION_LOOP)),
  [TASK_UPDATE_MOTORS] = DEFINE_TASK("UPDATE MOTORS", task_update_motors_fun, NULL, TASK_PRIORITY_REALTIME, TASK_PERIOD_HZ(FREQUENCY_ESC_UPDATE)),
  [TASK_TELEMETRY] = DEFINE_TASK("TELEMETRY", task_telemetry_fun, NULL, TASK_PRIORITY_LOW, TASK_PERIOD_HZ(FREQUENCY_TELEMETRY_UPDATE)),
  [TASK_BUZZER] = DEFINE_TASK("BUZZER", task_buzzer_fun, NULL, TASK_PRIORITY_IDLE, TASK_PERIOD_HZ(10)),
  [TASK_OSD] = DEFINE_TASK("OSD UPDATE", task_OSD_update_fun, NULL, TASK_PRIORITY_LOW, TASK_PERIOD_HZ(FREQUENCY_OSD_UPDATE)),
  [TASK_OSD_INIT] = DEFINE_TASK("OSD INITIALIZATION", NULL, task_OSD_init_check_fun, TASK_PRIORITY_LOW, TASK_PERIOD_HZ(FREQUENCY_OSD_UPDATE)),
  [TASK_USB_HANDLING] = DEFINE_TASK("USB CONNECTION", task_usb_communication_fun, NULL, TASK_PRIORITY_LOW, TASK_PERIOD_HZ(FREQUENCY_USB_CHECK)),
  [TASK_BLACKBOX] = DEFINE_TASK("BLACKBOX SAVE", task_blackbox_update_fun ,NULL , TASK_PRIORITY_HIGH, TASK_PERIOD_HZ(FREQUENCY_MAIN_LOOP)),
  [TASK_BLACKBOX_INIT] = DEFINE_TASK("BLACKBOX INITIALIZATION", NULL, task_blackbox_init_check_fun, TASK_PRIORITY_LOW, TASK_PERIOD_HZ(FREQUENCY_MAIN_LOOP)),
#if defined(USE_BARO)
[TASK_BARO_INIT] = DEFINE_TASK("BARO INITIALIZATION", NULL, task_baro_init_check_fun, TASK_PRIORITY_LOW, TASK_PERIOD_HZ(FREQUENCY_BARO)),
[TASK_BARO] = DEFINE_TASK("BARO READ&FILTER", task_baro_update_fun, NULL, TASK_PRIORITY_LOW, TASK_PERIOD_HZ(FREQUENCY_BARO)),
[TASK_ALT_HOLD] = DEFINE_TASK("ALTITUDE HOLD", task_alt_hold_fun, task_alt_hold_check_fun,TASK_PRIORITY_LOW,TASK_PERIOD_HZ(FREQUENCY_ALT_HOLD)),
#endif
};

static void task_system_fun(timeUs_t current_time, timeUs_t dt_us)
{
	main_scheduler.system_load = main_scheduler.system_load * 0.9f + 10 * (1 - (float)(main_scheduler.idle_time_counter) / (dt_us));
	main_scheduler.idle_time_counter = 0;

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

static void task_rx_save_fun(timeUs_t current_time, timeUs_t dt_us)
{
	rx_handling(current_time);
}

static void task_gyro_calibration_fun(timeUs_t current_time, timeUs_t dt_us) {
	gyro_calibration(&gyro_1, current_time);
}

static bool task_gyro_calibration_check_fun(timeUs_t current_time, timeUs_t dt_us) {
	// if gyro is calibrated remove this task from scheduler:
	if (gyro_1.calibrated) {
		remove_from_queue(&all_tasks[TASK_GYRO_CALIBRATION], &main_scheduler);
		// reset tasks stats:
		scheduler_reset_tasks_statistics(&main_scheduler);
		return false;
	}

	return true;
}

static void task_gyro_update_fun(timeUs_t current_time, timeUs_t dt_us)
{
	gyro_update(&gyro_1);
}

static bool task_gyro_update_check_fun(timeUs_t current_time, timeUs_t dt_us)
{
	return gyro_1.new_raw_data_flag;
}

static void task_acc_update_fun(timeUs_t current_time, timeUs_t dt_us)
{
	acc_update(&acc_1);
}

static bool task_acc_update_check_fun(timeUs_t current_time, timeUs_t dt_us)
{
	return acc_1.new_raw_data_flag;
}

static void task_update_motors_fun(timeUs_t current_time, timeUs_t dt_us) {
	update_motors();
}

static void task_main_loop_fun(timeUs_t current_time, timeUs_t dt_us)
{
	// prevent too big and too small dt:
	if (dt_us > 1.5f * TASK_PERIOD_HZ(FREQUENCY_MAIN_LOOP)) {
		dt_us = 1.5f * TASK_PERIOD_HZ(FREQUENCY_MAIN_LOOP);
	}
	else if (dt_us < 0.5f * TASK_PERIOD_HZ(FREQUENCY_MAIN_LOOP)) {
		dt_us = 0.5f * TASK_PERIOD_HZ(FREQUENCY_MAIN_LOOP);
	}
	acro(dt_us);
	gyro_1.new_filtered_data = false;
}

static bool task_main_loop_check_fun(timeUs_t current_time, timeUs_t dt_us)
{
	return gyro_1.new_filtered_data;
}

static void task_stabilization_fun(timeUs_t current_time, timeUs_t dt_us)
{
	//	prevent too big and too small dt:
	if (dt_us > 1.5f * TASK_PERIOD_HZ(FREQUENCY_STABILIZATION_LOOP))
	{
		dt_us = 1.5f * TASK_PERIOD_HZ(FREQUENCY_STABILIZATION_LOOP);
	}
	else if (dt_us < 0.5f * TASK_PERIOD_HZ(FREQUENCY_STABILIZATION_LOOP)) {
		dt_us = 0.5f * TASK_PERIOD_HZ(FREQUENCY_STABILIZATION_LOOP);
	}
	// update attitiude:
	att_update(dt_us);

	if (flight_mode == FLIGHT_MODE_STABLE || flight_mode == FLIGHT_MODE_ALT_HOLD) {
		// calculate corretions:
		stabilize(dt_us);
	}
}

static void task_telemetry_fun(timeUs_t current_time, timeUs_t dt_us)
{
	telemetry_fun_tab[get_Flight_Mode()](current_time);
}

static void task_buzzer_fun(timeUs_t current_time, timeUs_t dt_us)
{
	if (main_battery.status == BATTERY_LOW)
	{
		static timeUs_t previous_time;
		if (current_time - previous_time >= BUZZER_TIME_ON)
		{
			turn_off_BUZZER();

			if (current_time - previous_time >= (BUZZER_TIME_OFF + BUZZER_TIME_ON))
			{
				turn_on_BUZZER();
				previous_time = current_time;
			}
		}
	}
	else
	{
		turn_off_BUZZER();
	}
}

static void task_OSD_update_fun(timeUs_t current_time, timeUs_t dt_us)
{
	OSD_update_screen(&main_OSD, current_time);
	if (!OSD_draw_screen(&main_OSD, current_time)) {
		all_tasks[TASK_OSD].check_fun = task_OSD_update_check_fun;	// if screen not updated after 1 iteration add check_fun (it will be deleted after screen update):
	}

}

static bool task_OSD_update_check_fun(timeUs_t current_time, timeUs_t dt_us) {
	if (OSD_draw_screen(&main_OSD, current_time)) {
		// if updating is done wait for next update (scheduler will take care of it)
		all_tasks[TASK_OSD].check_fun = NULL;
	}
	// always return false:
	return false;
}

static bool task_OSD_init_check_fun(timeUs_t current_time, timeUs_t dt_us) {
	if (main_battery.status != BATTERY_NOT_CONNECTED) {
		if (OSD_init(&main_OSD, current_time)) {
			remove_from_queue(&all_tasks[TASK_OSD_INIT], &main_scheduler);
			add_to_queue(&all_tasks[TASK_OSD], &main_scheduler);
			scheduler_reset_tasks_statistics(&main_scheduler);
		}
	}
	return false;
}

static void task_usb_communication_fun(timeUs_t current_time, timeUs_t dt_us) {
	usb_communication();
}

static void task_blackbox_update_fun(timeUs_t current_time, timeUs_t dt_us) {
	blackbox_update(current_time);
}

static bool task_blackbox_init_check_fun(timeUs_t current_time, timeUs_t dt_us) {
	// if blackbox is initialized remove this task from scheduler:
	if (blackbox_init()) {
		remove_from_queue(&all_tasks[TASK_BLACKBOX_INIT], &main_scheduler);
		// reset tasks stats:
		scheduler_reset_tasks_statistics(&main_scheduler);

	}
	else {

		FailSafe_status = FAILSAFE_BLACKBOX_FULL;
		EXTI->SWIER |= EXTI_SWIER_SWIER15;
		if (current_time > 10000000) {
			remove_from_queue(&all_tasks[TASK_BLACKBOX_INIT], &main_scheduler);
		}
	}

	return false;
}

#if defined(USE_BARO)
static bool task_baro_init_check_fun(timeUs_t current_time, timeUs_t dt_us) {
	if (main_battery.status != BATTERY_NOT_CONNECTED) {
		if (MS5XXX_init(&baro_1)) {
			remove_from_queue(&all_tasks[TASK_BARO_INIT], &main_scheduler);
			add_to_queue(&all_tasks[TASK_BARO], &main_scheduler);
		}
	}
	return false;
}

static void task_baro_update_fun(timeUs_t current_time, timeUs_t dt_us)
{
	// process the last data read from the sensor (filtering):
	baro_calculate_altitude(&baro_1, current_time);
	baro_kalman_fusion(&baro_1, current_time);
	// this function only starts process of reading barometer sensor main part is in check_fun():
	// initialize pressure conversion:
	MS5XXX_start_conversion_preasure(MS5XXX_CMD_ADC_4096);
	// set .checkFun (check baro_update_check_fun() for further explanation):
	all_tasks[TASK_BARO].check_fun = task_baro_update_check_fun;
}

/*  This function is the main function of reading new data from depth sensor.
	 It is triggered according to the set frequency (TASK_READ_DEPTH_SENSOR_HZ).
	 Next, after CONVERSION_TIME raw pressure is read and temp conversion started.
	 Similarly after CONVERSION_TIME raw temperature is read.
	 At the end new values of temperature and pressure are calculated and updated.
	 Also at the end .checkFun is set to NULL to disabling this function
	 (it will be enable in .taskFun according to the scheduler).
 */
static bool task_baro_update_check_fun(timeUs_t current_time, timeUs_t dt_us)
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
		if (dt_us > MS5XXX_CONVERSION_TIME * 16)
		{
			// read pressure after conversion:
			MS5XXX_read_preasure(&baro_1);

			time_flag = current_time;
			// there is no need for reading temperature each time (it doesn't chage so much)
			if (counter++ < FREQUENCY_BARO / 2) {
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

static void task_alt_hold_fun(timeUs_t current_time, timeUs_t dt_us) {
	// prevent too big and too small dt:
	if (dt_us > 1.5f * TASK_PERIOD_HZ(FREQUENCY_ALT_HOLD)) {
		dt_us = 1.5f * TASK_PERIOD_HZ(FREQUENCY_ALT_HOLD);
	}
	else if (dt_us < 0.5f * TASK_PERIOD_HZ(FREQUENCY_ALT_HOLD)) {
		dt_us = 0.5f * TASK_PERIOD_HZ(FREQUENCY_ALT_HOLD);
	}
	alt_hold(dt_us);
}

static bool task_alt_hold_check_fun(timeUs_t current_time, timeUs_t dt_us) {
	return (flight_mode == FLIGHT_MODE_ALT_HOLD);
}

#endif

static void (*telemetry_fun_tab[FLIGHT_MODE_COUNT])(timeUs_t current_time) =
{ [FLIGHT_MODE_ALT_HOLD] = send_telemetry_stabilize,
	[FLIGHT_MODE_STABLE] = send_telemetry_stabilize,
	[FLIGHT_MODE_ACRO] = send_telemetry_acro };

