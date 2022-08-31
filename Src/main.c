/**
 ******************************************************************************
 * @file    main.c
 * @author  Ac6
 * @version V1.0
 * @date    01-December-2013
 * @brief   Default main function.
 ******************************************************************************
 */

#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "setup.h"
#include "stabilize.h"
#include "acro.h"
#include "ibus.h"
#include "connection.h"
#include "MPU6000.h"
#include "flash.h"
#include "tasks.h"
#include "i2c1.h"
#include "scheduler.h"

int main(void)
{

	setup();
	turn_OFF_BLUE_LED();
	setup_NVIC_1();
	delay_mili(1000);
	setup_MPU6000();
	setup_NVIC_2();
	delay_mili(500);
	flash_init();
	scheduler_initialization(&main_scheduler);

	time_flag0_2 = get_Global_Time();

	while (1)
	{

		static timeUs_t time_flag_debug;
		static uint16_t tik;
		static timeUs_t dt_test;

		if (get_Global_Time() - time_flag_debug >= SEC_TO_US(10))
		{

			time_flag_debug = get_Global_Time();
			tik = 0;
			dt_test = 0;
			debug_variable_1 = 0;
		}
		scheduler_execute(&main_scheduler);

		// if ((get_Global_Time() - time_flag0_2) >= SEC_TO_US(1. / FREQUENCY_PID_LOOP))
		// {
		// 	dt_global = get_Global_Time() - time_flag0_2;

		// 	time_flag0_2 += dt_global;

		// 	if (failsafe_PID_loop(&dt_global))
		// 	{
		// 		dt_test += 1;
		// 	}

		// 	tik++;

		// 	Ibus_save(dt_global);
		// 	rewrite_Gyro_Acc_data(dt_global);

		// 	PID_fun(dt_global);

		// 	if (0 != transmitting_is_Done && 0 != New_data_to_send)
		// 	{
		// 		// Transmit data
		// 		print(table_to_send, ALL_ELEMENTS_TO_SEND);
		// 	}
		// }
		// else
		// {
		// 	delay_micro(10);
		// }

		// if ((get_Global_Time() - time_flag0_3) >= SEC_TO_US(1.f / FREQUENCY_ESC_UPDATE))
		// {
		// 	time_flag0_3 = get_Global_Time();
		// 	update_motors(dt_global);
		// }

		if (USB_detected || blackbox_command == 2)
		{
			turn_OFF_RED_LED();
			turn_OFF_BLUE_LED();
			delay_mili(3000);
			turn_ON_RED_LED();
			delay_mili(1000);
			turn_ON_BLUE_LED();
			delay_mili(1000);
			turn_OFF_RED_LED();
			turn_OFF_BLUE_LED();
			delay_mili(1000);
			turn_ON_RED_LED();
			turn_ON_BLUE_LED();
			delay_mili(1000);
			turn_OFF_RED_LED();
			turn_OFF_BLUE_LED();
			delay_mili(1000);
			turn_ON_RED_LED();
			turn_ON_BLUE_LED();
			delay_mili(2000);

			print_flash(9);
		}
	}
}
