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


int main(void) {

	setup();
	turn_OFF_BLUE_LED();
	setup_NVIC_1();
	delay_mili(1000);
	setup_MPU6000();
	setup_NVIC_2();

	while (flash_read_status_register(FLASH_READ_STATUS_REGISTER_1) & 0x01) {
			delay_mili(500); //wait for erasing flash
		}
	flash_full_chip_erase();
	while (flash_read_status_register(FLASH_READ_STATUS_REGISTER_1) & 0x01) {
		delay_mili(500); //wait for erasing flash
	}
	time_flag0_2=get_Global_Time();

	while (1) {

		if ((get_Global_Time() - time_flag0_2) >= 1. / FREQUENCY_PID_LOOP) {
			dt=get_Global_Time()-time_flag0_2;
			time_flag0_2 +=dt;

			if (dt>2./FREQUENCY_PID_LOOP){
				dt=2./FREQUENCY_PID_LOOP;
				failsafe_type = 8;
				EXTI->SWIER |= EXTI_SWIER_SWIER15;
			}

			Ibus_save();
			rewrite_data();

			if (channels[6] < 1400) {
				acro();
				turn_OFF_RED_LED();
				turn_ON_BLUE_LED();
			}

			else if (channels[6] > 1450) {
				stabilize();
				turn_OFF_BLUE_LED();
				turn_ON_RED_LED();
			}

			if (0 != transmitting_is_Done && 0 != New_data_to_send) {
				// Transmit data
				print(table_to_send, ALL_ELEMENTS_TO_SEND);
			}
		} else {
			delay_micro(10);

		}

		if ((get_Global_Time() - time_flag0_3) >= 1.f / FREQUENCY_ESC_UPDATE) {
			time_flag0_3 = get_Global_Time();
			update_motors();
		}

		if (USB_detected || blackbox_command == 2) {
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

