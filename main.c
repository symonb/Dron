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

//for debugging only:
static int pik = 0;
static int pek = 0;
uint8_t test1;
uint8_t test2;
int main(void) {

	setup();
	turn_OFF_BLUE_LED();
	setup_NVIC_1();
	delay_mili(1000);
	setup_MPU6000();
	setup_NVIC_2();

	static float time_flag0_1 = 0;
	static float time_flag0_2 = 0;
	static float time_flag0_3 = 0;

	while (1) {
		if ((get_Global_Time() - time_flag0_1) >= 30) {
					time_flag0_1 = get_Global_Time();
					pik = 0;
					pek = 0;
				}
				pek++;

				if ((get_Global_Time() - time_flag0_2) >= 1. / FREQUENCY_PID_LOOP) {
					time_flag0_2 = get_Global_Time();
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
				}
				else {
					delay_micro(10);

				}

				if ((get_Global_Time() - time_flag0_3) >= 1.f / FREQUENCY_ESC_UPDATE) {
					time_flag0_3 = get_Global_Time();
					update_motors();
					pik++;
				}
	}

}





