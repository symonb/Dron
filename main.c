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


void update_motors();

//for debugging only:
static int pik = 0;
static int pek = 0;
uint8_t test1;
uint8_t test2;
int main(void) {

	setup();
	setup_MPU6000();
	setup_NVIC();

	static float time_flag0_1 = 0;
	static float time_flag0_2 = 0;
	static float time_flag0_3 = 0;

	static float time_flag1_2 = 0;

	uint8_t Who_I_am=0;
	SPI_enable();
	MPU6000_SPI_read(0x75,&Who_I_am,1);
	SPI_disable();

	while (1) {
		if ((get_Global_Time() - time_flag0_1) >= 10) {
					time_flag0_1 = get_Global_Time();
					pik = 0;
					pek = 0;
				}
				pek++;

				if ((get_Global_Time() - time_flag0_2) >= 1. / FREQUENCY_PID_LOOP) {
					time_flag0_2 = get_Global_Time();
					Ibus_save();
					rewrite_data();
					pik++;

					if ((get_Global_Time() - time_flag1_2) >= 1. / FREQUENCY_TELEMETRY_UPDATE) {
								time_flag1_2 = get_Global_Time();

					if (channels[6] < 1400) {
						acro();


					}
						turn_ON_LED();
					}
					else if (channels[6] > 1450) {
						stabilize();
						turn_OFF_LED();
					}

					if (0 != transmitting_is_Done && 0 != New_data_to_send) {
						// Transmit data
						print(table_to_send, ALL_ELEMENTS_TO_SEND);
					}
				}
				else {
					delay_mili(1);
				}

				if ((get_Global_Time() - time_flag0_3) >= 1. / FREQUENCY_ESC_UPDATE) {
					time_flag0_3 = get_Global_Time();
					update_motors();

				}


	}

}


void update_motors() {
	TIM3->CCR3 = *PWM_M1 - 1; 			//wypelneinie motor 1
	TIM3->CCR4 = *PWM_M2 - 1; 			//wypelneinie motor 2
	TIM2->CCR4 = *PWM_M3 - 1; 			//wypelneinie motor 3
	TIM2->CCR3 = *PWM_M4 - 1; 			//wypelneinie motor 4
}



