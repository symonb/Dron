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
#include "adc1.h"

#include "OSD.h"

void main(void)
{
	setup();
	turn_OFF_BLUE_LED();
	setup_NVIC_1();
	delay_mili(100);
	setup_MPU6000();
	setup_OSD();
	setup_NVIC_2();
	delay_mili(100);
	scheduler_initialization(&main_scheduler);

	while (1)
	{
		scheduler_execute(&main_scheduler);
	}
}
