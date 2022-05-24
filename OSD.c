/*
 * OSD.c
 *
 *  Created on: 04.09.2021
 *      Author: symon
 */

#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include <math.h>
#include "OSD.h"

static void CS_OSD_disable();
static void CS_OSD_enable();

void setup_OSD(){
	CS_OSD_disable();
}

static void CS_OSD_enable(){
	GPIOB->BSRR |= GPIO_BSRR_BR12;
}

static void CS_OSD_disable(){
	GPIOB->BSRR |= GPIO_BSRR_BS12;
}
