/*
 * flash.c

 *
 *  Created on: 04.09.2021
 *      Author: symon
 */
#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include <math.h>
#include "flash.h"
static void CS_flash_disable();
static void CS_flash_enable();

void setup_FLASH(){

 CS_flash_disable();

}


static void CS_flash_enable(){
	GPIOB->BSRR |= GPIO_BSRR_BR3;
}

static void CS_flash_disable(){
	GPIOB->BSRR |= GPIO_BSRR_BS3;
}

