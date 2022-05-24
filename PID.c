/*
 * PID.c
 *
 *  Created on: 08.03.2022
 *      Author: symon
 */

#include <math.h>
#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"

#include "stabilize.h"
#include "acro.h"
#include "PID.h"

void (*modeFun[FLIGHT_MODE_COUNT])(timeUs_t dt);
flight_mode_e get_Flight_Mode();

void PID_fun(timeUs_t time) {
	modeFun[get_Flight_Mode()](time);
}

void (*modeFun[FLIGHT_MODE_COUNT])(timeUs_t dt)=
{
	[FLIGHT_MODE_STABLE] = stabilize,
	[FLIGHT_MODE_ACRO] = acro };

flight_mode_e get_Flight_Mode() {
	return flight_mode;
}
