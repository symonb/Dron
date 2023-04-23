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
#include "string.h"
#include "battery.h"
#include "drivers/SPI2.h"

#include "OSD.h"

 //	digits and letters needs to match those in ASCII table ('0' in 48th position)
const char global_characters_tab[OSD_MAX_CHARACTER_NUMBER] = "################################# !###%%##()*+,-./0123456789:;<=> @ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_";
// tab with sequence for logo display, by default it is set as last 96 characters:
const uint8_t OSD_LOGO_characters_tab[96][54] = { {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 64, 85, 85, 0, 1, 85, 0, 1, 84, 0, 1, 84, 0, 1}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 64, 85, 80, 0, 84, 0, 0}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 64, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 168}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 5, 85, 85, 0, 21, 85, 0, 0, 85, 0, 0, 1}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 84, 85, 85, 80}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 64, 85, 84, 0, 85, 80, 0, 85, 0, 0, 80, 0, 2, 0, 0, 42, 0, 0, 42}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 21, 85, 85, 1, 85, 85, 0, 85, 85, 0, 5, 85, 0, 0, 85, 160, 0, 5, 168, 0, 0}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {84, 0, 1, 84, 0, 1, 84, 0, 1, 84, 0, 0, 84, 0, 0, 84, 2, 0, 84, 2, 0, 84, 2, 0, 84, 2, 128, 84, 2, 128, 84, 2, 128, 84, 2, 160, 84, 2, 160, 84, 2, 168, 84, 2, 168, 84, 2, 170, 84, 2, 170, 84, 2, 170}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 21, 85, 85, 21, 85, 85, 21, 85, 85, 5, 85, 85, 5, 85, 85, 5, 85, 85, 1, 85, 85, 0, 85, 85, 0, 85, 85, 0, 85, 85, 0, 21, 85, 0, 21, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 80, 85, 85, 64, 85, 84, 0, 85, 80, 0, 85, 64, 0, 85, 64, 0, 85, 0, 0}, {85, 85, 85, 85, 85, 85, 85, 85, 80, 85, 85, 0, 85, 84, 0, 85, 64, 0, 85, 0, 0, 84, 0, 0, 80, 0, 0, 64, 2, 160, 0, 10, 168, 0, 42, 170, 0, 170, 170, 0, 170, 170, 2, 170, 170, 10, 170, 170, 10, 170, 170, 10, 170, 170}, {80, 0, 0, 0, 0, 10, 0, 0, 170, 0, 42, 170, 0, 170, 170, 42, 170, 170, 42, 170, 170, 10, 170, 170, 2, 170, 170, 2, 170, 170, 0, 42, 170, 0, 42, 170, 128, 10, 170, 160, 10, 170, 160, 2, 170, 168, 2, 170, 168, 0, 170, 170, 0, 32}, {10, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 168, 170, 170, 128, 170, 160, 0, 170, 128, 0, 168, 0, 0, 160, 0, 10, 128, 0, 170, 0, 10, 170, 0, 42, 170, 0, 170, 170}, {160, 0, 0, 170, 128, 0, 170, 170, 0, 170, 170, 160, 170, 170, 160, 170, 170, 128, 170, 160, 0, 168, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, 170, 0, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170}, {21, 85, 85, 1, 85, 85, 0, 85, 85, 0, 21, 85, 0, 5, 85, 0, 1, 85, 0, 0, 85, 40, 0, 85, 170, 0, 5, 170, 128, 1, 170, 160, 1, 170, 168, 0, 170, 170, 0, 170, 170, 128, 170, 170, 128, 170, 170, 160, 170, 170, 168, 170, 170, 168}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 21, 85, 85, 5, 85, 85, 1, 85, 85, 0, 85, 85, 0, 21, 85, 0, 5, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 84, 85, 85, 64, 85, 85, 0, 85, 84, 0, 85, 80, 0, 85, 64, 2, 85, 0, 10, 84, 0, 42, 64, 2, 170}, {85, 85, 64, 85, 85, 0, 85, 80, 0, 85, 64, 0, 85, 0, 10, 84, 0, 42, 80, 0, 170, 64, 2, 170, 0, 10, 170, 0, 170, 170, 2, 170, 170, 10, 170, 170, 42, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170}, {0, 10, 170, 2, 170, 170, 42, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 168, 42, 170, 160, 10, 170, 160, 10, 170, 168, 42, 170, 170, 170, 170, 170, 170}, {170, 160, 0, 170, 168, 0, 170, 170, 128, 170, 170, 160, 170, 170, 160, 170, 170, 168, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170}, {21, 85, 85, 5, 85, 85, 5, 85, 85, 1, 85, 85, 1, 85, 85, 0, 85, 85, 0, 85, 85, 0, 21, 85, 0, 21, 85, 128, 5, 85, 128, 5, 85, 160, 1, 85, 160, 1, 85, 168, 0, 85, 168, 0, 85, 170, 0, 21, 170, 128, 5, 170, 128, 5}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 64}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {84, 0, 170, 85, 0, 170, 85, 0, 42, 85, 64, 10, 85, 80, 10, 85, 80, 2, 85, 84, 2, 85, 84, 0, 85, 85, 0, 85, 85, 0, 85, 85, 64, 85, 85, 64, 85, 85, 80, 85, 85, 80, 85, 85, 84, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {128, 5, 85, 160, 1, 85, 160, 0, 85, 168, 0, 85, 170, 0, 21, 170, 0, 5, 170, 128, 1, 170, 160, 0, 170, 168, 0, 42, 170, 0, 42, 170, 128, 10, 170, 168, 2, 170, 170, 0, 0, 170, 0, 0, 0, 0, 0, 0, 64, 0, 0, 85, 85, 0}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 170, 170, 170, 170, 170, 170, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {85, 85, 85, 85, 85, 85, 85, 85, 84, 85, 85, 64, 85, 85, 0, 85, 80, 0, 85, 0, 0, 80, 0, 2, 0, 0, 10, 0, 0, 170, 0, 10, 170, 2, 170, 170, 170, 170, 170, 170, 170, 170, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {80, 0, 128, 64, 0, 0, 0, 2, 0, 0, 40, 0, 0, 168, 2, 0, 160, 2, 0, 0, 10, 0, 0, 10, 128, 0, 42, 128, 0, 170, 160, 2, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {42, 170, 170, 42, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 42, 170, 170, 0, 0, 170, 0, 0, 0, 0, 0, 0}, {170, 128, 0, 170, 160, 0, 170, 160, 0, 170, 160, 0, 170, 168, 2, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 0, 10, 170, 0, 0, 0}, {2, 170, 170, 2, 170, 170, 42, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 0, 42, 170}, {170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170}, {170, 170, 160, 170, 170, 160, 170, 170, 160, 170, 170, 160, 170, 170, 168, 170, 170, 168, 170, 170, 168, 170, 170, 168, 170, 170, 168, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170}, {0, 5, 85, 0, 0, 85, 0, 0, 21, 2, 0, 5, 2, 128, 0, 2, 168, 0, 2, 170, 0, 2, 170, 128, 0, 168, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 0, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 5, 85, 85, 5, 85, 85, 0, 85, 85, 0, 21, 85, 0, 5, 85, 0, 0, 85, 160, 0, 21, 168, 0, 0, 170, 128, 0, 170, 160, 0, 170, 170, 128, 170, 170, 170, 170, 170, 170}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 170, 170, 170, 170, 170, 170}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 80, 85, 85, 0, 85, 80, 0, 84, 0, 0, 0, 0, 0, 0, 0, 42, 0, 2, 170, 0, 170, 170, 42, 170, 170, 170, 170, 170, 170, 170, 128}, {85, 85, 85, 85, 85, 80, 85, 85, 64, 85, 84, 0, 85, 80, 0, 84, 0, 2, 0, 0, 10, 0, 0, 170, 0, 2, 170, 0, 170, 170, 42, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 0, 0, 0}, {0, 10, 170, 0, 42, 170, 0, 170, 170, 10, 170, 170, 42, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 0, 0, 0}, {170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 0, 0, 0}, {170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 160, 0, 0, 0}, {170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 168, 170, 170, 160, 170, 170, 168, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 160, 168, 0, 0, 0, 0, 0, 0, 0, 0}, {170, 160, 1, 170, 168, 1, 170, 168, 1, 170, 168, 1, 170, 168, 0, 170, 168, 0, 170, 170, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 170, 168, 0, 170, 0, 0, 168, 0, 0, 0, 0, 1, 0, 0, 85, 0, 5, 85, 0, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 84, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 21, 5, 85, 85, 5, 85, 85, 5, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 0, 85, 85, 0, 85, 84, 0, 85, 84, 1, 85, 80, 1, 85, 0, 1, 0, 0, 5, 0, 0, 5, 0, 0, 5, 0, 0, 1, 85, 80, 0, 85, 84, 0, 85, 85, 64, 85, 85, 80, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 21, 85, 85, 21, 85, 85, 21, 85, 85, 21, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {64, 0, 0, 85, 85, 0, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {0, 0, 0, 0, 0, 0, 85, 64, 0, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 85, 0, 0, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {0, 0, 42, 0, 0, 0, 0, 0, 0, 0, 0, 0, 85, 85, 64, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {170, 170, 170, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {170, 170, 170, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {170, 168, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {0, 0, 0, 0, 0, 0, 0, 0, 1, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {0, 0, 1, 0, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 84, 85, 85, 82, 85, 85, 82, 85, 85, 84, 85, 85, 85, 85, 85, 85, 85, 85, 84, 85, 85, 82, 85, 85, 84, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 81, 85, 85, 72, 85, 85, 33, 85, 85, 133, 85, 85, 133, 85, 85, 161, 85, 85, 168, 85, 85, 40, 85, 85, 33, 85, 85, 133, 85, 85, 21, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85} };

void OSD_write_new_character(uint8_t* new_character_table, uint8_t character_number);
void OSD_read_character(uint8_t* table_to_save_character, uint8_t character_number);
static bool is_OSD_cleared();

static void OSD_enable_OSD_display();
static void OSD_disable_OSD_display();
static void OSD_characters_from_text(char* text, uint8_t* characters_tab);
static void OSD_characters_from_float(float number, uint8_t* characters_tab, uint8_t max_integer_digits);
static void OSD_characters_from_int(uint16_t number, uint8_t* characters_tab, uint8_t max_digits);
//static void OSD_characters_from_anything(char* text, uint8_t* characters_tab);
static void OSD_set_vertical_offset(int8_t offset);
static void OSD_set_horizontal_offset(int8_t offset);
static void OSD_NTSC_PAL_selection();
static void OSD_enable_auto_black_control();
static void OSD_blinking(uint8_t* character_number_tab, uint16_t first_character_position_on_display, uint16_t string_length);

osd_t main_OSD = { .calibrated = false, .chip_name = "MAX7456" };

// ------SPI2 with DMA usage-----
// reception:
void DMA1_Stream4_IRQHandler(void)
{
	// if stream4 transfer is completed:
	if (DMA1->HISR & DMA_HISR_TCIF4)
	{
		DMA1->HIFCR |= DMA_HIFCR_CTCIF4;
		DMA1_Stream4->CR &= ~DMA_SxCR_EN;
	}
}


void setup_OSD()
{
	CS_SPI2_disable();

	//	wait for saving changes (until OSD is not busy):
	while (is_OSD_busy())
	{
		; // wait
	}
	//	set all OSD parameters:
	OSD_enable_auto_black_control();
	OSD_set_horizontal_offset(-2);
	OSD_set_vertical_offset(4);
	OSD_NTSC_PAL_selection();
	OSD_clear_Display_Memory();
	delay_mili(50);
	OSD_enable_OSD_display();

	// // only for debugging:
	// uint8_t table[256];
	// for (int i = 0; i < 256; i++)
	// {
	// 	table[i] = i;
	// }
	// OSD_write_to_Display_Memory_16bit_AI(table, 1, 256);

	//	OSD_update_logo_characters();

	OSD_print_logo();

	delay_mili(1000);

	OSD_clear_Display_Memory();
	//	wait for saving changes (until OSD is not busy):
	while (is_OSD_cleared())
	{
		; // wait
	}
	main_OSD.calibrated = true;
}

void OSD_SPI_write(uint8_t instruction, uint8_t data)
{
	CS_SPI2_enable();

	SPI2_transmit_one(instruction);
	SPI2_transmit_one(data);

	CS_SPI2_disable();
}

void OSD_SPI_read(uint8_t address, uint8_t* memory_address)
{
	//	all registers send only 1 byte. There is one exeption - reading Display Memory in 16-bit mode
	// if you want read more bytes use SPI2_receive() function
	CS_SPI2_enable();
	SPI2_transmit_one(address | 0x80); // for reading adress|0x80
	SPI2_receive_one(memory_address);
	CS_SPI2_disable();
}

void OSD_write_new_character(uint8_t* new_character_table, uint8_t character_number)
{
	//	disable OSD display:
	OSD_disable_OSD_display();

	//	choose character that you want to write to:
	OSD_SPI_write(OSD_CMAH, character_number);
	//	start writing new character into memory:
	for (uint8_t i = 0; i < 54; i++)
	{ //	choose 4-pixel byte that you want to write to:
		OSD_SPI_write(OSD_CMAL, i);
		//	write values for this 4 pixels:
		OSD_SPI_write(OSD_CMDI, new_character_table[i]);
	}
	//	save changes:
	OSD_SPI_write(OSD_CMM, 0xA0);
	//	wait for saving changes (until OSD is not busy):
	while (is_OSD_busy())
	{
		; // wait
	}
	OSD_enable_OSD_display();
}

void OSD_read_character(uint8_t* table_to_save_character, uint8_t character_number)
{
	//	disable OSD display:
	OSD_disable_OSD_display();

	//	choose character that you want to read:
	OSD_SPI_write(OSD_CMAH, character_number);
	//	transfer character to shadow register:
	OSD_SPI_write(OSD_CMM, 0x50);
	//	start reading and saving character pixels:
	for (uint8_t i = 0; i < 54; i++)
	{ //	choose 4-pixel byte that you want to read:
		OSD_SPI_write(OSD_CMAL, i);
		//	read values of this 4 pixels:
		OSD_SPI_read(OSD_CMDO, &table_to_save_character[i]);
	}
	OSD_enable_OSD_display();
}

void OSD_clear_Display_Memory()
{
	uint8_t instruction;
	//	read value of register:
	OSD_SPI_read(OSD_DMM, &instruction);
	instruction |= 0x04;
	OSD_SPI_write(OSD_DMM, instruction);
	//	it will take approx. 20 [us]
}

void OSD_write_to_Display_Memory_8bit(uint8_t character_number, uint16_t character_position_on_display, uint8_t attributes)
{
	//	read current value of register:
	uint8_t instruction = 0x00;
	OSD_SPI_read(OSD_DMM, &instruction);
	//	set operation mode to 8-bit:
	instruction |= 0x40;
	OSD_SPI_write(OSD_DMM, instruction);
	//	prepare for writing address:
	OSD_SPI_write(OSD_DMAH, 0x00);
	//	set MSB:
	OSD_SPI_write(OSD_DMAH, character_position_on_display >> 8);
	//	set lower order bits:
	OSD_SPI_write(OSD_DMAL, 0xFF & character_position_on_display);

	//	set character address:
	OSD_SPI_write(OSD_DMDI, character_number);

	//	prepare for writing attributes:
	OSD_SPI_write(OSD_DMAH, 0x02);
	//	set MSB:
	OSD_SPI_write(OSD_DMAH, character_position_on_display >> 8);
	//	set lower order bits:
	OSD_SPI_write(OSD_DMAL, 0xFF & character_position_on_display);
	//	write attributes:
	OSD_SPI_write(OSD_DMDI, attributes);
}

void OSD_write_to_Display_Memory_16bit(uint8_t character_number, uint16_t character_position_on_display)
{
	//	read actual value of register:
	uint8_t instruction = 0x00;
	OSD_SPI_read(OSD_DMM, &instruction);
	//	set operation mode to 16-bit:
	instruction &= ~0x40;
	OSD_SPI_write(OSD_DMM, instruction);
	//	set Atributes:
	instruction |= 0x0;
	OSD_SPI_write(OSD_DMM, instruction);
	//	set MSB:
	OSD_SPI_write(OSD_DMAH, character_position_on_display >> 8);
	//	set lower order bits:
	OSD_SPI_write(OSD_DMAL, 0xFF & character_position_on_display);
	//	set character address:
	OSD_SPI_write(OSD_DMDI, character_number);
}

void OSD_write_to_Display_Memory_16bit_AI(uint8_t* character_number_tab, uint16_t first_character_position_on_display, uint16_t string_length)
{ //	You can NOT print 255th character with this function!

	//	define position on the display of the first character:
	//	set MSB:
	OSD_SPI_write(OSD_DMAH, first_character_position_on_display >> 8);
	//	set lower order bits:
	OSD_SPI_write(OSD_DMAL, 0xFF & first_character_position_on_display);

	//	set auto-increment mode, operation mode to 16-bit and define attributes:
	OSD_SPI_write(OSD_DMM, 0x01 | 0x00 | 0x00);

	//	write next characters of the string:
	CS_SPI2_enable();
	SPI2_transmit(character_number_tab, string_length);

	//	terminate auto-increment mode:
	SPI2_transmit_one(OSD_TERMINATE_AUTO_INCREMENT);
	CS_SPI2_disable();
}

bool is_OSD_busy()
{
	uint8_t status;
	OSD_SPI_read(OSD_STAT, &status);
	if (status & OSD_BUSY_FLAG)
	{
		return true;
	}
	return false;
}

static void OSD_enable_OSD_display()
{
	uint8_t instruction;
	//	enable OSD display:
	OSD_SPI_read(OSD_VM0, &instruction);
	instruction |= 0x0C;
	OSD_SPI_write(OSD_VM0, instruction);
}

static void OSD_disable_OSD_display()
{
	uint8_t instruction;
	//	disable OSD display:
	OSD_SPI_read(OSD_VM0, &instruction);
	instruction &= ~0x0C;
	OSD_SPI_write(OSD_VM0, instruction);
}

static bool is_OSD_cleared()
{
	uint8_t status;
	OSD_SPI_read(OSD_DMM, &status);
	if (status & OSD_CLEAR_DM)
	{
		return true;
	}
	return false;
}


static void OSD_characters_from_text(char* text, uint8_t* characters_tab)
{ //	it is working for standard ASCII symbols without small characters (compare global_character_table and ASCII table)
	for (uint16_t i = 0; i < strlen(text); i++)
	{
		characters_tab[i] = (uint8_t)text[i];
	}
}

static void OSD_characters_from_float(float number, uint8_t* characters_tab, uint8_t max_integer_digits)
{
	//	all float numbers are with 2 digits after point and have defined number of integer digits

	//	set point:
	*(characters_tab + max_integer_digits) = '.';
	//	set after point part:
	*(characters_tab + max_integer_digits + 1) = '0' + (uint16_t)(number * 10) % 10;
	*(characters_tab + max_integer_digits + 2) = '0' + (uint16_t)(number * 100) % 10;
	//	set integer part:
	for (uint8_t i = 1; i <= max_integer_digits; i++)
	{
		*(characters_tab + max_integer_digits - i) = '0' + (uint16_t)number % 10;
		number *= 0.1f;
	}
}

static void OSD_characters_from_int(uint16_t number, uint8_t* characters_tab, uint8_t max_digits)
{
	for (uint8_t i = 1; i <= max_digits; i++)
	{
		*(characters_tab + max_digits - i) = '0' + number % 10;
		number /= 10;
	}
}

// static void OSD_characters_from_anything(char* text, uint8_t* characters_tab)
// {
// 	// this function will find all elements in global array so can convert any text with
// 	// but is slow - for letters and digits use different functions
// 	uint16_t length = strlen(text);
// 	uint16_t counter = 0;
// 	for (uint16_t i = 0; i < length; i++)
// 	{
// 		while (text[i] != global_characters_tab[counter] && counter < OSD_MAX_CHARACTER_NUMBER)
// 		{
// 			counter++;
// 		}
// 		characters_tab[i] = counter;
// 		counter = 0;
// 	}
// }


static void OSD_set_vertical_offset(int8_t offset)
{
	//	max offset -32/+31 [px]
	OSD_SPI_write(OSD_VOS, 0x10 + offset); // 0 offset + offset
}

static void OSD_set_horizontal_offset(int8_t offset)
{
	//	max offset -15/+16 [px]
	OSD_SPI_write(OSD_HOS, 0x20 + offset); // 0 offset + offset
}

static void OSD_NTSC_PAL_selection()
{
#if defined(OSD_CAMERA_PAL)
	//	if arbitrary defined PAL camera:
	uint8_t instruction;
	OSD_SPI_read(OSD_VM0, &instruction, 1);
	instruction |= 0x40;
	OSD_SPI_write(OSD_VM0, &instruction, 1);

#elif defined(OSD_CAMERA_NTSC)
	//	if arbitrary defined NTSC camera:
	uint8_t instruction;
	OSD_SPI_read(OSD_VM0, &instruction, 1);
	instruction &= ~0x40;
	OSD_SPI_write(OSD_VM0, &instruction, 1);

#elif defined(OSD_AUTO_NTSC_PAL)
	//	if PAL/NTSC auto-selection:
	uint8_t status;
	uint8_t instruction;
	OSD_SPI_read(OSD_STAT, &status);
	if (status & OSD_NTSC_SIGNAL_DETECTED)
	{
		OSD_SPI_read(OSD_VM0, &instruction);
		instruction &= ~0x40;
		OSD_SPI_write(OSD_VM0, instruction);
	}
	else if (status & OSD_PAL_SIGNAL_DETECTED)
	{
		OSD_SPI_read(OSD_VM0, &instruction);
		instruction |= 0x40;
		OSD_SPI_write(OSD_VM0, instruction);
	}
	else
	{
		; // if nothing was detected don't care
	}
#endif
}

static void OSD_enable_auto_black_control()
{
	uint8_t instruction;
	OSD_SPI_read(OSD_OSDBL, &instruction);
	instruction &= ~0x10;
	OSD_SPI_write(OSD_OSDBL, instruction);
}

void OSD_print_logo()
{
	uint8_t OSD_LOGO_characters[96];
	for (uint8_t i = 0; i < 96; i++)
	{
		OSD_LOGO_characters[i] = 255 - 95 + i;
	}

	OSD_write_to_Display_Memory_16bit_AI(&OSD_LOGO_characters[0], OSD_LOGO_PLACEMENT, 24);
	OSD_write_to_Display_Memory_16bit_AI(&OSD_LOGO_characters[24], OSD_LOGO_PLACEMENT + 30, 24);
	OSD_write_to_Display_Memory_16bit_AI(&OSD_LOGO_characters[48], OSD_LOGO_PLACEMENT + 60, 24);
	OSD_write_to_Display_Memory_16bit_AI(&OSD_LOGO_characters[72], OSD_LOGO_PLACEMENT + 90, 23);
	// in auto-increment mode, it is impossible to print 255th character so, it has to be done separately:
	OSD_write_to_Display_Memory_16bit(255, OSD_LOGO_PLACEMENT + 90 + 23);
}

void OSD_print_battery_voltage()
{
	uint8_t battery_text_tab[11];
	OSD_characters_from_text("BAT:", battery_text_tab);
	OSD_characters_from_float(main_battery.voltage_filtered, &battery_text_tab[4], 2);
	OSD_write_to_Display_Memory_16bit_AI(battery_text_tab, OSD_BATTERY_VOLTAGE_PLACEMENT, 8);
	if (main_battery.cell_voltage < BATTERY_CELL_MIN_VOLTAGE)
	{
		OSD_characters_from_text("LOW BATTERY", battery_text_tab);
		OSD_blinking(battery_text_tab, OSD_WARNING_PLACEMENT - 6, 11);
	}
}

void OSD_print_battery_cell_voltage()
{
	uint8_t battery_text_tab[10];
	OSD_characters_from_text("CELL:", battery_text_tab);
	OSD_characters_from_float(main_battery.cell_voltage, &battery_text_tab[5], 1);
	OSD_write_to_Display_Memory_16bit_AI(battery_text_tab, OSD_BATTERY_CELL_VOLTAGE_PLACEMENT, 9);
}

void OSD_print_time()
{
	uint8_t time_text_tab[11];
	OSD_characters_from_text("TIM:00:00", time_text_tab);
	//	Calculate minutes:
	OSD_characters_from_int(get_Global_Time() / 1000000 / 60, &time_text_tab[4], 2);
	//	Calculate seconds:
	OSD_characters_from_int((get_Global_Time() / 1000000) % 60, &time_text_tab[7], 2);

	OSD_write_to_Display_Memory_16bit_AI(time_text_tab, OSD_TIMER_PLACEMENT, 9);
}

void OSD_print_flight_mode()
{
	static uint8_t text_tab[4];
	switch (get_Flight_Mode())
	{
	case FLIGHT_MODE_ACRO:
		OSD_characters_from_text("ACRO", text_tab);
		OSD_write_to_Display_Memory_16bit_AI(text_tab, OSD_FLIGHT_MODE_PLACEMENT, 4);
		break;
	case FLIGHT_MODE_STABLE:
		OSD_characters_from_text("STAB", text_tab);
		OSD_write_to_Display_Memory_16bit_AI(text_tab, OSD_FLIGHT_MODE_PLACEMENT, 4);
		break;

	default:

		break;
	}
}

static void OSD_blinking(uint8_t* character_number_tab, uint16_t first_character_position_on_display, uint16_t string_length)
{
	//	define position on the display of the first character:
	//	set MSB:
	OSD_SPI_write(OSD_DMAH, first_character_position_on_display >> 8);
	//	set lower order bits:
	OSD_SPI_write(OSD_DMAL, 0xFF & first_character_position_on_display);

	//	set auto-increment mode, operation mode to 16-bit and define attributes (blinking on):
	OSD_SPI_write(OSD_DMM, 0x01 | 0x00 | 0x10);

	//	write next characters of the string:
	CS_SPI2_enable();
	SPI2_transmit(character_number_tab, string_length);

	//	terminate auto-increment mode:
	SPI2_transmit_one(OSD_TERMINATE_AUTO_INCREMENT);
	CS_SPI2_disable();
}

void OSD_print_warnings()
{
	uint8_t text_tab[12];
	switch (FailSafe_status)
	{
	case FS_NO_FAILSAFE:
		OSD_characters_from_text("            ", text_tab);
		OSD_write_to_Display_Memory_16bit_AI(text_tab, OSD_WARNING_PLACEMENT - 6, 12);
		break;
	case FS_RX_TIMEOUT:
		OSD_characters_from_text("NO RX", text_tab);
		OSD_write_to_Display_Memory_16bit_AI(text_tab, OSD_WARNING_PLACEMENT - 2, 5);
		break;
	case FS_NO_PREARM:
		OSD_characters_from_text("NO PREARM", text_tab);
		OSD_write_to_Display_Memory_16bit_AI(text_tab, OSD_WARNING_PLACEMENT - 4, 9);
		break;
	case FS_GYRO_CALIBRATION:
		OSD_characters_from_text("GYRO CALIB", text_tab);
		OSD_write_to_Display_Memory_16bit_AI(text_tab, OSD_WARNING_PLACEMENT - 4, 10);
		break;
	default:
		break;

	}
}

void OSD_update_logo_characters()
{
	for (uint8_t i = 0; i < 96; i++)
	{
		OSD_write_new_character(OSD_LOGO_characters_tab[i], 255 - 95 + i);
	}
}
