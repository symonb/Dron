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
const uint8_t OSD_LOGO_characters_tab[96][OSD_CHARACTER_PIXELS_NUMBER] = { {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 64, 85, 85, 0, 1, 85, 0, 1, 84, 0, 1, 84, 0, 1}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 64, 85, 80, 0, 84, 0, 0}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 64, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 168}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 5, 85, 85, 0, 21, 85, 0, 0, 85, 0, 0, 1}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 84, 85, 85, 80}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 64, 85, 84, 0, 85, 80, 0, 85, 0, 0, 80, 0, 2, 0, 0, 42, 0, 0, 42}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 21, 85, 85, 1, 85, 85, 0, 85, 85, 0, 5, 85, 0, 0, 85, 160, 0, 5, 168, 0, 0}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {84, 0, 1, 84, 0, 1, 84, 0, 1, 84, 0, 0, 84, 0, 0, 84, 2, 0, 84, 2, 0, 84, 2, 0, 84, 2, 128, 84, 2, 128, 84, 2, 128, 84, 2, 160, 84, 2, 160, 84, 2, 168, 84, 2, 168, 84, 2, 170, 84, 2, 170, 84, 2, 170}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 21, 85, 85, 21, 85, 85, 21, 85, 85, 5, 85, 85, 5, 85, 85, 5, 85, 85, 1, 85, 85, 0, 85, 85, 0, 85, 85, 0, 85, 85, 0, 21, 85, 0, 21, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 80, 85, 85, 64, 85, 84, 0, 85, 80, 0, 85, 64, 0, 85, 64, 0, 85, 0, 0}, {85, 85, 85, 85, 85, 85, 85, 85, 80, 85, 85, 0, 85, 84, 0, 85, 64, 0, 85, 0, 0, 84, 0, 0, 80, 0, 0, 64, 2, 160, 0, 10, 168, 0, 42, 170, 0, 170, 170, 0, 170, 170, 2, 170, 170, 10, 170, 170, 10, 170, 170, 10, 170, 170}, {80, 0, 0, 0, 0, 10, 0, 0, 170, 0, 42, 170, 0, 170, 170, 42, 170, 170, 42, 170, 170, 10, 170, 170, 2, 170, 170, 2, 170, 170, 0, 42, 170, 0, 42, 170, 128, 10, 170, 160, 10, 170, 160, 2, 170, 168, 2, 170, 168, 0, 170, 170, 0, 32}, {10, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 168, 170, 170, 128, 170, 160, 0, 170, 128, 0, 168, 0, 0, 160, 0, 10, 128, 0, 170, 0, 10, 170, 0, 42, 170, 0, 170, 170}, {160, 0, 0, 170, 128, 0, 170, 170, 0, 170, 170, 160, 170, 170, 160, 170, 170, 128, 170, 160, 0, 168, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, 170, 0, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170}, {21, 85, 85, 1, 85, 85, 0, 85, 85, 0, 21, 85, 0, 5, 85, 0, 1, 85, 0, 0, 85, 40, 0, 85, 170, 0, 5, 170, 128, 1, 170, 160, 1, 170, 168, 0, 170, 170, 0, 170, 170, 128, 170, 170, 128, 170, 170, 160, 170, 170, 168, 170, 170, 168}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 21, 85, 85, 5, 85, 85, 1, 85, 85, 0, 85, 85, 0, 21, 85, 0, 5, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 84, 85, 85, 64, 85, 85, 0, 85, 84, 0, 85, 80, 0, 85, 64, 2, 85, 0, 10, 84, 0, 42, 64, 2, 170}, {85, 85, 64, 85, 85, 0, 85, 80, 0, 85, 64, 0, 85, 0, 10, 84, 0, 42, 80, 0, 170, 64, 2, 170, 0, 10, 170, 0, 170, 170, 2, 170, 170, 10, 170, 170, 42, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170}, {0, 10, 170, 2, 170, 170, 42, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 168, 42, 170, 160, 10, 170, 160, 10, 170, 168, 42, 170, 170, 170, 170, 170, 170}, {170, 160, 0, 170, 168, 0, 170, 170, 128, 170, 170, 160, 170, 170, 160, 170, 170, 168, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170}, {21, 85, 85, 5, 85, 85, 5, 85, 85, 1, 85, 85, 1, 85, 85, 0, 85, 85, 0, 85, 85, 0, 21, 85, 0, 21, 85, 128, 5, 85, 128, 5, 85, 160, 1, 85, 160, 1, 85, 168, 0, 85, 168, 0, 85, 170, 0, 21, 170, 128, 5, 170, 128, 5}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 64}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {84, 0, 170, 85, 0, 170, 85, 0, 42, 85, 64, 10, 85, 80, 10, 85, 80, 2, 85, 84, 2, 85, 84, 0, 85, 85, 0, 85, 85, 0, 85, 85, 64, 85, 85, 64, 85, 85, 80, 85, 85, 80, 85, 85, 84, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {128, 5, 85, 160, 1, 85, 160, 0, 85, 168, 0, 85, 170, 0, 21, 170, 0, 5, 170, 128, 1, 170, 160, 0, 170, 168, 0, 42, 170, 0, 42, 170, 128, 10, 170, 168, 2, 170, 170, 0, 0, 170, 0, 0, 0, 0, 0, 0, 64, 0, 0, 85, 85, 0}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 170, 170, 170, 170, 170, 170, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {85, 85, 85, 85, 85, 85, 85, 85, 84, 85, 85, 64, 85, 85, 0, 85, 80, 0, 85, 0, 0, 80, 0, 2, 0, 0, 10, 0, 0, 170, 0, 10, 170, 2, 170, 170, 170, 170, 170, 170, 170, 170, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {80, 0, 128, 64, 0, 0, 0, 2, 0, 0, 40, 0, 0, 168, 2, 0, 160, 2, 0, 0, 10, 0, 0, 10, 128, 0, 42, 128, 0, 170, 160, 2, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {42, 170, 170, 42, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 42, 170, 170, 0, 0, 170, 0, 0, 0, 0, 0, 0}, {170, 128, 0, 170, 160, 0, 170, 160, 0, 170, 160, 0, 170, 168, 2, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 0, 10, 170, 0, 0, 0}, {2, 170, 170, 2, 170, 170, 42, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 0, 42, 170}, {170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170}, {170, 170, 160, 170, 170, 160, 170, 170, 160, 170, 170, 160, 170, 170, 168, 170, 170, 168, 170, 170, 168, 170, 170, 168, 170, 170, 168, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170}, {0, 5, 85, 0, 0, 85, 0, 0, 21, 2, 0, 5, 2, 128, 0, 2, 168, 0, 2, 170, 0, 2, 170, 128, 0, 168, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 0, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 5, 85, 85, 5, 85, 85, 0, 85, 85, 0, 21, 85, 0, 5, 85, 0, 0, 85, 160, 0, 21, 168, 0, 0, 170, 128, 0, 170, 160, 0, 170, 170, 128, 170, 170, 170, 170, 170, 170}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 170, 170, 170, 170, 170, 170}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 80, 85, 85, 0, 85, 80, 0, 84, 0, 0, 0, 0, 0, 0, 0, 42, 0, 2, 170, 0, 170, 170, 42, 170, 170, 170, 170, 170, 170, 170, 128}, {85, 85, 85, 85, 85, 80, 85, 85, 64, 85, 84, 0, 85, 80, 0, 84, 0, 2, 0, 0, 10, 0, 0, 170, 0, 2, 170, 0, 170, 170, 42, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 0, 0, 0}, {0, 10, 170, 0, 42, 170, 0, 170, 170, 10, 170, 170, 42, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 0, 0, 0}, {170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 0, 0, 0}, {170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 160, 0, 0, 0}, {170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 168, 170, 170, 160, 170, 170, 168, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 160, 168, 0, 0, 0, 0, 0, 0, 0, 0}, {170, 160, 1, 170, 168, 1, 170, 168, 1, 170, 168, 1, 170, 168, 0, 170, 168, 0, 170, 170, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 170, 168, 0, 170, 0, 0, 168, 0, 0, 0, 0, 1, 0, 0, 85, 0, 5, 85, 0, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 84, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 21, 5, 85, 85, 5, 85, 85, 5, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 0, 85, 85, 0, 85, 84, 0, 85, 84, 1, 85, 80, 1, 85, 0, 1, 0, 0, 5, 0, 0, 5, 0, 0, 5, 0, 0, 1, 85, 80, 0, 85, 84, 0, 85, 85, 64, 85, 85, 80, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 21, 85, 85, 21, 85, 85, 21, 85, 85, 21, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {64, 0, 0, 85, 85, 0, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {0, 0, 0, 0, 0, 0, 85, 64, 0, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 85, 0, 0, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {0, 0, 42, 0, 0, 0, 0, 0, 0, 0, 0, 0, 85, 85, 64, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {170, 170, 170, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {170, 170, 170, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {170, 168, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {0, 0, 0, 0, 0, 0, 0, 0, 1, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {0, 0, 1, 0, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 84, 85, 85, 82, 85, 85, 82, 85, 85, 84, 85, 85, 85, 85, 85, 85, 85, 85, 84, 85, 85, 82, 85, 85, 84, 85, 85, 85, 85, 85, 85, 85, 85, 85}, {85, 85, 85, 85, 85, 85, 85, 85, 85, 81, 85, 85, 72, 85, 85, 33, 85, 85, 133, 85, 85, 133, 85, 85, 161, 85, 85, 168, 85, 85, 40, 85, 85, 33, 85, 85, 133, 85, 85, 21, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85} };

static void OSD_SPI_write(uint8_t instruction, uint8_t data);
static void OSD_SPI_read(uint8_t instruction, uint8_t* memory_address);
static void OSD_enable_OSD_display();
static void OSD_disable_OSD_display();
static void OSD_set_vertical_offset(int8_t offset);
static void OSD_set_horizontal_offset(int8_t offset);
static bool OSD_NTSC_PAL_selection();
static void OSD_enable_auto_black_control();
static void OSD_characters_from_text(char* text, uint8_t* characters_tab);
static void OSD_characters_from_float(float number, uint8_t* characters_tab, uint8_t integer_digits, uint8_t decimal_digits);
static void OSD_characters_from_int(uint16_t number, uint8_t* characters_tab, uint8_t max_digits);
// static void OSD_characters_from_anything(char* text, uint8_t* characters_tab);

static void OSD_warning_update(osd_t* OSD, timeUs_t current_time);
static void OSD_preset_buffers(osd_t* OSD);

osd_t main_OSD = { .calibrated = false, .chip_name = "MAX7456", .logo_time = SEC_TO_US(4) };

// ------SPI2 with DMA usage-----
// reception:
void DMA1_Stream4_IRQHandler(void)
{
	// if stream4 transfer is completed:
	if (DMA1->HISR & DMA_HISR_TCIF4)
	{ 	// trnasfer completed:
		DMA1->HIFCR |= DMA_HIFCR_CTCIF4;
		DMA1_Stream4->CR &= ~DMA_SxCR_EN;
		SPI2_CS_disable();
	}
	else if (DMA1->HISR & DMA_HISR_TEIF4) {
		// transfer error
		DMA1->HIFCR |= DMA_HIFCR_CTEIF4;
		DMA1_Stream4->CR &= ~DMA_SxCR_EN;
	}
}

/**
 * @brief Write 1 byte into register.
 * @param instruction register address or other instruction
 * @param data byte of data to write
 * @return None
*/
static void OSD_SPI_write(uint8_t instruction, uint8_t data)
{
	SPI2_CS_enable();

	SPI2_transmit_one(instruction);
	SPI2_transmit_one(data);

	SPI2_CS_disable();
}

/**
 * @brief Read 1 byte from registerc.
 * @param address register address, to be read from
 * @param memory_address pointer to variable to save read byte
 * @return None
 */
static void OSD_SPI_read(uint8_t address, uint8_t* memory_address)
{
	//	all registers send only 1 byte. There is one exeption - reading Display Memory in 16-bit mode
	// if you want read more bytes use SPI2_receive() function
	SPI2_CS_enable();
	SPI2_transmit_one(address | 0x80); // for reading: adress|0x80
	SPI2_receive_one(memory_address);
	SPI2_CS_disable();
}

/**
 *@brief Enable OSD printing characters.
 *@return None
*/
static void OSD_enable_OSD_display()
{
	uint8_t instruction;
	//	enable OSD display:
	OSD_SPI_read(OSD_VM0, &instruction);
	instruction |= 0x0C;
	OSD_SPI_write(OSD_VM0, instruction);
}

/**
 *@brief Disable OSD printing characters.
 *@return None
 *@note Used for updating characters.
*/
static void OSD_disable_OSD_display()
{
	uint8_t instruction;
	//	disable OSD display:
	OSD_SPI_read(OSD_VM0, &instruction);
	instruction &= ~0x0C;
	OSD_SPI_write(OSD_VM0, instruction);
}

/**
 *@brief set vertical offset
*@param offset value of offset up(>0) or down(<0) max offset values -32/+31 [px]
*@return None
*/
static void OSD_set_vertical_offset(int8_t offset)
{
	OSD_SPI_write(OSD_VOS, 0x10 + offset); // 0 offset + offset
}

/**
 *@brief set horizontal offset
*@param offset value of offset right(>0) or left(<0) max offset values -15/+16 [px]
*@return None
*/
static void OSD_set_horizontal_offset(int8_t offset)
{
	OSD_SPI_write(OSD_HOS, 0x20 + offset); // 0 offset + offset
}

/**
 * @brief configure module for PAL or NTSC input
 * @note #define OSD_CAMERA_PAL or OSD_CAMERA_NTSC or OSD_AUTO_NTSC_PAL
 * @retval true if correct signal was detected
 * @retval false if detected signal is not matching predefined or no signal detected
*/
static bool OSD_NTSC_PAL_selection()
{
	//	if input is not specified run PAL/NTSC auto-selection and configure for detected signal:
	uint8_t status;
	uint8_t instruction;
	OSD_SPI_read(OSD_STAT, &status);
	if (status & OSD_NTSC_SIGNAL_DETECTED)
	{
#if defined(OSD_CAMERA_PAL)	
		// if camera signal was defined as PAL and NTSC was detected:
		return false;
#endif
		// configure module for NTSC input
		OSD_SPI_read(OSD_VM0, &instruction);
		instruction &= ~0x40;
		OSD_SPI_write(OSD_VM0, instruction);
		return true;
	}
	else if (status & OSD_PAL_SIGNAL_DETECTED)
	{
#if defined(OSD_CAMERA_NTSC)	
		// if camera signal was defined as NTSC and PAL was detected:
		return false;
#endif
		// configure module for PAL input:
		OSD_SPI_read(OSD_VM0, &instruction);
		instruction |= 0x40;
		OSD_SPI_write(OSD_VM0, instruction);
		return true;
	}
	else
	{
		// if no signal was detected return false:
		return false;
	}
}

/**
 * @brief Enable auto black control.
 * @note By default it is enabled.
 * @return None
*/
static void OSD_enable_auto_black_control()
{
	uint8_t instruction;
	OSD_SPI_read(OSD_OSDBL, &instruction);
	instruction &= ~0x10;
	OSD_SPI_write(OSD_OSDBL, instruction);
}

/**
* @brief Convert text into chracters numbers for OSD display.
* @param text pointer to your text
* @param chracters_tab pointer for array for storing characters values
* @return None
**/
static void OSD_characters_from_text(char* text, uint8_t* characters_tab)
{ //	it is working for standard ASCII symbols without small characters (compare global_character_table and ASCII table)
	for (uint16_t i = 0; i < strlen(text); i++)
	{
		characters_tab[i] = (uint8_t)text[i];
	}
}

/**
* @brief Convert float number into chracters numbers for OSD display.
* @param  number float number
* @param chracters_tab pointer for array for storing characters values
* @param integer_digits how many digits of a integer part will be converted ('0' prefix if needed e.g. 01.23 or 001.23)
* @param decimal_digits how many digits of a decimal part will be converted (if 3 -> 1.234, if 1 -> 1.2)
* @return None
**/
static void OSD_characters_from_float(float number, uint8_t* characters_tab, uint8_t integer_digits, uint8_t decimal_digits)
{
	//	set point:
	*(characters_tab + integer_digits) = '.';
	//	set decimal part:
	uint16_t integer_part = number;
	for (uint8_t i = 1; i <= decimal_digits; i++)
	{
		number *= 10;
		*(characters_tab + integer_digits + i) = '0' + (uint16_t)number % 10;
	}

	//	set integer part:
	for (uint8_t i = 1; i <= integer_digits; i++)
	{
		*(characters_tab + integer_digits - i) = '0' + integer_part % 10;
		integer_part /= 10;
	}
}

/**
* @brief Convert integer number into chracters numbers for OSD display.
* @param  number integer number
* @param chracters_tab pointer for array for storing characters values
* @param max_digits how many digits of a intiger part will be converted ('0' prefix if needed e.g. 01 or 012)
* @return None
**/
static void OSD_characters_from_int(uint16_t number, uint8_t* characters_tab, uint8_t max_digits)
{
	for (uint8_t i = 1; i <= max_digits; i++)
	{
		*(characters_tab + max_digits - i) = '0' + number % 10;
		number /= 10;
	}
}

/**
* @brief Convert any string into characters numbers.
* @param char pointer for text string
* @param chracters_tab pointer for array for storing characters values
* @note This function search predefined array with all characters. This is slow. If you can, avoid using this function.
* @return None
**/
// static void OSD_characters_from_anything(char* text, uint8_t* characters_tab)
// {
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

// screen printning only if something changes:
/**
 *@brief Update warning message and clear if time has come.
*@param OSD Struct which contain buffers to update
*@param current_time Current time in [us]
*@return None
*/
static void OSD_warning_update(osd_t* OSD, timeUs_t current_time) {
	static timeUs_t last_time;
	switch (OSD_warning)
	{
	case WARNING_NONE:
		//	clear a warning message if time has passed:
		if (current_time - last_time > SEC_TO_US(OSD_WARNINGS_TIME)) {
			last_time = current_time;
			OSD_characters_from_text("                ", &OSD->shadow_buffer[OSD_WARNING_PLACEMENT - 7]);
		}
		break;
	case WARNING_NO_RX:
		OSD_characters_from_text("NO RX", &OSD->shadow_buffer[OSD_WARNING_PLACEMENT - 2]);
		last_time = current_time;
		break;
	case WARNING_RX_ERROR:
		OSD_characters_from_text("RX ERROR", &OSD->shadow_buffer[OSD_WARNING_PLACEMENT - 4]);
		last_time = current_time;
		break;
	case WARNING_NO_PREARM:
		OSD_characters_from_text("NO PREARM", &OSD->shadow_buffer[OSD_WARNING_PLACEMENT - 4]);
		last_time = current_time;
		break;
	case WARNING_THROTTLE_PREARM:
		OSD_characters_from_text("THROTTLE PREARM", &OSD->shadow_buffer[OSD_WARNING_PLACEMENT - 7]);

		last_time = current_time;
		break;
	case WARNING_GYRO_CALIBRATION:
		OSD_characters_from_text("GYRO CALIB", &OSD->shadow_buffer[OSD_WARNING_PLACEMENT - 4]);

		last_time = current_time;
		break;
	case WARNING_BLACKBOX_FULL:
		OSD_characters_from_text("BLACKBOX FULL", &OSD->shadow_buffer[OSD_WARNING_PLACEMENT - 6]);

		last_time = current_time;
		break;
	case WARNING_BATTERY_LOW:
		switch (main_battery.status) {
		case BATTERY_LOW:
			OSD_characters_from_text("LOW BATTERY", &OSD->shadow_buffer[OSD_WARNING_PLACEMENT - 6]);
			break;
		case BATTERY_VERY_LOW:
			OSD_characters_from_text("LAND NOW", &OSD->shadow_buffer[OSD_WARNING_PLACEMENT - 4]);

			break;
		default:
			break;
		}
		last_time = current_time;
		break;
	default:
		break;
	}
	OSD_warning = WARNING_NONE;
}

/**
*@brief Function which preset table with characters (most of the page will stay clear and only a few places will be updated)
*@param OSD Struct which contain buffers to preset,
*@return None
*/
static void OSD_preset_buffers(osd_t* OSD) {
	// set all character as ' ' page will be clear
	for (uint16_t i = 0; i < OSD_SCREEN_SIZE; ++i) {
		OSD_characters_from_text(" ", &OSD->buffer[i]);
		OSD_characters_from_text(" ", &OSD->shadow_buffer[i]);
	}
	// add all communicats:
	OSD_characters_from_text("TIM:--:--", &OSD->shadow_buffer[OSD_TIMER_PLACEMENT]);
	OSD_characters_from_text("BAT:--.-", &OSD->shadow_buffer[OSD_BATTERY_VOLTAGE_PLACEMENT]);
	OSD_characters_from_text("CELL:-.--", &OSD->shadow_buffer[OSD_BATTERY_CELL_VOLTAGE_PLACEMENT]);
	OSD_characters_from_text("MODE", &OSD->shadow_buffer[OSD_FLIGHT_MODE_PLACEMENT]);
	OSD->shadow_buffer[OSD_POINTER_PLACEMENT - 1] = 114;
	OSD->shadow_buffer[OSD_POINTER_PLACEMENT] = 115;
	OSD->shadow_buffer[OSD_POINTER_PLACEMENT + 1] = 116;


}


/**
 * @brief Initializate OSD module and print logo.
 * @param OSD pointer to struct to initialize
 * @param current_time current time in [us]
 * @retval True if setup copleted, false if in progress.
 * @note Non-blocking - needs to be called again if returned false.
 * */
bool OSD_init(osd_t* OSD, timeUs_t current_time)
{
	static uint8_t setup_stage;
	static timeUs_t time_flag;

	//	wait for saving changes (until OSD is not busy):
	switch (setup_stage)
	{
	case 0:
		OSD->calibrated = false;
		if (!OSD_is_busy()) {
			//	set all OSD parameters:
			OSD_enable_auto_black_control();
			OSD_set_horizontal_offset(-2);
			OSD_set_vertical_offset(4);
			OSD_NTSC_PAL_selection();
			OSD_clear_Display_Memory();
			OSD_preset_buffers(OSD);
			setup_stage++;
		}
		break;
	case 1:
		if (!OSD_is_busy()) {
			OSD_enable_OSD_display();
			OSD_print_logo();
			setup_stage++;
			time_flag = current_time;
		}

		// only for debugging:
		// uint8_t table[256];
		// for (int i = 0; i < 256; i++)
		// {
		// 	table[i] = i;
		// }
		// OSD_write_to_DM_16bit_AI(table, 1, 256);
		// delay_mili(10000);
		break;

		//	OSD_update_logo_characters();

	case 2:
		// wait some time to display a logo:
		if (current_time - time_flag > OSD->logo_time) {

			OSD_clear_Display_Memory();
			setup_stage++;
			time_flag = current_time;
		}
		break;
	case 3:
		//	wait for saving changes (until OSD is not busy):
		if (OSD_is_cleared()) {
			OSD->calibrated = true;
			// for reconecting battery with USB power:
			setup_stage = 0;
			return true;
		}
		break;
	default:
		break;
	}
	return false;
}

/**
 * @brief Clear Display Memory so screen is empty.
 * @return None
*/
void OSD_clear_Display_Memory()
{
	uint8_t instruction;
	//	read value of register:
	OSD_SPI_read(OSD_DMM, &instruction);
	instruction |= 0x04;
	OSD_SPI_write(OSD_DMM, instruction);
	//	it will take approx. 20 [us]
}

/**
 * @brief Check if OSD finieshed clearing Display Memory.
 * @return true if finished, false if not.
*/
bool OSD_is_cleared()
{
	uint8_t status;
	OSD_SPI_read(OSD_DMM, &status);
	if (!(status & OSD_DMM_CLEAR_DM))
	{
		return true;
	}
	return false;
}

/**
 * @brief Print one chracter with defined attributes.
* @param character_number number of the character to display
* @param character_poition_on_display position on the screen to print character <0-479>
* @param attributes additional attributes for character ( bits DMM[5:3])
* @return None
*/
void OSD_write_to_DM_8bit(uint8_t character_number, uint16_t character_position_on_display, uint8_t attributes)
{
	//	set operation mode to 8-bit:
	uint8_t instruction = OSD_DMM_MODE;
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

/**
 * @brief Print one chracter with predefined attributes.
* @param character_number number of the character to display
* @param character_poition_on_display position on the screen to print character <0-479>
* @note Attributes of character are copied from registerg DMM[5:3] automatically.
* @return None
*/
void OSD_write_to_DM_16bit(uint8_t character_number, uint16_t character_position_on_display)
{
	//	read current value of the register:
	uint8_t instruction = 0x00;
	OSD_SPI_read(OSD_DMM, &instruction);
	//	set operation mode to 16-bit:
	instruction &= ~OSD_DMM_MODE;
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

/**
* @brief Print following characters with predefined attributes.
* @param character_number_tab pointer to array with characters numbers to print
* @param first_character_poition_on_display position on the screen to print a first character <0-479>
* @param string_length number of characters to print
* @note Characters are printed in left-right and top-bottom order.
* @note Attributes of characters are copied from registerg DMM[5:3] automatically and are the same for all of the characters.
* @return None
*/
void OSD_write_to_DM_16bit_AI(uint8_t* character_number_tab, uint16_t first_character_position_on_display, uint16_t string_length)
{ //	You can NOT print 255th character with this function!

	//	define position on the display of the first character:
	//	set MSB:
	OSD_SPI_write(OSD_DMAH, first_character_position_on_display >> 8);
	//	set lower order bits:
	OSD_SPI_write(OSD_DMAL, 0xFF & first_character_position_on_display);

	//	set auto-increment mode, operation mode to 16-bit and define attributes:
	OSD_SPI_write(OSD_DMM, OSD_DMM_AUTO_INCREMENT);

	//	write next characters of the string:
	// it is neccesary to send after each character something so character tab has to be rewritten:
	uint8_t new_tab[string_length * 2];
	for (uint16_t i = 0; i < string_length;i++) {
		new_tab[i * 2] = character_number_tab[i];
		new_tab[i * 2 + 1] = OSD_DMDI;
	}
	SPI2_CS_enable();
	SPI2_transmit(new_tab, string_length * 2);

	//	terminate auto-increment mode:
	SPI2_transmit_one(OSD_TERMINATE_AUTO_INCREMENT);
	SPI2_CS_disable();
}

/**
* @brief Print following characters in blinking mode.
* @param character_number_tab pointer to array with characters numbers to print
* @param first_character_poition_on_display position on the screen to print a first character <0-479>
* @param string_length number of characters to print
* @note Characters are printed in left-right and top-bottom order.
* @note Attributes of characters are copied from registerg DMM[5:3] automatically and are the same for all of the characters.
* @return None
*/
void OSD_write_to_DM_16bit_AI_blinking(uint8_t* character_number_tab, uint16_t first_character_position_on_display, uint16_t string_length)
{
	//	define position on the display of the first character:
	//	set MSB:
	OSD_SPI_write(OSD_DMAH, first_character_position_on_display >> 8);
	//	set lower order bits:
	OSD_SPI_write(OSD_DMAL, 0xFF & first_character_position_on_display);

	//	set auto-increment mode, operation mode to 16-bit and define attributes (blinking on):
	OSD_SPI_write(OSD_DMM, OSD_DMM_AUTO_INCREMENT | OSD_DMM_BLK);

	//	write next characters of the string:
	// it is neccesary to send after each character something so character tab has to be rewritten:
	uint8_t new_tab[string_length * 2];
	for (uint16_t i = 0; i < string_length;i++) {
		new_tab[i * 2] = character_number_tab[i];
		new_tab[i * 2 + 1] = OSD_DMDI;
	}
	SPI2_CS_enable();
	SPI2_transmit(new_tab, string_length * 2);

	//	terminate auto-increment mode:
	SPI2_transmit_one(OSD_TERMINATE_AUTO_INCREMENT);
	SPI2_CS_disable();
}

/**
 * @brief Check if OSD module is busy.
 * @return true if busy, false if not.
*/
bool OSD_is_busy()
{
	uint8_t status;
	OSD_SPI_read(OSD_STAT, &status);
	if (status & OSD_BUSY_FLAG)
	{
		return true;
	}
	return false;
}

/**
 *@brief Read current character's pixels values.
 *@param table_to_save_character pointer for save character's pixels values
 *@param character_number number of character to save
 *@return None
*/
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

/**
 *@brief Update character look.
 *@param new_character_table pointer for new character pixels values
 *@param character_number number of character to update
 *@return None
*/
void OSD_write_new_character(const uint8_t* new_character_table, uint8_t character_number)
{
	//	disable OSD display:
	OSD_disable_OSD_display();

	//	choose character that you want to write to:
	OSD_SPI_write(OSD_CMAH, character_number);
	//	start writing new character into memory:
	for (uint8_t i = 0; i < OSD_CHARACTER_PIXELS_NUMBER; i++)
	{ //	choose 4-pixel byte that you want to write to:
		OSD_SPI_write(OSD_CMAL, i);
		//	write values for this 4 pixels:
		OSD_SPI_write(OSD_CMDI, new_character_table[i]);
	}
	//	save changes:
	OSD_SPI_write(OSD_CMM, 0xA0);
	//	wait for saving changes (until OSD is not busy):
	while (OSD_is_busy())
	{
		; // wait
	}
	OSD_enable_OSD_display();
}

/**
 *@brief Update logo look.
 *@param new_logo_characters_tab ponter to 2D array with new characters pixels values
 *@note Last 96 characters are defined as logo characters which create image 4X24 characters.
 *@return None
*/
void OSD_update_logo_characters(uint8_t** new_logo_characters_tab)
{
	for (uint8_t i = 0; i < 96; i++)
	{
		OSD_write_new_character(new_logo_characters_tab[i], 255 - 95 + i);
	}
}

// screen printning functions (not really fast, since they need to rewrite all arrays again):
/**
* @brief Prints a logo.
* @note It is using pulling functions (it is not really fast). Recomennded for initialization or other non-time-critical parts.
* @return None
*/
void OSD_print_logo()
{
	uint8_t OSD_LOGO_characters[96];
	for (uint8_t i = 0; i < 96; i++)
	{
		OSD_LOGO_characters[i] = 255 - 95 + i;
	}

	OSD_write_to_DM_16bit_AI(&OSD_LOGO_characters[0], OSD_LOGO_PLACEMENT, 24);
	OSD_write_to_DM_16bit_AI(&OSD_LOGO_characters[24], OSD_LOGO_PLACEMENT + 30, 24);
	OSD_write_to_DM_16bit_AI(&OSD_LOGO_characters[48], OSD_LOGO_PLACEMENT + 60, 24);
	OSD_write_to_DM_16bit_AI(&OSD_LOGO_characters[72], OSD_LOGO_PLACEMENT + 90, 23);
	// in auto-increment mode, it is impossible to print 255th character so, it has to be done separately:
	OSD_write_to_DM_16bit(255, OSD_LOGO_PLACEMENT + 90 + 23);
}

/**
* @brief Prints battery voltage.
* @note It is using pulling functions (it is not really fast). Recomennded for initialization or other non-time-critical parts.
* @return None
*/
void OSD_print_battery_voltage()
{
	uint8_t battery_text_tab[8];
	OSD_characters_from_text("BAT:", battery_text_tab);
	OSD_characters_from_float(main_battery.voltage_filtered, &battery_text_tab[4], 2, 1);
	OSD_write_to_DM_16bit_AI(battery_text_tab, OSD_BATTERY_VOLTAGE_PLACEMENT, sizeof(battery_text_tab));
}

/**
* @brief Prints battery cell voltage.
* @note It is using pulling functions (it is not really fast). Recomennded for initialization or other non-time-critical parts.
* @return None
*/
void OSD_print_battery_cell_voltage()
{
	uint8_t battery_text_tab[9];
	OSD_characters_from_text("CELL:", battery_text_tab);
	OSD_characters_from_float(main_battery.cell_voltage, &battery_text_tab[5], 1, 2);
	OSD_write_to_DM_16bit_AI(battery_text_tab, OSD_BATTERY_CELL_VOLTAGE_PLACEMENT, sizeof(battery_text_tab));
}

/**
* @brief Prints time from power up.
* @note It is using pulling functions (it is not really fast). Recomennded for initialization or other non-time-critical parts.
* @return None
*/
void OSD_print_time()
{
	uint8_t time_text_tab[9];
	OSD_characters_from_text("TIM:00:00", time_text_tab);
	//	Calculate minutes:
	OSD_characters_from_int(get_Global_Time() / 1000000 / 60, &time_text_tab[4], 2);
	//	Calculate seconds:
	OSD_characters_from_int((get_Global_Time() / 1000000) % 60, &time_text_tab[7], 2);

	OSD_write_to_DM_16bit_AI(time_text_tab, OSD_TIMER_PLACEMENT, sizeof(time_text_tab));
}

/**
* @brief Prints flight mode.
* @note It is using pulling functions (it is not really fast). Recomennded for initialization or other non-time-critical parts.
* @return None
*/
void OSD_print_flight_mode()
{
	static uint8_t text_tab[4];
	switch (get_Flight_Mode())
	{
	case FLIGHT_MODE_ACRO:
		OSD_characters_from_text("ACRO", text_tab);
		break;
	case FLIGHT_MODE_STABLE:
		OSD_characters_from_text("STAB", text_tab);
		break;
	case FLIGHT_MODE_ALT_HOLD:
		OSD_characters_from_text("ALTH", text_tab);
		break;

	default:

		break;
	}
	OSD_write_to_DM_16bit_AI(text_tab, OSD_FLIGHT_MODE_PLACEMENT, sizeof(text_tab));
}

/**
* @brief Prints a warning message and clear after timeout.
* @note It is using pulling functions (it is not really fast). Recomennded for initialization or other non-time-critical parts.
* @return None
*/
void OSD_print_warnings(timeUs_t current_time)
{
	static timeUs_t last_time;
	uint8_t text_tab[16];
	switch (OSD_warning)
	{
	case WARNING_NONE:
		//	clear a warning message if time has passed:
		if (current_time - last_time > SEC_TO_US(OSD_WARNINGS_TIME)) {
			last_time = current_time;
			OSD_characters_from_text("                ", text_tab);
			OSD_write_to_DM_16bit_AI(text_tab, OSD_WARNING_PLACEMENT - 7, 16);
		}
		break;
	case WARNING_NO_RX:
		OSD_characters_from_text("NO RX", text_tab);
		OSD_write_to_DM_16bit_AI(text_tab, OSD_WARNING_PLACEMENT - 2, 5);
		last_time = current_time;
		break;
	case WARNING_RX_ERROR:
		OSD_characters_from_text("RX ERROR", text_tab);
		OSD_write_to_DM_16bit_AI(text_tab, OSD_WARNING_PLACEMENT - 4, 8);
		last_time = current_time;
		break;
	case WARNING_NO_PREARM:
		OSD_characters_from_text("NO PREARM", text_tab);
		OSD_write_to_DM_16bit_AI(text_tab, OSD_WARNING_PLACEMENT - 4, 9);
		last_time = current_time;
		break;
	case WARNING_THROTTLE_PREARM:
		OSD_characters_from_text("THROTTLE PREARM", text_tab);
		OSD_write_to_DM_16bit_AI(text_tab, OSD_WARNING_PLACEMENT - 7, 15);
		last_time = current_time;
		break;
	case WARNING_GYRO_CALIBRATION:
		OSD_characters_from_text("GYRO CALIB", text_tab);
		OSD_write_to_DM_16bit_AI(text_tab, OSD_WARNING_PLACEMENT - 4, 10);
		last_time = current_time;
		break;
	case WARNING_BLACKBOX_FULL:
		OSD_characters_from_text("BLACKBOX FULL", text_tab);
		OSD_write_to_DM_16bit_AI(text_tab, OSD_WARNING_PLACEMENT - 6, 13);
		last_time = current_time;
		break;
	case WARNING_BATTERY_LOW:
		switch (main_battery.status) {
		case BATTERY_LOW:
			OSD_characters_from_text("LOW BATTERY", text_tab);
			OSD_write_to_DM_16bit_AI(text_tab, OSD_WARNING_PLACEMENT - 6, 11);
			break;
		case BATTERY_VERY_LOW:
			OSD_characters_from_text("LAND NOW", text_tab);
			OSD_write_to_DM_16bit_AI(text_tab, OSD_WARNING_PLACEMENT - 4, 8);
			break;
		default:
			break;
		}
		last_time = current_time;
		break;
	default:
		break;
	}
	OSD_warning = WARNING_NONE;
}

/**
 *@brief Update shadow buffer (new screen value).
*@param OSD Struct which contain buffers to update
*@param current_time Current time in [us]
*@return None
*/
void OSD_update_screen(osd_t* OSD, timeUs_t current_time) {
	//	Calculate minutes:
	OSD_characters_from_int(US_TO_SEC(current_time) / 60, &OSD->shadow_buffer[OSD_TIMER_PLACEMENT + 4], 2);
	//	Calculate seconds:
	OSD_characters_from_int((uint8_t)US_TO_SEC(current_time) % 60, &OSD->shadow_buffer[OSD_TIMER_PLACEMENT + 7], 2);
	OSD_characters_from_float(main_battery.voltage_filtered, &OSD->shadow_buffer[OSD_BATTERY_VOLTAGE_PLACEMENT + 4], 2, 1);
	OSD_characters_from_float(main_battery.cell_voltage, &OSD->shadow_buffer[OSD_BATTERY_CELL_VOLTAGE_PLACEMENT + 5], 1, 2);
	switch (get_Flight_Mode())
	{
	case FLIGHT_MODE_ACRO:
		OSD_characters_from_text("ACRO", &OSD->shadow_buffer[OSD_FLIGHT_MODE_PLACEMENT]);
		break;
	case FLIGHT_MODE_STABLE:
		OSD_characters_from_text("STAB ", &OSD->shadow_buffer[OSD_FLIGHT_MODE_PLACEMENT]);
		break;
	case FLIGHT_MODE_ALT_HOLD:
		OSD_characters_from_text("ALTH ", &OSD->shadow_buffer[OSD_FLIGHT_MODE_PLACEMENT]);
		break;
	default:
		break;
	}
	OSD_warning_update(OSD, current_time);
}

/**
 *@brief  Check if something new to update the screen and start sending.
*@param OSD Struct which contain buffers to print.
*@param current_time Current time in [us].
 *@return True if screen in updated, false if in progress.
 *@note Non-blocking (SPI is using DMA) so it has to be called again if returned false.
*/
bool OSD_draw_screen(osd_t* OSD, timeUs_t current_time) {
	// if DMA is free to send new values (it is disabled after transfer completion):
	if (DMA1_Stream4->CR & DMA_SxCR_EN) {
		return false;
	}

	static uint16_t pos;
	static uint8_t spi2_buffer[SPI_BUFFER_MAX_SIZE];
	uint8_t spi_buff_pos = 0;
	bool auto_inc = false;
	bool set_address = true;
	while (pos < OSD_SCREEN_SIZE && (spi_buff_pos <= SPI_BUFFER_MAX_SIZE - 9)) {
		if (OSD->buffer[pos] != OSD->shadow_buffer[pos]) {
			// update screen character:
			OSD->buffer[pos] = OSD->shadow_buffer[pos];

			if (set_address || !auto_inc) {
				// check if auto-increment mode is worthwhile:
				if (OSD->buffer[pos + 1] != OSD->shadow_buffer[pos + 1]) {
					// set autoincrement mode and other atributes:
					spi2_buffer[spi_buff_pos++] = OSD_DMM;
					spi2_buffer[spi_buff_pos++] = OSD_DMM_AUTO_INCREMENT;
					auto_inc = true;
				}
				else {
					// set normal mode and other atyributes:
					spi2_buffer[spi_buff_pos++] = OSD_DMM;
					spi2_buffer[spi_buff_pos++] = 0X00;
					auto_inc = false;
				}

				// set address of the character:
				spi2_buffer[spi_buff_pos++] = OSD_DMAH;
				spi2_buffer[spi_buff_pos++] = pos >> 8;
				spi2_buffer[spi_buff_pos++] = OSD_DMAL;
				spi2_buffer[spi_buff_pos++] = pos & 0xFF;

				set_address = false;
			}

			// set character: 
			spi2_buffer[spi_buff_pos++] = OSD_DMDI;
			spi2_buffer[spi_buff_pos++] = OSD->buffer[pos];
		}
		else {
			if (!set_address) {
				set_address = true;
				if (auto_inc) {
					spi2_buffer[spi_buff_pos++] = OSD_DMDI;
					spi2_buffer[spi_buff_pos++] = OSD_TERMINATE_AUTO_INCREMENT;
				}
			}
		}
		pos++;

	}

	if (auto_inc) {
		if (!set_address) {
			spi2_buffer[spi_buff_pos++] = OSD_DMDI;
			spi2_buffer[spi_buff_pos++] = OSD_TERMINATE_AUTO_INCREMENT;
		}

	}
	spi2_buffer[spi_buff_pos++] = OSD_DMM;
	spi2_buffer[spi_buff_pos++] = 0X00;

	// start DMA transfer if anything to send:
	if (spi_buff_pos != 0) {
		SPI2_CS_enable();
		SPI2_transmit_DMA(spi2_buffer, spi_buff_pos);
	}
	if (++pos >= OSD_SCREEN_SIZE) {
		pos = 0;
		return true;
	}
	return false;
}

