/*
 * global_functions.h
 *
 *  Created on: 03.05.2021
 *      Author: symon
 */

#ifndef GLOBAL_FUNCTIONS_H_
#define GLOBAL_FUNCTIONS_H_

flight_mode_e get_Flight_Mode();
void interrupts_LOCK();
void interrupts_UNLOCK();
void delay_mili(uint32_t);
void delay_micro(uint16_t);
timeUs_t get_Global_Time();
void TIM6_DAC_IRQHandler();
void TIM7_IRQHandler();
void set_motors(ThreeF);
void anti_windup(ThreeF*, PIDF*, PIDF*, PIDF*);
void turn_ON_BLUE_LED();
void turn_OFF_BLUE_LED();
void turn_ON_RED_LED();
void turn_OFF_RED_LED();
void toggle_BLUE_LED();
void toggle_RED_LED();
void turn_on_BUZZER();
void turn_off_BUZZER();
void toggle_BUZZER();

#endif /* GLOBAL_FUNCTIONS_H_ */
