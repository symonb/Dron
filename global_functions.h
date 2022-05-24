/*
 * global_functions.h
 *
 *  Created on: 03.05.2021
 *      Author: symon
 */

#ifndef GLOBAL_FUNCTIONS_H_
#define GLOBAL_FUNCTIONS_H_

void interrupts_LOCK();
void interrupts_UNLOCK();
void delay_mili(uint16_t);
void delay_micro(uint16_t);
timeUs_t get_Global_Time();
bool failsafe_PID_loop(timeUs_t *);
void TIM6_DAC_IRQHandler();
void TIM7_IRQHandler();
void set_motors(ThreeF);
void update_motors();
void anti_windup(ThreeF* , PIDF*,PIDF*, PIDF*);
void turn_ON_BLUE_LED();
void turn_OFF_BLUE_LED();
void turn_ON_RED_LED();
void turn_OFF_RED_LED();
uint16_t get_Dshot_checksum(uint16_t value);
uint16_t prepare_Dshot_package(uint16_t value);
void fill_Dshot_buffer(uint16_t m1_value,uint16_t m2_value,uint16_t m3_value,uint16_t m4_value);
void fill_Dshot_burst_buffer(uint16_t m1_value, uint16_t m2_value, uint16_t m3_value, uint16_t m4_value);
void prepare_OneShot_PWM();


#endif /* GLOBAL_FUNCTIONS_H_ */
