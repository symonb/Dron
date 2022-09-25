/*
 * filters.h
 *
 *  Created on: 24.09.2021
 *      Author: symon
 */

#ifndef FILTERS_H_
#define FILTERS_H_

#include <stdint.h>
#include "global_constants.h"

typedef struct
{
	uint8_t length;			 //	how many previous samples will be taken into calculation
	float *buffer;			 //	buffer for input measurements
	uint8_t buffer_index;	 //	variable to control position of the latest measurement
	float *impulse_response; //	coefficients for inputs values
	float output;			 //	final output value

} FIR_Filter;

typedef struct
{
	uint8_t filter_order;		  //	how many previous samples will be taken into calculation
	float *buffer_input;		  //	buffer for input measurements
	float *buffer_output;		  //	buffer for output values from filter
	uint8_t buffer_index;		  //	variable to control position of the latest measurement
	float *forward_coefficients;  //	coefficients for inputs values
	float *feedback_coefficients; //	coefficients for previous outputs values
	float output;				  //	final output value

} IIR_Filter;

void FIR_Filter_Init(FIR_Filter *fir);
float FIR_Filter_filtering(FIR_Filter *fir, float input);

void IIR_Filter_Init(IIR_Filter *fir);
float IIR_Filter_filtering(IIR_Filter *fir, float input);

void Gyro_Acc_filters_setup();
void Gyro_Acc_filtering(float *temporary);

#endif /* FILTERS_H_ */
