/*
 * filters.h
 *
 *  Created on: 24.09.2021
 *      Author: symon
 */

#ifndef FILTERS_H_
#define FILTERS_H_

#include<stdint.h>
#include"global_constants.h"

typedef struct {
	uint8_t length;
	float *buffer;
	uint8_t buffer_index;
	float *impulse_responce;
	float output;

}FIR_Filter;

typedef struct {
	uint8_t filter_order;
	float *buffer_input;
	float *buffer_output;
	uint8_t buffer_index;
	float *forward_coefficients;
	float *feedback_coefficients;
	float output;

}IIR_Filter;

void FIR_Filter_Init(FIR_Filter *fir);
float FIR_Filter_filtering(FIR_Filter *fir, float input);

void IIR_Filter_Init(IIR_Filter *fir);
float IIR_Filter_filtering(IIR_Filter *fir, float input);

#endif /* FILTERS_H_ */
