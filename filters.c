/*
 * filters.c
 *
 *  Created on: 24.09.2021
 *      Author: symon
 */
#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "filters.h"


static FIR_Filter average_gyro_X;
static FIR_Filter average_gyro_Y;
static FIR_Filter average_gyro_Z;
static FIR_Filter average_acc_X;
static FIR_Filter average_acc_Y;
static FIR_Filter average_acc_Z;


void FIR_Filter_Init(FIR_Filter *fir) {

//	Allocate memory for arrays
	fir->buffer = (float *) malloc(sizeof(fir->buffer) * fir->length);
	fir->impulse_responce = (float *) malloc(
			sizeof(fir->impulse_responce) * fir->length);

//	Clear filter buffer
	for (uint8_t i = 0; i < fir->length; i++) {
		fir->buffer[i] = 0.f;
		fir->impulse_responce[i] = 0.f;
	}

//	Clear buffer index
	fir->buffer_index = 0;

//	Clear filter output
	fir->output = 0.0f;

}

float FIR_Filter_filtering(FIR_Filter *fir, float input) {
//	Add new data to buffer
	fir->buffer[fir->buffer_index] = input;

//	Increment buffer_index and loop if necessary
	fir->buffer_index++;
	if (fir->buffer_index == fir->length) {
		fir->buffer_index = 0;
	}
//	Reset old output value
	fir->output = 0.f;

	uint8_t sum_index = fir->buffer_index;

//	Decrement sum_index and loop if necessary
	for (uint8_t i = 0; i < fir->length; i++) {
		if (sum_index > 0) {
			sum_index--;
		} else {
			sum_index = fir->length - 1;
		}
//	Compute new output (via convolution)
		fir->output += fir->impulse_responce[i] * fir->buffer[sum_index];
	}
	return fir->output;
}

void IIR_Filter_Init(IIR_Filter *iir) {

//	Allocate memory for arrays
	iir->buffer_input = (float *) malloc(
			sizeof(iir->buffer_input) * (iir->filter_order + 1));
	iir->buffer_output = (float *) malloc(
			sizeof(iir->buffer_output) * (iir->filter_order));
	iir->forward_coefficients = (float *) malloc(
			sizeof(iir->forward_coefficients) * (iir->filter_order + 1));
	iir->feedback_coefficients = (float *) malloc(
			sizeof(iir->feedback_coefficients) * (iir->filter_order));

//	Clear filter buffers and coefficients
	//FORWARD PART:
	for (uint8_t i = 0; i < iir->filter_order + 1; i++) {
		iir->buffer_input[i] = 0.f;
		iir->forward_coefficients[i] = 0.f;
	}
	//FEEDBACK PART:
	for (uint8_t i = 0; i < iir->filter_order; i++) {
		iir->buffer_output[i] = 0.f;
		iir->feedback_coefficients[i] = 0.f;
	}

//	Clear buffer index
	iir->buffer_index = 0.0f;

//	Clear filter output
	iir->output = 0.0f;

}
//DO POPRAWY SYTUACJA GDY BUFFER index is max
float IIR_Filter_filtering(IIR_Filter *iir, float input) {
//	Add new data to buffer_input
	iir->buffer_input[iir->buffer_index] = input;

//	Reset old output value
	iir->output = 0.f;

	uint8_t sum_index = iir->buffer_index;

//	FORWARD PART:
//	Decrement sum_index and loop if necessary
	for (uint8_t i = 0; i < iir->filter_order + 1; i++) {

//	Compute forward part of a new output
		iir->output += iir->forward_coefficients[i]
				* iir->buffer_input[sum_index];

		if (sum_index > 0) {
			sum_index--;
		} else {
			sum_index = iir->filter_order;
		}
	}

//	FEEDBACK PART:
	sum_index = iir->buffer_index;
//	Decrement sum_index and loop if necessary
	for (uint8_t i = 0; i < iir->filter_order; i++) {
		if (sum_index > 0) {
			sum_index--;
		} else {
			sum_index = iir->filter_order - 1;
		}
// Add feedback part of a new output
		iir->output += iir->feedback_coefficients[i]
				* iir->buffer_output[sum_index];
	}

//	Add new data to buffer_output
	iir->buffer_output[iir->buffer_index] = iir->output;

//	Increment buffer_index and loop if necessary
	iir->buffer_index++;
	if (iir->buffer_index > iir->filter_order) {
		iir->buffer_index = 0;
	}

	return iir->output;
}

void Gyro_Acc_average_filters_setup() {

	float value_gyro[6] = {0.135250,0.216229,0.234301,0.234301,0.216229,0.135250};

	average_gyro_X.length = 6;

	FIR_Filter_Init(&average_gyro_X);

	for (uint8_t i = 0; i < average_gyro_X.length; i++) {
		average_gyro_X.impulse_responce[i] = value_gyro[i];
	}

	average_gyro_Y.length = 6;

	FIR_Filter_Init(&average_gyro_Y);
	for (uint8_t i = 0; i < average_gyro_Y.length; i++) {
		average_gyro_Y.impulse_responce[i] = value_gyro[i];
	}

	average_gyro_Z.length = 6;

	FIR_Filter_Init(&average_gyro_Z);
	for (uint8_t i = 0; i < average_gyro_Z.length; i++) {
		average_gyro_Z.impulse_responce[i] = value_gyro[i];
	}

	float value_acc[6] = {0.135250,0.216229,0.234301,0.234301,0.216229,0.135250};

	average_acc_X.length = 6;
		FIR_Filter_Init(&average_acc_X);
	for (uint8_t i = 0; i < average_acc_X.length; i++) {
		average_acc_X.impulse_responce[i] = value_acc[i];
	}

	average_acc_Y.length = 6;

	FIR_Filter_Init(&average_acc_Y);
	for (uint8_t i = 0; i < average_acc_Y.length; i++) {
		average_acc_Y.impulse_responce[i] = value_acc[i];
	}

	average_acc_Z.length = 6;

	FIR_Filter_Init(&average_acc_Z);
	for (uint8_t i = 0; i < average_acc_Z.length; i++) {
		average_acc_Z.impulse_responce[i] = value_acc[i];
	}
}
void Gyro_Acc_filtering(float*temporary){

Gyro_Acc[0] = FIR_Filter_filtering(&average_gyro_X,
		temporary[0] - GYRO_ROLL_OFFSET);
Gyro_Acc[1] = FIR_Filter_filtering(&average_gyro_Y,
		temporary[1] - GYRO_PITCH_OFFSET);
Gyro_Acc[2] = FIR_Filter_filtering(&average_gyro_Z,
		temporary[2] - GYRO_YAW_OFFSET);
Gyro_Acc[3] = FIR_Filter_filtering(&average_acc_X,
		temporary[3] - ACC_ROLL_OFFSET);
Gyro_Acc[4] = FIR_Filter_filtering(&average_acc_Y,
		temporary[4] - ACC_PITCH_OFFSET);
Gyro_Acc[5] = FIR_Filter_filtering(&average_acc_Z,
		temporary[5] - ACC_YAW_OFFSET);
}
