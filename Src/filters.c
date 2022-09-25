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

#if defined(USE_FIR_FILTERS)
static FIR_Filter filter_gyro_X;
static FIR_Filter filter_gyro_Y;
static FIR_Filter filter_gyro_Z;
static FIR_Filter filter_acc_X;
static FIR_Filter filter_acc_Y;
static FIR_Filter filter_acc_Z;
#elif defined(USE_IIR_FILTERS)
static IIR_Filter filter_gyro_X;
static IIR_Filter filter_gyro_Y;
static IIR_Filter filter_gyro_Z;
static IIR_Filter filter_acc_X;
static IIR_Filter filter_acc_Y;
static IIR_Filter filter_acc_Z;
#endif

void FIR_Filter_Init(FIR_Filter *fir)
{

	//	Allocate memory for arrays
	fir->buffer = (float *)malloc(sizeof(fir->buffer) * fir->length);
	fir->impulse_response = (float *)malloc(
		sizeof(fir->impulse_response) * fir->length);

	//	Clear filter buffer
	for (uint8_t i = 0; i < fir->length; i++)
	{
		fir->buffer[i] = 0.f;
		fir->impulse_response[i] = 0.f;
	}

	//	Clear buffer index
	fir->buffer_index = 0;

	//	Clear filter output
	fir->output = 0.0f;
}

float FIR_Filter_filtering(FIR_Filter *fir, float input)
{
	//	Add new data to buffer
	fir->buffer[fir->buffer_index] = input;

	//	Increment buffer_index and loop if necessary
	fir->buffer_index++;
	if (fir->buffer_index == fir->length)
	{
		fir->buffer_index = 0;
	}
	//	Reset old output value
	fir->output = 0.f;

	uint8_t sum_index = fir->buffer_index;

	//	Decrement sum_index and loop if necessary
	for (uint8_t i = 0; i < fir->length; i++)
	{
		if (sum_index > 0)
		{
			sum_index--;
		}
		else
		{
			sum_index = fir->length - 1;
		}
		//	Compute new output (via convolution)
		fir->output += fir->impulse_response[i] * fir->buffer[sum_index];
	}
	return fir->output;
}

void IIR_Filter_Init(IIR_Filter *iir)
{
	//	Allocate memory for arrays
	iir->buffer_input = (float *)malloc(
		sizeof(iir->buffer_input) * (iir->filter_order + 1));
	iir->buffer_output = (float *)malloc(
		sizeof(iir->buffer_output) * (iir->filter_order));
	iir->forward_coefficients = (float *)malloc(
		sizeof(iir->forward_coefficients) * (iir->filter_order + 1));
	iir->feedback_coefficients = (float *)malloc(
		sizeof(iir->feedback_coefficients) * (iir->filter_order));

	//	Clear filter buffers and coefficients

	for (uint8_t i = 0; i < iir->filter_order + 1; i++)
	{
		// FORWARD PART:
		iir->buffer_input[i] = 0.f;
		iir->forward_coefficients[i] = 0.f;
		// FEEDBACK PART:
		iir->buffer_output[i] = 0.f;
		iir->feedback_coefficients[i] = 0.f;
	}

	//	Clear buffer index
	iir->buffer_index = 0;

	//	Clear filter output
	iir->output = 0.0f;
}

float IIR_Filter_filtering(IIR_Filter *iir, float input)
{
	//	Add new data to buffer_input:
	iir->buffer_input[iir->buffer_index] = input;

	//	Reset old output value:
	iir->output = 0.f;

	// save position of the latest sample:
	uint8_t sum_index = iir->buffer_index;

	//	FORWARD PART:
	//	Decrement sum_index and loop if necessary
	for (uint8_t i = 0; i < iir->filter_order + 1; i++)
	{
		//	Compute forward part of a new output:
		iir->output += iir->forward_coefficients[i] * iir->buffer_input[sum_index];

		if (sum_index > 0)
		{
			sum_index--;
		}
		else
		{
			sum_index = iir->filter_order;
		}
	}

	//	FEEDBACK PART:
	sum_index = iir->buffer_index;
	//	Decrement sum_index and loop if necessary
	for (uint8_t i = 0; i < iir->filter_order; i++)
	{
		if (sum_index > 0)
		{
			sum_index--;
		}
		else
		{
			sum_index = iir->filter_order - 1;
		}
		// Add feedback part of a new output
		iir->output -= iir->feedback_coefficients[i] * iir->buffer_output[sum_index];
	}

	//	Add new data to buffer_output
	iir->buffer_output[iir->buffer_index] = iir->output;

	//	Increment buffer_index and loop if necessary
	iir->buffer_index++;
	if (iir->buffer_index > iir->filter_order)
	{
		iir->buffer_index = 0;
	}

	return iir->output;
}

void Gyro_Acc_filters_setup()
{
#if defined(USE_FIR_FILTERS)

	//	X axis:
	filter_gyro_X.length = GYRO_FILTERS_ORDER;

	FIR_Filter_Init(&filter_gyro_X);

	for (uint8_t i = 0; i < filter_gyro_X.length; i++)
	{
		filter_gyro_X.impulse_response[i] = GYRO_FILTER_X_COEF[i];
	}

	//	Y axis:
	filter_gyro_Y.length = GYRO_FILTERS_ORDER;

	FIR_Filter_Init(&filter_gyro_Y);

	for (uint8_t i = 0; i < filter_gyro_Y.length; i++)
	{
		filter_gyro_Y.impulse_response[i] = GYRO_FILTER_Y_COEF[i];
	}

	//	Z axis:
	filter_gyro_Z.length = GYRO_FILTERS_ORDER;

	FIR_Filter_Init(&filter_gyro_Z);

	for (uint8_t i = 0; i < filter_gyro_Z.length; i++)
	{
		filter_gyro_Z.impulse_response[i] = GYRO_FILTER_Z_COEF[i];
	}

	//	X axis:
	filter_acc_X.length = ACC_FILTERS_ORDER;

	FIR_Filter_Init(&filter_acc_X);

	for (uint8_t i = 0; i < filter_acc_X.length; i++)
	{
		filter_acc_X.impulse_response[i] = ACC_FILTER_X_COEF[i];
	}

	//	Y axis:
	filter_acc_Y.length = ACC_FILTERS_ORDER;

	FIR_Filter_Init(&filter_acc_Y);

	for (uint8_t i = 0; i < filter_acc_Y.length; i++)
	{
		filter_acc_Y.impulse_response[i] = ACC_FILTER_Y_COEF[i];
	}

	//	Z axis:
	filter_acc_Z.length = ACC_FILTERS_ORDER;

	FIR_Filter_Init(&filter_acc_Z);

	for (uint8_t i = 0; i < filter_acc_Z.length; i++)
	{
		filter_acc_Z.impulse_response[i] = ACC_FILTER_Z_COEF[i];
	}

#elif defined(USE_IIR_FILTERS)

	//	X axis:
	filter_gyro_X.filter_order = GYRO_FILTERS_ORDER;

	IIR_Filter_Init(&filter_gyro_X);

	for (uint8_t i = 0; i < filter_gyro_X.filter_order + 1; i++)
	{
		filter_gyro_X.forward_coefficients[i] = GYRO_FILTER_X_FORW_COEF[i];
	}

	for (uint8_t i = 0; i < filter_gyro_X.filter_order; i++)
	{
		filter_gyro_X.feedback_coefficients[i] = GYRO_FILTER_X_BACK_COEF[i];
	}

	//	Y axis:
	filter_gyro_Y.filter_order = GYRO_FILTERS_ORDER;

	IIR_Filter_Init(&filter_gyro_Y);

	for (uint8_t i = 0; i < filter_gyro_Y.filter_order + 1; i++)
	{
		filter_gyro_Y.forward_coefficients[i] = GYRO_FILTER_Y_FORW_COEF[i];
	}

	for (uint8_t i = 0; i < filter_gyro_Y.filter_order; i++)
	{
		filter_gyro_Y.feedback_coefficients[i] = GYRO_FILTER_Y_BACK_COEF[i];
	}

	//	Z axis:
	filter_gyro_Z.filter_order = GYRO_FILTERS_ORDER;

	IIR_Filter_Init(&filter_gyro_Z);

	for (uint8_t i = 0; i < filter_gyro_Z.filter_order + 1; i++)
	{
		filter_gyro_Z.forward_coefficients[i] = GYRO_FILTER_Z_FORW_COEF[i];
	}

	for (uint8_t i = 0; i < filter_gyro_Z.filter_order; i++)
	{
		filter_gyro_Z.feedback_coefficients[i] = GYRO_FILTER_Y_BACK_COEF[i];
	}

	//	ACC MEASUREMENTS:

	//	X axis:
	filter_acc_X.filter_order = ACC_FILTERS_ORDER;

	IIR_Filter_Init(&filter_acc_X);

	for (uint8_t i = 0; i < filter_acc_X.filter_order + 1; i++)
	{
		filter_acc_X.forward_coefficients[i] = ACC_FILTER_X_FORW_COEF[i];
	}

	for (uint8_t i = 0; i < filter_acc_X.filter_order; i++)
	{
		filter_acc_X.feedback_coefficients[i] = ACC_FILTER_X_BACK_COEF[i];
	}

	//	Y axis:
	filter_acc_Y.filter_order = ACC_FILTERS_ORDER;

	IIR_Filter_Init(&filter_acc_Y);

	for (uint8_t i = 0; i < filter_acc_Y.filter_order + 1; i++)
	{
		filter_acc_Y.forward_coefficients[i] = ACC_FILTER_Y_FORW_COEF[i];
	}

	for (uint8_t i = 0; i < filter_acc_Y.filter_order; i++)
	{
		filter_acc_Y.feedback_coefficients[i] = ACC_FILTER_Y_BACK_COEF[i];
	}

	//	Z axis:
	filter_acc_Z.filter_order = ACC_FILTERS_ORDER;

	IIR_Filter_Init(&filter_acc_Z);

	for (uint8_t i = 0; i < filter_acc_Z.filter_order + 1; i++)
	{
		filter_acc_Z.forward_coefficients[i] = ACC_FILTER_Z_FORW_COEF[i];
	}

	for (uint8_t i = 0; i < filter_acc_Z.filter_order; i++)
	{
		filter_acc_Z.feedback_coefficients[i] = ACC_FILTER_Y_BACK_COEF[i];
	}

#endif
}

void Gyro_Acc_filtering(float *temporary)
{
#if defined(USE_FIR_FILTERS)
	Gyro_Acc[0] = FIR_Filter_filtering(&filter_gyro_X,
									   temporary[0] - GYRO_ROLL_OFFSET);
	Gyro_Acc[1] = FIR_Filter_filtering(&filter_gyro_Y,
									   temporary[1] - GYRO_PITCH_OFFSET);
	Gyro_Acc[2] = FIR_Filter_filtering(&filter_gyro_Z,
									   temporary[2] - GYRO_YAW_OFFSET);
	Gyro_Acc[3] = FIR_Filter_filtering(&filter_acc_X,
									   temporary[3] - ACC_ROLL_OFFSET);
	Gyro_Acc[4] = FIR_Filter_filtering(&filter_acc_Y,
									   temporary[4] - ACC_PITCH_OFFSET);
	Gyro_Acc[5] = FIR_Filter_filtering(&filter_acc_Z,
									   temporary[5] - ACC_YAW_OFFSET);

#elif defined(USE_IIR_FILTERS)

	Gyro_Acc[0] = IIR_Filter_filtering(&filter_gyro_X,
									   temporary[0] - GYRO_ROLL_OFFSET);
	Gyro_Acc[1] = IIR_Filter_filtering(&filter_gyro_Y,
									   temporary[1] - GYRO_PITCH_OFFSET);
	Gyro_Acc[2] = IIR_Filter_filtering(&filter_gyro_Z,
									   temporary[2] - GYRO_YAW_OFFSET);
	Gyro_Acc[3] = IIR_Filter_filtering(&filter_acc_X,
									   temporary[3] - ACC_ROLL_OFFSET);
	Gyro_Acc[4] = IIR_Filter_filtering(&filter_acc_Y,
									   temporary[4] - ACC_PITCH_OFFSET);
	Gyro_Acc[5] = IIR_Filter_filtering(&filter_acc_Z,
									   temporary[5] - ACC_YAW_OFFSET);
#endif
}
