/*
 * filters.c
 *
 *  Created on: 24.09.2021
 *      Author: symon
 */
#include "stm32f4xx.h"
#include <math.h>
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
static FIR_Filter filter_D_term;
#elif defined(USE_IIR_FILTERS)
static IIR_Filter filter_gyro_X;
static IIR_Filter filter_gyro_Y;
static IIR_Filter filter_gyro_Z;
static IIR_Filter filter_acc_X;
static IIR_Filter filter_acc_Y;
static IIR_Filter filter_acc_Z;
static IIR_Filter filter_D_term;
#elif defined(USE_BIQUAD_FILTERS)
static biquad_Filter_t filter_gyro_X;
static biquad_Filter_t filter_gyro_Y;
static biquad_Filter_t filter_gyro_Z;
static biquad_Filter_t filter_acc_X;
static biquad_Filter_t filter_acc_Y;
static biquad_Filter_t filter_acc_Z;
static biquad_Filter_t filter_D_term_roll;
static biquad_Filter_t filter_D_term_pitch;
static biquad_Filter_t filter_D_term_yaw;
#endif

void FIR_filter_init(FIR_Filter *fir)
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

float FIR_filter_apply(FIR_Filter *fir, float input)
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

void IIR_filter_init(IIR_Filter *iir)
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

float IIR_filter_apply(IIR_Filter *iir, float input)
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

void biquad_filter_init(biquad_Filter_t *filter, biquad_Filter_type filter_type, float filter_frequency_Hz, float quality_factor, uint16_t sampling_frequency_Hz)
{
	// save filter info:
	filter->frequency = filter_frequency_Hz;
	filter->Q_factor = quality_factor;
	const float omega = 2.f * M_PI * filter->frequency / sampling_frequency_Hz;
	const float sn = sinf(omega);
	const float cs = cosf(omega);
	const float alpha = sn / (2.0f * filter->Q_factor);
	// implementation from datasheet:https://www.ti.com/lit/an/slaa447/slaa447.pdf <-- not everything good
	// or even better: http://shepazu.github.io/Audio-EQ-Cookbook/audio-eq-cookbook.html <-- probably resource for above

	/*
	general formula for 2nd order filter:

			   b0*z^-2 + b1*z^-1 + b2
	H(z^-1) = ------------------------
			   a0*z^-2 + a1*z^-1 + a2

	main analog prototypes:

	Low-pass:
				 1
	H(s) = ---------------
			s^2 + s/Q + 1

	Notch:
			   s^2 + 1
	H(s) = ---------------
			s^2 + s/Q + 1

	BPF (skirt gain where Q is max. gain):
				 s
	H(s) = ---------------
			s^2 + s/Q + 1

	Q is quality factor
	In this datasheet conversion from s -> z domain is done with Bilinear transform with pre-warping
	*/

	const float a0 = 1 + alpha;
	switch (filter_type)
	{
	case BIQUAD_LPF:

		filter->b0 = filter->b1 * 0.5f;
		filter->b1 = 1 - cs;
		filter->b2 = filter->b0;
		filter->a1 = -2 * cs;
		filter->a2 = 1 - alpha;
		break;
	case BIQUAD_NOTCH:
		filter->b0 = 1;
		filter->b1 = -2 * cs;
		filter->b2 = 1;
		filter->a1 = filter->b1;
		filter->a2 = 1 - alpha;
		break;
	case BIQUAD_BPF:
		filter->b0 = alpha;
		filter->b1 = 0;
		filter->b2 = -alpha;
		filter->a1 = -2 * cs;
		filter->a2 = 1 - alpha;
		break;
	}

	// oust a0 coefficient:
	filter->b0 /= a0;
	filter->b1 /= a0;
	filter->b2 /= a0;
	filter->a1 /= a0;
	filter->a2 /= a0;

	// set previous values as 0:
	filter->x1 = 0;
	filter->x2 = 0;
	filter->y1 = 0;
	filter->y2 = 0;
}

float biquad_filter_apply_DF1(biquad_Filter_t *filter, float input)
{
	/* compute result */
	const float result = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2 - filter->a1 * filter->y1 - filter->a2 * filter->y2;

	/* shift x1 to x2, input to x1 */
	filter->x2 = filter->x1;
	filter->x1 = input;

	/* shift y1 to y2, result to y1 */
	filter->y2 = filter->y1;
	filter->y1 = result;

	return result;
}

float biquad_filter_apply_DF2(biquad_Filter_t *filter, float input)
{
	//	this is transposed direct form 2 is a little more precised for float number implementation:

	const float result = filter->b0 * input + filter->x1;

	filter->x1 = filter->b1 * input - filter->a1 * result + filter->x2;
	filter->x2 = filter->b2 * input - filter->a2 * result;

	return result;
}

void Gyro_Acc_filters_setup()
{
#if defined(USE_FIR_FILTERS)

	//	X axis:
	filter_gyro_X.length = GYRO_FILTERS_ORDER;

	FIR_filter_init(&filter_gyro_X);

	for (uint8_t i = 0; i < filter_gyro_X.length; i++)
	{
		filter_gyro_X.impulse_response[i] = GYRO_FILTER_X_COEF[i];
	}

	//	Y axis:
	filter_gyro_Y.length = GYRO_FILTERS_ORDER;

	FIR_filter_init(&filter_gyro_Y);

	for (uint8_t i = 0; i < filter_gyro_Y.length; i++)
	{
		filter_gyro_Y.impulse_response[i] = GYRO_FILTER_Y_COEF[i];
	}

	//	Z axis:
	filter_gyro_Z.length = GYRO_FILTERS_ORDER;

	FIR_filter_init(&filter_gyro_Z);

	for (uint8_t i = 0; i < filter_gyro_Z.length; i++)
	{
		filter_gyro_Z.impulse_response[i] = GYRO_FILTER_Z_COEF[i];
	}

	//	X axis:
	filter_acc_X.length = ACC_FILTERS_ORDER;

	FIR_filter_init(&filter_acc_X);

	for (uint8_t i = 0; i < filter_acc_X.length; i++)
	{
		filter_acc_X.impulse_response[i] = ACC_FILTER_X_COEF[i];
	}

	//	Y axis:
	filter_acc_Y.length = ACC_FILTERS_ORDER;

	FIR_filter_init(&filter_acc_Y);

	for (uint8_t i = 0; i < filter_acc_Y.length; i++)
	{
		filter_acc_Y.impulse_response[i] = ACC_FILTER_Y_COEF[i];
	}

	//	Z axis:
	filter_acc_Z.length = ACC_FILTERS_ORDER;

	FIR_filter_init(&filter_acc_Z);

	for (uint8_t i = 0; i < filter_acc_Z.length; i++)
	{
		filter_acc_Z.impulse_response[i] = ACC_FILTER_Z_COEF[i];
	}

#elif defined(USE_IIR_FILTERS)

	//	X axis:
	filter_gyro_X.filter_order = GYRO_FILTERS_ORDER;

	IIR_filter_init(&filter_gyro_X);

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

	IIR_filter_init(&filter_gyro_Y);

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

	IIR_filter_init(&filter_gyro_Z);

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

	IIR_filter_init(&filter_acc_X);

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

	IIR_filter_init(&filter_acc_Y);

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

	IIR_filter_init(&filter_acc_Z);

	for (uint8_t i = 0; i < filter_acc_Z.filter_order + 1; i++)
	{
		filter_acc_Z.forward_coefficients[i] = ACC_FILTER_Z_FORW_COEF[i];
	}

	for (uint8_t i = 0; i < filter_acc_Z.filter_order; i++)
	{
		filter_acc_Z.feedback_coefficients[i] = ACC_FILTER_Y_BACK_COEF[i];
	}

#elif defined(USE_BIQUAD_FILTERS)

	biquad_filter_init(&filter_gyro_X, BIQUAD_LPF, BIQUAD_LPF_CUTOFF, BIQUAD_LPF_Q, FREQUENCY_IMU_READING);
	biquad_filter_init(&filter_gyro_Y, BIQUAD_LPF, BIQUAD_LPF_CUTOFF, BIQUAD_LPF_Q, FREQUENCY_IMU_READING);
	biquad_filter_init(&filter_gyro_Z, BIQUAD_LPF, BIQUAD_LPF_CUTOFF, BIQUAD_LPF_Q, FREQUENCY_IMU_READING);

	biquad_filter_init(&filter_acc_X, BIQUAD_LPF, BIQUAD_LPF_CUTOFF, BIQUAD_LPF_Q, FREQUENCY_IMU_READING);
	biquad_filter_init(&filter_acc_Y, BIQUAD_LPF, BIQUAD_LPF_CUTOFF, BIQUAD_LPF_Q, FREQUENCY_IMU_READING);
	biquad_filter_init(&filter_acc_Z, BIQUAD_LPF, BIQUAD_LPF_CUTOFF, BIQUAD_LPF_Q, FREQUENCY_IMU_READING);

#endif
}

void Gyro_Acc_filtering(float *temporary)
{
#if defined(USE_FIR_FILTERS)
	Gyro_Acc[0] = FIR_filter_apply(&filter_gyro_X,
								   temporary[0] - GYRO_ROLL_OFFSET);
	Gyro_Acc[1] = FIR_filter_apply(&filter_gyro_Y,
								   temporary[1] - GYRO_PITCH_OFFSET);
	Gyro_Acc[2] = FIR_filter_apply(&filter_gyro_Z,
								   temporary[2] - GYRO_YAW_OFFSET);
	Gyro_Acc[3] = FIR_filter_apply(&filter_acc_X,
								   temporary[3] - ACC_ROLL_OFFSET);
	Gyro_Acc[4] = FIR_filter_apply(&filter_acc_Y,
								   temporary[4] - ACC_PITCH_OFFSET);
	Gyro_Acc[5] = FIR_filter_apply(&filter_acc_Z,
								   temporary[5] - ACC_YAW_OFFSET);

#elif defined(USE_IIR_FILTERS)

	Gyro_Acc[0] = IIR_filter_apply(&filter_gyro_X,
								   temporary[0] - GYRO_ROLL_OFFSET);
	Gyro_Acc[1] = IIR_filter_apply(&filter_gyro_Y,
								   temporary[1] - GYRO_PITCH_OFFSET);
	Gyro_Acc[2] = IIR_filter_apply(&filter_gyro_Z,
								   temporary[2] - GYRO_YAW_OFFSET);
	Gyro_Acc[3] = IIR_filter_apply(&filter_acc_X,
								   temporary[3] - ACC_ROLL_OFFSET);
	Gyro_Acc[4] = IIR_filter_apply(&filter_acc_Y,
								   temporary[4] - ACC_PITCH_OFFSET);
	Gyro_Acc[5] = IIR_filter_apply(&filter_acc_Z,
								   temporary[5] - ACC_YAW_OFFSET);

#elif defined(USE_BIQUAD_FILTERS)
	Gyro_Acc[0] = biquad_filter_apply_DF2(&filter_gyro_X, temporary[0] - GYRO_ROLL_OFFSET);
	Gyro_Acc[1] = biquad_filter_apply_DF2(&filter_gyro_Y, temporary[1] - GYRO_PITCH_OFFSET);
	Gyro_Acc[2] = biquad_filter_apply_DF2(&filter_gyro_Z, temporary[2] - GYRO_YAW_OFFSET);

	Gyro_Acc[3] = biquad_filter_apply_DF2(&filter_acc_X, temporary[3] - ACC_ROLL_OFFSET);
	Gyro_Acc[4] = biquad_filter_apply_DF2(&filter_acc_Y, temporary[4] - ACC_PITCH_OFFSET);
	Gyro_Acc[5] = biquad_filter_apply_DF2(&filter_acc_Z, temporary[5] - ACC_YAW_OFFSET);
#endif
}

void D_term_lowpass_setup()
{
#if defined(USE_FIR_FILTERS)

#elif defined(USE_IIR_FILTERS)
	filter_D_term_pitch.filter_order = D_TERM_FILTER_ORDER;
	IIR_filter_init(&filter_D_term_pitch);
	for (uint8_t i = 0; i < filter_D_term_pitch.filter_order + 1; i++)
	{
		filter_D_term_pitch.forward_coefficients[i] = D_TERM_FILTER_FORW_COEF[i];
	}

	for (uint8_t i = 0; i < filter_D_term_pitch.filter_order; i++)
	{
		filter_D_term_pitch.feedback_coefficients[i] = D_TERM_FILTER_BACK_COEF[i];
	}

	filter_D_term_roll.filter_order = D_TERM_FILTER_ORDER;
	IIR_filter_init(&filter_D_term_roll);
	for (uint8_t i = 0; i < filter_D_term_roll.filter_order + 1; i++)
	{
		filter_D_term_roll.forward_coefficients[i] = D_TERM_FILTER_FORW_COEF[i];
	}

	for (uint8_t i = 0; i < filter_D_term_roll.filter_order; i++)
	{
		filter_D_term_roll.feedback_coefficients[i] = D_TERM_FILTER_BACK_COEF[i];
	}

	filter_D_term_yaw.filter_order = D_TERM_FILTER_ORDER;
	IIR_filter_init(&filter_D_term_yaw);
	for (uint8_t i = 0; i < filter_D_term_yaw.filter_order + 1; i++)
	{
		filter_D_term_yaw.forward_coefficients[i] = D_TERM_FILTER_FORW_COEF[i];
	}

	for (uint8_t i = 0; i < filter_D_term_yaw.filter_order; i++)
	{
		filter_D_term_yaw.feedback_coefficients[i] = D_TERM_FILTER_BACK_COEF[i];
	}

#elif defined(USE_BIQUAD_FILTERS)
	biquad_filter_init(&filter_D_term_pitch, BIQUAD_LPF, BIQUAD_LPF_CUTOFF, BIQUAD_LPF_Q, FREQUENCY_MAIN_LOOP);
	biquad_filter_init(&filter_D_term_roll, BIQUAD_LPF, BIQUAD_LPF_CUTOFF, BIQUAD_LPF_Q, FREQUENCY_MAIN_LOOP);
	biquad_filter_init(&filter_D_term_yaw, BIQUAD_LPF, BIQUAD_LPF_CUTOFF, BIQUAD_LPF_Q, FREQUENCY_MAIN_LOOP);
#endif
}

void D_term_filtering(ThreeF *input)
{
#if defined(USE_FIR_FILTERS)
	input->pitch = FIR_filter_apply(&filter_D_term_pitch, input->pitch);
	input->roll = FIR_filter_apply(&filter_D_term_roll, input->roll);
	input->yaw = FIR_filter_apply(&filter_D_term_yaw, input->yaw);
#elif defined(USE_IIR_FILTERS)
	input->pitch = IIR_filter_apply(&filter_D_term_pitch, input->pitch);
	input->roll = IIR_filter_apply(&filter_D_term_roll, input->roll);
	input->yaw = IIR_filter_apply(&filter_D_term_yaw, input->yaw);
#elif defined(USE_BIQUAD_FILTERS)
	input->pitch = IIR_filter_apply(&filter_D_term_pitch, input->pitch);
	input->roll = IIR_filter_apply(&filter_D_term_roll, input->roll);
	input->yaw = IIR_filter_apply(&filter_D_term_yaw, input->yaw);
#endif
}