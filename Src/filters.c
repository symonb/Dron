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
#include"stdlib.h"
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
#if defined(USE_RPM_FILTER_GYRO)
static RPM_filter_t rpm_filter_gyro;
#endif
#if defined(USE_RPM_FILTER_ACC)
static RPM_filter_t rpm_filter_acc;
#endif

static void biquad_filter_update(biquad_Filter_t* filter, biquad_Filter_type filter_type, float filter_frequency_Hz, float quality_factor, uint16_t sampling_frequency_Hz);
#if defined(USE_RPM_FILTER_GYRO)||defined(USE_RPM_FILTER_ACC)
static void biquad_filter_copy_coefficients(biquad_Filter_t* copy_from_filter, biquad_Filter_t* copy_to_filter);
static void RPM_filter_update(RPM_filter_t* filter);
#endif
void FIR_filter_init(FIR_Filter* fir)
{

	//	Allocate memory for arrays
	fir->buffer = (float*)malloc(sizeof(*(fir->buffer)) * fir->length);
	fir->impulse_response = (float*)malloc(
		sizeof(*(fir->impulse_response)) * fir->length);

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

float FIR_filter_apply(FIR_Filter* fir, float input)
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

void IIR_filter_init(IIR_Filter* iir)
{
	//	Allocate memory for arrays
	iir->buffer_input = (float*)malloc(
		sizeof(*(iir->buffer_input)) * (iir->filter_order + 1));
	iir->buffer_output = (float*)malloc(
		sizeof(*(iir->buffer_output)) * (iir->filter_order));
	iir->forward_coefficients = (float*)malloc(
		sizeof(*(iir->forward_coefficients)) * (iir->filter_order + 1));
	iir->feedback_coefficients = (float*)malloc(
		sizeof(*(iir->feedback_coefficients)) * (iir->filter_order));

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

float IIR_filter_apply(IIR_Filter* iir, float input)
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

void biquad_filter_init(biquad_Filter_t* filter, biquad_Filter_type filter_type, float filter_frequency_Hz, float quality_factor, uint16_t sampling_frequency_Hz)
{
	biquad_filter_update(filter, filter_type, filter_frequency_Hz, quality_factor, sampling_frequency_Hz);

	// set previous values as 0:
	filter->x1 = 0;
	filter->x2 = 0;
	filter->y1 = 0;
	filter->y2 = 0;
}

static void biquad_filter_update(biquad_Filter_t* filter, biquad_Filter_type filter_type, float filter_frequency_Hz, float quality_factor, uint16_t sampling_frequency_Hz)
{
	// save filter info:
	filter->frequency = filter_frequency_Hz;
	filter->Q_factor = quality_factor;
	const float omega = 2.f * M_PI * filter_frequency_Hz / sampling_frequency_Hz;
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

	filter->a0 = 1 + alpha;
	switch (filter_type)
	{
	case BIQUAD_LPF:

		filter->b0 = (1 - cs) * 0.5f;
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
	filter->b0 /= filter->a0;
	filter->b1 /= filter->a0;
	filter->b2 /= filter->a0;
	filter->a1 /= filter->a0;
	filter->a2 /= filter->a0;
}

float biquad_filter_apply_DF1(biquad_Filter_t* filter, float input)
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

float biquad_filter_apply_DF2(biquad_Filter_t* filter, float input)
{
	//	this is transposed direct form 2 is a little more precised for float number implementation:

	const float result = filter->b0 * input + filter->x1;

	filter->x1 = filter->b1 * input - filter->a1 * result + filter->x2;
	filter->x2 = filter->b2 * input - filter->a2 * result;

	return result;
}

#if defined(USE_RPM_FILTER_GYRO)||defined(USE_RPM_FILTER_ACC)
static void biquad_filter_copy_coefficients(biquad_Filter_t* copy_from_filter, biquad_Filter_t* copy_to_filter)
{
	copy_to_filter->a0 = copy_from_filter->a0;
	copy_to_filter->a1 = copy_from_filter->a1;
	copy_to_filter->a2 = copy_from_filter->a2;

	copy_to_filter->b0 = copy_from_filter->b0;
	copy_to_filter->b1 = copy_from_filter->b1;
	copy_to_filter->b2 = copy_from_filter->b2;
}

void RPM_filter_init(RPM_filter_t* filter, uint16_t sampling_frequency_Hz)
{
	filter->harmonics = RPM_MAX_HARMONICS;
	filter->q_factor = RPM_Q_FACTOR;
	filter->frequency = sampling_frequency_Hz;
	filter->max_filtered_frequency = 0.48 * sampling_frequency_Hz;
	const float default_freq = 100; // only for initialization doesn't really matter

	// initialize notch filters:
	for (uint8_t axis = 0; axis < 3; axis++)
	{
		for (uint8_t motor = 0; motor < MOTORS_COUNT; motor++)
		{
			for (uint8_t harmonic = 0; harmonic < RPM_MAX_HARMONICS; harmonic++)
			{
				biquad_filter_init(&(filter->notch_filters[axis][motor][harmonic]), BIQUAD_NOTCH, default_freq, filter->q_factor, sampling_frequency_Hz);
				filter->weight[axis][motor][harmonic] = 1;
			}
		}
	}
}

static void RPM_filter_update(RPM_filter_t* filter)
{
	const uint8_t sec_in_min = 60; // for conversion from Hz to rpm
	float frequency;			   // frequency for filtering
	// each motor introduces its own frequency (with harmonics) but for every axes noises are the same;

	for (uint8_t motor = 0; motor < MOTORS_COUNT; motor++)
	{
		for (uint8_t harmonic = 0; harmonic < RPM_MAX_HARMONICS; harmonic++)
		{
			frequency = (float)motors_rpm[motor] * (harmonic + 1) / sec_in_min;
			if (frequency > RPM_MIN_FREQUENCY_HZ)
			{
				if (frequency < filter->max_filtered_frequency)
				{
					// each axis has the same noises from motors, so compute it once and next copy values:
					biquad_filter_update(&(filter->notch_filters[0][motor][harmonic]), BIQUAD_NOTCH, frequency, filter->q_factor, filter->frequency);
					biquad_filter_copy_coefficients(&(filter->notch_filters[0][motor][harmonic]), &(filter->notch_filters[1][motor][harmonic]));
					biquad_filter_copy_coefficients(&(filter->notch_filters[0][motor][harmonic]), &(filter->notch_filters[2][motor][harmonic]));

					// fade out if reaching minimal frequency:
					if (frequency < (RPM_MIN_FREQUENCY_HZ + RPM_FADE_RANGE_HZ))
					{
						filter->weight[0][motor][harmonic] = (float)(frequency - RPM_MIN_FREQUENCY_HZ) / (RPM_FADE_RANGE_HZ);
						filter->weight[1][motor][harmonic] = filter->weight[0][motor][harmonic];
						filter->weight[2][motor][harmonic] = filter->weight[0][motor][harmonic];
					}
					else
					{
						filter->weight[0][motor][harmonic] = 1;
						filter->weight[1][motor][harmonic] = 1;
						filter->weight[2][motor][harmonic] = 1;
					}
				}
				else
				{
					filter->weight[0][motor][harmonic] = 0;
					filter->weight[1][motor][harmonic] = 0;
					filter->weight[2][motor][harmonic] = 0;
				}
			}
			else
			{
				filter->weight[0][motor][harmonic] = 0;
				filter->weight[1][motor][harmonic] = 0;
				filter->weight[2][motor][harmonic] = 0;
			}
		}
	}
}
#endif

float RPM_filter_apply(RPM_filter_t* filter, uint8_t axis, float input)
{
	float result = input;
	// Iterate over all notches on axis and apply each one to value.
	// Order of application doesn't matter because biquads are linear time-invariant filters.
	for (uint8_t motor = 0; motor < MOTORS_COUNT; motor++)
	{
		for (uint8_t harmonic = 0; harmonic < RPM_MAX_HARMONICS; harmonic++)
		{
			result = filter->weight[axis][motor][harmonic] * biquad_filter_apply_DF1(&(filter->notch_filters[axis][motor][harmonic]), result) + (1 - filter->weight[axis][motor][harmonic]) * result;
		}
	}

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

	biquad_filter_init(&filter_gyro_X, BIQUAD_LPF, BIQUAD_LPF_CUTOFF, BIQUAD_LPF_Q, FREQUENCY_MAIN_LOOP);
	biquad_filter_init(&filter_gyro_Y, BIQUAD_LPF, BIQUAD_LPF_CUTOFF, BIQUAD_LPF_Q, FREQUENCY_MAIN_LOOP);
	biquad_filter_init(&filter_gyro_Z, BIQUAD_LPF, BIQUAD_LPF_CUTOFF, BIQUAD_LPF_Q, FREQUENCY_MAIN_LOOP);

	biquad_filter_init(&filter_acc_X, BIQUAD_LPF, BIQUAD_LPF_CUTOFF, BIQUAD_LPF_Q, FREQUENCY_STABILIZATION_LOOP);
	biquad_filter_init(&filter_acc_Y, BIQUAD_LPF, BIQUAD_LPF_CUTOFF, BIQUAD_LPF_Q, FREQUENCY_STABILIZATION_LOOP);
	biquad_filter_init(&filter_acc_Z, BIQUAD_LPF, BIQUAD_LPF_CUTOFF, BIQUAD_LPF_Q, FREQUENCY_STABILIZATION_LOOP);

#endif

#if defined(USE_RPM_FILTER_GYRO)
	RPM_filter_init(&rpm_filter_gyro, FREQUENCY_MAIN_LOOP);
#endif
#if defined(USE_RPM_FILTER_ACC)
	RPM_filter_init(&rpm_filter_acc, FREQUENCY_STABILIZATION_LOOP);
#endif
}

void gyro_filtering(const float* temporary)
{
#if defined(USE_FIR_FILTERS)
	Gyro_Acc[0] = FIR_filter_apply(&filter_gyro_X,
		temporary[0] - gyro_1.offset.roll);
	Gyro_Acc[1] = FIR_filter_apply(&filter_gyro_Y,
		temporary[1] - gyro_1.offset.pitch);
	Gyro_Acc[2] = FIR_filter_apply(&filter_gyro_Z,
		temporary[2] - gyro_1.offset.yaw);

#elif defined(USE_IIR_FILTERS)

	Gyro_Acc[0] = IIR_filter_apply(&filter_gyro_X,
		temporary[0] - gyro_1.offset.roll);
	Gyro_Acc[1] = IIR_filter_apply(&filter_gyro_Y,
		temporary[1] - gyro_1.offset.pitch);
	Gyro_Acc[2] = IIR_filter_apply(&filter_gyro_Z,
		temporary[2] - gyro_1.offset.yaw);

#elif defined(USE_BIQUAD_FILTERS)
	Gyro_Acc[0] = biquad_filter_apply_DF2(&filter_gyro_X, temporary[0] - gyro_1.offset.roll);
	Gyro_Acc[1] = biquad_filter_apply_DF2(&filter_gyro_Y, temporary[1] - gyro_1.offset.pitch);
	Gyro_Acc[2] = biquad_filter_apply_DF2(&filter_gyro_Z, temporary[2] - gyro_1.offset.yaw);
#endif
#if defined(USE_RPM_FILTER_GYRO)

	// update coefficients:
	RPM_filter_update(&rpm_filter_gyro);
	// next apply rpm filtering:
	Gyro_Acc[0] = RPM_filter_apply(&rpm_filter_gyro, 0, Gyro_Acc[0]);
	Gyro_Acc[1] = RPM_filter_apply(&rpm_filter_gyro, 1, Gyro_Acc[1]);
	Gyro_Acc[2] = RPM_filter_apply(&rpm_filter_gyro, 2, Gyro_Acc[2]);
#endif
}

void acc_filtering(const float* temporary)
{
#if defined(USE_FIR_FILTERS)
	Gyro_Acc[3] = FIR_filter_apply(&filter_acc_X,
		temporary[0] - ACC_ROLL_OFFSET);
	Gyro_Acc[4] = FIR_filter_apply(&filter_acc_Y,
		temporary[1] - ACC_PITCH_OFFSET);
	Gyro_Acc[5] = FIR_filter_apply(&filter_acc_Z,
		temporary[2] - ACC_YAW_OFFSET);

#elif defined(USE_IIR_FILTERS)

	Gyro_Acc[3] = IIR_filter_apply(&filter_acc_X,
		temporary[0] - ACC_ROLL_OFFSET);
	Gyro_Acc[4] = IIR_filter_apply(&filter_acc_Y,
		temporary[1] - ACC_PITCH_OFFSET);
	Gyro_Acc[5] = IIR_filter_apply(&filter_acc_Z,
		temporary[2] - ACC_YAW_OFFSET);

#elif defined(USE_BIQUAD_FILTERS)
	Gyro_Acc[3] = biquad_filter_apply_DF2(&filter_acc_X, temporary[0] - ACC_ROLL_OFFSET);
	Gyro_Acc[4] = biquad_filter_apply_DF2(&filter_acc_Y, temporary[1] - ACC_PITCH_OFFSET);
	Gyro_Acc[5] = biquad_filter_apply_DF2(&filter_acc_Z, temporary[2] - ACC_YAW_OFFSET);
#endif
#if defined(USE_RPM_FILTER_ACC)

	// update coefficients:
	RPM_filter_update(&rpm_filter_acc);
	// next apply rpm filtering:
	Gyro_Acc[3] = RPM_filter_apply(&rpm_filter_acc, 0, Gyro_Acc[3]);
	Gyro_Acc[4] = RPM_filter_apply(&rpm_filter_acc, 1, Gyro_Acc[4]);
	Gyro_Acc[5] = RPM_filter_apply(&rpm_filter_acc, 2, Gyro_Acc[5]);

#endif
}

void setup_D_term_filters()
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

void D_term_filtering(ThreeF* input)
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
	input->pitch = biquad_filter_apply_DF2(&filter_D_term_pitch, input->pitch);
	input->roll = biquad_filter_apply_DF2(&filter_D_term_roll, input->roll);
	input->yaw = biquad_filter_apply_DF2(&filter_D_term_yaw, input->yaw);
#endif
}
