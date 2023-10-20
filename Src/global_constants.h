/*
 * global_constants.h
 *
 *  Created on: 18.04.2021
 *      Author: symon
 */

#ifndef GLOBAL_CONSTANTS_H_
#define GLOBAL_CONSTANTS_H_
#include <stdbool.h>

 //------DIFFERENT_CONSTANTS------------
#define MAX(a, b) ((a > b) ? a : b)	 // choose greater value
#define MIN(a, b) ((a < b) ? a : b)
#define GYRO_ACC_SIZE 7				 // 3 for gyro, 3 acc and 1 for temperature
#define ALL_ELEMENTS_TO_SEND 14		 // telemetry information different values
#define GYRO_TO_DPS (2000.f / 32767.f) // convert gyro register into degrees per second unit
#define RAD_TO_DEG (180.f / M_PI)
#define DEG_TO_RAD (M_PI / 180.f)
#define GYRO_TO_RAD (1.f / 32.767f * DEG_TO_RAD)  // convert gyro register into rad/second unit
#define ACC_TO_GRAVITY (1.f / 4096.f)				  // convert acc register into gravity unit
#define SPI_TIMEOUT_VALUE 0.01f						  // time for SPI_failsafe activation [s]
#define I2C_TIMEOUT_VALUE 0.0001f						  // time for I2C_failsafe activation [s]
#define TASK_PERIOD_KHZ(kHz) (1000000000 / (kHz)) // converts frequency in [kHz] into period in [us]
#define TASK_PERIOD_HZ(Hz) (1000000 / (Hz))		  // converts frequency in [Hz] into [us]
#define SEC_TO_US(s) ((s)*1000000)				  // converts [s] into [us]
#define MS_TO_US(s) ((s)*1000)					  // converts [ms] into [us]
#define US_TO_SEC(s) ((s) / 1000000.f)			  // converts [us] into [s]

//-------------BATTERY------------
#define BATTERY_SCALE 11
#define ADC_REFERENCE_VOLTAGE 3.3f	  //	[V]
#define BATTERY_CELL_MIN_VOLTAGE 3.4f //	[V]
#define BATTERY_CELL_WARNING_VOLTAGE 3.55f //	[V]
#define BATTERY_CELL_MAX_VOLTAGE 4.2f //	[V]
#define USE_BATTERY_SAG_COMPENSATION

//-------------BUZZER-------------
#define BUZZER_TIME_ON 0.2f	//	[s]
#define BUZZER_TIME_OFF 0.8f //	[s]

//-----------RX_SETTINGS----------
#define CHANNELS 10 // how many channels you want use (4 for steering; 14 is max)
#define MAX_RX_SIGNAL 2050
#define MIN_RX_SIGNAL 950
#define MAX_NO_SIGNAL_TIME 1	  // 	time after RX_failsafe occurs [s]
#define ARM_VALUE 1600			  //	value of arming switch to arm drone
#define PREARM_VALUE 1600		  //	value of prearming switch to prearm drone
#define MAX_ARM_THROTTLE_VAL 1100 //	maximum value of throttle that allow for arming
#define USE_PREARM				  //	if you want use PREARM define USE_PREARM
#define PREARM_CHANNEL 8		  //	and PREARM_CHANNEL
#define ARM_CHANNEL 4
#define FLIGHT_MODE_CHANNEL 6
#define BLACKBOX_CHANNEL 7 //	switch to turn on blackbox data collection

// ---------------RATES-----------------
#define RATES_USE_NO_EXPO // if you want use expo or not RATES_USE_EXPO/RATES_USE_NO_EXPO

#if defined(RATES_USE_EXPO)
/* Rates description:
similar to Betaflight 4.X Actual Rates but not identical
x - stick position <-1; 1> (normalized deflection from center of the stick)
y - output <0; max> (no more than 2000)
a - Max value		 <0; 2000>	maximal rotation speed (+-)
b - Center rate		 <0; 1000>	define central part of the scope
c - Expo			 <0; 1>		if you want add more curve

y = x*b + (c*x^5 + x^3*(1-c))*(a-b)

notes:
you can make linear dependence (a=b)
but in general it is curved even with c = 0
*/
#define RATES_MAX_RATE_P 900	// <0; 2000>
#define RATES_CENTER_RATE_P 270 // <1; 1000>
#define RATES_EXPO_P 0.55f		// <0; 1>

#define RATES_MAX_RATE_R 900	// <0; 2000>
#define RATES_CENTER_RATE_R 270 // <1; 1000>
#define RATES_EXPO_R 0.55f		// <0; 1>

#define RATES_MAX_RATE_Y 650	// <0; 2000>
#define RATES_CENTER_RATE_Y 300 // <1; 1000>
#define RATES_EXPO_Y 0.42f		// <0; 1>

#elif defined(RATES_USE_NO_EXPO)
/* Rates description:
same as above with expo = 0;
x - stick position <-1; 1> (normalized deflection from center of the stick)
y - output <0; max> (no more than 2000)
a - Max value		 <0; 2000>	maximal rotation speed (+-)
b - Center rate		 <0; 1000>	define central part of the scope

y = x*b + x^3*(a-b)

notes:
linear dependance for a=b otherwise curve (max for b = 0)
*/

#define RATES_MAX_RATE_P 750	// <0; 2000>
#define RATES_CENTER_RATE_P 270 // <1; 1000>
#define RATES_EXPO_P 0

#define RATES_MAX_RATE_R 750	// <0; 2000>
#define RATES_CENTER_RATE_R 270 // <1; 1000>
#define RATES_EXPO_R 0

#define RATES_EXPO_Y 0
#define RATES_MAX_RATE_Y 650	// <0; 2000>
#define RATES_CENTER_RATE_Y 290 // <1; 1000>

#define THROTTLE_MIN 1000
#define THROTTLE_MAX 2000

#endif

//------------ESC_PROTOCOLS----------
#define ESC_PROTOCOL_BDSHOT // ESC_PROTOCOL_PWM, ESC_PROTOCOL_ONESHOT125, ESC_PROTOCOL_BDSHOT, ESC_PROTOCOL_DSHOT, ESC_PROTOCOL_DSHOT_BURST (not ended yet)
#if defined(ESC_PROTOCOL_BDSHOT)
#define BIT_BANGING_V1
#endif
#define DSHOT_MODE 300 // 150 300 600 1200

#define DSHOT_BUFFER_LENGTH 18 // 16 bits of Dshot and 2 for clearing
#define DSHOT_PWM_FRAME_LENGTH 35
#define DSHOT_1_LENGTH 26
#define DSHOT_0_LENGTH 13

#if defined(BIT_BANGING_V1)
#define DSHOT_BB_BUFFER_LENGTH 18  // 16 bits of Dshot and 2 for clearing - used when bit-banging dshot used
#define DSHOT_BB_FRAME_LENGTH 140  //	how many counts of timer gives one bit frame
#define DSHOT_BB_FRAME_SECTIONS 7 // in how many sections is bit frame divided (must be factor of DSHOT_BB_FRAME_LENGTH)
#define DSHOT_BB_1_LENGTH 5
#define DSHOT_BB_0_LENGTH 2
#define BDSHOT_RESPONSE_LENGTH 21
#define BDSHOT_RESPONSE_BITRATE (DSHOT_MODE * 4 / 3) // in my tests this value was not 5/4 * DSHOT_MODE as documentation suggests but 4/3*DSHOT_MODE
#define BDSHOT_RESPONSE_OVERSAMPLING 3				 // it has to be a factor of DSHOT_BB_FRAME_LENGTH * DSHOT_MODE / BDSHOT_RESPONSE_BITRATE

#elif defined(BIT_BANGING_V2)
#define DSHOT_BB_BUFFER_LENGTH 18 // 16 bits of Dshot and 2 for clearing - used when bit-banging dshot used
#define DSHOT_BB_FRAME_LENGTH 35
#define DSHOT_BB_1_LENGTH 26
#define DSHOT_BB_0_LENGTH 13
#define DSHOT_BB_FRAME_SECTIONS 3
#define BDSHOT_RESPONSE_BUFFER_LENGTH 21
#endif

//----------MOTORS_AND_CORRECTIONS-------
#define TPA_MAX_VALUE 0.5f		// Throttle PID atennuation (when full throttle PID values are 0.7 of original values (if this is set to 0.3))
#define TPA_BREAKPOINT 1650	// throttle value when attenuation begins (when throttle vary between 1000-2000 attenuation will start at 1500 and increase up to 2000)
#define MAX_I_CORRECTION 200 // maximal I_corr for PIDs between <0;1000>
#define MOTOR_OUTPUT_MIN 1040
#define MOTOR_OUTPUT_MAX 2000
#define MOTORS_COUNT 4		  // how many motors are used
#define MOTOR_1 3			  // PA3
#define MOTOR_2 0			  // PB0
#define MOTOR_3 1			  // PB1
#define MOTOR_4 2			  // PA2
#define MOTOR_POLES_NUMBER 14 // how many poles have your motors (usually 14 or 12)

//----------STABILIZE_SETINGS------
#define MAX_ROLL_ANGLE 45
#define MAX_PITCH_ANGLE 45

#define STABILIZE_FILTER_MAGDWICK // STABILIZE_FILTER_MAGDWICK, STABILIZE_FILTER_MAHONY, STABILIZE_FILTER_COMPLEMENTARY
#define MAGDWICK_NEW			  // MAGDWICK_ORIGINAL, MAGDWICK_NEW

#define GYRO_PART 0.995f // complementary filter
#define ACC_PART 0.005f	 // complementary filter

//---------ALTITUDE_HOLDING----------
#define THR_ALT_P 1
#define THROTTLE_HOLD_LOW 1300			// min throttle value for which altitude will be hold (range = range_up - range_low)
#define THROTTLE_HOLD_UP 1700			// max throttle value for which altitude will be hold (range = range_up - range_low)
#define MAX_RISING_RATE  1.5f			// [m/s]
#define MAX_DECLINE_RATE  1.5f			// [m/s]


//---------FREQUENCY_SETTINGS--------
#define FREQUENCY_MAIN_LOOP 2000			//	[Hz]   IF YOU'RE USING PWM MAX is 500[Hz] (little less), IF DSHOT you can go up to 8 [kHz]
#define FREQUENCY_STABILIZATION_LOOP 200 	//	[Hz]
#define FREQUENCY_ACC_SAMPLING	1000		//	[Hz]
#define FREQUENCY_ESC_UPDATE 2000		 	//	[Hz]	SHOULD BE AT LEAST AS MAIN_LOOP
#define FREQUENCY_RX_READING 200		 	//	[Hz]	set more than it is possible (>150)
#define FREQUENCY_TELEMETRY_UPDATE 1	 	//	[Hz]
#define FREQUENCY_SYSTEM_CHECK 	30	 		//	[Hz]
#define FREQUENCY_OSD_UPDATE 30			 	//	[Hz]
#define FREQUENCY_USB_CHECK 50				//	[Hz]
#define FREQUENCY_BLACKBOX FREQUENCY_MAIN_LOOP/(1<<BLACKBOX_SAMPLE_RATE)				//	[Hz]
#define FREQUENCY_BARO 100					//	[Hz]
#define FREQUENCY_ALT_HOLD 50				//	[Hz]

//--------------------FLASH-------------------
#define USE_FLASHFS	// USE_FLASHFS required for USB MSC and BLACKBOX  

//--------------BLACKBOX-------------------
#define USE_BLACKBOX // USE_BLACKBOX if you want BLACKBOX
/* define which parameters you would like to save:
 * they would be saved as debug parameters
 * BLACKBOX_DEBUG_GYRO_RAW				- 3 values save gyro values without filtering
 * BLACKBOX_DEBUG_ACC_RAW				- 3 values save acc values without filtering
 * BLACKBOX_DEBUG_CHANNELS_RAW			- 4 values save raw channels values without filtering
 * BLACKBOX_DEBUG_BARO					- 4 values PID values + baro.vel
 * BLACKBOX_DEBUG_RPM_ERR				- 4 values bdshot rpm error * 1000
 * BLACKBOX_DEBUG_ATTITIUDE				- 3 Euler angles roll, pitch, yaw * 100 [deg]
 */

#define BLACKBOX_DEBUG_BARO	
#define BLACKBOX_SAMPLE_RATE 1		//	sampling is 1/2^BLACKBOX_SAMPLE_RATE so for 0 it is equal to main PID loop frequency, for half frequency of main loop set this as 1

#define REQUIRE_CC_ARM_PRINTF_SUPPORT
#define REQUIRE_PRINTF_LONG_SUPPORT

typedef enum
{
	BLACKBOX_IDLE,
	BLACKBOX_STARTED,
	BLACKBOX_COLLECTING_DATA,
	BLACKBOX_SUSPENDED,
	BLACKBOX_STOPPED
}blackbox_e;

//---------------------OSD--------------------

#define OSD_BATTERY_VOLTAGE_PLACEMENT 361
#define OSD_BATTERY_CELL_VOLTAGE_PLACEMENT 370
#define OSD_TIMER_PLACEMENT 380
#define OSD_FLIGHT_MODE_PLACEMENT 23
#define OSD_LOGO_PLACEMENT 123
#define OSD_WARNING_PLACEMENT 315
#define OSD_POINTER_PLACEMENT 195
#define OSD_AUTO_NTSC_PAL //	options:	OSD_AUTO_NTSC_PAL/OSD_CAMERA_PAL/OSD_CAMERA_NTSC
#define OSD_WARNINGS_TIME 1 // [s] time of showing warnings (after that it will be cleared)

//-----------------I2C----------------
#define USE_I2C1


//------------------SENSORS-------------------
#define USE_BARO
// how is IMU mounted refering to drone axes 
// imagine all rotations required to align sensor axes with drone axes
// you rotate always in sensor frame 
// first rotate along sensor Z axis
// next rotate along sensor Y axis 
// at the end rotate along sensor X axis
#define ACC_GYRO_MOUNT_Z 180	
#define ACC_GYRO_MOUNT_Y 0
#define ACC_GYRO_MOUNT_X 0
// #define USE_MAG

//-------------------FILTERS------------------
#define USE_BIQUAD_FILTERS										  // USE_FIR_FILTERS or USE_IIR_FILTERS or USE_BIQUAD_FILTERS
#if defined(USE_FIR_FILTERS) || defined(USE_IIR_FILTERS)
#define GYRO_FILTERS_ORDER 2
#define ACC_FILTERS_ORDER 2
#define D_TERM_FILTER_ORDER 2
#endif

#define USE_RC_SMOOTHING	// if you want use rc smoothing USE_RC_SMOOTHING

#define BIQUAD_LPF_CUTOFF_GYRO 90	    //	[Hz]
#define BIQUAD_LPF_CUTOFF_ACC 10		//	[Hz]
#define BIQUAD_LPF_CUTOFF_D_TERM 90	//	[Hz]
#define BIQUAD_LPF_CUTOFF_FF_TERM 120	//  [Hz]
#define BIQUAD_LPF_CUTOFF_RC 70			//  [Hz]

#define BIQUAD_LPF_Q (1.f / sqrtf(2)) 	//	Q factor for Low-pass filters
#define BIQUAD_NOTCH_Q 10				//	Q factor for notch filters


// RPM filter:
#if defined(ESC_PROTOCOL_BDSHOT)
#define USE_RPM_FILTER_GYRO // if you want use RPM_filter for gyro	USE_RPM_FILTER_GYRO
// #define USE_RPM_FILTER_ACC	// if you want use RPM_filter for acc USE_RPM_FILTER_ACC
#endif

#define RPM_MIN_FREQUENCY_HZ 100 // all frequencies <= than this value will not be filtered by RPM filter
#define RPM_FADE_RANGE_HZ 50	// fade out notch when approaching RPM_MIN_FREQUENCY_HZ (turn it off for RPM_MIN_FREQUENCY_HZ)
#define RPM_Q_FACTOR 500		// Q factor for all notches. It is VERY HIGH therefore notches are really narrow and selective
#define RPM_MAX_HARMONICS 3		// max. number of filtered harmonics

//	remember to define coefficients in global_variables.c

//-----------OFFSETS and CALIBRATIONS VALUE----------
#define GYRO_STARTUP_CALIB_DURATION 1.5f // time when gyro bias is measured in [s]
#define GYRO_STARTUP_CALIB_MAX_DEV 0.5f // maximum deviation accepted for gyro calibration [degree/s]



//-------------DEBUGGING--------------

// #define IMU_TEST // for IMU SELFTEST functions

#endif /* GLOBAL_CONSTANTS_H_ */
