///*
// * calibration.c
// *
// *  Created on: 11.01.2021
// *      Author: symon
// */
//
//#include "stm32l0xx.h"
//#include "stm32l0xx_nucleo.h"
//#include "MPU6050.h"
//
//extern typedef struct{
//	int32_t roll;
//	int32_t pitch;
//	int32_t yaw;
//}Three;
//
//Three gyro_calibration();
//Three gyro_calibration();
//
//Three GYRO_OFFSET;
//
//extern int16_t Gyro_Acc[];
//
#include "stm32l0xx.h"
#include "stm32l0xx_nucleo.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "MPU6050.h"
#include "calibration.h"

typedef struct {
	double x;
	double y;
	double z;
} ThreeXYZ;



void gyro_calibration() {

	ThreeXYZ G_calibration = { 0, 0, 0 };
	static double time_flag0_1 = 0;
	static int counter = 0;
	uint16_t total_number=1000;
	while (counter < total_number) {
		if (get_Global_Time() - time_flag0_1 > 0.01) {
			time_flag0_1 = get_Global_Time();
			rewrite_data();
			G_calibration.x += (double)Gyro_Acc[0]/total_number;
			G_calibration.y += (double)Gyro_Acc[1]/total_number;
			G_calibration.z += (double)Gyro_Acc[2]/total_number;
			counter++;
		} else {
			delay_micro(500);
		}
	}
}


void acc_calibration() {

	ThreeXYZ A_calibration = { 0, 0, 0 };
	static double time_flag0_1 = 0;
	static int licznik = 0;

	while (licznik < 1000) {
		if (get_Global_Time() - time_flag0_1 > 0.01) {
			time_flag0_1 = get_Global_Time();
			rewrite_data();
			A_calibration.x += Gyro_Acc[3]/1000.;
			A_calibration.y += Gyro_Acc[4]/1000.;
			A_calibration.z += Gyro_Acc[5]/1000.;
			licznik++;
		} else {
			delay_micro(500);
		}
	}
}
