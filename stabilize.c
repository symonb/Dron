/*
 * stabilize.c
 *s
 */
#include <math.h>
#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "MPU6000.h"
#include "quaternions.h"


static ThreeF gangles = { 0, 0, 0 };
static Quaternion q_gyro_dryf = { 1, 0, 0, 0 };

static double acc_angle_roll = 0;
static double acc_angle_pitch = 0;
static double gyro_angle_roll = 0;
static double gyro_angle_pitch = 0;
static ThreeF acc_angle = { 0, 0, 0 };
static Quaternion q_acc;
static Quaternion q_gyro = { 1, 0, 0, 0 };
static double dt;

static void gyro_angles(Quaternion *q_gyro);
static void complementary_filter();
static void acc_angles();
static ThreeF corrections();
static ThreeF Corrections_from_quaternion();

static void anti_windup();

// err values - difference between set value and measured value:
static ThreeF err = { 0, 0, 0 };
static ThreeF sum_err = { 0, 0, 0 };
static ThreeF last_err = { 0, 0, 0 };
static ThreeF D_corr = { 0, 0, 0 };
static ThreeF last_D_corr = { 0, 0, 0 };
static Three Rates = { 400, 400, 400 };

//4s
static PID R_PID = { 0.31, 0.0001, 0.009 };
static PID P_PID = { 0.31, 0.0001, 0.008 };
static PID Y_PID = {0.2, 0.00, 0.01 };

// 3s
//static PID R_PID = { 0.14, 0.03, 0.014 };
//static PID P_PID = { 0.14, 0.03, 0.013 };
//static PID Y_PID = { 2, 0.11, 0.0005 };

//for debugging only:
static int puk2 = 0;
static double czas_trwania_petli = 0;

void stabilize() {

	czas_trwania_petli = get_Global_Time();

	static double time_flag1_1;
	static double time_flag1_2;

	dt = (get_Global_Time() - time_flag1_1);
	time_flag1_1 = get_Global_Time();

	// dryf zyroskopu:
//	gyro_angles(&q_gyro_dryf);
//	gangles= Quaternion_to_Euler_angles(q_gyro_dryf);
//	gyro_angle_roll = gangles.roll;
//	gyro_angle_pitch = gangles.pitch;

	complementary_filter();
	set_motors(Corrections_from_quaternion());

	if ((get_Global_Time() - time_flag1_2) >= 1. / FREQUENCY_TELEMETRY_UPDATE) {
		time_flag1_2 = get_Global_Time();
		puk2++;
		//wypisywanie korekcji pitch P I D i roll P I D; k�t�w; zadanych wartosci
//		table_to_send[0] = P_PID.P * err.pitch * 500. / 32768. + 1000;
//		table_to_send[1] = P_PID.I * sum_err.pitch * 500. / 32768. + 1000;
//		table_to_send[2] = P_PID.D * D_corr.pitch * 500. / 32768. + 1000;
//		table_to_send[3] = R_PID.P * err.roll * 500. / 32768. + 1000;
//		table_to_send[4] = R_PID.I * sum_err.roll * 500. / 32768. + 1000;
//		table_to_send[5] = R_PID.D * D_corr.roll * 500. / 32768. + 1000;
//		table_to_send[6] = (global_euler_angles.roll / MAX_ROLL_ANGLE * 50) + 1000;
//		table_to_send[7] = (global_euler_angles.pitch / MAX_PITCH_ANGLE * 50) + 1000;
//		table_to_send[8] = 10 * (gyro_angle_roll + 360);
//		table_to_send[9] = 10 * (gyro_angle_pitch + 360);
//		table_to_send[10] = 10 * (acc_angle_roll + 360);
//		table_to_send[11] = 10 * (acc_angle_pitch + 360);
//		table_to_send[12] = channels[1] - 500;
//		table_to_send[13] = channels[0] - 500;

//FOR SECOND APP TO MONITOR ALL FREE ERROR PITCH ROLL YAW NO ANGLES
		table_to_send[0] = P_PID.P * err.pitch * 500. / 32768. + 1000;
		table_to_send[1] = P_PID.I * sum_err.pitch * 500. / 32768. + 1000;
		table_to_send[2] = P_PID.D * D_corr.pitch * 500. / 32768. + 1000;
		table_to_send[3] = R_PID.P * err.roll * 500. / 32768. + 1000;
		table_to_send[4] = R_PID.I * sum_err.roll * 500. / 32768. + 1000;
		table_to_send[5] = R_PID.D * D_corr.roll * 500. / 32768. + 1000;
		table_to_send[6] = (global_euler_angles.roll / MAX_ROLL_ANGLE * 50)
				+ 1000;
		table_to_send[7] = (global_euler_angles.pitch / MAX_PITCH_ANGLE * 50)
				+ 1000;
		table_to_send[8] = Y_PID.P * err.yaw * 500. / 32768. + 1000;
		table_to_send[9] = Y_PID.I * sum_err.yaw * 500. / 32768. + 1000;
		table_to_send[10] = Y_PID.D * D_corr.yaw * 500. / 32768. + 1000;
		table_to_send[11] = (10*global_euler_angles.yaw) + 1500;
		table_to_send[12] = channels[1] - 500;
		table_to_send[13] = channels[0] - 500;

		czas_trwania_petli = get_Global_Time() - czas_trwania_petli;
		New_data_to_send = 1;
	}

}

//static void acc_angles() {
//	double acc_filter_rate = 0.05;
//	acc_angle_roll = (1 - acc_filter_rate) * acc_angle_roll
//			+ acc_filter_rate
//					* (atan2(Gyro_Acc[4], Gyro_Acc[5]) * RAD_TO_DEG + ROLL_OFFSET);
//	acc_angle_pitch = (1 - acc_filter_rate) * acc_angle_pitch
//			+ acc_filter_rate
//					* (-atan2(Gyro_Acc[3], Gyro_Acc[5]) * RAD_TO_DEG + PITCH_OFFSET);
//
//}
static void acc_angles(Quaternion *q_gyro) {

	static ThreeF gravity_estimated = { 0, 0, 0 };
	ThreeF acc_vector = { Gyro_Acc[3], Gyro_Acc[4], Gyro_Acc[5] };
	static double norm;
	gravity_estimated = Rotate_Vector_with_Quaternion(acc_vector, *q_gyro, 1);

	norm = sqrtf(
			gravity_estimated.roll * gravity_estimated.roll
					+ gravity_estimated.pitch * gravity_estimated.pitch
					+ gravity_estimated.yaw * gravity_estimated.yaw);
	gravity_estimated.roll /= norm;
	gravity_estimated.pitch /= norm;
	gravity_estimated.yaw /= norm;

	double acc_filter_rate = 0.1;
	if (gravity_estimated.yaw >= 0) {
		q_acc.w = (1 - acc_filter_rate) * q_acc.w
				+ acc_filter_rate * sqrtf(0.5 * (gravity_estimated.yaw + 1));

		q_acc.x = (1 - acc_filter_rate) * q_acc.x
				+ acc_filter_rate
						* (-gravity_estimated.pitch
								/ sqrtf(2 * (gravity_estimated.yaw + 1)));
		q_acc.y = (1 - acc_filter_rate) * q_acc.y
				+ acc_filter_rate
						* (gravity_estimated.roll
								/ sqrtf(2 * (gravity_estimated.yaw + 1)));
		q_acc.z = 0;
	} else {
		q_acc.w = (1 - acc_filter_rate) * q_acc.w
				+ acc_filter_rate
						* (-gravity_estimated.pitch
								/ sqrtf(2 * (1 - gravity_estimated.yaw)));
		q_acc.x = (1 - acc_filter_rate) * q_acc.x
				+ acc_filter_rate * sqrtf(0.5 * (1 - gravity_estimated.yaw));
		q_acc.y = 0;
		q_acc.z = (1 - acc_filter_rate) * q_acc.z
				+ acc_filter_rate
						* (gravity_estimated.roll
								/ sqrtf(2 * (1 - gravity_estimated.yaw)));
	}
	//after LERP it is needed to normalize q_acc:
	q_acc = quaternion_multiply(q_acc, 1 / quaternion_norm(q_acc));

}

//static void gyro_angles(ThreeF *gyro_angles) {
//	gyro_angles->roll += Gyro_Acc[0] * dt / (GYRO_TO_DPS);
//	gyro_angles->pitch += Gyro_Acc[1] * dt
//			/ (GYRO_TO_DPS);
//	gyro_angles->roll += gyro_angles->pitch
//			* sin(
//					Gyro_Acc[2] * dt / (GYRO_TO_DPS)
//							/ RAD_TO_DEG);
//	gyro_angles->pitch -= gyro_angles->roll
//			* sin(
//					Gyro_Acc[2] * dt / (GYRO_TO_DPS)
//							/ RAD_TO_DEG);
//
//}
static void gyro_angles(Quaternion *q_gyro) {
	*q_gyro = Rotate_Quaternion(*q_gyro);
}
//static void complementary_filter() {
//	gyro_angles(&global_euler_angles);
//	acc_angles();
//	global_euler_angles.roll = ACC_PART * acc_angle_roll + GYRO_PART * global_euler_angles.roll;
//	global_euler_angles.pitch = ACC_PART * acc_angle_pitch + GYRO_PART * global_euler_angles.pitch;
//}
static void complementary_filter() {
	gyro_angles(&q_gyro);
	acc_angles(&q_gyro);

	// to accomplish complementary filter q_acc need to have, a little effect so it need to be reduce by combining with identity quaternion =[1,0,0,0] which was multiplied with (1-ACC_PART) so:
	const Quaternion IDENTITY_QUATERNION = { 1 - ACC_PART, 0, 0, 0 };
	Quaternion delta_q_acc;
	delta_q_acc = quaternions_sum(IDENTITY_QUATERNION,
			quaternion_multiply(q_acc, ACC_PART));
	delta_q_acc = quaternion_multiply(delta_q_acc,
			1 / quaternion_norm(delta_q_acc));

	q_gyro = quaternions_multiplication(delta_q_acc, q_gyro);
	global_euler_angles = Quaternion_to_Euler_angles(q_gyro);

	global_euler_angles.roll *= -1;
	global_euler_angles.pitch *= -1;
	global_euler_angles.yaw *= -1;

}

static ThreeF corrections() {
	static ThreeF corr;
	static ThreeF last_channels;
	err.roll = ((channels[0] - 1500) * 32768 / 500.
			- global_angles.roll * 32768 / MAX_ROLL_ANGLE);
	err.pitch = ((channels[1] - 1500) * 32768 / 500.
			- global_angles.pitch * 32768 / MAX_PITCH_ANGLE);
	err.yaw = (channels[3] - 1500) * 32768 / 500.
			- (Gyro_Acc[2]) * 1000 / Rates.yaw;

	//	estimate Integral by sum (I term):
	sum_err.roll += err.roll * dt;
	sum_err.pitch += err.pitch * dt;
	sum_err.yaw += err.yaw * dt;

//	//low-pass filter
//	D_corr.roll= ((err.roll-last_err.roll)/dt+last_D_corr.roll)/2.;
//	D_corr.pitch=((err.pitch-last_err.pitch)/dt+last_D_corr.pitch)/2.;
//	D_corr.yaw=((err.yaw-last_err.yaw)/dt+last_D_corr.yaw)/2.;

	D_corr.roll = -(Gyro_Acc[0]) * 1000 / MAX_ROLL_ANGLE
			+ (channels[0] - last_channels.roll) / 500. * 32768 / dt;
	D_corr.pitch = -(Gyro_Acc[1]) * 1000 / MAX_PITCH_ANGLE
			+ (channels[1] - last_channels.pitch) / 500. * 32768 / dt;
	D_corr.yaw = (err.yaw - last_err.yaw) / dt;

	anti_windup();

	//	calculate corrections:
	corr.roll = (R_PID.P * err.roll + R_PID.I * sum_err.roll
			+ R_PID.D * D_corr.roll) * 500 / 32768.;
	corr.pitch = (P_PID.P * err.pitch + P_PID.I * sum_err.pitch
			+ P_PID.D * D_corr.pitch) * 500 / 32768.;
	corr.yaw =
			(Y_PID.P * err.yaw + Y_PID.I * sum_err.yaw + Y_PID.D * D_corr.yaw)
					* 500 / 32768.;

	//	set current errors as last errors:
	last_err.roll = err.roll;
	last_err.pitch = err.pitch;
	last_err.yaw = err.yaw;

	last_D_corr.roll = D_corr.roll;
	last_D_corr.pitch = D_corr.pitch;
	last_D_corr.yaw = D_corr.yaw;

	last_channels.roll = channels[0];
	last_channels.pitch = channels[1];
	last_channels.yaw = channels[3];

	return corr;
}

static ThreeF Corrections_from_quaternion() {

	static ThreeF corr;
	static ThreeF set_angles;
	static ThreeF last_channels;
	static Quaternion set_position_quaternion;
	static Quaternion error_quaternion;

	set_angles.roll = (channels[0] - 1500) / 500. * MAX_ROLL_ANGLE;
	set_angles.pitch = (channels[1] - 1500) / 500. * MAX_PITCH_ANGLE;
	set_angles.yaw += (channels[3] - 1500) / 500. * Rates.yaw  * dt;

	//define quaternion of desired position (global) :
	set_position_quaternion = Euler_angles_to_Quaternion(set_angles);
	//to achieve the shortest path it is required to choose between q and -q so at first check cos(alfa) between quaternions
	if (skalar_quaternions_multiplication(q_gyro,set_position_quaternion)<0){
		set_position_quaternion=quaternion_multiply(set_position_quaternion,-1);
	}

	//compute error quaternion (quaternion by which actual position quaternion has to be multiplied to achieve desired position quaternion)
	//NOTE my position quaternion is already conjunction of quaternion of global position ( so it is from local to global position) so counting error is made in this way:
	error_quaternion = quaternions_multiplication(q_gyro,
			set_position_quaternion);
	//because error_quaternion (imaginary part of it) is vector in global (earth) coordinate system to have local error and then correction it can be rotate by matrix or quaternion rotation (quaternion of actual position)
	err.roll = error_quaternion.x;
	err.pitch = error_quaternion.y;
	err.yaw = error_quaternion.z;

	err = Rotate_Vector_with_Quaternion(err, q_gyro, 0);

	// PLEACE CHANGE THIS:
	err.roll *= 32768;
	err.pitch *= 32768;
	err.yaw *= 32768;

	//	estimate Integral by sum (I term):
	sum_err.roll += err.roll * dt;
	sum_err.pitch += err.pitch * dt;
	sum_err.yaw += err.yaw * dt;

	//	//low-pass filter
	//	D_corr.roll= ((err.roll-last_err.roll)/dt+last_D_corr.roll)/2.;
	//	D_corr.pitch=((err.pitch-last_err.pitch)/dt+last_D_corr.pitch)/2.;
	//	D_corr.yaw=((err.yaw-last_err.yaw)/dt+last_D_corr.yaw)/2.;

	D_corr.roll = -(Gyro_Acc[0]) * 1000 / MAX_ROLL_ANGLE
			+ (channels[0] - last_channels.roll) / 500. * 32768 / dt;
	D_corr.pitch = -(Gyro_Acc[1]) * 1000 / MAX_PITCH_ANGLE
			+ (channels[1] - last_channels.pitch) / 500. * 32768 / dt;
	D_corr.yaw = (err.yaw - last_err.yaw) / dt;

	anti_windup();

	//	calculate corrections:
	corr.roll = (R_PID.P * err.roll + R_PID.I * sum_err.roll
			+ R_PID.D * D_corr.roll) * 500 / 32768.;
	corr.pitch = (P_PID.P * err.pitch + P_PID.I * sum_err.pitch
			+ P_PID.D * D_corr.pitch) * 500 / 32768.;
	corr.yaw =
			(Y_PID.P * err.yaw + Y_PID.I * sum_err.yaw + Y_PID.D * D_corr.yaw)
					* 500 / 32768.;

	//	set current errors as last errors:
	last_err.roll = err.roll;
	last_err.pitch = err.pitch;
	last_err.yaw = err.yaw;

	last_D_corr.roll = D_corr.roll;
	last_D_corr.pitch = D_corr.pitch;
	last_D_corr.yaw = D_corr.yaw;

	last_channels.roll = channels[0];
	last_channels.pitch = channels[1];
	last_channels.yaw = channels[3];

	return corr;
}

static void anti_windup() {
	if (channels[4] > 1600) {
		int16_t max_I_correction = 300;
		if ((sum_err.roll * R_PID.I * 500. / 32768.) > max_I_correction) {
			sum_err.roll = max_I_correction / R_PID.I / 500. * 32768.;
		} else if ((sum_err.roll * R_PID.I * 500. / 32768.)
				< -max_I_correction) {
			sum_err.roll = -max_I_correction / R_PID.I / 500. * 32768.;
		}
		if ((sum_err.pitch * P_PID.I * 500. / 32768.) > max_I_correction) {
			sum_err.pitch = max_I_correction / P_PID.I / 500. * 32768.;
		} else if ((sum_err.pitch * P_PID.I * 500 / 32768.)
				< -max_I_correction) {
			sum_err.pitch = -max_I_correction / P_PID.I / 500. * 32768.;
		}
		if ((sum_err.yaw * Y_PID.I * 500. / 32768.) > max_I_correction) {
			sum_err.yaw = max_I_correction / Y_PID.I / 500. * 32768.;
		} else if ((sum_err.yaw * Y_PID.I * 500 / 32768.) < -max_I_correction) {
			sum_err.yaw = -max_I_correction / Y_PID.I / 500. * 32768.;
		}
	} else {			// quad is disarmed so turn off I term of corrections
		sum_err.roll = 0;
		sum_err.pitch = 0;
		sum_err.yaw = 0;
	}

	int16_t max_D_correction = 300;
	if ((D_corr.roll * R_PID.D * 500. / 32768.) > max_D_correction
			|| (D_corr.roll * R_PID.D * 500. / 32768.) < -max_D_correction) {
		D_corr.roll = last_D_corr.roll;
	}
	if (D_corr.pitch * P_PID.D * 500. / 32768. > max_D_correction
			|| D_corr.pitch * P_PID.D * 500. / 32768. < -max_D_correction) {
		D_corr.pitch = last_D_corr.pitch;
	}
	if (D_corr.yaw * Y_PID.D * 500. / 32768. > max_D_correction
			|| D_corr.yaw * Y_PID.D * 500. / 32768. < -max_D_correction) {
		D_corr.yaw = last_D_corr.yaw;
	}

}
