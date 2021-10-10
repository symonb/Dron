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
static Quaternion q_global_position = { 1, 0, 0, 0 };
static float dt;

static void gyro_angles(Quaternion *q_gyro);
static void complementary_filter();
static void madgwick_filter();
static void acc_angles();
static ThreeF corrections();
static ThreeF Corrections_from_quaternion(Quaternion position_quaternion);

// err values - difference between set value and measured value:
static ThreeF err = { 0, 0, 0 };
static ThreeF sum_err = { 0, 0, 0 };
static ThreeF last_err = { 0, 0, 0 };
static ThreeF D_corr = { 0, 0, 0 };
static ThreeF F_corr = { 0, 0, 0 };
static ThreeF last_D_corr = { 0, 0, 0 };
static Three Rates = { 400, 400, 400 };

//4s
static PIDF R_PIDF = { 0.4, 0.001, 0.0004, 0.005 };
static PIDF P_PIDF = { 0.4, 0.001, 0.0004 , 0.005 };
static PIDF Y_PIDF = { 1, 0.01, 0.01, 0.01 };

// 3s
//static PIDF R_PIDF = { 0.14, 0.03, 0.014, 0.014 };
//static PIDF P_PIDF = { 0.14, 0.03, 0.013, 0.013 };
//static PIDF Y_PIDF = { 2, 0.11, 0.0005, 0.0005 };

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

//complementary_filter();
	madgwick_filter();
	set_motors(Corrections_from_quaternion(q_gyro));

	if ((get_Global_Time() - time_flag1_2) >= 1. / FREQUENCY_TELEMETRY_UPDATE) {
		time_flag1_2 = get_Global_Time();
		puk2++;
		//wypisywanie korekcji pitch P I D i roll P I D; k¹tów; zadanych wartosci
//		table_to_send[0] = P_PIDF.P * err.pitch * 500.f  + 1000;
//		table_to_send[1] = P_PIDF.I * sum_err.pitch * 500.f  + 1000;
//		table_to_send[2] = P_PIDF.D * D_corr.pitch * 500.f + 1000;
//		table_to_send[3] = R_PIDF.P * err.roll * 500.f + 1000;
//		table_to_send[4] = R_PIDF.I * sum_err.roll * 500.f  + 1000;
//		table_to_send[5] = R_PIDF.D * D_corr.roll * 500.f + 1000;
//		table_to_send[6] = (global_euler_angles.roll / MAX_ROLL_ANGLE * 50) + 1000;
//		table_to_send[7] = (global_euler_angles.pitch / MAX_PITCH_ANGLE * 50) + 1000;
//		table_to_send[8] = 10 * (gyro_angle_roll + 360);
//		table_to_send[9] = 10 * (gyro_angle_pitch + 360);
//		table_to_send[10] = 10 * (acc_angle_roll + 360);
//		table_to_send[11] = 10 * (acc_angle_pitch + 360);
//		table_to_send[12] = channels[1] - 500;
//		table_to_send[13] = channels[0] - 500;

//FOR SECOND APP TO MONITOR ALL FREE ERROR PITCH ROLL YAW NO ANGLES
		table_to_send[0] = P_PIDF.P * err.pitch * 500.f  + 1000;
		table_to_send[1] = P_PIDF.I * sum_err.pitch * 500.f + 1000;
		table_to_send[2] = P_PIDF.D * D_corr.pitch * 500.f + 1000;
		table_to_send[3] = R_PIDF.P * err.roll * 500.f + 1000;
		table_to_send[4] = R_PIDF.I * sum_err.roll * 500.f + 1000;
		table_to_send[5] = R_PIDF.D * D_corr.roll * 500.f + 1000;
		table_to_send[6] = (global_euler_angles.roll / MAX_ROLL_ANGLE * 50)
				+ 1000;
		table_to_send[7] = (global_euler_angles.pitch / MAX_PITCH_ANGLE * 50)
				+ 1000;
		table_to_send[8] = Y_PIDF.P * err.yaw * 500.f + 1000;
		table_to_send[9] = Y_PIDF.I * sum_err.yaw * 500.f + 1000;
		table_to_send[10] = Y_PIDF.D * D_corr.yaw * 500.f + 1000;
		table_to_send[11] = (10 * global_euler_angles.yaw) + 1500;
		table_to_send[12] = channels[1] - 500;
		table_to_send[13] = channels[0] - 500;

		czas_trwania_petli = get_Global_Time() - czas_trwania_petli;
		New_data_to_send = 1;
	}

}

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

	double acc_filter_rate = 0.1; // modification it is basically a weighted average (it gives much more smooth acc_reading)
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

static void gyro_angles(Quaternion *q_gyro) {
	*q_gyro = Rotate_Quaternion(*q_gyro);
}

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

static void madgwick_filter() {

//calculate derivative of q as usually
	q_global_position.w = q_gyro.w;
	q_global_position.x = -q_gyro.x;
	q_global_position.y = -q_gyro.y;
	q_global_position.z = -q_gyro.z;
	static Quaternion q_prim;
	static Quaternion angular_velocity;
	const float GYRO_TO_RAD = 1.f / 32.768f * DEG_TO_RAD;
	angular_velocity.w = 0;
	angular_velocity.x = Gyro_Acc[0] * GYRO_TO_RAD;
	angular_velocity.y = Gyro_Acc[1] * GYRO_TO_RAD;
	angular_velocity.z = Gyro_Acc[2] * GYRO_TO_RAD;

	q_prim = quaternion_multiply(
			quaternions_multiplication(q_global_position, angular_velocity),
			0.5f);

	static float min_fun[3];
	static Quaternion acc_reading;
	const float ACC_TO_GRAVITY = 1.f / 4096;
	acc_reading.w = 0;
	acc_reading.x = Gyro_Acc[3] * ACC_TO_GRAVITY;
	acc_reading.y = Gyro_Acc[4] * ACC_TO_GRAVITY;
	acc_reading.z = Gyro_Acc[5] * ACC_TO_GRAVITY;

	acc_reading = quaternion_multiply(acc_reading,
			1.f / quaternion_norm(acc_reading));

	min_fun[0] = 2
			* (q_global_position.x * q_global_position.z
					- q_global_position.w * q_global_position.y)
			- acc_reading.x;
	min_fun[1] = 2
			* (q_global_position.w * q_global_position.x
					+ q_global_position.y * q_global_position.z)
			- acc_reading.y;
	min_fun[2] = 2
			* (0.5f - q_global_position.x * q_global_position.x
					- q_global_position.y * q_global_position.y)
			- acc_reading.z;

	static Quaternion delta_min_fun;

	delta_min_fun.w = -2 * q_global_position.y * min_fun[0]
			+ 2 * q_global_position.x * min_fun[1];
	delta_min_fun.x = 2 * q_global_position.z * min_fun[0]
			+ 2 * q_global_position.w * min_fun[1]
			- 4 * q_global_position.x * min_fun[2];
	delta_min_fun.y = -2 * q_global_position.w * min_fun[0]
			+ 2 * q_global_position.z * min_fun[1]
			- 4 * q_global_position.y * min_fun[2];
	delta_min_fun.z = 2 * q_global_position.x * min_fun[0]
			+ 2 * q_global_position.y * min_fun[1];

	//normalize the gradient
	delta_min_fun = quaternion_multiply(delta_min_fun,
			1.f / quaternion_norm(delta_min_fun));

	static float coefficient_Beta = 0.073;

	q_gyro = quaternions_sum(q_gyro,
			quaternion_multiply(
					quaternion_conjugate(
							quaternions_sub(q_prim,
									quaternion_multiply(delta_min_fun,
											coefficient_Beta))), dt));

	//normalize quaternion:
	q_gyro = quaternion_multiply(q_gyro, 1.f / quaternion_norm(q_gyro));

	global_euler_angles = Quaternion_to_Euler_angles(q_gyro);

	global_euler_angles.roll *= -1;
	global_euler_angles.pitch *= -1;
	global_euler_angles.yaw *= -1;

}

static ThreeF corrections() {
	static ThreeF corr;
	static ThreeF last_channels;
	err.roll = ((channels[0] - 1500)  / 500.
			- global_angles.roll  / MAX_ROLL_ANGLE);
	err.pitch = ((channels[1] - 1500)/ 500.
			- global_angles.pitch  / MAX_PITCH_ANGLE);
	err.yaw = (channels[3] - 1500)  / 500.
			- (Gyro_Acc[2]) * 0.0305185f/ Rates.yaw;

	//	estimate Integral by sum (I term):
	sum_err.roll += err.roll * dt;
	sum_err.pitch += err.pitch * dt;
	sum_err.yaw += err.yaw * dt;



	//D correction will be divide for measurements and set-point corrections:

	D_corr.roll = -Gyro_Acc[0]  * 0.0305185f;
	D_corr.pitch = -Gyro_Acc[1] * 0.0305185f;
	D_corr.yaw = (err.yaw - last_err.yaw) / dt;

	F_corr.roll = (channels[0] - last_channels.roll)/500.f / dt;
	F_corr.pitch = (channels[1] - last_channels.pitch)/500.f  / dt;
	F_corr.yaw = 0;

	anti_windup(&sum_err,&R_PIDF,&P_PIDF,&Y_PIDF);

	//	calculate corrections:
	corr.roll = (R_PIDF.P * err.roll + R_PIDF.I * sum_err.roll
			+ R_PIDF.D * D_corr.roll) * 500 / 32768.;
	corr.pitch = (P_PIDF.P * err.pitch + P_PIDF.I * sum_err.pitch
			+ P_PIDF.D * D_corr.pitch) * 500 / 32768.;
	corr.yaw = (Y_PIDF.P * err.yaw + Y_PIDF.I * sum_err.yaw
			+ Y_PIDF.D * D_corr.yaw) * 500 / 32768.;

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

static ThreeF Corrections_from_quaternion(Quaternion position_quaternion) {

	static ThreeF corr;
	static ThreeF set_angles;
	static Three last_channels;
	static Quaternion set_position_quaternion;
	static Quaternion error_quaternion;

	set_angles.roll = (channels[0] - 1500) / 500.f * MAX_ROLL_ANGLE;
	set_angles.pitch = (channels[1] - 1500) / 500.f * MAX_PITCH_ANGLE;
	set_angles.yaw += (channels[3] - 1500) / 500.f * Rates.yaw * dt;

	//define quaternion of desired position (global) :
	set_position_quaternion = Euler_angles_to_Quaternion(set_angles);
	//to achieve the shortest path it is required to choose between q and -q so at first check cos(alfa) between quaternions
	if (skalar_quaternions_multiplication(position_quaternion,
			set_position_quaternion) < 0) {
		set_position_quaternion = quaternion_multiply(set_position_quaternion,
				-1);
	}

	//compute error quaternion (quaternion by which actual position quaternion has to be multiplied to achieve desired position quaternion)
	//NOTE my position quaternion is already conjunction of quaternion of global position ( so it is from local to global position) so counting error is made in this way:
	error_quaternion = quaternions_multiplication(position_quaternion,
			set_position_quaternion);
	//because error_quaternion (imaginary part of it) is vector in global (earth) coordinate system to have local error and then correction it can be rotate by matrix or quaternion rotation (quaternion of actual position)
	err.roll = error_quaternion.x;
	err.pitch = error_quaternion.y;
	err.yaw = error_quaternion.z;

	err = Rotate_Vector_with_Quaternion(err, position_quaternion, 0);

	//	estimate Integral by sum (I term):
	sum_err.roll += err.roll * dt;
	sum_err.pitch += err.pitch * dt;
	sum_err.yaw += err.yaw * dt;

	//D correction will be divide for measurements and set-point corrections:

	D_corr.roll = -Gyro_Acc[0]  * 0.0305185f;
	D_corr.pitch = -Gyro_Acc[1] * 0.0305185f;
	D_corr.yaw = (err.yaw - last_err.yaw) / dt;

	F_corr.roll = (channels[0] - last_channels.roll)/500.f / dt;
	F_corr.pitch = (channels[1] - last_channels.pitch)/500.f  / dt;
	F_corr.yaw = 0;

	anti_windup(&sum_err,&R_PIDF,&P_PIDF,&Y_PIDF);

	//	calculate corrections:
	corr.roll = (R_PIDF.P * err.roll + R_PIDF.I * sum_err.roll
			+ R_PIDF.D * D_corr.roll+R_PIDF.F*F_corr.roll) * 500;
	corr.pitch = (P_PIDF.P * err.pitch + P_PIDF.I * sum_err.pitch
			+ P_PIDF.D * D_corr.pitch+P_PIDF.F*F_corr.pitch) * 500 ;
	corr.yaw = (Y_PIDF.P * err.yaw + Y_PIDF.I * sum_err.yaw
			+ Y_PIDF.D * D_corr.yaw+Y_PIDF.F*F_corr.yaw) * 500;

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

