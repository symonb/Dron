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
#include "flash.h"
#include "scheduler.h"
#include "stabilize.h"

static Quaternion q_acc = { 1, 0, 0, 0 };
static Quaternion q_gyro = { 1, 0, 0, 0 };

static Quaternion gyro_angles(Quaternion q_position, float dt);
static Quaternion acc_angles();
static void complementary_filter(float dt);
static void madgwick_filter(float dt);
static void mahony_filter(float dt);

static ThreeF corrections(float dt);
static ThreeF corrections_from_quaternion(Quaternion position_quaternion, float dt);

// err values - difference between set value and measured value:
static ThreeF err = { 0, 0, 0 };
static ThreeF sum_err = { 0, 0, 0 };
static ThreeF last_err = { 0, 0, 0 };
static ThreeF D_corr = { 0, 0, 0 };
static ThreeF F_corr = { 0, 0, 0 };
static ThreeF last_D_corr = { 0, 0, 0 };
static Three Rates = { 400, 400, 400 };

//4s
// DRON DRUK 3D:
//static PIDF R_PIDF = { 260, 180, 0.27, 2.5 };
//static PIDF P_PIDF = { 260, 180, 0.27, 2.5 };
//static PIDF Y_PIDF = { 1200, 300, 200, 0 };

//DRON CARBON:
static PIDF R_PIDF = {150 , 190, 0.175, 0.2 };
static PIDF P_PIDF = { 200, 210, 0.225, 0.25 };
static PIDF Y_PIDF = { 500, 200, 70, 0 };


void stabilize(timeUs_t time) {

	static float dt;

	dt = US_TO_SEC(time);

#if defined(STABILIZE_FILTER_MAGDWICK)
	madgwick_filter(dt);
#elif defined(STABILIZE_FILTER_MAHONY)
	mahony_filter(dt);
#elif defined(STABILIZE_FILTER_COMPLEMENTARY)
	complementary_filter(dt);
#endif

	set_motors(corrections_from_quaternion(q_global_position,dt));

	if ((get_Global_Time() - time_flag1_2) >= SEC_TO_US(1.f / FREQUENCY_TELEMETRY_UPDATE)) {
		time_flag1_2 = get_Global_Time();

		//send_telemetry_stabilize(time_flag1_2);
	}

}

static Quaternion gyro_angles(Quaternion q_position, float dt) {

	static Quaternion q_prim;
	static Quaternion angular_velocity;

	angular_velocity.w = 0;
	angular_velocity.x = Gyro_Acc[0] * GYRO_TO_RAD;
	angular_velocity.y = Gyro_Acc[1] * GYRO_TO_RAD;
	angular_velocity.z = Gyro_Acc[2] * GYRO_TO_RAD;

	q_prim = quaternion_multiply(
			quaternions_multiplication(angular_velocity, q_position), -0.5f);

	q_position = quaternions_sum(q_position, quaternion_multiply(q_prim, dt));

	//normalize quaternion:
	q_position = quaternion_multiply(q_position,
			1.f / quaternion_norm(q_position));

	return q_position;

}

static Quaternion acc_angles(Quaternion q_position) {

	static ThreeF gravity_estimated = { 0, 0, 0 };
	ThreeF acc_vector = { Gyro_Acc[3] * ACC_TO_GRAVITY, Gyro_Acc[4]
			* ACC_TO_GRAVITY, Gyro_Acc[5] * ACC_TO_GRAVITY };
	static double norm;

	gravity_estimated = Rotate_Vector_with_Quaternion(acc_vector,
			quaternion_conjugate(q_position));

	// normalize vector:
	norm = sqrtf(
			gravity_estimated.roll * gravity_estimated.roll
					+ gravity_estimated.pitch * gravity_estimated.pitch
					+ gravity_estimated.yaw * gravity_estimated.yaw);
	gravity_estimated.roll /= norm;
	gravity_estimated.pitch /= norm;
	gravity_estimated.yaw /= norm;

	double acc_filter_rate = 0.9f; // modification: it is basically a weighted average (it gives much more smooth acc_reading)
	if (gravity_estimated.yaw >= 0) {
		q_position.w = (1 - acc_filter_rate) * q_acc.w
				+ acc_filter_rate * sqrtf(0.5f * (gravity_estimated.yaw + 1));

		q_position.x = (1 - acc_filter_rate) * q_acc.x
				+ acc_filter_rate
						* (-gravity_estimated.pitch
								/ sqrtf(2 * (gravity_estimated.yaw + 1)));
		q_position.y = (1 - acc_filter_rate) * q_acc.y
				+ acc_filter_rate
						* (gravity_estimated.roll
								/ sqrtf(2 * (gravity_estimated.yaw + 1)));
		q_position.z = 0;
	} else {
		q_position.w = (1 - acc_filter_rate) * q_acc.w
				+ acc_filter_rate
						* (-gravity_estimated.pitch
								/ sqrtf(2 * (1 - gravity_estimated.yaw)));
		q_position.x = (1 - acc_filter_rate) * q_acc.x
				+ acc_filter_rate * sqrtf(0.5f * (1 - gravity_estimated.yaw));
		q_position.y = 0;
		q_position.z = (1 - acc_filter_rate) * q_acc.z
				+ acc_filter_rate
						* (gravity_estimated.roll
								/ sqrtf(2 * (1 - gravity_estimated.yaw)));
	}
	//after LERP it is needed to normalize q_acc:
	q_position = quaternion_multiply(q_position,
			1.f / quaternion_norm(q_position));

	return q_position;

}



static void complementary_filter(float dt) {
	q_gyro = gyro_angles(q_global_position, dt);
	q_acc = acc_angles(q_gyro);

	// to accomplish complementary filter q_acc need to have, a little effect so it need to be reduce by combining with identity quaternion =[1,0,0,0] which was multiplied with (1-ACC_PART) so:
	const Quaternion IDENTITY_QUATERNION = { 1 - ACC_PART, 0, 0, 0 };
	Quaternion delta_q_acc;
	delta_q_acc = quaternions_sum(IDENTITY_QUATERNION,
			quaternion_multiply(q_acc, ACC_PART));
	delta_q_acc = quaternion_multiply(delta_q_acc,
			1.f / quaternion_norm(delta_q_acc));

	q_global_position = quaternions_multiplication(delta_q_acc, q_gyro);
	global_euler_angles = Quaternion_to_Euler_angles(q_global_position);

}

static void madgwick_filter(float dt) {

	static Quaternion q_prim;
	static Quaternion angular_velocity;

	angular_velocity.w = 0;
	angular_velocity.x = Gyro_Acc[0] * GYRO_TO_RAD;
	angular_velocity.y = Gyro_Acc[1] * GYRO_TO_RAD;
	angular_velocity.z = Gyro_Acc[2] * GYRO_TO_RAD;

	static float error_function[3];
	static Quaternion acc_reading;

	acc_reading.w = 0;
	acc_reading.x = Gyro_Acc[3] * ACC_TO_GRAVITY;
	acc_reading.y = Gyro_Acc[4] * ACC_TO_GRAVITY;
	acc_reading.z = Gyro_Acc[5] * ACC_TO_GRAVITY;

	//normalize acc_reading:
	acc_reading = quaternion_multiply(acc_reading,
			1.f / quaternion_norm(acc_reading));

#if defined(MAGDWICK_ORIGINAL)

	q_prim = quaternion_multiply(
			quaternions_multiplication(q_global_position, angular_velocity),
			0.5f);

	//	compute values of error_function:
	error_function[0] = 2
	* (q_global_position.x * q_global_position.z
			- q_global_position.w * q_global_position.y)
	- acc_reading.x;
	error_function[1] = 2
	* (q_global_position.w * q_global_position.x
			+ q_global_position.y * q_global_position.z)
	- acc_reading.y;
	error_function[2] = 2
	* (0.5f - q_global_position.x * q_global_position.x
			- q_global_position.y * q_global_position.y)
	- acc_reading.z;

	// compute Jacobian^T*error_function:
	static Quaternion delta_error_function;

	delta_error_function.w = -2 * q_global_position.y * error_function[0]
	+ 2 * q_global_position.x * error_function[1];

	delta_error_function.x = 2 * q_global_position.z * error_function[0]
	+ 2 * q_global_position.w * error_function[1]
	- 4 * q_global_position.x * error_function[2];

	delta_error_function.y = -2 * q_global_position.w * error_function[0]
	+ 2 * q_global_position.z * error_function[1]
	- 4 * q_global_position.y * error_function[2];

	delta_error_function.z = 2 * q_global_position.x * error_function[0]
	+ 2 * q_global_position.y * error_function[1];

	//normalize the gradient
	delta_error_function = quaternion_multiply(delta_error_function,
			1.f / quaternion_norm(delta_error_function));

	static float coefficient_Beta = 0.15;//0.073 was

	q_global_position = quaternions_sum(q_global_position,
			quaternion_multiply(
					quaternions_sub(q_prim,
							quaternion_multiply(delta_error_function,
									coefficient_Beta)), dt));

	//normalize quaternion:
	q_global_position = quaternion_multiply(q_global_position,
			1.f / quaternion_norm(q_global_position));

	// for debugging compute Euler angles from quaternion:
	global_euler_angles = Quaternion_to_Euler_angles(
			quaternion_conjugate(q_global_position));

#elif defined(MAGDWICK_NEW)
	//teoretycznie poprawne dla kwaternionu transformacji z ukladu global do lokal:
	q_prim = quaternion_multiply(
			quaternions_multiplication(angular_velocity, q_global_position),
			-0.5f);

//	compute values of error_function:
	error_function[0] = 2
			* (q_global_position.w * q_global_position.y
					+ q_global_position.x * q_global_position.z)
			- acc_reading.x;
	error_function[1] = 2
			* (q_global_position.y * q_global_position.z
					- q_global_position.w * q_global_position.x)
			- acc_reading.y;
	error_function[2] = 2
			* (0.5f - q_global_position.x * q_global_position.x
					- q_global_position.y * q_global_position.y)
			- acc_reading.z;

//compute Jacobian^T*error_function:
	static Quaternion delta_error_function;

	delta_error_function.w = 2 * q_global_position.y * error_function[0]
			- 2 * q_global_position.x * error_function[1];
	delta_error_function.x = 2 * q_global_position.z * error_function[0]
			- 2 * q_global_position.w * error_function[1]
			- 4 * q_global_position.x * error_function[2];
	delta_error_function.y = 2 * q_global_position.w * error_function[0]
			+ 2 * q_global_position.z * error_function[1]
			- 4 * q_global_position.y * error_function[2];
	delta_error_function.z = 2 * q_global_position.x * error_function[0]
			+ 2 * q_global_position.y * error_function[1];

	//normalize the gradient
	delta_error_function = quaternion_multiply(delta_error_function,
			1.f / quaternion_norm(delta_error_function));

	static float coefficient_Beta = 0.15;				//0.073 was

	q_global_position = quaternions_sum(q_global_position,
			quaternion_multiply(
					quaternions_sub(q_prim,
							quaternion_multiply(delta_error_function,
									coefficient_Beta)), dt));
	//normalize quaternion:
	q_global_position = quaternion_multiply(q_global_position,
			1.f / quaternion_norm(q_global_position));

	// for debugging compute Euler angles from quaternion:
	global_euler_angles = Quaternion_to_Euler_angles(q_global_position);
#endif

}

static void mahony_filter(float dt) {

	static Quaternion q_prim;
	static Quaternion angular_velocity;

	static Quaternion acc_reading;
	static PID mahony_omega_PI = { 1, 0.001, 0 };
	static Quaternion omega_corr;
	static Quaternion sum_omega_corr;
	static Quaternion gravity_q;
	static const ThreeF gravity_vector_global = { 0, 0, 1 };
	static ThreeF gravity_vector = { 0, 0, 1 };

	angular_velocity.w = 0;
	angular_velocity.x = Gyro_Acc[0] * GYRO_TO_RAD;
	angular_velocity.y = Gyro_Acc[1] * GYRO_TO_RAD;
	angular_velocity.z = Gyro_Acc[2] * GYRO_TO_RAD;

	acc_reading.w = 0;
	acc_reading.x = Gyro_Acc[3] * ACC_TO_GRAVITY;
	acc_reading.y = Gyro_Acc[4] * ACC_TO_GRAVITY;
	acc_reading.z = Gyro_Acc[5] * ACC_TO_GRAVITY;

	//normalize acc_reading:
	acc_reading = quaternion_multiply(acc_reading,
			1.f / quaternion_norm(acc_reading));

	// transform gravity vector from global frame to local frame (with use of gyro measurements):
	q_prim = quaternion_multiply(
			quaternions_multiplication(angular_velocity, q_global_position),
			-0.5f);

	gravity_vector = Rotate_Vector_with_Quaternion(gravity_vector_global,
			quaternions_sum(q_global_position,
					quaternion_multiply(q_prim, dt)));

	gravity_q.w = 0;
	gravity_q.x = gravity_vector.roll;
	gravity_q.y = gravity_vector.pitch;
	gravity_q.z = gravity_vector.yaw;

	//normalize gravity_q:
	gravity_q = quaternion_multiply(gravity_q,
			1.f / quaternion_norm(gravity_q));

	// calculate value of correction:
	//* NOTE: multiplication of pure quaternions is not giving pure quaternion, but vector part is equal to vectors multiplication *
	omega_corr = quaternions_multiplication(acc_reading, gravity_q);

	// integrate corrections:
	sum_omega_corr.x += omega_corr.x * dt;
	sum_omega_corr.y += omega_corr.y * dt;
	sum_omega_corr.z += omega_corr.z * dt;

	// calculate angular_velocity with correction:
	angular_velocity.x = angular_velocity.x
			+ mahony_omega_PI.I * sum_omega_corr.x
			+ mahony_omega_PI.P * omega_corr.x;
	angular_velocity.y = angular_velocity.y
			+ mahony_omega_PI.I * sum_omega_corr.y
			+ mahony_omega_PI.P * omega_corr.y;
	angular_velocity.z = angular_velocity.z
			+ mahony_omega_PI.I * sum_omega_corr.z
			+ mahony_omega_PI.P * omega_corr.z;

	q_prim = quaternion_multiply(
			quaternions_multiplication(angular_velocity, q_global_position),
			-0.5f);

	q_global_position = quaternions_sum(q_global_position,
			quaternion_multiply(q_prim, dt));

	//normalize quaternion:
	q_global_position = quaternion_multiply(q_global_position,
			1.f / quaternion_norm(q_global_position));

	// for debugging compute Euler angles from quaternion:
	global_euler_angles = Quaternion_to_Euler_angles(q_global_position);
}

static ThreeF corrections(float dt) {
	static ThreeF corr;

	err.roll = ((channels[0] - 1500) / 500.
			- global_angles.roll / MAX_ROLL_ANGLE);
	err.pitch = ((channels[1] - 1500) / 500.
			- global_angles.pitch / MAX_PITCH_ANGLE);
	err.yaw = (channels[3] - 1500) / 500.
			- (Gyro_Acc[2]) * 0.0305185f / Rates.yaw;

	//	estimate Integral by sum (I term):
	sum_err.roll += err.roll * dt;
	sum_err.pitch += err.pitch * dt;
	sum_err.yaw += err.yaw * dt;

	//D correction will be divide for measurements and set-point corrections:
	D_corr.roll = -Gyro_Acc[0] * 0.0305185f;
	D_corr.pitch = -Gyro_Acc[1] * 0.0305185f;
	D_corr.yaw = (err.yaw - last_err.yaw) / dt;

	F_corr.roll = (channels[0] - channels_previous_values[0])  *0.002f / dt;
	F_corr.pitch = (channels[1] - channels_previous_values[1]) *0.002f / dt;
	F_corr.yaw = 0;

	anti_windup(&sum_err, &R_PIDF, &P_PIDF, &Y_PIDF);

	//	calculate corrections:
	corr.roll = (R_PIDF.P * err.roll + R_PIDF.I * sum_err.roll
			+ R_PIDF.D * D_corr.roll) * 2;
	corr.pitch = (P_PIDF.P * err.pitch + P_PIDF.I * sum_err.pitch
			+ P_PIDF.D * D_corr.pitch) * 2;
	corr.yaw = (Y_PIDF.P * err.yaw + Y_PIDF.I * sum_err.yaw
			+ Y_PIDF.D * D_corr.yaw) * 2;

	//	set current errors as last errors:
	last_err.roll = err.roll;
	last_err.pitch = err.pitch;
	last_err.yaw = err.yaw;

	last_D_corr.roll = D_corr.roll;
	last_D_corr.pitch = D_corr.pitch;
	last_D_corr.yaw = D_corr.yaw;

	return corr;
}

static ThreeF corrections_from_quaternion(Quaternion position_quaternion, float dt) {

	static ThreeF corr;
	static ThreeF set_angles;
	static Quaternion set_position_quaternion;
	static Quaternion error_quaternion;
	static uint8_t drone_was_armed = 0;

	set_angles.roll = (channels[0] - 1500) / 500.f * MAX_ROLL_ANGLE;
	set_angles.pitch = (channels[1] - 1500) / 500.f * MAX_PITCH_ANGLE;
	set_angles.yaw += (channels[3] - 1500) / 500.f * Rates.yaw * dt;



	if (set_angles.yaw > 180) {
		set_angles.yaw -= 360;
	}

	else if (set_angles.yaw < -180) {
		set_angles.yaw += 360;
	}


#if defined(BLACKBOX_SAVE_SET_ANGLES)
	global_variable_monitor[0] = set_angles.roll;
	global_variable_monitor[1] = set_angles.pitch;
	global_variable_monitor[2] = set_angles.yaw;
#endif

	//reset error for yaw after arming drone:

	if (channels[4] > ARM_VALUE) {
		if (!drone_was_armed) {
			drone_was_armed = 1;
			set_angles.yaw = global_euler_angles.yaw;
		}
	} else {
		drone_was_armed = 0;
	}

	//define quaternion of desired position (quaternion transformation from global to body frame):
	set_position_quaternion = Euler_angles_to_Quaternion(set_angles);

#if defined(MAGDWICK_ORIGINAL)

	//to achieve the shortest path it is required to choose between q and -q, so at first check cos(alfa) between quaternions:
	if (quaternions_skalar_multiplication(position_quaternion,
					quaternion_conjugate(set_position_quaternion)) < 0) {
		set_position_quaternion = quaternion_multiply(set_position_quaternion,
				-1);
	}
	//compute error quaternion (quaternion by which actual position quaternion has to be multiplied to achieve desired position quaternion):
	error_quaternion = quaternions_multiplication(
			quaternion_conjugate(position_quaternion),
			quaternion_conjugate(set_position_quaternion));

#elif defined(MAGDWICK_NEW)||defined( STABILIZE_FILTER_MAHONY ) || defined(STABILIZE_FILTER_COMPLEMENTARY)

	//to achieve the shortest path it is required to choose between q and -q, so at first check cos(alfa) between quaternions:
	if (quaternions_skalar_multiplication(position_quaternion,
			set_position_quaternion) < 0) {
		set_position_quaternion = quaternion_multiply(set_position_quaternion,
				-1);
	}

	//compute error quaternion (quaternion by which actual position quaternion has to be multiplied to achieve desired position quaternion):
	error_quaternion = quaternions_multiplication(position_quaternion,
			quaternion_conjugate(set_position_quaternion));

#endif

	//set imaginary part of q_error as errors:
	err.roll = error_quaternion.x;
	err.pitch = error_quaternion.y;
	err.yaw = error_quaternion.z;

	// opcja 2 do dokoñczenia !!! trzeba zmniejszyc PIDy
//	static ThreeF err_omega;
//	static float last_err_yaw;
//	err_omega.yaw = (err.yaw-last_err_yaw)/dt;
//
//	last_err_yaw = err.yaw;
//
//	err_omega.yaw= err_omega.yaw - Gyro_Acc[2] *0.0305185f/ Rates.yaw;
//
//	err.yaw=err_omega.yaw;

	//	estimate Integral by sum (I term):
	sum_err.roll += err.roll * dt;
	sum_err.pitch += err.pitch * dt;
	sum_err.yaw += err.yaw * dt;

	//D correction will be divide for measurements and set-point corrections:
	D_corr.roll = -Gyro_Acc[0] * 0.0305185f;
	D_corr.pitch = -Gyro_Acc[1] * 0.0305185f;
	D_corr.yaw = (err.yaw - last_err.yaw) / dt;

	F_corr.roll = (channels[0] - channels_previous_values[0]) *0.002f / dt;
	F_corr.pitch = (channels[1] - channels_previous_values[1]) *0.002f / dt;
	F_corr.yaw = 0;

	anti_windup(&sum_err, &R_PIDF, &P_PIDF, &Y_PIDF);

	//	calculate corrections:
	corr.roll = (R_PIDF.P * err.roll + R_PIDF.I * sum_err.roll
			+ R_PIDF.D * D_corr.roll + R_PIDF.F * F_corr.roll) * 2;
	corr.pitch = (P_PIDF.P * err.pitch + P_PIDF.I * sum_err.pitch
			+ P_PIDF.D * D_corr.pitch + P_PIDF.F * F_corr.pitch) * 2;
	corr.yaw = (Y_PIDF.P * err.yaw + Y_PIDF.I * sum_err.yaw
			+ Y_PIDF.D * D_corr.yaw + Y_PIDF.F * F_corr.yaw) * 2;

	//	set current errors as last errors:
	last_err.roll = err.roll;
	last_err.pitch = err.pitch;
	last_err.yaw = err.yaw;

	last_D_corr.roll = D_corr.roll;
	last_D_corr.pitch = D_corr.pitch;
	last_D_corr.yaw = D_corr.yaw;

	return corr;
}

void send_telemetry_stabilize(timeUs_t time){
	//FOR SECOND APP TO MONITOR ALL FREE ERROR PITCH ROLL YAW
			table_to_send[0] = P_PIDF.P * err.pitch + 1000;
			table_to_send[1] = P_PIDF.I * sum_err.pitch + 1000;
			table_to_send[2] = P_PIDF.D * D_corr.pitch + 1000;
			table_to_send[3] = R_PIDF.P * err.roll + 1000;
			table_to_send[4] = R_PIDF.I * sum_err.roll + 1000;
			table_to_send[5] = R_PIDF.D * D_corr.roll + 1000;
			table_to_send[6] = (global_euler_angles.roll / MAX_ROLL_ANGLE * 50)
					+ 1000;
			table_to_send[7] = (global_euler_angles.pitch / MAX_PITCH_ANGLE * 50)
					+ 1000;
			table_to_send[8] = Y_PIDF.P * err.yaw + 1000;
			table_to_send[9] = Y_PIDF.I * sum_err.yaw + 1000;
			table_to_send[10] = Y_PIDF.D * D_corr.yaw + 1000;
			table_to_send[11] = (10 * global_euler_angles.yaw) + 1500;
			table_to_send[12] = channels[1] - 500;
			table_to_send[13] = channels[0] - 500;

			New_data_to_send = 1;
}
