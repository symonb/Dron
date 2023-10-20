/*
 * stabilize.c
 *s
 */
#include <math.h>
#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "sensors/MPU6000.h"
#include "math/quaternions.h"
#include "flash.h"
#include "telemetry.h"
#include "stabilize.h"
#include "tasks.h"

#if defined(STABILIZE_FILTER_COMPLEMENTARY)
static quaternion_t gyro_angles(quaternion_t q_position, float dt);
static quaternion_t acc_angles(quaternion_t q_position);
static void complementary_filter(float dt_s);
#elif defined(STABILIZE_FILTER_MAGDWICK)
static void madgwick_filter(float dt_s);
#elif defined(STABILIZE_FILTER_MAHONY)
static void mahony_filter(float dt_s);
#endif

static threef_t corrections_from_quaternion(quaternion_t attitude_quaternion);

void att_update(timeUs_t dt_us)
{
	float dt = US_TO_SEC(dt_us);
#if defined(STABILIZE_FILTER_MAGDWICK)
	madgwick_filter(dt);
#elif defined(STABILIZE_FILTER_MAHONY)
	mahony_filter(dt);
#elif defined(STABILIZE_FILTER_COMPLEMENTARY)
	complementary_filter(dt);
#endif
}

void stabilize(timeUs_t dt_us) {

	float dt = US_TO_SEC(dt_us);
	// from correction calculated from angles we need to convert them into float <-1,1> as desired rotation speed for next PID controller:
	static float temp;
	// save yaw value since value calculated below is not desired
	temp = desired_rotation_speed.yaw;
	desired_rotation_speed = corrections_from_quaternion(q_global_attitude);
	static float I_roll;
	static float I_pitch;
	I_roll += desired_rotation_speed.roll * att_PIDF.I * dt;
	I_pitch += desired_rotation_speed.pitch * att_PIDF.I * dt;
	if (flight_mode == FLIGHT_MODE_ACRO || Arming_status != ARMED) {
		I_roll = 0;
		I_pitch = 0;
	}

	desired_rotation_speed.roll = (att_PIDF.P * desired_rotation_speed.roll + I_roll) * RATES_MAX_RATE_R;
	desired_rotation_speed.pitch = (att_PIDF.P * desired_rotation_speed.pitch + I_pitch) * RATES_MAX_RATE_P;
	//	write an old value of yaw rotaation speed (calculated from sticks position not angles):
	desired_rotation_speed.yaw = temp;
}

#if defined(STABILIZE_FILTER_COMPLEMENTARY)
static quaternion_t gyro_angles(quaternion_t q_position, float dt)
{

	quaternion_t q_prim;
	quaternion_t angular_velocity = { 0,Gyro_Acc[0] * DEG_TO_RAD ,Gyro_Acc[1] * DEG_TO_RAD ,Gyro_Acc[2] * DEG_TO_RAD };

	q_prim = quaternion_multiply(
		quaternions_multiplication(angular_velocity, q_position), -0.5f);

	q_position = quaternions_sum(q_position, quaternion_multiply(q_prim, dt));

	// normalize quaternion:
	q_position = quaternion_multiply(q_position,
		1.f / quaternion_norm(q_position));

	return q_position;
}

static quaternion_t acc_angles(quaternion_t q_position)
{

	threef_t acc_vector = { Gyro_Acc[3], Gyro_Acc[4], Gyro_Acc[5] };
	static quaternion_t q_acc;

	threef_t gravity_estimated = quaternion_rotate_vector(acc_vector,
		quaternion_conjugate(q_position));

	// normalize vector:
	double norm = sqrtf(
		gravity_estimated.roll * gravity_estimated.roll + gravity_estimated.pitch * gravity_estimated.pitch + gravity_estimated.yaw * gravity_estimated.yaw);
	gravity_estimated.roll /= norm;
	gravity_estimated.pitch /= norm;
	gravity_estimated.yaw /= norm;

	double acc_filter_rate = 0.9f; // modification: it is basically a weighted average (it gives much more smooth acc_reading)
	if (gravity_estimated.yaw >= 0)
	{
		q_position.w = (1 - acc_filter_rate) * q_acc.w + acc_filter_rate * sqrtf(0.5f * (gravity_estimated.yaw + 1));

		q_position.x = (1 - acc_filter_rate) * q_acc.x + acc_filter_rate * (-gravity_estimated.pitch / sqrtf(2 * (gravity_estimated.yaw + 1)));
		q_position.y = (1 - acc_filter_rate) * q_acc.y + acc_filter_rate * (gravity_estimated.roll / sqrtf(2 * (gravity_estimated.yaw + 1)));
		q_position.z = 0;
	}
	else
	{
		q_position.w = (1 - acc_filter_rate) * q_acc.w + acc_filter_rate * (-gravity_estimated.pitch / sqrtf(2 * (1 - gravity_estimated.yaw)));
		q_position.x = (1 - acc_filter_rate) * q_acc.x + acc_filter_rate * sqrtf(0.5f * (1 - gravity_estimated.yaw));
		q_position.y = 0;
		q_position.z = (1 - acc_filter_rate) * q_acc.z + acc_filter_rate * (gravity_estimated.roll / sqrtf(2 * (1 - gravity_estimated.yaw)));
	}
	// after LERP it is needed to normalize q_acc:
	q_acc = quaternion_multiply(q_position,
		1.f / quaternion_norm(q_position));

	return q_acc;
}

static void complementary_filter(float dt)
{
	quaternion_t q_gyro = gyro_angles(q_global_attitude, dt);
	quaternion_t q_acc = acc_angles(q_gyro);

	// to accomplish complementary filter q_acc need to have, a little effect so it need to be reduce by combining with identity quaternion =[1,0,0,0] which was multiplied with (1-ACC_PART) so:
	const quaternion_t IDENTITY_QUATERNION = { 1 - ACC_PART, 0, 0, 0 };
	quaternion_t delta_q_acc;
	delta_q_acc = quaternions_sum(IDENTITY_QUATERNION,
		quaternion_multiply(q_acc, ACC_PART));
	delta_q_acc = quaternion_multiply(delta_q_acc,
		1.f / quaternion_norm(delta_q_acc));

	q_global_attitude = quaternions_multiplication(delta_q_acc, q_gyro);
	global_euler_angles = quaternion_to_euler_angles(q_global_attitude);
}
#elif defined(STABILIZE_FILTER_MAGDWICK)
static void madgwick_filter(float dt)
{

	quaternion_t q_prim;
	quaternion_t angular_velocity = { 0,Gyro_Acc[0] * DEG_TO_RAD ,Gyro_Acc[1] * DEG_TO_RAD ,Gyro_Acc[2] * DEG_TO_RAD };

	float error_function[3];
	quaternion_t acc_reading = { 0, Gyro_Acc[3] ,Gyro_Acc[4] ,Gyro_Acc[5] };

	// normalize acc_reading:
	acc_reading = quaternion_multiply(acc_reading,
		1.f / quaternion_norm(acc_reading));

#if defined(MAGDWICK_ORIGINAL)

	q_prim = quaternion_multiply(
		quaternions_multiplication(q_global_attitude, angular_velocity),
		0.5f);

	//	compute values of error_function:
	error_function[0] = 2 * (q_global_attitude.x * q_global_attitude.z - q_global_attitude.w * q_global_attitude.y) - acc_reading.x;
	error_function[1] = 2 * (q_global_attitude.w * q_global_attitude.x + q_global_attitude.y * q_global_attitude.z) - acc_reading.y;
	error_function[2] = 2 * (0.5f - q_global_attitude.x * q_global_attitude.x - q_global_attitude.y * q_global_attitude.y) - acc_reading.z;

	// compute Jacobian^T*error_function:
	quaternion_t delta_error_function;

	delta_error_function.w = -2 * q_global_attitude.y * error_function[0] + 2 * q_global_attitude.x * error_function[1];

	delta_error_function.x = 2 * q_global_attitude.z * error_function[0] + 2 * q_global_attitude.w * error_function[1] - 4 * q_global_attitude.x * error_function[2];

	delta_error_function.y = -2 * q_global_attitude.w * error_function[0] + 2 * q_global_attitude.z * error_function[1] - 4 * q_global_attitude.y * error_function[2];

	delta_error_function.z = 2 * q_global_attitude.x * error_function[0] + 2 * q_global_attitude.y * error_function[1];

	// normalize the gradient
	delta_error_function = quaternion_multiply(delta_error_function,
		1.f / quaternion_norm(delta_error_function));

	float coefficient_Beta = 0.15; // 0.073 was

	q_global_attitude = quaternions_sum(q_global_attitude,
		quaternion_multiply(
			quaternions_sub(q_prim,
				quaternion_multiply(delta_error_function,
					coefficient_Beta)),
			dt));

	// normalize quaternion:
	q_global_attitude = quaternion_multiply(q_global_attitude,
		1.f / quaternion_norm(q_global_attitude));

	// for debugging compute Euler angles from quaternion:
	global_euler_angles = quaternion_to_euler_angles(
		quaternion_conjugate(q_global_attitude));

#elif defined(MAGDWICK_NEW)
	// teoretycznie poprawne dla kwaternionu transformacji z ukladu global do lokal:
	q_prim = quaternion_multiply(
		quaternions_multiplication(angular_velocity, q_global_attitude), -0.5f);

	//	compute values of error_function:
	error_function[0] = 2 * (q_global_attitude.w * q_global_attitude.y + q_global_attitude.x * q_global_attitude.z) - acc_reading.x;
	error_function[1] = 2 * (q_global_attitude.y * q_global_attitude.z - q_global_attitude.w * q_global_attitude.x) - acc_reading.y;
	error_function[2] = 2 * (0.5f - q_global_attitude.x * q_global_attitude.x - q_global_attitude.y * q_global_attitude.y) - acc_reading.z;

	// compute Jacobian^T*error_function:
	quaternion_t delta_error_function;

	delta_error_function.w = 2 * q_global_attitude.y * error_function[0] - 2 * q_global_attitude.x * error_function[1];
	delta_error_function.x = 2 * q_global_attitude.z * error_function[0] - 2 * q_global_attitude.w * error_function[1] - 4 * q_global_attitude.x * error_function[2];
	delta_error_function.y = 2 * q_global_attitude.w * error_function[0] + 2 * q_global_attitude.z * error_function[1] - 4 * q_global_attitude.y * error_function[2];
	delta_error_function.z = 2 * q_global_attitude.x * error_function[0] + 2 * q_global_attitude.y * error_function[1];

	// normalize the gradient
	delta_error_function = quaternion_multiply(delta_error_function,
		1.f / quaternion_norm(delta_error_function));

	float coefficient_Beta = 0.01f; // 0.073f from original article

	q_global_attitude = quaternions_sum(q_global_attitude,
		quaternion_multiply(
			quaternions_sub(q_prim,
				quaternion_multiply(delta_error_function,
					coefficient_Beta)),
			dt));
	// normalize quaternion:
	q_global_attitude = quaternion_multiply(q_global_attitude,
		1.f / quaternion_norm(q_global_attitude));

	// for debugging compute Euler angles from quaternion:
	global_euler_angles = quaternion_to_euler_angles(q_global_attitude);
#endif
}

#elif defined(STABILIZE_FILTER_MAHONY)
static void mahony_filter(float dt)
{
	quaternion_t q_prim;
	quaternion_t angular_velocity = { 0,Gyro_Acc[0] * DEG_TO_RAD ,Gyro_Acc[1] * DEG_TO_RAD ,Gyro_Acc[2] * DEG_TO_RAD };

	quaternion_t acc_reading = { 0,Gyro_Acc[3] ,Gyro_Acc[4] ,Gyro_Acc[5] };
	const PID_t mahony_omega_PI = { 1, 0.001, 0 };
	quaternion_t omega_corr;
	static quaternion_t sum_omega_corr;
	quaternion_t gravity_q;
	const threef_t gravity_vector_global = { 0, 0, 1 };


	// normalize acc_reading:
	acc_reading = quaternion_multiply(acc_reading,
		1.f / quaternion_norm(acc_reading));


	q_prim = quaternion_multiply(
		quaternions_multiplication(angular_velocity, q_global_attitude),
		-0.5f)

		// transform gravity vector from global frame to local frame (with use of gyro measurements):
		threef_t gravity_vector = quaternion_rotate_vector(gravity_vector_global,
			quaternions_sum(q_global_attitude,
				quaternion_multiply(q_prim, dt)));

	gravity_q.w = 0;
	gravity_q.x = gravity_vector.roll;
	gravity_q.y = gravity_vector.pitch;
	gravity_q.z = gravity_vector.yaw;

	// normalize gravity_q:
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
	angular_velocity.x = angular_velocity.x + mahony_omega_PI.I * sum_omega_corr.x + mahony_omega_PI.P * omega_corr.x;
	angular_velocity.y = angular_velocity.y + mahony_omega_PI.I * sum_omega_corr.y + mahony_omega_PI.P * omega_corr.y;
	angular_velocity.z = angular_velocity.z + mahony_omega_PI.I * sum_omega_corr.z + mahony_omega_PI.P * omega_corr.z;

	q_prim = quaternion_multiply(
		quaternions_multiplication(angular_velocity, q_global_attitude),
		-0.5f);

	q_global_attitude = quaternions_sum(q_global_attitude,
		quaternion_multiply(q_prim, dt));

	// normalize quaternion:
	q_global_attitude = quaternion_multiply(q_global_attitude,
		1.f / quaternion_norm(q_global_attitude));

	// compute Euler angles from quaternion:
	global_euler_angles = quaternion_to_euler_angles(q_global_attitude);
}
#endif

static threef_t corrections_from_quaternion(quaternion_t attitude_quaternion)
{
	threef_t err = { 0, 0, 0 };


	quaternion_t set_attitude_quaternion;
	quaternion_t error_quaternion;
	static bool drone_was_armed;

	desired_angles.yaw = global_euler_angles.yaw;

	if (desired_angles.yaw > 180)
	{
		desired_angles.yaw -= 360;
	}

	else if (desired_angles.yaw < -180)
	{
		desired_angles.yaw += 360;
	}

	// reset error for yaw after arming drone:

	if (!drone_was_armed && Arming_status == ARMED)
	{
		drone_was_armed = true;
		desired_angles.yaw = global_euler_angles.yaw;
	}
	else if (Arming_status != ARMED)
	{
		drone_was_armed = false;
	}

	// define quaternion of desired attitude (quaternion transformation from global to body frame):
	set_attitude_quaternion = quaternion_from_euler_angles(desired_angles);

#if defined(MAGDWICK_ORIGINAL)

	// to achieve the shortest path it is required to choose between q and -q, so at first check cos(alfa) between quaternions:
	if (quaternions_scalar_multiplication(attitude_quaternion,
		quaternion_conjugate(set_attitude_quaternion)) < 0)
	{
		set_attitude_quaternion = quaternion_multiply(set_attitude_quaternion,
			-1);
	}
	// compute error quaternion (quaternion by which actual position quaternion has to be multiplied to achieve desired position quaternion):
	error_quaternion = quaternions_multiplication(
		quaternion_conjugate(attitude_quaternion),
		quaternion_conjugate(set_attitude_quaternion));

#elif defined(MAGDWICK_NEW) || defined(STABILIZE_FILTER_MAHONY) || defined(STABILIZE_FILTER_COMPLEMENTARY)

	// to achieve the shortest path it is required to choose between q and -q, so at first check cos(alfa) between quaternions:
	if (quaternions_scalar_multiplication(attitude_quaternion,
		set_attitude_quaternion) < 0)
	{
		set_attitude_quaternion = quaternion_multiply(set_attitude_quaternion,
			-1);
	}

	// compute error quaternion (quaternion by which actual position quaternion has to be multiplied to achieve desired attitude quaternion):
	error_quaternion = quaternions_multiplication(attitude_quaternion,
		quaternion_conjugate(set_attitude_quaternion));

#endif

	// set imaginary part of q_error as errors:
	err.roll = error_quaternion.x;
	err.pitch = error_quaternion.y;
	err.yaw = error_quaternion.z;

	return err;
}

void send_telemetry_stabilize(timeUs_t time)
{
	// FOR SECOND APP TO MONITOR ALL FREE ERROR PITCH ROLL YAW
	// table_to_send[0] = P_PIDF.P * err.pitch + 1000;
	// table_to_send[1] = P_PIDF.I * sum_err.pitch + 1000;
	// table_to_send[2] = P_PIDF.D * D_corr.pitch + 1000;
	// table_to_send[3] = R_PIDF.P * err.roll + 1000;
	// table_to_send[4] = R_PIDF.I * sum_err.roll + 1000;
	// table_to_send[5] = R_PIDF.D * D_corr.roll + 1000;
	// table_to_send[6] = (global_euler_angles.roll / MAX_ROLL_ANGLE * 50) + 1000;
	// table_to_send[7] = (global_euler_angles.pitch / MAX_PITCH_ANGLE * 50) + 1000;
	// table_to_send[8] = Y_PIDF.P * err.yaw + 1000;
	// table_to_send[9] = Y_PIDF.I * sum_err.yaw + 1000;
	// table_to_send[10] = Y_PIDF.D * D_corr.yaw + 1000;
	// table_to_send[11] = (10 * global_euler_angles.yaw) + 1500;
	// table_to_send[12] = receiver.channels[1] - 500;
	// table_to_send[13] = receiver.channels[0] - 500;

	print(table_to_send, ALL_ELEMENTS_TO_SEND);
}
