/*
 * quaternions.c
 *
 *  Created on: 28.05.2021
 *      Author: symon
 */

#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "quaternions.h"

#include <math.h>

threef_t quaternion_rotate_vector(threef_t vector, quaternion_t q) {

	threef_t vector_before = vector;
	vector.roll = (q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)
		* vector_before.roll + 2 * (q.x * q.y - q.w * q.z) * vector_before.pitch
		+ 2 * (q.x * q.z + q.w * q.y) * vector_before.yaw;
	vector.pitch = 2 * (q.x * q.y + q.w * q.z) * vector_before.roll
		+ (q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z) * vector_before.pitch
		+ 2 * (q.y * q.z - q.w * q.x) * vector_before.yaw;
	vector.yaw = 2 * (q.x * q.z - q.w * q.y) * vector_before.roll
		+ 2 * (q.y * q.z + q.w * q.x) * vector_before.pitch
		+ (q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z) * vector_before.yaw;
	return vector;
}


threef_t quaternion_to_euler_angles(quaternion_t q) {
	threef_t angles;

	// yaw (z-axis rotation)
	float siny_cosp = 2 * (-q.w * q.z + q.x * q.y);
	float cosy_cosp = q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z;
	angles.yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;


	// pitch (y-axis rotation)
	angles.pitch = asinf(2 * (-q.w * q.y - q.x * q.z)) * RAD_TO_DEG;

	// roll (x-axis rotation)
	float sinr_cosp = 2 * (-q.w * q.x + q.y * q.z);
	float cosr_cosp = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;
	angles.roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

	return angles;
}
quaternion_t quaternion_from_euler_angles(threef_t euler_angles) {

	float cr = cosf(euler_angles.roll * 0.5f * DEG_TO_RAD);
	float sr = sinf(euler_angles.roll * 0.5f * DEG_TO_RAD);
	float cp = cosf(euler_angles.pitch * 0.5f * DEG_TO_RAD);
	float sp = sinf(euler_angles.pitch * 0.5f * DEG_TO_RAD);
	float cy = cosf(euler_angles.yaw * 0.5f * DEG_TO_RAD);
	float sy = sinf(euler_angles.yaw * 0.5f * DEG_TO_RAD);

	quaternion_t q;
	q.w = cr * cp * cy + sr * sp * sy;
	q.x = -sr * cp * cy + cr * sp * sy;
	q.y = -cr * sp * cy - sr * cp * sy;
	q.z = -cr * cp * sy + sr * sp * cy;
	q = quaternion_multiply(q, 1.f / quaternion_norm(q));
	return q;
}

quaternion_t quaternions_multiplication(quaternion_t q1, quaternion_t q2) {
	quaternion_t q3;
	q3.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
	q3.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
	q3.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
	q3.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;

	return q3;
}
quaternion_t quaternions_sum(quaternion_t q1, quaternion_t q2) {
	quaternion_t q3;
	q3.w = q1.w + q2.w;
	q3.x = q1.x + q2.x;
	q3.y = q1.y + q2.y;
	q3.z = q1.z + q2.z;

	return q3;
}
quaternion_t quaternions_sub(quaternion_t q1, quaternion_t q2) {
	quaternion_t q3;
	q3.w = q1.w - q2.w;
	q3.x = q1.x - q2.x;
	q3.y = q1.y - q2.y;
	q3.z = q1.z - q2.z;

	return q3;
}
quaternion_t quaternion_multiply(quaternion_t q1, float x) {
	q1.w *= x;
	q1.x *= x;
	q1.y *= x;
	q1.z *= x;

	return q1;
}
float quaternion_norm(quaternion_t q1) {
	return sqrtf(q1.w * q1.w + q1.x * q1.x + q1.y * q1.y + q1.z * q1.z);
}

quaternion_t quaternion_conjugate(quaternion_t q1) {
	q1.x *= -1;
	q1.y *= -1;
	q1.z *= -1;
	return q1;
}

float quaternions_scalar_multiplication(quaternion_t q1, quaternion_t q2) {
	return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
}
