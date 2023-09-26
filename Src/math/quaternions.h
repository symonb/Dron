/*
 * quaternions.h
 *
 *  Created on: 28.05.2021
 *      Author: symon
 */

#ifndef QUATERNIONS_H_
#define QUATERNIONS_H_


quaternion_t quaternions_multiplication(quaternion_t q1, quaternion_t q2);
quaternion_t quaternion_multiply(quaternion_t q1, float x);
quaternion_t quaternions_sum(quaternion_t q1, quaternion_t q2);
quaternion_t quaternions_sub(quaternion_t q1, quaternion_t q2);
quaternion_t quaternion_conjugate(quaternion_t q1);
float quaternion_norm(quaternion_t q1);
float quaternions_scalar_multiplication(quaternion_t q1, quaternion_t q2);

threef_t quaternion_rotate_vector(threef_t vector,quaternion_t q);
quaternion_t euler_angles_to_quaternion(threef_t euler_angles);
threef_t quaternion_to_euler_angles(quaternion_t q);

#endif /* QUATERNIONS_H_ */
