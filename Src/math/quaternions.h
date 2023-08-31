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
float quaternions_skalar_multiplication(quaternion_t q1, quaternion_t q2);

threef_t Rotate_Vector_with_Quaternion(threef_t vector,quaternion_t q);
quaternion_t Euler_angles_to_Quaternion(threef_t euler_angles);
threef_t Quaternion_to_Euler_angles(quaternion_t q);

#endif /* QUATERNIONS_H_ */
