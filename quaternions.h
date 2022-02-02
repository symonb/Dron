/*
 * quaternions.h
 *
 *  Created on: 28.05.2021
 *      Author: symon
 */

#ifndef QUATERNIONS_H_
#define QUATERNIONS_H_


Quaternion quaternions_multiplication(Quaternion q1, Quaternion q2);
Quaternion quaternion_multiply(Quaternion q1, float x);
Quaternion quaternions_sum(Quaternion q1, Quaternion q2);
Quaternion quaternions_sub(Quaternion q1, Quaternion q2);
Quaternion quaternion_conjugate(Quaternion q1);
float quaternion_norm(Quaternion q1);
float quaternions_skalar_multiplication(Quaternion q1, Quaternion q2);

ThreeF Rotate_Vector_with_Quaternion(ThreeF vector,Quaternion q);
Quaternion Euler_angles_to_Quaternion(ThreeF euler_angles);
ThreeF Quaternion_to_Euler_angles(Quaternion q);

#endif /* QUATERNIONS_H_ */
