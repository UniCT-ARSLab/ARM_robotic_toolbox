/*
 * quaternion.h
 *
 *  Created on: Oct 14, 2026
 *      Author: corrado
 */

#ifndef LIB_CORE_INC_QUATERNION_H_
#define LIB_CORE_INC_QUATERNION_H_

typedef struct {
	float w,x,y,z;
} Quaternion;

float Quaternions_Dot(Quaternion * q0, Quaternion * q1);
float Quaternions_Angle(Quaternion * q0, Quaternion * q1);
void Quaternions_Slerp(Quaternion * result, Quaternion * q0, Quaternion * q1, float omega, float t);
void Quaternion_print(Quaternion * q);

#endif /* LIB_CORE_INC_QUATERNION_H_ */
