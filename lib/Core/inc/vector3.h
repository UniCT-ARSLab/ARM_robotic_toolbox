/*
 * vector3.h
 *
 *  Created on: Sep 3, 2026
 *      Author: corrado
 */

#ifndef CORE_INC_VECTOR3_H_
#define CORE_INC_VECTOR3_H_

typedef struct {
	float	x,y,z;
} Vector3;

void vector3_zero(Vector3 * v);
void vector3_init(Vector3 * v, float x, float y, float z);
void vector3_init_v(Vector3 * v, float * d);
void vector3_add(Vector3 * res, Vector3 * a, Vector3 * b);
void vector3_sub(Vector3 * res, Vector3 * a, Vector3 * b);
float vector3_norm(Vector3 * a);
float vector3_diff_norm(Vector3 * a, Vector3 * b);
float vector3_dot(Vector3 * a, Vector3 * b);
void vector3_cross(Vector3 * res, Vector3 * a, Vector3 * b);
void vector3_multiply(Vector3 * res, Vector3 * a, float b);

#endif /* CORE_INC_VECTOR3_H_ */
