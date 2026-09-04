/*
 * vector3.c
 *
 *  Created on: Sep 3, 2026
 *      Author: corrado
 */

#include <math.h>
#include "vector3.h"

void vector3_zero(Vector3 * v)
{
	v->x = 0.0;
	v->y = 0.0;
	v->z = 0.0;
}

void vector3_init(Vector3 * v, float x, float y, float z)
{
	v->x = x;
	v->y = y;
	v->z = z;
}

void vector3_init_v(Vector3 * v, float * d)
{
	v->x = d[0];
	v->y = d[1];
	v->z = d[2];
}

void vector3_add(Vector3 * res, Vector3 * a, Vector3 * b)
{
	res->x = a->x +  b->x;
	res->y = a->y +  b->y;
	res->z = a->z +  b->z;
}

void vector3_sub(Vector3 * res, Vector3 * a, Vector3 * b)
{
	res->x = a->x -  b->x;
	res->y = a->y -  b->y;
	res->z = a->z -  b->z;
}

float vector3_norm(Vector3 * a)
{
	return sqrt(a->x*a->x + a->y*a->y + a->z*a->z);
}

float vector3_diff_norm(Vector3 * a, Vector3 * b)
{
	Vector3 tmp;
	vector3_sub(&tmp, a, b);
	return vector3_norm(&tmp);
}

float vector3_dot(Vector3 * a, Vector3 * b)
{
	return a->x * b->x + a->y * b->y + a->z * b->z;
}

void vector3_cross(Vector3 * res, Vector3 * a, Vector3 * b)
{
	res->x = a->y * b->z - a->z * b->y;
	res->y = a->z * b->x - a->x * b->z;
	res->z = a->x * b->y - a->y * b->x;
}

void vector3_multiply(Vector3 * res, Vector3 * a, float b)
{
	res->x = a->x * b;
	res->y = a->y * b;
	res->z = a->z * b;
}
