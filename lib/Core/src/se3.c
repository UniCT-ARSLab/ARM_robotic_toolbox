/*
 * se3.c
 *
 *  Created on: Aug 31, 2026
 *      Author: corrado
 */

#include "arm_rtb.h"

#include <stdio.h>
#include <stdlib.h>


void SE3_delete(SE3_mat * mat)
{
	rtb_free(mat->pData);
}

arm_status SE3_Rx(SE3_mat * mat, float angle)
{
	float * pData = rtb_allocate(sizeof(float) * 16);
	if (pData == NULL)
		return ARM_MATH_NOMEM;
	arm_mat_init_f32(mat, 4, 4, pData);
	return SE3_Rx_N(mat, angle);
}

arm_status SE3_Rx_N(SE3_mat * mat, float angle)
{
	float c = cos(angle);
	float s = sin(angle);
	SE3(mat, 0, 0) = 1;		SE3(mat, 0, 1) = 0;		SE3(mat, 0, 2) = 0;		SE3(mat, 0, 3) = 0;
	SE3(mat, 1, 0) = 0;		SE3(mat, 1, 1) = c;		SE3(mat, 1, 2) = -s;	SE3(mat, 1, 3) = 0;
	SE3(mat, 2, 0) = 0;		SE3(mat, 2, 1) = s;		SE3(mat, 2, 2) = c;		SE3(mat, 2, 3) = 0;
	SE3(mat, 3, 0) = 0;		SE3(mat, 3, 1) = 0;		SE3(mat, 3, 2) = 0;		SE3(mat, 3, 3) = 1;
	return ARM_MATH_SUCCESS;
}

arm_status SE3_Ry(SE3_mat * mat, float angle)
{
	float * pData = rtb_allocate(sizeof(float) * 16);
	if (pData == NULL)
		return ARM_MATH_NOMEM;
	arm_mat_init_f32(mat, 4, 4, pData);
	return SE3_Ry_N(mat, angle);
}

arm_status SE3_Ry_N(SE3_mat * mat, float angle)
{
	float c = cos(angle);
	float s = sin(angle);
	SE3(mat, 0, 0) = c;		SE3(mat, 0, 1) = 0;		SE3(mat, 0, 2) = s;		SE3(mat, 0, 3) = 0;
	SE3(mat, 1, 0) = 0;		SE3(mat, 1, 1) = 1;		SE3(mat, 1, 2) = 0;		SE3(mat, 1, 3) = 0;
	SE3(mat, 2, 0) = -s;	SE3(mat, 2, 1) = 0;		SE3(mat, 2, 2) = c;		SE3(mat, 2, 3) = 0;
	SE3(mat, 3, 0) = 0;		SE3(mat, 3, 1) = 0;		SE3(mat, 3, 2) = 0;		SE3(mat, 3, 3) = 1;
	return ARM_MATH_SUCCESS;
}

arm_status SE3_Rz(SE3_mat * mat, float angle)
{
	float * pData = rtb_allocate(sizeof(float) * 16);
	if (pData == NULL)
		return ARM_MATH_NOMEM;
	arm_mat_init_f32(mat, 4, 4, pData);
	return SE3_Rz_N(mat, angle);
}

arm_status SE3_Rz_N(SE3_mat * mat, float angle)
{
	float c = cos(angle);
	float s = sin(angle);
	SE3(mat, 0, 0) = c;		SE3(mat, 0, 1) = -s;	SE3(mat, 0, 2) = 0;		SE3(mat, 0, 3) = 0;
	SE3(mat, 1, 0) = s;		SE3(mat, 1, 1) = c;		SE3(mat, 1, 2) = 0;		SE3(mat, 1, 3) = 0;
	SE3(mat, 2, 0) = 0;		SE3(mat, 2, 1) = 0;		SE3(mat, 2, 2) = 1;		SE3(mat, 2, 3) = 0;
	SE3(mat, 3, 0) = 0;		SE3(mat, 3, 1) = 0;		SE3(mat, 3, 2) = 0;		SE3(mat, 3, 3) = 1;
	return ARM_MATH_SUCCESS;
}

arm_status SE3_Trans(SE3_mat * mat, float * xyz)
{
	float * pData = rtb_allocate(sizeof(float) * 16);
	if (pData == NULL)
		return ARM_MATH_NOMEM;
	arm_mat_init_f32(mat, 4, 4, pData);
	return SE3_Trans_N(mat, xyz);
}

arm_status SE3_Trans_N(SE3_mat * mat, float * xyz)
{
	SE3(mat, 0, 0) = 1;		SE3(mat, 0, 1) = 0;		SE3(mat, 0, 2) = 0;		SE3(mat, 0, 3) = xyz[0];
	SE3(mat, 1, 0) = 0;		SE3(mat, 1, 1) = 1;		SE3(mat, 1, 2) = 0;		SE3(mat, 1, 3) = xyz[1];
	SE3(mat, 2, 0) = 0;		SE3(mat, 2, 1) = 0;		SE3(mat, 2, 2) = 1;		SE3(mat, 2, 3) = xyz[2];
	SE3(mat, 3, 0) = 0;		SE3(mat, 3, 1) = 0;		SE3(mat, 3, 2) = 0;		SE3(mat, 3, 3) = 1;
	return ARM_MATH_SUCCESS;
}

arm_status SE3_from_Quat_N(SE3_mat * mat, Quaternion * q)
{
	SE3(mat, 0, 0) = 1 - 2 * (q->y*q->y + q->z*q->z);
	SE3(mat, 0, 1) = 2 * (q->x*q->y - q->w*q->z);
	SE3(mat, 0, 2) = 2 * (q->x*q->z + q->w*q->y);
	SE3(mat, 0, 3) = 0;

	SE3(mat, 1, 0) = 2 * (q->x*q->y + q->w*q->z);
	SE3(mat, 1, 1) = 1 - 2 * (q->x*q->x + q->z*q->z);
	SE3(mat, 1, 2) = 2 * (q->y*q->z - q->w*q->x);
	SE3(mat, 1, 3) = 0;

	SE3(mat, 2, 0) = 2 * (q->x*q->z - q->w*q->y);
	SE3(mat, 2, 1) = 2 * (q->y*q->z + q->w*q->x);
	SE3(mat, 2, 2) = 1 - 2 * (q->y*q->y + q->x*q->x);
	SE3(mat, 2, 3) = 0;

	SE3(mat, 3, 0) = 0;
	SE3(mat, 3, 1) = 0;
	SE3(mat, 3, 2) = 0;
	SE3(mat, 3, 3) = 1;

	return ARM_MATH_SUCCESS;
}

arm_status SE3_to_Quat(SE3_mat * mat, Quaternion * q)
{
	float t12p = (SE3(mat, 0, 1) + SE3(mat, 1, 0)); t12p = t12p * t12p;
	float t13p = (SE3(mat, 0, 2) + SE3(mat, 2, 0)); t13p = t13p * t13p;
    float t23p = (SE3(mat, 1, 2) + SE3(mat, 2, 1)); t23p = t23p * t23p;

    float t12m = (SE3(mat, 0, 1) - SE3(mat, 1, 0)); t12m = t12m * t12m;
    float t13m = (SE3(mat, 0, 2) - SE3(mat, 2, 0)); t13m = t13m * t13m;
    float t23m = (SE3(mat, 1, 2) - SE3(mat, 2, 1)); t23m = t23m * t23m;

    float d1 = ( SE3(mat, 0, 0) + SE3(mat, 1, 1) + SE3(mat, 2, 2) + 1); d1 = d1 * d1;
    float d2 = ( SE3(mat, 0, 0) - SE3(mat, 1, 1) - SE3(mat, 2, 2) + 1); d2 = d2 * d2;
    float d3 = (-SE3(mat, 0, 0) + SE3(mat, 1, 1) - SE3(mat, 2, 2) + 1); d3 = d3 * d3;
    float d4 = (-SE3(mat, 0, 0) - SE3(mat, 1, 1) + SE3(mat, 2, 2) + 1); d4 = d4 * d4;

    q->w = sqrt(d1 + t23m + t13m + t12m) / 4.0;
    q->x = sqrt(t23m + d2 + t12p + t13p) / 4.0;
    q->y = sqrt(t13m + t12p + d3 + t23p) / 4.0;
    q->z = sqrt(t12m + t13p + t23p + d4) / 4.0;

    if (q->w >= q->x && q->w >= q->y && q->w >= q->z) {
    	q->x = copysign(q->x, SE3(mat, 2, 1) - SE3(mat, 1, 2));
        q->y = copysign(q->y, SE3(mat, 0, 2) - SE3(mat, 2, 0));
        q->z = copysign(q->z, SE3(mat, 1, 0) - SE3(mat, 0, 1));
    }
    else if (q->x >= q->w && q->x >= q->y && q->x >= q->z) {
    	q->w = copysign(q->w, SE3(mat, 2, 1) - SE3(mat, 1, 2));
        q->y = copysign(q->y, SE3(mat, 1, 0) + SE3(mat, 0, 1));
        q->z = copysign(q->z, SE3(mat, 0, 2) + SE3(mat, 2, 0));
    }
    else if (q->y >= q->w && q->y >= q->x && q->x >= q->z) {
    	q->w = copysign(q->w, SE3(mat, 0, 2) - SE3(mat, 2, 0));
        q->x = copysign(q->x, SE3(mat, 1, 0) + SE3(mat, 0, 1));
        q->z = copysign(q->z, SE3(mat, 2, 1) + SE3(mat, 1, 2));
    }
    else if (q->z >= q->w && q->z >= q->x && q->z >= q->y) {
    	q->w = copysign(q->w, SE3(mat, 1, 0) - SE3(mat, 0, 1));
        q->x = copysign(q->x, SE3(mat, 0, 2) + SE3(mat, 2, 0));
        q->y = copysign(q->y, SE3(mat, 2, 1) + SE3(mat, 1, 2));
    }
    else {
    	printf("SE3_to_Quat: INVALID CONDITION\n\r");
    	for (;;) ;
    }

	return ARM_MATH_SUCCESS;
}

void Quaternion_Scale(Quaternion * res, Quaternion * q, float f)
{
	res->w = q->w * f;
	res->x = q->x * f;
	res->y = q->y * f;
	res->z = q->z * f;
}

void Quaternions_Add(Quaternion * res, Quaternion * q0, Quaternion * q1)
{
	res->w = q0->w + q1->w;
	res->x = q0->x + q1->x;
	res->y = q0->y + q1->y;
	res->z = q0->z + q1->z;
}

float Quaternions_Dot(Quaternion * q0, Quaternion * q1)
{
	return q0->w * q1->w + q0->x * q1->x + q0->y * q1->y + q0->z * q1->z;
}

float Quaternions_Angle(Quaternion * q0, Quaternion * q1)
{
	float dot = Quaternions_Dot(q0, q1);

	// shortest path
	if (dot < 0) dot = -dot;

	// saturate in order to avoid float approximations
	if (dot > 1.0) dot = 1.0;

	return acos(dot);
}


void Quaternions_Slerp(Quaternion * result, Quaternion * q0, Quaternion * q1, float omega, float t)
{
	float s = sin(omega);
	float a0 = sin((1 - t) * omega) / s;
	float a1 = sin(t * omega) / s;
	Quaternion q0_a0, q1_a1;

	Quaternion_Scale(&q0_a0, q0, a0);
	Quaternion_Scale(&q1_a1, q1, a1);

	Quaternions_Add(result, &q0_a0, &q1_a1);
}

void Quaternion_print(Quaternion * q)
{
	printf("[%6.3f, %6.3f, %6.3f, %6.3f]\n\r",
			q->w, q->x, q->y, q->z);
}

arm_status SE3_I(SE3_mat * mat)
{
	float * pData = rtb_allocate(sizeof(float) * 16);
	if (pData == NULL)
		return ARM_MATH_NOMEM;
	arm_mat_init_f32(mat, 4, 4, pData);
	SE3(mat, 0, 0) = 1;		SE3(mat, 0, 1) = 0;		SE3(mat, 0, 2) = 0;		SE3(mat, 0, 3) = 0;
	SE3(mat, 1, 0) = 0;		SE3(mat, 1, 1) = 1;		SE3(mat, 1, 2) = 0;		SE3(mat, 1, 3) = 0;
	SE3(mat, 2, 0) = 0;		SE3(mat, 2, 1) = 0;		SE3(mat, 2, 2) = 1;		SE3(mat, 2, 3) = 0;
	SE3(mat, 3, 0) = 0;		SE3(mat, 3, 1) = 0;		SE3(mat, 3, 2) = 0;		SE3(mat, 3, 3) = 1;
	return ARM_MATH_SUCCESS;
}

arm_status SE3_Multiply_N(SE3_mat * result, SE3_mat * a, SE3_mat * b)
{
	arm_mat_mult_f32(a, b, result);
	return ARM_MATH_SUCCESS;
}

arm_status SE3_Multiply(SE3_mat * result, SE3_mat * a, SE3_mat * b)
{
	float * pData = rtb_allocate(sizeof(float) * 16);
	if (pData == NULL)
		return ARM_MATH_NOMEM;
	return SE3_Multiply_N(result, a, b);
}


arm_status SE3_copy(SE3_mat * result, SE3_mat * a)
{
	if (SE3_I(result) == ARM_MATH_NOMEM)
		return ARM_MATH_NOMEM;
	memcpy(result->pData, a->pData, 16 * sizeof(float));
	return ARM_MATH_SUCCESS;
}

void SE3_copy_N(SE3_mat * result, SE3_mat * a)
{
	memcpy(result->pData, a->pData, 16 * sizeof(float));
}

void SE3_origin(SE3_mat * mat, Vector3 * v)
{
	v->x = SE3(mat, 0, 3);
	v->y = SE3(mat, 1, 3);
	v->z = SE3(mat, 2, 3);
}

void SE3_set_origin(SE3_mat * mat, Vector3 * v)
{
	SE3(mat, 0, 3) = v->x;
	SE3(mat, 1, 3) = v->y;
	SE3(mat, 2, 3) = v->z;
}

void SE3_basis(SE3_mat * mat,SO3_mat * basis)
{
	SO3(basis, 0, 0) = SE3(mat, 0, 0);
	SO3(basis, 0, 1) = SE3(mat, 0, 1);
	SO3(basis, 0, 2) = SE3(mat, 0, 2);
	SO3(basis, 1, 0) = SE3(mat, 1, 0);
	SO3(basis, 1, 1) = SE3(mat, 1, 1);
	SO3(basis, 1, 2) = SE3(mat, 1, 2);
	SO3(basis, 2, 0) = SE3(mat, 2, 0);
	SO3(basis, 2, 1) = SE3(mat, 2, 1);
	SO3(basis, 2, 2) = SE3(mat, 2, 2);
}

void SE3_basis_x(SE3_mat * mat, Vector3 * v)
{
	v->x = SE3(mat, 0, 0);
	v->y = SE3(mat, 1, 0);
	v->z = SE3(mat, 2, 0);
}

void SE3_basis_y(SE3_mat * mat, Vector3 * v)
{
	v->x = SE3(mat, 0, 1);
	v->y = SE3(mat, 1, 1);
	v->z = SE3(mat, 2, 1);
}

void SE3_basis_z(SE3_mat * mat, Vector3 * v)
{
	v->x = SE3(mat, 0, 2);
	v->y = SE3(mat, 1, 2);
	v->z = SE3(mat, 2, 2);
}

#define TOL		1e-9

void SE3_to_rpy(SE3_mat * mat, float * roll, float * pitch, float * yaw)
{
	// zyx sequence
	// algorithm copied from python spatial math
	if (fabs ( fabs(SE3(mat, 2, 0)) - 1 ) < TOL) {
		//singularity
		*roll = 0;
		if (SE3(mat, 2, 0) < 0) {
			*yaw = - atan2(SE3(mat,0,1), SE3(mat,0,2));
		}
		else {
			*yaw = atan2(- SE3(mat,0,1), - SE3(mat,0,2));
		}
		float c = SE3(mat, 2, 0);
		if (c < -1.0) c = -1.0;
		if (c > 1.0) c = 1.0;
		*pitch = -asin(c);
	}
	else {
		*roll = atan2(SE3(mat,2,1), SE3(mat,2,2));
		*yaw = atan2(SE3(mat,1,0), SE3(mat,0,0));
		float r0_0 = fabs(SE3(mat, 0, 0));
		float r1_0 = fabs(SE3(mat, 1, 0));
		float r2_1 = fabs(SE3(mat, 2, 1));
		float r2_2 = fabs(SE3(mat, 2, 2));

		if (r0_0 >= r1_0 && r0_0 >= r2_1 && r0_0 >= r2_2) {
			*pitch = - atan(SE3(mat,2,0) * cos(*yaw) / SE3(mat, 0, 0));
		}
		else if (r1_0 >= r0_0 && r1_0 >= r2_1 && r1_0 >= r2_2) {
			*pitch = - atan(SE3(mat,2,0) * sin(*yaw) / SE3(mat, 1, 0));
		}
		else if (r2_1 >= r0_0 && r2_1 >= r1_0 && r2_1 >= r2_2) {
			*pitch = - atan(SE3(mat,2,0) * sin(*roll) / SE3(mat, 2, 1));
		}
		else if (r2_2 >= r0_0 && r2_2 >= r1_0 && r2_2 >= r2_1) {
			*pitch = - atan(SE3(mat,2,0) * cos(*roll) / SE3(mat, 2, 2));
		}
		else {
			printf("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAa\n\r");
			for (;;) ;
		}
	}
//	*yaw = atan2(SE3(mat,1,0), SE3(mat,0,0));
//	*pitch = asin(-SE3(mat,2,0));
//	*roll = atan2(SE3(mat,2,1), SE3(mat,2,2));
}

#define BLUE_COLOR "\033[34m"
#define RED_COLOR "\033[31m"
#define WHITE_COLOR "\033[37m"

void SE3_print(SE3_mat * m)
{
	int r, c;
	printf("[");
	for (r = 0; r < 4; r++) {
		if (r > 0) printf(" ");
		printf("[");
		for (c = 0; c < 4; c++) {
			if (r == 3)
				printf(WHITE_COLOR);
			else if (r < 3 && c == 0)
				printf(BLUE_COLOR);
			else if (r < 3 && c == 3)
				printf(RED_COLOR);
			printf("%6.3f  ", m->pData[r * 4 + c]);
			if (c < 3)
				printf(",");
		}
		if (r < 3)
			printf("],\n");
		else
			printf("]");
	}
	printf("]\n");
}
