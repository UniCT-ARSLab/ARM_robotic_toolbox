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

void SE3_to_rpy(SE3_mat * mat, float * roll, float * pitch, float * yaw)
{
	*yaw = asin(-SE3(mat,2,0));
	*pitch = atan2(SE3(mat,1,0), SE3(mat,0,0));
	*roll = atan2(SE3(mat,2,1), SE3(mat,2,2));
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
