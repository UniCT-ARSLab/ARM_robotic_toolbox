/*
 * so3.c
 *
 *  Created on: Sep 6, 2026
 *      Author: corrado
 */

#include "arm_rtb.h"

arm_status SO3_I(SO3_mat * mat)
{
	float * pData = rtb_allocate(sizeof(float) * 9);
	if (pData == NULL)
		return ARM_MATH_NOMEM;
	arm_mat_init_f32(mat, 3, 3, pData);
	SO3(mat, 0, 0) = 1;		SO3(mat, 0, 1) = 0;		SO3(mat, 0, 2) = 0;
	SO3(mat, 1, 0) = 0;		SO3(mat, 1, 1) = 1;		SO3(mat, 1, 2) = 0;
	SO3(mat, 2, 0) = 0;		SO3(mat, 2, 1) = 0;		SO3(mat, 2, 2) = 1;
	return ARM_MATH_SUCCESS;
}

arm_status SO3_Skew(Vector3 * v, SO3_mat * mat)
{
	SO3(mat, 0, 0) = 0;		SO3(mat, 0, 1) = -v->z;	SO3(mat, 0, 2) = v->y;
	SO3(mat, 1, 0) = v->z;	SO3(mat, 1, 1) = 0;		SO3(mat, 1, 2) = -v->x;
	SO3(mat, 2, 0) = -v->y;	SO3(mat, 2, 1) = v->x;	SO3(mat, 2, 2) = 0;
	return ARM_MATH_SUCCESS;
}

void SO3_delete(SO3_mat * mat)
{
	rtb_free(mat->pData);
}

