/*
 * se3.h
 *
 *  Created on: Aug 31, 2026
 *      Author: corrado
 */

#ifndef INC_SE3_H_
#define INC_SE3_H_

#include "arm_math.h"
#include "vector3.h"

#define ARM_MATH_NOMEM			-10

typedef enum {
	SE3_X = 0,
	SE3_Y,
	SE3_Z
} SE3_Axis;

typedef arm_matrix_instance_f32 	SE3_mat;
typedef arm_matrix_instance_f32 	SO3_mat;

#define SE3(m,row,col)		m->pData[row * 4 + col]
#define SO3(m,row,col)		m->pData[row * 3 + col]

void * rtb_allocate(int size);
void rtb_free(void * ptr);

void mat_multiply_scalar(arm_matrix_instance_f32 * m, float scalar);

void SE3_delete(SE3_mat * mat);

arm_status SO3_I(SO3_mat * mat);
void SO3_delete(SO3_mat * mat);
arm_status SO3_Skew(Vector3 * v, SO3_mat * mat);

arm_status SE3_Rx(SE3_mat * mat, float angle);
arm_status SE3_Rx_N(SE3_mat * mat, float angle);
arm_status SE3_Ry(SE3_mat * mat, float angle);
arm_status SE3_Ry_N(SE3_mat * mat, float angle);
arm_status SE3_Rz(SE3_mat * mat, float angle);
arm_status SE3_Rz_N(SE3_mat * mat, float angle);
arm_status SE3_Trans(SE3_mat * mat, float * xyz);
arm_status SE3_Trans_N(SE3_mat * mat, float * xyz);
arm_status SE3_I(SE3_mat * mat);

arm_status SE3_Multiply_N(SE3_mat * result, SE3_mat * a, SE3_mat * b);
arm_status SE3_Multiply(SE3_mat * result, SE3_mat * a, SE3_mat * b);
void SE3_copy_N(SE3_mat * result, SE3_mat * a);
void SE3_origin(SE3_mat * mat, Vector3 * v);
void SE3_basis(SE3_mat * mat,SO3_mat * basis);
void SE3_basis_x(SE3_mat * mat, Vector3 * v);
void SE3_basis_y(SE3_mat * mat, Vector3 * v);
void SE3_basis_z(SE3_mat * mat, Vector3 * v);

void SE3_print(SE3_mat * m);

#endif /* INC_SE3_H_ */
