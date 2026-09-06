/*
 * se3.h
 *
 *  Created on: Aug 31, 2026
 *      Author: corrado
 */

#ifndef INC_SE3_H_
#define INC_SE3_H_

#include "so3.h"


typedef enum {
	SE3_X = 0,
	SE3_Y,
	SE3_Z
} SE3_Axis;

typedef arm_matrix_instance_f32 	SE3_mat;

#define SE3(m,row,col)		m->pData[row * 4 + col]

void SE3_delete(SE3_mat * mat);

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
