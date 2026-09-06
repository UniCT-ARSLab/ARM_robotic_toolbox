/*
 * so3.h
 *
 *  Created on: Sep 6, 2026
 *      Author: corrado
 */

#ifndef LIB_CORE_INC_SO3_H_
#define LIB_CORE_INC_SO3_H_

#include "arm_math.h"
#include "vector3.h"

#define SO3(m,row,col)		m->pData[row * 3 + col]

typedef arm_matrix_instance_f32 	SO3_mat;

arm_status SO3_I(SO3_mat * mat);
void SO3_delete(SO3_mat * mat);
arm_status SO3_Skew(Vector3 * v, SO3_mat * mat);



#endif /* LIB_CORE_INC_SO3_H_ */
