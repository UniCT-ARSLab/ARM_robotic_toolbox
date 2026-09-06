/*
 * arm_rtb.h
 *
 *  Created on: Sep 6, 2026
 *      Author: corrado
 */

#ifndef LIB_CORE_INC_ARM_RTB_H_
#define LIB_CORE_INC_ARM_RTB_H_

#include "so3.h"
#include "se3.h"
#include "manipulator.h"

void * rtb_allocate(int size);
void rtb_free(void * ptr);

void mat_multiply_scalar(arm_matrix_instance_f32 * m, float scalar);

#define ARM_MATH_NOMEM			-10


#endif /* LIB_CORE_INC_ARM_RTB_H_ */
