/*
 * arm_rtb.c
 *
 *  Created on: Sep 6, 2026
 *      Author: corrado
 */

#include "arm_rtb.h"
#include <stdlib.h>

__attribute__((weak)) void * rtb_allocate(int size)
{
	return malloc(size);
}

__attribute__((weak)) void rtb_free(void * ptr)
{
	free(ptr);
}

void mat_multiply_scalar(arm_matrix_instance_f32 * m, float scalar)
{
	int i;
	for (i = 0; i < m->numCols * m->numRows;i++)
		m->pData[i] *= scalar;
}

