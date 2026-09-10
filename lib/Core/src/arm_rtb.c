/*
 * arm_rtb.c
 *
 *  Created on: Sep 6, 2026
 *      Author: corrado
 */

#include "arm_rtb.h"
#include <stdio.h>
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

void mat_print(arm_matrix_instance_f32 * m)
{
	int r, c;
	printf("[");
	for (r = 0; r < m->numRows; r++) {
		if (r > 0) printf(" ");
		printf("[");
		for (c = 0; c < m->numCols; c++) {
			printf("%6.3f  ", m->pData[r * m->numCols + c]);
			if (c < m->numCols - 1)
				printf(",");
		}
		if (r < m->numRows)
			printf("],\n");
		else
			printf("]");
	}
	printf("]\n");
}

