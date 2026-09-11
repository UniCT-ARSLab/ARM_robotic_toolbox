/*
 * openarm.c
 *
 *  Created on: Aug 31, 2026
 *      Author: corrado
 */

#include "manipulator.h"

Manipulator openarm_model;

#include <stdio.h>

#include "stm32h7xx_hal.h"

#define check(s)	\
	if (s == ARM_MATH_NOMEM) { \
		printf("Not enough memory\n");	\
		for (;;) ; \
	}


/*
┌──────┬─────────────────────────────┬───────┬─────────────────────┬───────────────────────────────────────────────┐
│ link │            link             │ joint │       parent        │              ETS: parent to link              │
├──────┼─────────────────────────────┼───────┼─────────────────────┼───────────────────────────────────────────────┤
│    0 │ world                       │       │ BASE                │ SE3()                                         │
│    1 │ openarm_body_link0          │       │ world               │ SE3()                                         │
│    2 │ openarm_right_link0         │       │ openarm_body_link0  │ SE3(0, -0.031, 0.698; 90°, -0°, 0°)           │
│    3 │ openarm_right_link1         │     0 │ openarm_right_link0 │ SE3(0, 0, 0.0625) ⊕ Rz(q0)                    │
│    4 │ openarm_right_link2         │     1 │ openarm_right_link1 │ SE3(-0.0301, 0, 0.06; 90°, -0°, 0°) ⊕ Rx(-q1) │
│    5 │ openarm_right_link3         │     2 │ openarm_right_link2 │ SE3(0.0301, 0, 0.06625) ⊕ Rz(q2)              │
│    6 │ openarm_right_link4         │     3 │ openarm_right_link3 │ SE3(0, 0.0315, 0.1537) ⊕ Ry(q3)               │
│    7 │ openarm_right_link5         │     4 │ openarm_right_link4 │ SE3(0, -0.0315, 0.0955) ⊕ Rz(q4)              │
│    8 │ openarm_right_link6         │     5 │ openarm_right_link5 │ SE3(0.0375, 0, 0.1205) ⊕ Rx(q5)               │
│    9 │ openarm_right_link7         │     6 │ openarm_right_link6 │ SE3(-0.0375, 0, 0) ⊕ Ry(q6)                   │
│   10 │ openarm_right_hand          │       │ openarm_right_link7 │ SE3(0, 0, 0.1001)                             │
│   11 │ @openarm_right_hand_tcp     │       │ openarm_right_hand  │ SE3(0, 0, 0.08)                               │
│   12 │ @openarm_right_left_finger  │     7 │ openarm_right_hand  │ SE3(0, 0.006, 0.015) ⊕ ty(q7)                 │
│   13 │ @openarm_right_right_finger │     8 │ openarm_right_hand  │ SE3(0, -0.006, 0.015) ⊕ ty(-q8)               │
└──────┴─────────────────────────────┴───────┴─────────────────────┴───────────────────────────────────────────────┘

 */

arm_status openarm_model_init(void)
{
	check(manipulator_init(&openarm_model, 19, 7));

	check(manipulator_add_T_link(&openarm_model, 0, -0.031, 0.698));
	check(manipulator_add_R_link(&openarm_model, PI/2, SE3_X));
	check(manipulator_add_T_link(&openarm_model, 0, 0, 0.065));
	check(manipulator_add_joint(&openarm_model, SE3_Z));	// q0

	check(manipulator_add_T_link(&openarm_model, -0.0301, 0, 0.06));
	check(manipulator_add_R_link(&openarm_model, PI/2, SE3_X));
	check(manipulator_add_joint_ex(&openarm_model, SE3_X, 0, -1.0));	// -q1

	check(manipulator_add_T_link(&openarm_model, 0.0301, 0, 0.06625));
	check(manipulator_add_joint(&openarm_model, SE3_Z));	// q2

	check(manipulator_add_T_link(&openarm_model, 0, 0.0315, 0.1537));
	check(manipulator_add_joint(&openarm_model, SE3_Y));	// q3

	check(manipulator_add_T_link(&openarm_model, 0, -0.0315, 0.0955));
	check(manipulator_add_joint(&openarm_model, SE3_Z));	// q4

	check(manipulator_add_T_link(&openarm_model, 0.0375, 0, 0.1205));
	check(manipulator_add_joint(&openarm_model, SE3_X));	// q5

	check(manipulator_add_T_link(&openarm_model, -0.0375, 0, 0));
	check(manipulator_add_joint(&openarm_model, SE3_Y));	// q6

	check(manipulator_add_T_link(&openarm_model, 0, 0, 0.1001));
	check(manipulator_add_T_link(&openarm_model, 0, 0, 0.08));

	check(manipulator_prepare(&openarm_model));

	return ARM_MATH_SUCCESS;
}

void wait_char(void);

void openarm_model_test(void)
{
	printf("Creating openarm\n");
	openarm_model_init();
	printf("Done\n");
	float q[] = {-0.35, 0.0, 0.0, 0.35, 0.0, 0.0, 1.05};
	//float q[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	manipulator_set_joints(&openarm_model, q);
	SE3_mat ee = manipulator_ee_pose(&openarm_model);
	manipulator_jacob_info(&openarm_model);
	printf("Pinv(jacobe):\n");
	mat_print(&openarm_model.inv_jacobe);
	printf("End-Effector:\n");
	SE3_print(&ee);

	float dq[7];
	SE3_mat target;
	SE3_copy(&target, &ee);
	SE3(&target, 2, 3) = SE3(&target, 2, 3) + 0.2;
	printf("Target Pose:\n");
	SE3_print(&target);

	printf("Starting Servoing:\n");
	assert(manipulator_servo_start(&openarm_model,&target, 1.0, 0.01) == ARM_MATH_SUCCESS);

	while (1) {
		uint32_t st = HAL_GetTick();
		bool ret = manipulator_servo_control(&openarm_model, dq);
		manipulator_move_joints(&openarm_model, 0.01, dq);
		uint32_t en = HAL_GetTick();
		printf("elapsed: %ld ms, %d:", (en - st), ret);
		int i;
		for (i = 0; i < 7;i++)
			printf("%.3f ", dq[i]);
		printf("\n");
		if (ret)
			break;
		SE3_mat ee = manipulator_ee_pose(&openarm_model);
		SE3_print(&ee);
		//wait_char();
	}
}

