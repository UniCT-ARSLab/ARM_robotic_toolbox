/*
 * openarm.c
 *
 *  Created on: Aug 31, 2026
 *      Author: corrado
 */

#include "openarm_model.h"
#include "openarm_driver.h"

#include <stdio.h>

#include "stm32h7xx_hal.h"

#define check(s)	\
	if (s == ARM_MATH_NOMEM) { \
		printf("Not enough memory\n");	\
		for (;;) ; \
	}

#define MODEL(side)		(side == ARM_LEFT ? &openarm_left_model : &openarm_right_model)

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

/*
┌──────┬────────────────────────────┬───────┬────────────────────┬────────────────────────────────────────────────┐
│ link │            link            │ joint │       parent       │              ETS: parent to link               │
├──────┼────────────────────────────┼───────┼────────────────────┼────────────────────────────────────────────────┤
│    0 │ world                      │       │ BASE               │ SE3()                                          │
│    1 │ openarm_body_link0         │       │ world              │ SE3()                                          │
│    2 │ openarm_left_link0         │       │ openarm_body_link0 │ SE3(0, 0.031, 0.698; -90°, -0°, 0°)            │
│    3 │ openarm_left_link1         │     0 │ openarm_left_link0 │ SE3(0, 0, 0.0625) ⊕ Rz(q0)                     │
│    4 │ openarm_left_link2         │     1 │ openarm_left_link1 │ SE3(-0.0301, 0, 0.06; -90°, -0°, 0°) ⊕ Rx(-q1) │
│    5 │ openarm_left_link3         │     2 │ openarm_left_link2 │ SE3(0.0301, 0, 0.06625) ⊕ Rz(q2)               │
│    6 │ openarm_left_link4         │     3 │ openarm_left_link3 │ SE3(0, 0.0315, 0.1537) ⊕ Ry(q3)                │
│    7 │ openarm_left_link5         │     4 │ openarm_left_link4 │ SE3(0, -0.0315, 0.0955) ⊕ Rz(q4)               │
│    8 │ openarm_left_link6         │     5 │ openarm_left_link5 │ SE3(0.0375, 0, 0.1205) ⊕ Rx(q5)                │
│    9 │ openarm_left_link7         │     6 │ openarm_left_link6 │ SE3(-0.0375, 0, 0) ⊕ Ry(-q6)                   │
│   10 │ openarm_left_hand          │       │ openarm_left_link7 │ SE3(0, 0, 0.1001)                              │
│   11 │ @openarm_left_hand_tcp     │       │ openarm_left_hand  │ SE3(0, 0, 0.08)                                │
│   12 │ @openarm_left_left_finger  │     7 │ openarm_left_hand  │ SE3(0, 0.006, 0.015) ⊕ ty(q7)                  │
│   13 │ @openarm_left_right_finger │     8 │ openarm_left_hand  │ SE3(0, -0.006, 0.015) ⊕ ty(-q8)                │
└──────┴────────────────────────────┴───────┴────────────────────┴────────────────────────────────────────────────┘
 */


arm_status openarm_model_init(t_openarm_robot * robot)
{
	if (robot->side == ARM_RIGHT) {
		check(manipulator_init(&robot->model, 19, 7));

		check(manipulator_add_T_link(&robot->model, 0, -0.031, 0.698));
		check(manipulator_add_R_link(&robot->model, PI/2, SE3_X));
		check(manipulator_add_T_link(&robot->model, 0, 0, 0.065));
		check(manipulator_add_joint(&robot->model, SE3_Z));	// q0

		check(manipulator_add_T_link(&robot->model, -0.0301, 0, 0.06));
		check(manipulator_add_R_link(&robot->model, PI/2, SE3_X));
		check(manipulator_add_joint_ex(&robot->model, SE3_X, 0, -1.0));	// -q1

		check(manipulator_add_T_link(&robot->model, 0.0301, 0, 0.06625));
		check(manipulator_add_joint(&robot->model, SE3_Z));	// q2

		check(manipulator_add_T_link(&robot->model, 0, 0.0315, 0.1537));
		check(manipulator_add_joint(&robot->model, SE3_Y));	// q3

		check(manipulator_add_T_link(&robot->model, 0, -0.0315, 0.0955));
		check(manipulator_add_joint(&robot->model, SE3_Z));	// q4

		check(manipulator_add_T_link(&robot->model, 0.0375, 0, 0.1205));
		check(manipulator_add_joint(&robot->model, SE3_X));	// q5

		check(manipulator_add_T_link(&robot->model, -0.0375, 0, 0));
		check(manipulator_add_joint(&robot->model, SE3_Y));	// q6

		check(manipulator_add_T_link(&robot->model, 0, 0, 0.1001));
		check(manipulator_add_T_link(&robot->model, 0, 0, 0.08));

		check(manipulator_prepare(&robot->model));
	}

	// -------------------------------------------------------------------
	else {
		check(manipulator_init(&robot->model, 19, 7));

		check(manipulator_add_T_link(&robot->model, 0, 0.031, 0.698));
		check(manipulator_add_R_link(&robot->model, -PI/2, SE3_X));
		check(manipulator_add_T_link(&robot->model, 0, 0, 0.065));
		check(manipulator_add_joint(&robot->model, SE3_Z));	// q0

		check(manipulator_add_T_link(&robot->model, -0.0301, 0, 0.06));
		check(manipulator_add_R_link(&robot->model, -PI/2, SE3_X));
		check(manipulator_add_joint_ex(&robot->model, SE3_X, 0, -1.0));	// -q1

		check(manipulator_add_T_link(&robot->model, 0.0301, 0, 0.06625));
		check(manipulator_add_joint(&robot->model, SE3_Z));	// q2

		check(manipulator_add_T_link(&robot->model, 0, 0.0315, 0.1537));
		check(manipulator_add_joint(&robot->model, SE3_Y));	// q3

		check(manipulator_add_T_link(&robot->model, 0, -0.0315, 0.0955));
		check(manipulator_add_joint(&robot->model, SE3_Z));	// q4

		check(manipulator_add_T_link(&robot->model, 0.0375, 0, 0.1205));
		check(manipulator_add_joint(&robot->model, SE3_X));	// q5

		check(manipulator_add_T_link(&robot->model, -0.0375, 0, 0));
		check(manipulator_add_joint_ex(&robot->model, SE3_Y, 0, -1.0));	// -q6

		check(manipulator_add_T_link(&robot->model, 0, 0, 0.1001));
		check(manipulator_add_T_link(&robot->model, 0, 0, 0.08));

		check(manipulator_prepare(&robot->model));
	}

	return ARM_MATH_SUCCESS;
}


void wait_char(void);

void openarm_model_set_joints(t_openarm_robot * robot, float * q, float speed)
{
	manipulator_set_joints(&robot->model, q);
	openarm_set_joints(robot->canif, q, speed);
}

void openarm_jacob_info(t_openarm_robot * robot)
{
	manipulator_jacob_info(&robot->model);
}

void openarm_model_show_ee_pose(t_openarm_robot * robot)
{
	SE3_mat ee = manipulator_ee_pose(&robot->model);
	printf("End-Effector:\n");
	SE3_print(&ee);
}

void openarm_model_servo_drive3(t_openarm_robot * robot, float * xyz)
{
	Manipulator * model = &robot->model;

	SE3_mat ee = manipulator_ee_pose(model);

	SE3_mat target;
	SE3_copy(&target, &ee);

	Vector3 new_origin;
	vector3_init_v(&new_origin, xyz);
	SE3_set_origin(&target, &new_origin);

	printf("Target Pose:\n");
	SE3_print(&target);

	printf("Starting Servoing:\n");
	assert(manipulator_servo_start(model, &target, 1.0, 0.05) == ARM_MATH_SUCCESS);

	float dq[7];

	while (1) {
		uint32_t st = HAL_GetTick();
		bool ret = manipulator_servo_control(model, dq);
		manipulator_move_joints(model, 0.001, dq);
		uint32_t en = HAL_GetTick();
		//printf("elapsed: %ld ms, %d:", (en - st), ret);
//		int i;
//		for (i = 0; i < 7;i++)
//			printf("%.3f ", dq[i]);
//		printf("\n");
		if (ret)
			break;
		SE3_mat ee = manipulator_ee_pose(model);
		//SE3_print(&ee);
		//wait_char();
	}
}

void openarm_model_servo_drive(t_openarm_robot * robot, SE3_mat * target, float delta_t, float gain, float threshold, bool drive)
{
	Manipulator * model = &robot->model;

//	printf("Target Pose:\n");
//	SE3_print(target);

	//printf("Starting Servoing:\n");
	assert(manipulator_servo_start(model, target, gain, threshold) == ARM_MATH_SUCCESS);

	float dq[7];

	while (1) {
		//uint32_t st = HAL_GetTick();
		bool ret = manipulator_servo_control(model, dq);
		manipulator_move_joints(model, delta_t, dq);
		if (drive) {
			float j[8];
			manipulator_get_joints(model, j);
			openarm_set_joints(robot->canif, j, 10 * M_PI);
//			int i;
//			for (i = 0; i < 7;i++)
//				printf("%6.3f, ", j[i]);
//			printf("\n");
		}
		//uint32_t en = HAL_GetTick();
		HAL_Delay(delta_t * 1000);
		if (ret)
			break;
		if (drive) {
			// read motor position
		}
		//SE3_mat ee = manipulator_ee_pose(&openarm_model);
		//SE3_print(&ee);
	}
	if (drive)
		openarm_set_zero_speed(robot->canif);
}


void openarm_model_segment_drive(t_openarm_robot * robot, SE3_mat * target, t_speed_profile sp, float delta_t, float gain, float threshold, bool drive)
{
	Vector3 error;
	t_motion mot;
	Quaternion q_start, q_end;
	float omega;

	Manipulator * model = &robot->model;

	SE3_mat ee = manipulator_ee_pose(model);
	SE3_copy_N(&model->segment_start, &ee);

	SE3_mat * start = &model->segment_start;

	// the quaternions relative to start and end pose
	SE3_to_Quat(start, &q_start);
	SE3_to_Quat(target, &q_end);
	// the angle between the two quaternions
	omega = Quaternions_Angle(&q_start, &q_end);

	error.x = SE3(target, 0, 3) - SE3(start, 0, 3);
	error.y = SE3(target, 1, 3) - SE3(start, 1, 3);
	error.z = SE3(target, 2, 3) - SE3(start, 2, 3);

	printf("Start Pose:\n");
	SE3_print(start);
	Quaternion_print(&q_start);

	printf("Target Pose:\n");
	SE3_print(target);
	Quaternion_print(&q_end);

	printf("Quaternion angle = %f\n\r", omega);

	float distance = vector3_norm(&error);

	vr_start_motion(&mot, sp, distance);

	float pos;
	float dq[7];

	uint32_t last_tm = HAL_GetTick();
	while (!vr_next_motion(&mot, delta_t, &pos)) {
		uint32_t tm = HAL_GetTick();
		CAN_process(robot->canif);
		// interpolate position
		float fraction = pos / distance;

		Quaternion virtual_robot_q;
		Quaternions_Slerp(&virtual_robot_q, &q_start, &q_end, omega, fraction);

		SE3_from_Quat_N(&model->virtual_robot_temp, &virtual_robot_q);
		SE3(&model->virtual_robot_temp, 0, 3) = SE3(start, 0, 3) + error.x * fraction;
		SE3(&model->virtual_robot_temp, 1, 3) = SE3(start, 1, 3) + error.y * fraction;
		SE3(&model->virtual_robot_temp, 2, 3) = SE3(start, 2, 3) + error.z * fraction;

		//printf("Step %f:", fraction);
		//Quaternion_print(&virtual_robot_q);
		//SE3_print(&openarm_model.virtual_robot_temp);

		float dt = (tm - last_tm) / 1000.0;
		last_tm = tm;
		int delay_ticks = 0;

		float real_dt;
		if (dt < delta_t) {
			delay_ticks = (delta_t - dt) * 1000;
			real_dt = delta_t;
		}
		else {
			delay_ticks = 0;
			real_dt = dt;
		}
		//printf("Elapsed  %f, Expected %f, Delay Ticks %d, %f, %f\n\r", dt, delta_t, delay_ticks, pos, distance);

		// start motion
		assert(manipulator_servo_start(model, &model->virtual_robot_temp, gain, threshold) == ARM_MATH_SUCCESS);
		// make motion
		manipulator_servo_control(model, dq);
		manipulator_move_joints(model, real_dt, dq);
		if (drive) {
			float j[8];
			manipulator_get_joints(model, j);
			openarm_set_joints(robot->canif, j, 10 * M_PI);
			openarm_get_joints(robot->canif);
		}
		HAL_Delay(delay_ticks);
	}
	openarm_model_servo_drive(robot, target, delta_t, gain, threshold, drive);
}


void openarm_model_segment_start(t_openarm_robot * robot, SE3_mat * target, t_speed_profile sp, float delta_t, float gain, float threshold)
{
	Manipulator * model = &robot->model;

	SE3_mat ee = manipulator_ee_pose(model);
	SE3_copy_N(&model->segment_start, &ee);

	SE3_mat * start = &model->segment_start;

	// the quaternions relative to start and end pose
	SE3_to_Quat(start, &model->segment.q_start);
	SE3_to_Quat(target, &model->segment.q_end);
	// the angle between the two quaternions
	model->segment.omega = Quaternions_Angle(&model->segment.q_start, &model->segment.q_end);

	model->segment.error.x = SE3(target, 0, 3) - SE3(start, 0, 3);
	model->segment.error.y = SE3(target, 1, 3) - SE3(start, 1, 3);
	model->segment.error.z = SE3(target, 2, 3) - SE3(start, 2, 3);

//	printf("Start Pose:\n");
//	SE3_print(start);
//	Quaternion_print(&model->segment.q_start);
//
//	printf("Target Pose:\n");
//	SE3_print(target);
//	Quaternion_print(&model->segment.q_end);
//
//	printf("Quaternion angle = %f\n\r", model->segment.omega);

	model->segment.distance = vector3_norm(&model->segment.error);

	vr_start_motion(&model->segment.mot, sp, model->segment.distance);

	model->segment.gain = gain;
	model->segment.threshold = threshold;
	model->segment.target = target;

}

//#define VERBOSE

bool openarm_model_segment_step(t_openarm_robot * robot, float delta_t, bool drive)
{
	float pos;
	float dq[7];

	Manipulator * model = &robot->model;
	t_motion * mot = &model->segment.mot;
	SE3_mat * start = &model->segment_start;

	if (!vr_next_motion(mot, delta_t, &pos)) {
		CAN_process(robot->canif);

		// interpolate position
		float fraction = pos / model->segment.distance;

		Quaternion virtual_robot_q;
		Quaternions_Slerp(&virtual_robot_q,
						&model->segment.q_start, &model->segment.q_end,
						model->segment.omega, fraction);

		SE3_from_Quat_N(&model->virtual_robot_temp, &virtual_robot_q);
		SE3(&model->virtual_robot_temp, 0, 3) = SE3(start, 0, 3) + model->segment.error.x * fraction;
		SE3(&model->virtual_robot_temp, 1, 3) = SE3(start, 1, 3) + model->segment.error.y * fraction;
		SE3(&model->virtual_robot_temp, 2, 3) = SE3(start, 2, 3) + model->segment.error.z * fraction;

#ifdef VERBOSE
		printf("Step %f:", fraction);
		Quaternion_print(&virtual_robot_q);
		//SE3_print(&model->virtual_robot_temp);
#endif
		//printf("Elapsed  %f, Expected %f, Delay Ticks %d, %f, %f\n\r", dt, delta_t, delay_ticks, pos, distance);

		// start motion
		assert(manipulator_servo_start(model, &model->virtual_robot_temp, model->segment.gain, model->segment.threshold) == ARM_MATH_SUCCESS);
		// make motion
		manipulator_servo_control(model, dq);
		manipulator_move_joints(model, delta_t, dq);
		if (drive) {
			float j[8];
			manipulator_get_joints(model, j);
			openarm_set_joints(robot->canif, j, 10 * M_PI);
			openarm_get_joints(robot->canif);
		}
		return false;
	}
	else {
		openarm_model_servo_drive(robot, model->segment.target, delta_t, model->segment.gain, model->segment.threshold, drive);
		return true;
	}
}


#if 0
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
		for (i = 0; i < 7;i++)  openarm_model_test();

			printf("%.3f ", dq[i]);
		printf("\n");
		if (ret)
			break;
		SE3_mat ee = manipulator_ee_pose(&openarm_model);
		SE3_print(&ee);
		//wait_char();
	}
}
#endif
