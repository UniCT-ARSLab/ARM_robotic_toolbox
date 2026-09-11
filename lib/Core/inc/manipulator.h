/*
 * manipulator.h
 *
 *  Created on: Aug 31, 2026
 *      Author: corrado
 */

#ifndef INC_MANIPULATOR_H_
#define INC_MANIPULATOR_H_

#include <stdbool.h>
#include "arm_rtb.h"

typedef arm_matrix_instance_f32 	Jacob_mat;

typedef struct {
	SE3_mat *target_pose;
	SE3_mat error_matrix, ee_inv;
	float threshold;
	float gain;
} t_servo_info;

typedef struct {
	int	num_links;
	int link_index;

	int	num_joints;
	int joint_index;

	SE3_mat *transforms;
	SE3_mat *global_transforms;
	int *joint_indexes;

	SE3_Axis *joint_axis;

	float * joint_angles;
	float * joint_offsets;
	float * joint_signs;

	SE3_mat new_t, prev_t;

	Jacob_mat jacob0, jacobe, jacobe_T;

	SO3_mat  r_ee, r_ee_T, origin_skew, RT_p_hat;
	arm_matrix_instance_f32 jacob_transform, jacobe_T_jacobe, inv_jacobe_T_jacobe, inv_jacobe;

	t_servo_info servo;

} Manipulator;

arm_status manipulator_init(Manipulator * m, int n_links, int n_joints);
arm_status manipulator_prepare(Manipulator * m);
arm_status manipulator_add_link(Manipulator * m, SE3_mat * t);
arm_status manipulator_add_T_link(Manipulator * m, float x, float y, float z);
arm_status manipulator_add_R_link(Manipulator * m, float angle, SE3_Axis axis);
arm_status manipulator_add_joint(Manipulator * m, SE3_Axis axis);
arm_status manipulator_add_joint_ex(Manipulator * m, SE3_Axis axis, float offset, float sign);
arm_status manipulator_set_joints(Manipulator * m, float * joints);
arm_status manipulator_move_joints(Manipulator * m, float delta_t, float * joint_speeds);
arm_status manipulator_compute_transform(Manipulator * m);
SE3_mat manipulator_ee_pose(Manipulator * m);
arm_status manipulator_jacobians(Manipulator * m);
void manipulator_jacob_info(Manipulator * m);
arm_status manipulator_servo_start(Manipulator * m, SE3_mat * target_pose, float gain, float threshold);
bool manipulator_servo_control(Manipulator * m, float * joint_speeds);


#endif /* INC_MANIPULATOR_H_ */
