/*
 * manipulator.c
 *
 *  Created on: Aug 31, 2026
 *      Author: corrado
 */


#include "arm_rtb.h"

#include <assert.h>
#include <stdio.h>

#define math_success(v)		assert(v == ARM_MATH_SUCCESS)
//#define TRANSFORM_DEBUG

arm_status manipulator_init(Manipulator * m, int n_links, int n_joints)
{
	// transforms
	SE3_mat *t = rtb_allocate(n_links * sizeof(SE3_mat));
	if (t == NULL)
		return ARM_MATH_NOMEM;

	// global transforms
	SE3_mat *gt = rtb_allocate(n_links * sizeof(SE3_mat));
	if (t == NULL)
		return ARM_MATH_NOMEM;

	// joint indexes
	int * j = rtb_allocate(n_joints * sizeof(int));
	if (j == NULL)
		return ARM_MATH_NOMEM;

	// joint axis
	SE3_Axis * jax = rtb_allocate(n_joints * sizeof(SE3_Axis));
	if (jax == NULL)
		return ARM_MATH_NOMEM;

	// joint angles
	float * ja = rtb_allocate(n_joints * sizeof(float));
	if (ja == NULL)
		return ARM_MATH_NOMEM;

	// joint signs
	float * js = rtb_allocate(n_joints * sizeof(float));
	if (js == NULL)
		return ARM_MATH_NOMEM;

	// joint offsets
	float * jo = rtb_allocate(n_joints * sizeof(float));
	if (jo == NULL)
		return ARM_MATH_NOMEM;

	if (SE3_I(&m->new_t) == ARM_MATH_NOMEM)
		return ARM_MATH_NOMEM;
	if (SE3_I(&m->prev_t) == ARM_MATH_NOMEM)
		return ARM_MATH_NOMEM;

	m->transforms = t;
	m->joint_indexes = j;
	m->joint_axis = jax;
	m->joint_angles = ja;
	m->joint_signs = js;
	m->joint_offsets = jo;
	m->num_joints = n_joints;
	m->num_links = n_links;
	m->link_index = 0;
	m->joint_index = 0;
	m->global_transforms = gt;
	int i;
	for (i = 0; i < n_links;i++)
		m->global_transforms[i].pData = NULL;

	m->servo.error_matrix.pData = NULL;
	return ARM_MATH_SUCCESS;
}

arm_status manipulator_add_link(Manipulator * m, SE3_mat * t)
{
	SE3_mat mat;
	if (SE3_I(&mat) == ARM_MATH_NOMEM)
		return ARM_MATH_NOMEM;
	SE3_copy_N(&mat, t);
	m->transforms[m->link_index] = mat;
	m->link_index++;
	return ARM_MATH_SUCCESS;
}

arm_status manipulator_add_T_link(Manipulator * m, float x, float y, float z)
{
	SE3_mat mat;
	float t[3] = {x, y, z};
	if (SE3_Trans(&mat, t) == ARM_MATH_NOMEM)
		return ARM_MATH_NOMEM;
	m->transforms[m->link_index] = mat;
	m->link_index++;
	return ARM_MATH_SUCCESS;
}

arm_status manipulator_add_R_link(Manipulator * m, float angle, SE3_Axis axis)
{
	SE3_mat mat;
	switch (axis) {
	case SE3_X:
		if (SE3_Rx(&mat, angle) == ARM_MATH_NOMEM)
			return ARM_MATH_NOMEM;
		break;
	case SE3_Y:
		if (SE3_Ry(&mat, angle) == ARM_MATH_NOMEM)
			return ARM_MATH_NOMEM;
		break;
	case SE3_Z:
		if (SE3_Rz(&mat, angle) == ARM_MATH_NOMEM)
			return ARM_MATH_NOMEM;
		break;
	}
	m->transforms[m->link_index] = mat;
	m->link_index++;
	return ARM_MATH_SUCCESS;
}

arm_status manipulator_add_joint(Manipulator * m, SE3_Axis axis)
{
	SE3_mat mat;
	if (SE3_I(&mat) == ARM_MATH_NOMEM)
		return ARM_MATH_NOMEM;
	m->transforms[m->link_index] = mat;
	m->joint_axis[m->joint_index] = axis;
	m->joint_indexes[m->joint_index] = m->link_index;
	m->joint_offsets[m->joint_index] = 0.;
	m->joint_signs[m->joint_index] = 1.;
	m->link_index++;
	m->joint_index++;
	return ARM_MATH_SUCCESS;
}

arm_status manipulator_add_joint_ex(Manipulator * m, SE3_Axis axis, float offset, float sign)
{
	arm_status ret = manipulator_add_joint(m, axis);
	if (ret != ARM_MATH_SUCCESS)
		return ret;
	m->joint_offsets[m->joint_index - 1] = offset;
	m->joint_signs[m->joint_index - 1] = sign;

	return ARM_MATH_SUCCESS;
}

arm_status manipulator_prepare(Manipulator * m)
{
	// the jacobian of the base (jacob0)
	float * f = rtb_allocate(sizeof(float) * 6 * m->num_joints);
	if (f == NULL)
		return ARM_MATH_NOMEM;
	arm_mat_init_f32(&m->jacob0, 6, m->num_joints, f);

	// the jacobian of the end-effector (jacobe)
	f = rtb_allocate(sizeof(float) * 6 * m->num_joints);
	if (f == NULL)
		return ARM_MATH_NOMEM;
	arm_mat_init_f32(&m->jacobe, 6, m->num_joints, f);

	// the jacobe transposed
	f = rtb_allocate(sizeof(float) * 6 * m->num_joints);
	if (f == NULL)
		return ARM_MATH_NOMEM;
	arm_mat_init_f32(&m->jacobe_T, m->num_joints, 6, f);

	// the jacobe transposed * jacobe, useful to compute the pseudo inverse
	f = rtb_allocate(sizeof(float) * m->num_joints * m->num_joints);
	if (f == NULL)
		return ARM_MATH_NOMEM;
	arm_mat_init_f32(&m->jacobe_T_jacobe, m->num_joints, m->num_joints, f);

	// the inv(jacobe transposed * jacobe), useful to compute the pseudo inverse
	f = rtb_allocate(sizeof(float) * m->num_joints * m->num_joints);
	if (f == NULL)
		return ARM_MATH_NOMEM;
	arm_mat_init_f32(&m->inv_jacobe_T_jacobe, m->num_joints, m->num_joints, f);

	// the jacobe pseudo inverse
	f = rtb_allocate(sizeof(float) * 6 * m->num_joints);
	if (f == NULL)
		return ARM_MATH_NOMEM;
	arm_mat_init_f32(&m->inv_jacobe, m->num_joints, 6, f);

	// the matrix useful to transform jacob0 to jacobe
	f = rtb_allocate(sizeof(float) * 6 * 6);
	if (f == NULL)
		return ARM_MATH_NOMEM;
	int i;
	for (i = 0; i < 36;i++) f[i] = 0.0;
	arm_mat_init_f32(&m->jacob_transform, 6, 6, f);

	int ret = SO3_I(&m->r_ee);
	if (ret != ARM_MATH_SUCCESS)
		return ret;
	ret = SO3_I(&m->r_ee_T);
	if (ret != ARM_MATH_SUCCESS)
		return ret;
	ret = SO3_I(&m->origin_skew);
	if (ret != ARM_MATH_SUCCESS)
		return ret;
	ret = SO3_I(&m->RT_p_hat);
	return ret;
}


arm_status manipulator_set_joints(Manipulator * m, float * joints)
{
	int i;
	for (i = 0; i < m->num_joints;i++) {
		float a = joints[i];
		a = a * m->joint_signs[i] + m->joint_offsets[i];
		m->joint_angles[i] = a;

		int jindex = m->joint_indexes[i];
		SE3_Axis axis = m->joint_axis[i];

		switch (axis) {
		case SE3_X:
			SE3_Rx_N(&m->transforms[jindex], a);
			break;
		case SE3_Y:
			SE3_Ry_N(&m->transforms[jindex], a);
			break;
		case SE3_Z:
			SE3_Rz_N(&m->transforms[jindex], a);
			break;
		}
	}
	int ret = manipulator_compute_transform(m);
	if (ret != ARM_MATH_SUCCESS)
		return ret;
	return manipulator_jacobians(m);
}

arm_status manipulator_compute_transform(Manipulator * m)
{
	int i;
	SE3_copy_N(&m->prev_t,&m->transforms[0]);
	if (m->global_transforms[0].pData == NULL) {
		if (SE3_I(&m->global_transforms[0]) == ARM_MATH_NOMEM)
			return ARM_MATH_NOMEM;
	}
	SE3_copy_N(&m->global_transforms[0], &m->prev_t);

#ifdef TRANSFORM_DEBUG
	printf("Index: 0\n");
	printf("T:\n");SE3_print(&m->transforms[0]);
	printf("GT:\n");SE3_print(&m->global_transforms[0]);
	printf("\n");
#endif

	for (i = 1; i < m->num_links;i++) {
		SE3_Multiply_N(&m->new_t, &m->prev_t, &m->transforms[i]);
		if (m->global_transforms[i].pData == NULL) {
			if (SE3_I(&m->global_transforms[i]) == ARM_MATH_NOMEM)
				return ARM_MATH_NOMEM;
		}
		SE3_copy_N(&m->global_transforms[i], &m->new_t);
#ifdef TRANSFORM_DEBUG
		printf("Index: %d\n",i);
		printf("T:\n");SE3_print(&m->transforms[i]);
		printf("GT:\n");SE3_print(&m->global_transforms[i]);
		printf("\n");
#endif
		SE3_copy_N(&m->prev_t, &m->new_t);
	}
	return ARM_MATH_SUCCESS;
}

#define EE	(m->global_transforms[m->num_links - 1])

SE3_mat manipulator_ee_pose(Manipulator * m)
{
	return EE;
}

static void _jacob_make_transform(Manipulator * m, SO3_mat * rot_T, Vector3 * origin)
{
	int r,c;
	//SO3_Skew(origin, &m->origin_skew);
	//arm_mat_mult_f32(rot_T, &m->origin_skew, &m->RT_p_hat);
	//mat_multiply_scalar(&m->RT_p_hat, -1.0);
	for (r = 0; r < 3; r++) {
		for (c = 0; c < 3; c++) {
			MAT_EL(m->jacob_transform, r, c) = SO3(rot_T, r, c);
			MAT_EL(m->jacob_transform, r + 3, c + 3) = SO3(rot_T, r, c);
			//MAT_EL(m->jacob_transform, r, c + 3) = SO3((&m->RT_p_hat), r, c);
		}
	}
}

arm_status manipulator_jacobians(Manipulator * m)
{
	int i;

	SE3_mat ee_pose = manipulator_ee_pose(m);
	Vector3 ee_origin;
	SE3_origin(&ee_pose, &ee_origin);
	for (i = 0; i < m->num_joints;i++) {
		int jindex = m->joint_indexes[i];
		SE3_mat * t = &m->global_transforms[jindex];

		Vector3 origin, j_v_i, j_w_i, dist;
		SE3_origin(t, &origin);

		vector3_sub(&dist, &ee_origin, &origin);

		switch(m->joint_axis[i]) {
		case SE3_X:
			SE3_basis_x(t, &j_w_i);
			break;
		case SE3_Y:
			SE3_basis_y(t, &j_w_i);
			break;
		case SE3_Z:
			SE3_basis_z(t, &j_w_i);
			break;
		}
		//printf("%d: %f, %f, %f\n", i, j_w_i.x, j_w_i.y, j_w_i.z);
		vector3_multiply(&j_w_i, &j_w_i, m->joint_signs[i]);
		vector3_cross(&j_v_i, &j_w_i, &dist);
		//printf("%d: %f, %f, %f\n", i, j_v_i.x, j_v_i.y, j_v_i.z);
		MAT_EL(m->jacob0, 0, i) = j_v_i.x;
		MAT_EL(m->jacob0, 1, i) = j_v_i.y;
		MAT_EL(m->jacob0, 2, i) = j_v_i.z;
		MAT_EL(m->jacob0, 3, i) = j_w_i.x;
		MAT_EL(m->jacob0, 4, i) = j_w_i.y;
		MAT_EL(m->jacob0, 5, i) = j_w_i.z;
	}
	// compute jacobe
	SE3_basis(&m->global_transforms[m->num_links - 1], &m->r_ee);
	math_success(arm_mat_trans_f32(&m->r_ee, &m->r_ee_T));
	_jacob_make_transform(m, &m->r_ee_T, &ee_origin);
	math_success(arm_mat_mult_f32(&m->jacob_transform, &m->jacob0, &m->jacobe));

	// compute JE^T
	math_success(arm_mat_trans_f32(&m->jacobe, &m->jacobe_T));

	// Compute pinv sx or dx on the basis of num of rows and columns
	if (m->jacobe.numCols > m->jacobe.numRows) {
		// pinv dx
		// compute JE*JE^T
		math_success(arm_mat_mult_f32(&m->jacobe, &m->jacobe_T, &m->jacobe_T_jacobe));

		// compute inv(JE*JE^T)
		math_success(arm_mat_inverse_f32(&m->jacobe_T_jacobe, &m->inv_jacobe_T_jacobe));

		// compute JE^T*inv(JE*JE^T)
		math_success(arm_mat_mult_f32(&m->jacobe_T, &m->inv_jacobe_T_jacobe, &m->inv_jacobe));
	}
	else {
		// pinv sx
		// compute JE^T*JE
		math_success(arm_mat_mult_f32(&m->jacobe_T, &m->jacobe, &m->jacobe_T_jacobe));

		// compute inv(JE^T*JE)
		math_success(arm_mat_inverse_f32(&m->jacobe_T_jacobe, &m->inv_jacobe_T_jacobe));

		// compute inv(JE^T*JE)*JE^T
		math_success(arm_mat_mult_f32(&m->inv_jacobe_T_jacobe, &m->jacobe_T, &m->inv_jacobe));
	}

	return ARM_MATH_SUCCESS;
}

void manipulator_jacob_info(Manipulator * m)
{
	printf("Jacob0:\n");
	mat_print(&m->jacob0);
	printf("Jacob Transform:\n");
	mat_print(&m->jacob_transform);
	printf("Jacobe:\n");
	mat_print(&m->jacobe);
}

arm_status manipulator_servo_start(Manipulator * m, SE3_mat * target_pose, float gain, float threshold)
{
	if (m->servo.error_matrix.pData == NULL) {
		if (SE3_I(&m->servo.error_matrix) == ARM_MATH_NOMEM)
			return ARM_MATH_NOMEM;
		if (SE3_I(&m->servo.ee_inv) == ARM_MATH_NOMEM)
			return ARM_MATH_NOMEM;
	}
	m->servo.target_pose = target_pose;
	m->servo.gain = gain;
	m->servo.threshold = threshold;
	return ARM_MATH_SUCCESS;
}

bool manipulator_servo_control(Manipulator * m, float * joint_speeds)
{
	float err_roll, err_pitch, err_yaw;
	float ee_speed_vector[6], error_vector[6];
	math_success(arm_mat_inverse_f32(&EE, &m->servo.ee_inv));
	math_success(arm_mat_mult_f32(&m->servo.ee_inv, m->servo.target_pose, &m->servo.error_matrix));
	SE3_to_rpy(&m->servo.error_matrix, &err_roll, &err_pitch, &err_yaw);
	error_vector[0] = SE3(&m->servo.error_matrix, 0, 3);
	error_vector[1] = SE3(&m->servo.error_matrix, 1, 3);
	error_vector[2] = SE3(&m->servo.error_matrix, 2, 3);
	error_vector[3] = err_yaw;
	error_vector[4] = err_pitch;
	error_vector[5] = err_roll;

	int i;
	float norm = 0;
	for (i = 0; i < 6;i++) {
		ee_speed_vector[i] = error_vector[i] * m->servo.gain;
		norm += fabs(ee_speed_vector[i]);
	}

	arm_matrix_instance_f32	svec;
	svec.numCols = 1;
	svec.numRows = 6;
	svec.pData = ee_speed_vector;

	arm_matrix_instance_f32	dq;
	dq.numCols = 1;
	dq.numRows = m->num_joints;
	dq.pData = joint_speeds;
	math_success(arm_mat_mult_f32(&m->inv_jacobe, &svec, &dq));

	return norm < m->servo.threshold;
}
