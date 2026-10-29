/*
 * openarm.h
 *
 *  Created on: Sep 2, 2026
 *      Author: corrado
 */

#ifndef INC_OPENARM_MODEL_H_
#define INC_OPENARM_MODEL_H_

#include "manipulator.h"
#include "virtual_robot.h"
#include "openarm_driver.h"
#include "can_utils.h"

typedef struct {
	Manipulator model;
	t_arm_side side;
	t_can_interface * canif;
} t_openarm_robot;

arm_status openarm_model_init(t_openarm_robot * robot);
void openarm_model_set_joints(t_openarm_robot * robot, float * q, float speed);
void openarm_model_show_ee_pose(t_openarm_robot * robot);
void openarm_model_servo_drive3(t_openarm_robot * robot, float * xyz);
void openarm_model_servo_drive(t_openarm_robot * robot, SE3_mat * target, float delta_t, float gain, float threshold, bool drive);
void openarm_model_segment_drive(t_openarm_robot * robot, SE3_mat * target, t_speed_profile sp, float delta_t, float gain, float threshold, bool drive);
void openarm_model_segment_start(t_openarm_robot * robot, SE3_mat * target, t_speed_profile sp, float delta_t, float gain, float threshold);
bool openarm_model_segment_step(t_openarm_robot * robot, float delta_t, bool drive);
void openarm_model_test(void);
void openarm_jacob_info(t_openarm_robot * robot);


#endif /* INC_OPENARM_MODEL_H_ */
