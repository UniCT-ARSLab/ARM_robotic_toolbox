/*
 * openarm.h
 *
 *  Created on: Sep 2, 2026
 *      Author: corrado
 */

#ifndef INC_OPENARM_MODEL_H_
#define INC_OPENARM_MODEL_H_

#include "manipulator.h"

arm_status openarm_model_init(void);
void openarm_model_set_joints(float * q);
void openarm_model_show_ee_pose(void);
void openarm_model_servo_drive3(float * xyz);
void openarm_model_servo_drive(SE3_mat * target);
void openarm_model_test(void);
void openarm_jacob_info(void);


#endif /* INC_OPENARM_MODEL_H_ */
