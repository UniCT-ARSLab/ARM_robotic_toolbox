/*
 * virtual_robot.h
 *
 *  Created on: Oct 21, 2026
 *      Author: corrado
 */

#ifndef LIB_CORE_INC_VIRTUAL_ROBOT_H_
#define LIB_CORE_INC_VIRTUAL_ROBOT_H_

#include <stdbool.h>

typedef enum {
	ACCEL,
	CRUISE,
	DECEL,
	TARGET
} t_phase;

typedef struct {
	float vmax;
	float accel;
	float decel;
} t_speed_profile;

typedef struct {
	float target;
	float p;
	float v;
	float decel_distance;
	t_speed_profile speed_profile;
	t_phase phase;
} t_motion;

void vr_start_motion(t_motion * m, t_speed_profile sp, float target);
bool vr_next_motion(t_motion * m, float delta_t, float * pos);

#endif /* LIB_CORE_INC_VIRTUAL_ROBOT_H_ */
