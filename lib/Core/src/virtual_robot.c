/*
 * virtual_robot.c
 *
 *  Created on: Oct 21, 2026
 *      Author: corrado
 */

#include <math.h>
#include "virtual_robot.h"

void vr_start_motion(t_motion * m, t_speed_profile sp, float target)
{
    m->target = target;
    m->v = 0;  // current speed
    m->p = 0;  // current position
    m->phase = ACCEL;
    m->decel_distance = 0.5 * sp.vmax * sp.vmax / sp.decel;
    m->speed_profile = sp;
}


bool vr_next_motion(t_motion * m, float delta_t, float * pos)
{
    float distance, new_v;
    switch(m->phase) {
    case ACCEL:
        m->p = m->p + m->v * delta_t + m->speed_profile.accel * delta_t * delta_t / 2;
        m->v = m->v + m->speed_profile.accel  * delta_t;
        distance = m->target - m->p;
        if (distance < 0)
            distance = 0;
        if (m->v >= m->speed_profile.vmax) {
            m->v = m->speed_profile.vmax;
            m->phase = CRUISE;
        }
        else if (distance <= m->decel_distance) {
            float v_exp = sqrt(2 * m->speed_profile.decel * distance);
            if (v_exp < m->v)
                m->phase = DECEL;
        }
        break;

    case CRUISE:
        m->p = m->p + m->speed_profile.vmax * delta_t;
        distance = m->target - m->p;
        if (distance <= m->decel_distance)
            m->phase = DECEL;
        break;

    case DECEL:
    	m->p = m->p + m->v * delta_t - m->speed_profile.decel * delta_t * delta_t / 2.0;
        new_v = m->v - m->speed_profile.decel * delta_t;
        if (m->v >= 0)
        	m->v = new_v;
        if (m->p >= m->target) {
        	m->v = 0;
        	m->p = m->target;
        	m->phase = TARGET;
        }
        break;

    case TARGET:
        m->v = 0;
        m->p = m->target;
        break;
    }

    *pos = m->p;
    return (m->phase == TARGET);

}
