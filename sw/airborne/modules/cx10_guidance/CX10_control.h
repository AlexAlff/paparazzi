/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/wall_avoidance/WA_control.h
 * @brief Laser range finder wall avoider
 *
 * Will avoid walls using laser range finder data assuming L/F/R/T
 */

#ifndef WA_CONTROL_H_
#define WA_CONTROL_H_

#include <std.h>
#include "PID/cx10_pid.h"
#include "../../math/pprz_algebra_int.h"

// Macros
#define CX10_NO_READING 1.8 // Any value above this is treated as a NaN, actual laser should return value above 8190
#define CX10_GUIDANCE_FREQ 512.0
#define CX10_TO_RAD_MULTI 0.0174533 // Probably already exists in Papa
#define CX10_SAFETY_ANGLE (7.0 * CX10_TO_RAD_MULTI) // 7 degrees is used, cuz its safe
#define CX10_MAX_ANGLE (35.0 * CX10_TO_RAD_MULTI) // Anything above 35 is dangerous
#define CX10_G 9.81
#define CX10_READING_FILTER_TAU (0.057875) // To filter PID differentiated term

#define CX10_PSI_DOT (M_PI_2 / CX10_GUIDANCE_FREQ) // Heading increment size
#define CX10_YAW_DETECTION_RATE (30 * CX10_GUIDANCE_FREQ) // Heading change rate
#define CX10_DES_DIST 1.0 // For controller

// Settings
extern float CX10_RANGE; //Laser range
extern float CX10_desired_dist_left; //PID desired dist
extern float CX10_desired_dist_front; //PID desired dist
extern float CX10_desired_dist_right; //PID desired dist
extern float CX10_KP;
extern float CX10_KD_before;
extern float CX10_KI;

// Vars
extern float integ_windup_band_error;
extern float integ_windup_band_d_error;
extern float CX10_KD;

// Implement own horizontal guidance module
#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_MODULE

// Implement pre-existing GUIDANCE_V_MODE_HOVER vertical guidance control
#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_HOVER

// Implement own Horizontal loops
extern void guidance_h_module_init(void);
extern void guidance_h_module_enter(void);
extern void guidance_h_module_read_rc(void);
extern void guidance_h_module_run(bool in_flight);

extern void ctrl_module_init(struct cx10_pid_data* pid_left, struct cx10_pid_data* pid_front, struct cx10_pid_data* pid_right);
extern void ctrl_module_run(bool in_flight, struct Int32Eulers*, float left_reading, float front_reading, float right_reading,
					 bool left, bool front, bool right);

#endif /* CTRL_MODULE_DEMO_H_ */
