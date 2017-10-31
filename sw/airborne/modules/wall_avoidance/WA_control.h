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

// Settings
extern float WA_control_pr_ff_gain;  // Pitch/Roll
extern float WA_control_pr_d_gain;
extern float WA_control_y_ff_gain;   // Yaw
extern float WA_control_y_d_gain;


// Implement own horizontal guidance module
#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_MODULE

// Implement pre-existing GUIDANCE_V_MODE_HOVER vertical guidance control
#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_HOVER

// Implement own Horizontal loops
extern void guidance_h_module_init(void);
extern void guidance_h_module_enter(void);
extern void guidance_h_module_read_rc(void);
extern void guidance_h_module_run(bool in_flight);

#endif /* CTRL_MODULE_DEMO_H_ */
