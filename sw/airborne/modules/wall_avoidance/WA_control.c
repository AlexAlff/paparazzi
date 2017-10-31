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
 */

#include "modules/wall_avoidance/WA_control.h"
#include "state.h"
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "modules/wall_avoidance/wall_avoidance.h"


struct rc_commands_struct {
  int rc_x;
  int rc_y;
  int rc_z;
  int rc_t;
} rc_commands;

struct cx10_pid_data
{
	// Angles in radians, error in meters
	float last;// = LASER_RANGE;
	float error_sum;// = 0.0;
};
struct cx10_pid_data cx10_pid_left;
struct cx10_pid_data cx10_pid_front={LASER_RANGE,0};

struct cx10_pid_data cx10_pid_right;

float pid_cx10(float desired_distance, float current_distance, struct cx10_pid_data*);

float left_reading;
float front_reading;
float right_reading;

float desired_dist_left = 0.0;
float desired_dist_front = 2;
float desired_dist_right = 0.0;

//float ctrl_module_demo_pr_ff_gain = 0.2f;  // Pitch/Roll
//float ctrl_module_demo_pr_d_gain = 0.1f;
//float ctrl_module_demo_y_ff_gain = 0.4f;   // Yaw
//float ctrl_module_demo_y_d_gain = 0.05f;

float CX_10_SAFETY_ANGLE = - 10.0 * 0.0174533; // rad
float CX_10_MAX_ANGLE = 25.0 * 0.0174533; // rad

float to_rad_multi = 0.0174533;
float cx_10_first_current = 0;
float cx_10_KP = 0.5;
float cx_10_KD = 0.05;
float cx_10_KI = 0.0;

float cx_10_dist_KD_standby = 0.1;
float cx_10_rel_KD_standby = 0.1;

float init_heading;

void ctrl_module_init(void);
void ctrl_module_run(bool in_flight);

void ctrl_module_init(void)
{
	rc_commands.rc_x = 0;
	rc_commands.rc_y = 0;
	rc_commands.rc_z = 0;
	rc_commands.rc_t = 0;
	init_heading = ANGLE_BFP_OF_REAL(stateGetNedToBodyEulers_f() -> psi); //heading
	cx_10_KD = cx_10_KD * 120; // Times frequency
}

// simple rate control without reference model nor attitude
void ctrl_module_run(bool in_flight)
{
	// your control
	// printf("%f %f \n", cx10_pid_front.last, cx10_pid_front.error_sum); // print struct values

	//cx10_pid_front.error_sum = 0;

	left_reading = laser_telemetry[0];
	front_reading = laser_telemetry[1];
	right_reading = laser_telemetry[2];

	float desired_pitch;
	float desired_roll = -0.5 * to_rad_multi; // rad

	if(front_reading < LASER_RANGE)
	{
		printf("Front reading: %f\n", front_reading);
		desired_pitch = pid_cx10(desired_dist_front, front_reading, &cx10_pid_front);  // rad
	}
	else
	{
		printf("Front reading: %f, ", front_reading);
		desired_pitch = CX_10_SAFETY_ANGLE;
		printf("standby pitch: %f\n", desired_pitch / to_rad_multi);
	}
    struct Int32Eulers sp_cmd_i;
    sp_cmd_i.phi = ANGLE_BFP_OF_REAL(desired_roll); // roll
    sp_cmd_i.theta = ANGLE_BFP_OF_REAL(desired_pitch); //pitch

    sp_cmd_i.psi = init_heading;

    stabilization_attitude_set_rpy_setpoint_i(&sp_cmd_i);
    stabilization_attitude_run(in_flight);
}


////////////////////////////////////////////////////////////////////
// Call our controller
// Implement own Horizontal loops
void guidance_h_module_init(void)
{
	ctrl_module_init();
}

void guidance_h_module_enter(void)
{
	ctrl_module_init();
}

void guidance_h_module_read_rc(void)
{
	// -MAX_PPRZ to MAX_PPRZ
//	printf("h read rc\n");
//	rc_commands.rc_t = radio_control.values[RADIO_THROTTLE];
//	rc_commands.rc_x = radio_control.values[RADIO_ROLL];
//	rc_commands.rc_y = radio_control.values[RADIO_PITCH];
//	rc_commands.rc_z = radio_control.values[RADIO_YAW];
}

void guidance_h_module_run(bool in_flight)
{
  // Call full inner-/outerloop / horizontal-/vertical controller:
  ctrl_module_run(in_flight);
}

//void guidance_v_module_init(void)
//{
//  // initialization of your custom vertical controller goes here
//}
//
//// Implement own Vertical loops
//void guidance_v_module_enter(void)
//{
//  // your code that should be executed when entering this vertical mode goes here
//}
//
//void guidance_v_module_run(UNUSED bool in_flight)
//{
//  // your vertical controller goes here
//}


float pid_cx10(float desired_distance, float current_distance, struct cx10_pid_data* pid_data)
{
	float d_input = pid_data->last - current_distance;
	pid_data->last = current_distance;
	float error = desired_distance - current_distance;
	pid_data->error_sum += error;

	float KP_factor = cx_10_KP * error;
	//float KI_factor = cx_10_KI * pid_data.error_sum;
	float KD_factor = cx_10_KD * d_input;
//	//Non-linear behavior, for KP implement that on approach KP / 5, and KD / 5 on distancing
//	if((abs(error) < cx_10_dist_KD_standby) && (abs(KD_factor) < 0.17))
//	{
//		KD_factor *= cx_10_rel_KD_standby;
//	}
//	if((KP_factor < - CX_10_SAFETY_ANGLE) && (d_input > 0)) // Limit initial P angle so drone doesn't fly towards wall!
//	{
//		KP_factor = - CX_10_SAFETY_ANGLE;
//	} //Need to test signs before implementing, simulation was only in one direction
	float desired_angle = KP_factor + KD_factor; // + cx_10_KI * pid_data.error_sum
	if(desired_angle < - CX_10_MAX_ANGLE)
	{
		desired_angle = - CX_10_MAX_ANGLE;
	}
	if(desired_angle > CX_10_MAX_ANGLE)
	{
		desired_angle = CX_10_MAX_ANGLE;
	}
	printf("error: %f, KP factor: %f, d_input: %f, KD factor: %f,desired angle: %f\n \n\n", error, KP_factor / to_rad_multi, d_input, KD_factor / to_rad_multi, desired_angle / to_rad_multi);
	return desired_angle;
}

