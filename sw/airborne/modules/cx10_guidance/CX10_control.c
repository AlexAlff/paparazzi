/*
 * @file modules/wall_avoidance/WA_control.h
 * @brief Follow wall contour based on laser range finders
 *
 */

// Guidance_h_module_run() contains navigation logic
// cx10_pid is the controller, with a function and struct
// ctrl_module manages the PID, saving its output for roll and pitch to sp_cmd_i
// corner_exploration and yaw_detection contain the navigation functionality, coupled with their respective structs
// Real laser returns values in mm, whereas the module uses meters.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "state.h"
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "filters/low_pass_filter.h"
#include "generated/modules.h"

#include "laser_emulation/cx10_laser_emulation.h"
#include "CX10_control.h"
#include "helper_functions/helpers.h"
#include "exploration_mode/exploration_mode.h"
#include "yaw_detection/yaw_detection.h"

// Variable declarations
int last_yaw_detection_tick_count = 0;
float desired_heading; // in rad
bool wall_locked_flag = FALSE;
bool ctrl_init_flag;
int printing_counter = 0;

// PID constants
float CX10_desired_dist_left  = CX10_DES_DIST;
float CX10_desired_dist_front = CX10_DES_DIST;
float CX10_desired_dist_right = 0.0;
float CX10_RANGE = LASER_RANGE;
float CX10_KP = 0.24;
float CX10_KD; // depends on frequency -> initialized in init function
float CX10_KD_before = 0.18;
float CX10_KI = 0.0002;
float integ_windup_band_error = 0.36; // error when integral term kicks in
float integ_windup_band_d_error = 0.0003; // differentiated error when integral term kicks in (times freq for vel in m/s)

// Struct inits
struct cx10_yaw_detection_state_struct cx10_yaw_detection_state;
struct cx10_corner_exploration_struct cx10_corner_exploration;

struct cx10_pid_data cx10_pid_left;
struct cx10_pid_data cx10_pid_front;
struct cx10_pid_data cx10_pid_right;

Butterworth2LowPass butter_left_reading;
Butterworth2LowPass butter_front_reading;
Butterworth2LowPass butter_right_reading;

void guidance_h_module_init(void)
{
	ctrl_module_init(&cx10_pid_left, &cx10_pid_front, &cx10_pid_right);
	exploration_state_init(&cx10_yaw_detection_state, &cx10_corner_exploration);
}

void guidance_h_module_enter(void)
{
	guidance_h_module_init();
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
	printing_counter += 1;
	struct Int32Eulers sp_cmd_i;
	float left_reading  = laser_telemetry[0];
	float front_reading = laser_telemetry[1];
	float right_reading = laser_telemetry[2];

	if(cx10_corner_exploration.corner_exploring_initialized)
	{
		// If corner exploration flag set to true, continue exploring
		ctrl_init_flag = FALSE;
		if(printing_counter >= 100){printf("Continue exploration\n");}
		explore_corner(&cx10_yaw_detection_state, &cx10_corner_exploration, &sp_cmd_i, left_reading, front_reading, right_reading,
					   &cx10_pid_left, &cx10_pid_front, &cx10_pid_right, wall_locked_flag);
		last_yaw_detection_tick_count = 0;
	}
	else if(cx10_yaw_detection_state.yaw_detection_flag)
	{
		// If yaw detection flag set to true, continue yaw detection
		ctrl_init_flag = FALSE;
		if(printing_counter >= 100){printf("Continue detection\n");}
		yaw_detection(&cx10_yaw_detection_state, &sp_cmd_i, left_reading, front_reading);
	}
//	else if(wall_locked_flag && last_yaw_detection_tick_count >  CX10_YAW_DETECTION_RATE)
//	{
//		// if there hasn't been a wall detection for CX10_YAW_DETECTION_RATE counts and locked on to wall, sets yaw detection flag to true
//		ctrl_init_flag = FALSE;
//		printf("Init yaw detection\n");
//		yaw_detection(&cx10_yaw_detection_state, &sp_cmd_i, left_reading, front_reading);
//		last_yaw_detection_tick_count = 0;
//	}
	else if(front_reading < CX10_NO_READING)
	{
		// If not doing parallel detection or already exploring corner, and there's a front reading, sets corner exploration flag to true
		ctrl_init_flag = FALSE;
		printf("Init corner exploration\n");
		explore_corner(&cx10_yaw_detection_state, &cx10_corner_exploration, &sp_cmd_i, left_reading, front_reading, right_reading,
					   &cx10_pid_left, &cx10_pid_front, &cx10_pid_right, wall_locked_flag);
	}
	else if(wall_locked_flag)
	{
		// If locked on to wall and not time to detect wall parallelism, just cruise along wall
		if(printing_counter >= 100){printf("Cruising\n");}
		if(!ctrl_init_flag)
		{
			guidance_h_module_init();
			ctrl_init_flag = TRUE;
		}
		float cruising_pitch = - CX10_SAFETY_ANGLE;
		// if(stateGetNedToBodyEulers_f() -> theta < cruising_pitch)
		// {
		// 	cruising_pitch += 5 * CX10_TO_RAD_MULTI; // Limit pitch aggressively because this is a manly drone
		// }
		ctrl_module_run(in_flight, &sp_cmd_i, left_reading, front_reading, right_reading, TRUE, FALSE, FALSE);
		sp_cmd_i.theta = ANGLE_BFP_OF_REAL(cruising_pitch);  // pitch
		last_yaw_detection_tick_count += 1;
	}
	else
    {
		//if not wall_locked, safest mode therefore set as else
		if(printing_counter >= 100){printf("Not locked so controlling\n");}
    	if(!ctrl_init_flag)
		{
			ctrl_init_flag = TRUE;
			guidance_h_module_init();
		}
    	ctrl_module_run(in_flight, &sp_cmd_i, left_reading, front_reading, right_reading, TRUE, FALSE, FALSE);
    }
    stabilization_attitude_set_rpy_setpoint_i(&sp_cmd_i);
    stabilization_attitude_run(in_flight);
    if(printing_counter >= 100)
	{
		printf("wall lock status: %i\n", wall_locked_flag);
		printing_counter = 0;
	}
}

// tau : f_c/ f_s * 2 * pi
void ctrl_module_init(struct cx10_pid_data* pid_left, struct cx10_pid_data* pid_front,
		struct cx10_pid_data* pid_right)
{
	wall_locked_flag = FALSE;
	desired_heading = stateGetNedToBodyEulers_f() -> psi; //heading
	CX10_KD = CX10_KD_before * CX10_GUIDANCE_FREQ; // Times frequency

	float left_init_reading  = laser_telemetry[0] > CX10_RANGE ? CX10_RANGE : laser_telemetry[0];
	float front_init_reading = laser_telemetry[1] > CX10_RANGE ? CX10_RANGE : laser_telemetry[1];
	float right_init_reading = laser_telemetry[2] > CX10_RANGE ? CX10_RANGE : laser_telemetry[2];

	float left_init_distance  = left_init_reading  * cos(stateGetNedToBodyEulers_f()-> phi);
	float front_init_distance = front_init_reading * cos(stateGetNedToBodyEulers_f()-> theta);
	float right_init_distance = right_init_reading * cos(stateGetNedToBodyEulers_f()-> phi);
	init_butterworth_2_low_pass(&butter_left_reading,  CX10_READING_FILTER_TAU, 1 / CX10_GUIDANCE_FREQ, left_init_distance);
	init_butterworth_2_low_pass(&butter_front_reading, CX10_READING_FILTER_TAU, 1 / CX10_GUIDANCE_FREQ, front_init_distance);
	init_butterworth_2_low_pass(&butter_right_reading, CX10_READING_FILTER_TAU, 1 / CX10_GUIDANCE_FREQ, right_init_distance);
	pid_left ->wall_locked_flag = FALSE;
	pid_front->wall_locked_flag = FALSE;
	pid_right->wall_locked_flag = FALSE;
	pid_left ->last = 0.;
	pid_front->last = 0.;
	pid_right->last = 0.;
	pid_left ->pid_error_sum = 0.;
	pid_front->pid_error_sum = 0.;
	pid_right->pid_error_sum = 0.;
	pid_left ->reading_lost_tick_count = 0.;
	pid_front->reading_lost_tick_count = 0.;
	pid_right->reading_lost_tick_count = 0.;
}

void ctrl_module_run(bool in_flight, struct Int32Eulers* commands, float left_reading, float front_reading, float right_reading,
					 bool pid_left, bool pid_front, bool pid_right)
// Takes laser readings as input and will stabilize/'hover' drone certain distance from wall saving its outputs into sp_cmd_i
{
	if(in_flight)
	{
		float front_distance = front_reading * cos(stateGetNedToBodyEulers_f()-> theta);
		float left_distance = left_reading * cos(stateGetNedToBodyEulers_f()-> phi);
		float right_distance = right_reading * cos(stateGetNedToBodyEulers_f()-> phi);

		float desired_pitch = 0;
		float desired_roll = 0;

		// Based on desired pid directions (function's last three bool arguments) return desired pitch/roll commands
		if(pid_front)
		{
			desired_pitch = pid_cx10(front_reading, CX10_desired_dist_front, front_distance,
					&cx10_pid_front, &butter_front_reading);
		}
		if(pid_left)
		{
			desired_roll = pid_cx10(left_reading, CX10_desired_dist_left, left_distance,
				&cx10_pid_left, &butter_left_reading);
		}
		else if(pid_right)
		{
			desired_roll = pid_cx10(right_reading, CX10_desired_dist_right, right_distance,
				&cx10_pid_right, &butter_right_reading);
		}

		// Set locked wall flag based on desired pids
		if(pid_front && !(pid_left || pid_right))
		{
			wall_locked_flag = cx10_pid_front.wall_locked_flag;
		}
		if(pid_front && (pid_left || pid_right))
		{
			wall_locked_flag = cx10_pid_front.wall_locked_flag && (cx10_pid_left.wall_locked_flag || cx10_pid_right.wall_locked_flag);
		}
		if(!pid_front && (pid_left || pid_right))
		{
			wall_locked_flag = cx10_pid_left.wall_locked_flag || cx10_pid_right.wall_locked_flag;
		}

		float total_angle = acos(cos(desired_roll)*cos(desired_pitch)) / CX10_TO_RAD_MULTI;
		float angle_factor = max_angle_limiting_factor(desired_pitch, desired_roll, CX10_MAX_ANGLE);
		desired_pitch *= angle_factor;
		desired_roll *= angle_factor;
		total_angle = acos(cos(desired_roll)*cos(desired_pitch)) / CX10_TO_RAD_MULTI;
		commands->phi = ANGLE_BFP_OF_REAL(desired_roll); // roll
		commands->theta = ANGLE_BFP_OF_REAL(desired_pitch); //pitch
		commands->psi = ANGLE_BFP_OF_REAL(desired_heading);

		DOWNLINK_SEND_CX_10_TUNING(DefaultChannel, 			DefaultDevice, &total_angle,
					&cx10_pid_left.reading_lost_tick_count, &cx10_pid_left.current_distance,
					&cx10_pid_left.desired_angle, 			&cx10_pid_left.KP_factor,
					&cx10_pid_left.KI_factor, 				&cx10_pid_left.KD_factor,
					&cx10_pid_front.desired_distance, 		&cx10_pid_front.current_distance,
					&cx10_pid_front.desired_angle, 			&cx10_pid_front.KP_factor,
					&cx10_pid_front.KI_factor, 				&cx10_pid_front.KD_factor);
	}
}
