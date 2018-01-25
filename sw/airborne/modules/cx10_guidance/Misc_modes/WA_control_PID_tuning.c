/*
 * @file modules/wall_avoidance/WA_control.h
 * @brief Laser range finder wall avoider
 *
 */

// Copy into CX10_control.c for PID tuning

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

// Variable declarations
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
float CX10_KI = 0.0002;// 0.004;
float integ_windup_band_error = 0.36; // error when integral term kicks in
float integ_windup_band_d_error = 0.0003; // differentiated error when integral term kicks in (times freq for vel in m/s)
bool non_linearity_flag = TRUE; // Set to false to turn off non-linear behavior

// Struct inits
struct cx10_pid_data cx10_pid_left  = {0.,0.};
struct cx10_pid_data cx10_pid_front = {0.,0.};
struct cx10_pid_data cx10_pid_right = {0.,0.};

Butterworth2LowPass butter_left_reading;
Butterworth2LowPass butter_front_reading;
Butterworth2LowPass butter_right_reading;

// tau : f_c/ f_s * 2 * pi
void ctrl_module_init(struct cx10_pid_data* pid_left, struct cx10_pid_data* pid_front,
		struct cx10_pid_data* pid_right)
{
	wall_locked_flag = FALSE;
	desired_heading = stateGetNedToBodyEulers_f() -> psi; //heading
	CX10_KD = CX10_KD_before * CX10_GUIDANCE_FREQ; // Times frequency
	// Filter: tau =  0.2617 works good

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
	float front_distance = front_reading * cos(stateGetNedToBodyEulers_f()-> theta);
	float left_distance = left_reading * cos(stateGetNedToBodyEulers_f()-> phi);
	float right_distance = right_reading * cos(stateGetNedToBodyEulers_f()-> phi);

	float desired_pitch;
	float desired_roll;

	// Based on desired pid directions (function's last three bool arguments) return desired pitch/roll commands
	if(pid_front)
	{
		desired_pitch = pid_cx10(front_reading, CX10_desired_dist_front, front_distance,
				&cx10_pid_front, &butter_front_reading);
	}
	else
	{
		desired_pitch = 0;
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
	else
	{
		desired_roll = 0;
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

////////////////////////////////////////////////////////////////////
// Call our controller
// Implement own Horizontal loops
void guidance_h_module_init(void)
{
	ctrl_module_init(&cx10_pid_left, &cx10_pid_front, &cx10_pid_right);
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
	sp_cmd_i.theta = ANGLE_BFP_OF_REAL(0.0);
	sp_cmd_i.phi = ANGLE_BFP_OF_REAL(0.0);
	//printf("init: %i,%i,%i\n", sp_cmd_i.theta, sp_cmd_i.phi, sp_cmd_i.psi);
	float left_reading  = laser_telemetry[0];
	float front_reading = laser_telemetry[1];
	float right_reading = laser_telemetry[2];

	ctrl_module_run(in_flight, &sp_cmd_i, left_reading, front_reading, right_reading, TRUE, FALSE, FALSE);
	//sp_cmd_i.phi = ANGLE_BFP_OF_REAL(CX10_SAFETY_ANGLE);
    stabilization_attitude_set_rpy_setpoint_i(&sp_cmd_i);
    stabilization_attitude_run(in_flight);
    if(printing_counter >= 100)
	{
		printf("%i\n", wall_locked_flag);
		printing_counter = 0;
	}
}
