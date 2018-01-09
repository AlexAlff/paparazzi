/*
 * @file modules/wall_avoidance/WA_control.h
 * @brief Laser range finder wall avoider
 *
 */

// TODO: ctrl init flags
// What do I do when drone's scanned 180 degrees and there's still a front reading?
// Find way to implement yaw detection direction (for exploration phase end, if it was going CCW, it's going to need to look for wall CW, and vise versa)

// Using 120 Hertz time constant instead of 520... it's working though
#include "state.h"
#include <stdio.h>
#include <stdlib.h>
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "filters/low_pass_filter.h"
#include "generated/modules.h"
#include <math.h>
#include "../cx10_guidance/cx10_laser_emulation.h"
#include "../cx10_guidance/WA_control.h"

// Macros
#define NO_READING 3.0 // Any value above this is a NaN
#define GUIDANCE_FREQ 512.0 // Using 120 Hz in code with CX10_LASER_EMULATION_PERIODIC_PERIOD
#define TO_RAD_MULTI 0.0174533 // Probably already exists in Papa
#define CX10_SAFETY_ANGLE (6.0 * TO_RAD_MULTI) // 6.5 degrees is gewd
#define CX10_MAX_ANGLE (40.0 * TO_RAD_MULTI) // Anything above 35 is dangerous
#define G 9.81

#define PSI_DOT (M_PI_2 / GUIDANCE_FREQ)
#define YAW_DETECTION_RATE (30 * GUIDANCE_FREQ)
#define DES_DIST 0.7

struct cx10_pid_data
{
	// Angles in degrees, error in meters
	float last;// = CX10_RANGE; // Should equate to laser range when LR is small
	float pid_error_sum;// = 0.0;
	bool wall_locked_flag;
	float desired_angle;
	float reading_lost_tick_count;
	float KP_factor; // these are used for messages
	float KI_factor;
	float KD_factor;
	float desired_distance;
	float current_distance;
};
struct cx10_detection_state_struct
{
	int tick_count;
	float desired_heading;
	bool yaw_detection_flag;
	bool wall_detected_flag;
	float final_heading;
};
struct cx10_corner_exploration_struct
{
	bool corner_exploring_flag;
	bool corner_exploring_initialized;
	bool brake_flag;
	bool CW_90_deg_flag;
	bool CCW_270_deg_flag;
	float initial_heading;
	float last_front_reading;
};
struct cx10_detection_state_struct cx10_detection_state;
struct cx10_corner_exploration_struct cx10_corner_exploration;

// Prototypes
float max_angle_limiting_factor(float des_pitch, float des_roll);
float heading_mod_2pi(float a);
float GetHeadingError(float initial, float final);
float pid_cx10(float reading, float desired_distance, float current_distance,
			   struct cx10_pid_data*, Butterworth2LowPass*);
void ctrl_module_init(struct cx10_pid_data* pid_left, struct cx10_pid_data* pid_front, struct cx10_pid_data* pid_right);
void ctrl_module_run(bool in_flight, struct Int32Eulers*, float left_reading, float front_reading, float right_reading,
					 bool left, bool front, bool right);
void explore_corner(struct cx10_detection_state_struct*, struct cx10_corner_exploration_struct*, 
					struct Int32Eulers*, float left_reading, float front_reading, float right_reading);
void yaw_detection(struct cx10_detection_state_struct*, struct Int32Eulers*, float left_reading, float front_reading);

// Variable declarations
int last_yaw_detection_tick_count = 1;
float desired_heading; // in rad
bool wall_locked_flag = FALSE;
bool ctrl_init_flag;
int printing_counter = 0;

// PID constants
float CX10_desired_dist_left  = DES_DIST;
float CX10_desired_dist_front = DES_DIST;
float CX10_desired_dist_right = 0.0;
float CX10_RANGE = LASER_RANGE;
float CX10_KP = 0.55;
float CX10_KD; // depends on frequency -> initialized in init function
float CX10_KD_before = 0.15;
float CX10_KI = 0.004;

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
	CX10_KD = CX10_KD_before * GUIDANCE_FREQ; // Times frequency
	// Filter: tau =  0.2617 works good

	float left_init_reading  = laser_telemetry[0] > CX10_RANGE ? CX10_RANGE : laser_telemetry[0];
	float front_init_reading = laser_telemetry[1] > CX10_RANGE ? CX10_RANGE : laser_telemetry[1];
	float right_init_reading = laser_telemetry[2] > CX10_RANGE ? CX10_RANGE : laser_telemetry[2];

	float left_init_distance  = left_init_reading  * cos(stateGetNedToBodyEulers_f()-> phi);
	float front_init_distance = front_init_reading * cos(stateGetNedToBodyEulers_f()-> theta);
	float right_init_distance = right_init_reading * cos(stateGetNedToBodyEulers_f()-> phi);
	init_butterworth_2_low_pass(&butter_left_reading,  2*0.2617, CX10_LASER_EMULATION_PERIODIC_PERIOD, left_init_distance);
	init_butterworth_2_low_pass(&butter_front_reading, 2*0.2617, CX10_LASER_EMULATION_PERIODIC_PERIOD, front_init_distance);
	init_butterworth_2_low_pass(&butter_right_reading, 2*0.2617, CX10_LASER_EMULATION_PERIODIC_PERIOD, right_init_distance);
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

	float total_angle = acos(cos(desired_roll)*cos(desired_pitch)) / TO_RAD_MULTI;
	float angle_factor = max_angle_limiting_factor(desired_pitch, desired_roll);
	desired_pitch *= angle_factor;
	desired_roll *= angle_factor;
	total_angle = acos(cos(desired_roll)*cos(desired_pitch)) / TO_RAD_MULTI;
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

	if(cx10_corner_exploration.corner_exploring_initialized)
	{
		// Continue exploring corner until finished
		ctrl_init_flag = FALSE;
		if(printing_counter >= 100){printf("Continue exploration\n");}
		explore_corner(&cx10_detection_state, &cx10_corner_exploration, &sp_cmd_i, left_reading, front_reading, right_reading);
	}
	else if(cx10_detection_state.yaw_detection_flag)
	{
		// Continue yaw detection until finished
		ctrl_init_flag = FALSE;
		if(printing_counter >= 100){printf("Continue detection\n");}
		yaw_detection(&cx10_detection_state, &sp_cmd_i, left_reading, front_reading);
	}
	else if(wall_locked_flag && last_yaw_detection_tick_count > YAW_DETECTION_RATE) // check if flag set to TRUE
	{
		// if there hasn't been a wall detection for YAW_DETECTION_RATE counts and locked on to wall
		ctrl_init_flag = FALSE;
		printf("Init yaw detection\n");
		yaw_detection(&cx10_detection_state, &sp_cmd_i, left_reading, front_reading);
		last_yaw_detection_tick_count = 0;
	}
	else if(front_reading < NO_READING)
	{
		// If not doing parallel detection or already exploring corner, and there's a front reading, initiate corner exploration
		ctrl_init_flag = FALSE;
		printf("Init corner exploration\n");
		explore_corner(&cx10_detection_state, &cx10_corner_exploration, &sp_cmd_i, left_reading, front_reading, right_reading);
	}
	else if(wall_locked_flag)
	{
		// If locked on to wall and not time to detect wall parallelism, just cruise along wall
		if(printing_counter >= 100){printf("Cruising\n");}
		float cruising_pitch = - CX10_SAFETY_ANGLE;
		if(stateGetNedToBodyEulers_f() -> theta < cruising_pitch)
		{
			cruising_pitch += 5 * TO_RAD_MULTI; // Limit pitch aggressively because this is a manly drone
		}
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
//    if(front_reading < 0.3) // in case of emergency?
//    {
//    	cx10_cornering_init(TRUE, &cornering_state, 0);
//    	wall_locked_flag = FALSE;
//    	printf("front < 0.3 -> collision avoidance... front_reading = %f\n", front_reading);
//    	sp_cmd_i.theta = ANGLE_BFP_OF_REAL(pid_cx10(front_reading, CX10_desired_dist_front,
//    			front_reading / (cos(stateGetNedToBodyEulers_f()-> theta)), &cx10_pid_front,
//    					&butter_front_reading));
//    	sp_cmd_i.phi = ANGLE_BFP_OF_REAL(pid_cx10(left_reading, CX10_desired_dist_left,
//    			left_reading / (cos(stateGetNedToBodyEulers_f()-> phi)), &cx10_pid_left,
//    					&butter_left_reading));
//    }
	//printf("final: %i,%i,%i\n", sp_cmd_i.theta, sp_cmd_i.phi, sp_cmd_i.psi);
    stabilization_attitude_set_rpy_setpoint_i(&sp_cmd_i);
    stabilization_attitude_run(in_flight);
    if(printing_counter >= 100)
	{
		printf("%i\n", wall_locked_flag);
		printing_counter = 0;
	}
}




float pid_cx10(float reading, float desired_distance, float current_distance, struct cx10_pid_data* pid_data,
		Butterworth2LowPass* buttered_distance)
{ //Angles in rad, converted to degree for message
	float desired_angle;
	float KP_factor = 0;
	float KI_factor = 0;
	float KD_factor = 0;
	if(reading < NO_READING)
	{
		float filtered_current_distance = update_butterworth_2_low_pass(buttered_distance, current_distance);
		float d_input = pid_data->last - filtered_current_distance;
		pid_data->last = filtered_current_distance;
		float pid_error = desired_distance - current_distance;
		// Lock info for navigation
		if(fabs(pid_error) < 0.15 && fabs(d_input) < 0.001)
		{
			pid_data->wall_locked_flag = TRUE;
		}
		else
		{
			pid_data->wall_locked_flag = FALSE;
		}
		KI_factor = 0.0;
		KD_factor = CX10_KD * d_input;
		KP_factor = CX10_KP * pid_error;

		// Non-linear behavior, need to compact when done with logic
		if(d_input > 0.0) // when approaching wall
		{
			if(pid_error < 0.0) // before passing desired distance
			{
				KP_factor *= 0.5; // Doesn't fly towards wall as fast on initial approach
				KD_factor *= 1.5; // More breaking when approaching wall, before passing desired distance
// 12/12 remove comment after checked
				if(KP_factor < - CX10_SAFETY_ANGLE)
				{
					KP_factor = - CX10_SAFETY_ANGLE; // when approaching wall, before passing desired distance
				}
			}
			if(pid_error > 0.0) // after passing desired distance
			{
				KP_factor *= 1.0; // rebound from wall after passing desired distance
				//printf("rebound from wall after passing desired distance");
			}
		}
		if((d_input < 0.0) && (pid_error < 0.0)){
			KP_factor *= 0.5; // Rebounding from wall, after passing desired distance
			KD_factor *= 0.5;
			//printf("Rebounding from wall, after passing desired distance");
		}

		desired_angle = KP_factor + KD_factor;
		if(fabs(pid_error) < 0.05)
		{
			pid_data->pid_error_sum += pid_error;
			KI_factor = CX10_KI * pid_data->pid_error_sum;
			desired_angle += KI_factor;
		}
		// Limit the maximum angle
		if(desired_angle < - CX10_MAX_ANGLE)
		{
			desired_angle = - CX10_MAX_ANGLE;
		}
		if(desired_angle > CX10_MAX_ANGLE)
		{
			desired_angle = CX10_MAX_ANGLE;
		}
	}
	else
	{
		pid_data->wall_locked_flag = FALSE;
		if(pid_data->last == 0) // in case: haven't locked on to wall yet
		{
			desired_angle = - CX10_SAFETY_ANGLE;
		}
		else // in case wall reading lost while PID active (overshoot), gradually decrease last desired angle
		{
			float max_lost_time = 0.5 * GUIDANCE_FREQ;// during 0.5 seconds after losing wall
			if(pid_data->reading_lost_tick_count < max_lost_time)
				{
					float lost_time_percent = pid_data->reading_lost_tick_count / max_lost_time;
					desired_angle = pid_data->desired_angle * (1 - 0.5 * lost_time_percent) * TO_RAD_MULTI;
					pid_data->reading_lost_tick_count += 1;
				}
			else // after 0.5s, reset behavior
			{
				desired_angle = - CX10_SAFETY_ANGLE;
				pid_data->reading_lost_tick_count = 0;
				pid_data->last = 0; // probably not needed... depends on module use
			}
		}
	}
	// Save data
	pid_data->current_distance = current_distance;
	pid_data->desired_distance = desired_distance;
	pid_data->desired_angle = desired_angle / TO_RAD_MULTI;
	pid_data->KP_factor = KP_factor / TO_RAD_MULTI;
	pid_data->KI_factor = KI_factor / TO_RAD_MULTI;
	pid_data->KD_factor = KD_factor / TO_RAD_MULTI;

	return desired_angle;
}

float max_angle_limiting_factor(float des_pitch, float des_roll)
{
	float cos_max = cos(CX10_MAX_ANGLE);
	float limiting_factor = 1.0;
	for(float factor_test = 1.0 ; factor_test >= 0.6; factor_test -= 0.01)
	{
		float p2 = factor_test * des_pitch;
		float r2 = factor_test * des_roll;
		if(cos(p2)*cos(r2) > cos_max)
		{
			limiting_factor = factor_test;
			break;
		}
	}
		return limiting_factor;
}

void yaw_detection(struct cx10_detection_state_struct* detection_state, struct Int32Eulers* commands, float left_reading, float front_reading)
{
	detection_state->yaw_detection_flag = TRUE;
	if(detection_state->wall_detected_flag == FALSE && front_reading > NO_READING)
	{
		// Yaw CW until front laser detects a wall
		desired_heading = stateGetNedToBodyEulers_f() -> psi - PSI_DOT;
	}
	else if(front_reading < NO_READING && detection_state->wall_detected_flag == FALSE)
	{
		// Save required yaw CCW for drone to be parallel to wall
		detection_state->final_heading = stateGetNedToBodyEulers_f()->psi + M_PI_2 - atan(front_reading/left_reading);
		detection_state->wall_detected_flag = TRUE;
	}
	else if(heading_mod_2pi(GetHeadingError(stateGetNedToBodyEulers_f() ->psi, detection_state->final_heading)) > 1 * TO_RAD_MULTI)
	{
		// Yaw CCW until drone parallel to wall, exploration algorithm should jump here when complete
		desired_heading = stateGetNedToBodyEulers_f() ->psi += PSI_DOT;
	}
	else
	{
		// Once drone parallel to wall, save final heading and exit yaw detection
		desired_heading = detection_state->final_heading;
		detection_state->wall_detected_flag = FALSE;
		detection_state->yaw_detection_flag = FALSE;
		last_yaw_detection_tick_count = 0;
	}
	commands->psi = ANGLE_BFP_OF_REAL(desired_heading);
	commands->theta = ANGLE_BFP_OF_REAL(0.0);
	commands->phi = ANGLE_BFP_OF_REAL(0.0);
}

void explore_corner(struct cx10_detection_state_struct* detection_state, struct cx10_corner_exploration_struct* corner_exploration, 
					struct Int32Eulers* commands, float left_reading, float front_reading, float right_reading)
{
	if(front_reading < NO_READING)
	{
		corner_exploration->last_front_reading = front_reading;
	}
	if(!corner_exploration->corner_exploring_initialized)
	// initialize corner exploration
	{
		printf("1\n");
		corner_exploration->initial_heading = stateGetNedToBodyEulers_f() -> psi;
		ctrl_module_init(&cx10_pid_left, &cx10_pid_front, &cx10_pid_right);
		ctrl_module_run(TRUE, commands, left_reading, front_reading, right_reading, TRUE, TRUE, FALSE);
		corner_exploration->brake_flag = TRUE;
		corner_exploration->corner_exploring_initialized = TRUE;
		corner_exploration->CW_90_deg_flag = TRUE;
	}
	else if(corner_exploration->brake_flag)
		// brake drone with pid front and left
		// stabilization on corner done twice, once at initialization, and a 2nd time after first 90 CW scan
	{
		printf("OK WE DOING SOME BRAKING UP IN HERE\n");
		ctrl_module_run(TRUE, commands, left_reading, front_reading, right_reading, TRUE, TRUE, FALSE);
		if(wall_locked_flag == TRUE)
		{
			corner_exploration->brake_flag = FALSE;
			if(!corner_exploration->CW_90_deg_flag)
			{
				corner_exploration->CCW_270_deg_flag = TRUE;
			}
		}
	}
	else if(front_reading < NO_READING)
	{
		printf("2\n");
		if(corner_exploration->CW_90_deg_flag)
		{
			printf("2a\n");
			if(heading_mod_2pi(GetHeadingError(stateGetNedToBodyEulers_f() ->psi, corner_exploration->initial_heading + M_PI_2)) > 1 * TO_RAD_MULTI)
			{
				printf("2a1\n");
				// Yaw 90 degrees CW until there's no front reading
				desired_heading = stateGetNedToBodyEulers_f() ->psi += PSI_DOT;
			}
			else
			{
				// If lose wall reading after 90 deg CW scan, stabilize drone to wall and scan 270 deg CCW
				printf("2a2\n");
				corner_exploration->CW_90_deg_flag = FALSE;
				corner_exploration->brake_flag = TRUE;
			}
		}
		else if(corner_exploration->CCW_270_deg_flag)
		{
			printf("2b\n");
			// Yaw 270 degrees CCW until there's no front reading (180 deg from initial heading)
			if(heading_mod_2pi(GetHeadingError(stateGetNedToBodyEulers_f()->psi, corner_exploration->initial_heading - M_PI_2)) > 1 * TO_RAD_MULTI)
			{
				printf("2b1\n");
				desired_heading = stateGetNedToBodyEulers_f() ->psi -= PSI_DOT;
			}
			else
			{
				printf("2b2\n");
				desired_heading = corner_exploration->initial_heading - M_PI;
				corner_exploration->CCW_270_deg_flag = FALSE;
				guidance_h_module_init(); // full 180 degree scan complete and there's still a front reading... WHAT DO WE DO???
			}
		}
	}
	else
	{
		// If lost reading, reset cornering state
		printf("3\n");
		desired_heading = stateGetNedToBodyEulers_f()->psi;
		corner_exploration->corner_exploring_flag = FALSE;
		corner_exploration->brake_flag = FALSE;
		corner_exploration->CW_90_deg_flag = FALSE;
		corner_exploration->CCW_270_deg_flag = FALSE;
		// proceed to yaw detection (wall parallelism)
		detection_state->wall_detected_flag = TRUE;
		detection_state->final_heading = M_PI_2 - atan(corner_exploration->last_front_reading/left_reading);
	}
	if(!corner_exploration->brake_flag)
	{
		commands->phi = ANGLE_BFP_OF_REAL(0.0);
		commands->theta = ANGLE_BFP_OF_REAL(0.0);
	}
	commands->psi = ANGLE_BFP_OF_REAL(desired_heading);
}

float heading_mod_2pi(float a)
{
	// Returns negative float modulus in rad
    float ans = fmodf(a, (2 * M_PI));
    if(ans < 0)
    {
        ans += (2 * M_PI);
    }
    return ans;
}

float GetHeadingError(float initial, float final)
{
	float diff = final - initial;
	float absDiff = abs(diff);
	if (absDiff <= M_PI)
	{
		return absDiff == M_PI ? absDiff : diff;
	}
	else if (final > initial)
	{
		return absDiff - (2 * M_PI);
	}
	else
	{
		return (2 * M_PI) - absDiff;
	}
}