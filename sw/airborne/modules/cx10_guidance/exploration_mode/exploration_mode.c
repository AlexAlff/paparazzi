#include <stdio.h>
#include <stdlib.h>
#include "../helper_functions/helpers.h"
#include "exploration_mode.h"
#include "state.h"


void exploration_state_init(struct cx10_yaw_detection_state_struct* detection_state, struct cx10_corner_exploration_struct* corner_exploration)
{
	detection_state->wall_detected_flag = FALSE;
	detection_state->yaw_detection_flag = FALSE;

	corner_exploration->corner_exploring_flag = FALSE;
	corner_exploration->corner_exploring_initialized = FALSE;
	corner_exploration->brake_flag = FALSE;
}

void explore_corner(struct cx10_yaw_detection_state_struct* detection_state, struct cx10_corner_exploration_struct* corner_exploration,
					struct Int32Eulers* commands, float left_reading, float front_reading, float right_reading,
                    struct cx10_pid_data* pid_left, struct cx10_pid_data* pid_front, struct cx10_pid_data* pid_right, bool wall_locked)
{
	if(front_reading < CX10_NO_READING)
	{ // Used for yaw detection
		corner_exploration->last_front_reading = front_reading;
	}
	if(!corner_exploration->corner_exploring_initialized)
	// initialize corner exploration
	{
		corner_exploration->initial_heading = stateGetNedToBodyEulers_f() -> psi;
		corner_exploration->desired_heading = corner_exploration->initial_heading;
		ctrl_module_init(pid_left, pid_front, pid_right);
		ctrl_module_run(TRUE, commands, left_reading, front_reading, right_reading, TRUE, TRUE, FALSE);
		corner_exploration->brake_flag = TRUE;
		corner_exploration->corner_exploring_initialized = TRUE;
	}
	else if(corner_exploration->brake_flag)
		// brake drone with pid front and left
		// stabilization on corner
	{
		ctrl_module_run(TRUE, commands, left_reading, front_reading, right_reading, TRUE, TRUE, FALSE);
		if(wall_locked == TRUE)
		{
			corner_exploration->brake_flag = FALSE;
		}
	}
	else if(front_reading < CX10_NO_READING)
	{
		if(heading_mod_2pi(GetHeadingError(stateGetNedToBodyEulers_f() ->psi, corner_exploration->initial_heading + M_PI)) > 1 * CX10_TO_RAD_MULTI)
		{
			// Yaw 180 degrees CW until there's no front reading
			corner_exploration->desired_heading += CX10_PSI_DOT;
		}
		else
		{
			corner_exploration->desired_heading = corner_exploration->initial_heading + M_PI;
			guidance_h_module_init(); // full 180 degree scan complete and there's still a front reading... WHAT DO WE DO???
		}
	}
	else
	{
		// If lost reading, reset cornering state
		corner_exploration->desired_heading = stateGetNedToBodyEulers_f()->psi;
		corner_exploration->corner_exploring_flag = FALSE;
		corner_exploration->brake_flag = FALSE;
		corner_exploration->corner_exploring_initialized = FALSE;
		// proceed to yaw detection (wall parallelism)
		detection_state->yaw_detection_flag = TRUE;
		detection_state->wall_detected_flag = TRUE;
		detection_state->final_heading = stateGetNedToBodyEulers_f()->psi + M_PI_2 - atan(corner_exploration->last_front_reading/left_reading);
		detection_state->desired_heading = corner_exploration->desired_heading;
		//printf("Wall parallelism angle: %f deg\n", detection_state->final_heading / CX10_TO_RAD_MULTI);
	}
	if(!corner_exploration->brake_flag)
	{
		commands->phi = ANGLE_BFP_OF_REAL(0.0);
		commands->theta = ANGLE_BFP_OF_REAL(0.0);
	}
	commands->psi = ANGLE_BFP_OF_REAL(corner_exploration->desired_heading);
}

