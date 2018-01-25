#include "../../math/pprz_algebra_int.h"
#include "yaw_detection.h"
#include "state.h"
#include "../CX10_control.h"
#include "../helper_functions/helpers.h"

void yaw_detection(struct cx10_yaw_detection_state_struct* detection_state, struct Int32Eulers* commands, float left_reading, float front_reading)
{
	if(!detection_state->yaw_detection_flag)
	{
		//printf("1\n");
		detection_state->yaw_detection_flag = TRUE;
		detection_state->desired_heading = stateGetNedToBodyEulers_f() -> psi;
	}

	if(!detection_state->wall_detected_flag && front_reading > CX10_NO_READING)
	{
		//printf("2\n");
		// Yaw CCW until front laser detects a wall
		detection_state->desired_heading -= - CX10_PSI_DOT;
	}
	else if(front_reading < CX10_NO_READING && !detection_state->wall_detected_flag)
	{
		//printf("3\n");
		// Save required yaw CW for drone to be parallel to wall
		detection_state->desired_heading = stateGetNedToBodyEulers_f()->psi;
		detection_state->final_heading = stateGetNedToBodyEulers_f()->psi + M_PI_2 - atan(front_reading/left_reading);
		detection_state->wall_detected_flag = TRUE;
	}
	else if(heading_mod_2pi(GetHeadingError(stateGetNedToBodyEulers_f() ->psi, detection_state->final_heading)) > 1 * CX10_TO_RAD_MULTI)
	{
		//printf("4\n");
		// Yaw CCW until drone parallel to wall, exploration algorithm should jump here when complete
		detection_state->desired_heading += CX10_PSI_DOT;
		//printf("CW, check final heading: %f\nCurrent heading: %f\n", detection_state->final_heading / CX10_TO_RAD_MULTI, stateGetNedToBodyEulers_f()->psi/ CX10_TO_RAD_MULTI);
	}
	else
	{
		//printf("5\n");
		// Once drone parallel to wall, save final heading and exit yaw detection
		detection_state->desired_heading = detection_state->final_heading;
		detection_state->wall_detected_flag = FALSE;
		detection_state->yaw_detection_flag = FALSE;
	}
	commands->psi = ANGLE_BFP_OF_REAL(detection_state->desired_heading);
	commands->theta = ANGLE_BFP_OF_REAL(0.0);
	commands->phi = ANGLE_BFP_OF_REAL(0.0);
}
