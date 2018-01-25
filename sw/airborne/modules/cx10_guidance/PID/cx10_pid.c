#include "filters/low_pass_filter.h"
#include "cx10_pid.h"
#include "../CX10_control.h"


float pid_cx10(float reading, float desired_distance, float current_distance, struct cx10_pid_data* pid_data,
		Butterworth2LowPass* buttered_distance)
{ //Angles in rad, converted to degree for message
	float desired_angle;
	float KP_factor = 0;
	float KI_factor = 0;
	float KD_factor = 0;
	if(reading <  CX10_NO_READING)
	{
		float filtered_current_distance = update_butterworth_2_low_pass(buttered_distance, current_distance);
		float d_input = pid_data->last - filtered_current_distance;
		pid_data->last = filtered_current_distance;
		float pid_error = desired_distance - current_distance;
		// Lock info for navigation
		//printf("d_input: %f, e: %f\n", d_input, pid_error);
		if(fabs(pid_error) < integ_windup_band_error && fabs(d_input) < integ_windup_band_d_error)
		{
			//velocity : d_input x  CX10_GUIDANCE_FREQ
			pid_data->wall_locked_flag = TRUE;
		}
		else
		{
			pid_data->wall_locked_flag = FALSE;
		}
		KI_factor = 0.0;
		KD_factor = CX10_KD * d_input;
		KP_factor = CX10_KP * pid_error;

		// Non-linear behavior

		if((d_input > 0.0) && (pid_error < 0.0) && !pid_data->wall_locked_flag) // when approaching wall before passing desired distance
			{
				if(KP_factor < - CX10_SAFETY_ANGLE)
				{
					KP_factor = - CX10_SAFETY_ANGLE; // when approaching wall, before passing desired distance
				}
			}
		if((d_input < 0.0) && (pid_error < 0.0)) // after wall rebound after passing desired distance
		{
			KD_factor *= 0.7; // Assuming drone has stabilized at this point
			//printf("Rebounding from wall, after passing desired distance");
		}

		desired_angle = KP_factor + KD_factor;
		if((fabs(pid_error) < integ_windup_band_error) && fabs(d_input) < integ_windup_band_d_error)
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
			float max_lost_time = 0.5 *  CX10_GUIDANCE_FREQ;// during 0.5 seconds after losing wall
			if(pid_data->reading_lost_tick_count < max_lost_time)
				{
					float lost_time_percent = pid_data->reading_lost_tick_count / max_lost_time;
					desired_angle = pid_data->desired_angle * (1 - 0.5 * lost_time_percent) *  CX10_TO_RAD_MULTI;
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
	pid_data->desired_angle = desired_angle /  CX10_TO_RAD_MULTI;
	pid_data->KP_factor = KP_factor /  CX10_TO_RAD_MULTI;
	pid_data->KI_factor = KI_factor /  CX10_TO_RAD_MULTI;
	pid_data->KD_factor = KD_factor /  CX10_TO_RAD_MULTI;

	return desired_angle;
}