#ifndef CX10_PID
#define CX10_PID

#include "filters/low_pass_filter.h"

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

extern float pid_cx10(float reading, float desired_distance, float current_distance, struct cx10_pid_data* pid_data, Butterworth2LowPass* buttered_distance);
#endif