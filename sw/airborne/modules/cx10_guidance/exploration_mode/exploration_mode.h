#ifndef CX10_SAFE_MODE
#define CX10_SAFE_MODE

#include "../../../math/pprz_algebra_int.h"
#include "../CX10_control.h"
#include "../yaw_detection/yaw_detection.h"

struct cx10_corner_exploration_struct
{
	bool corner_exploring_flag;
	bool corner_exploring_initialized;
	bool brake_flag;
	float initial_heading;
	float last_front_reading;
	float desired_heading;
};

extern void exploration_state_init(struct cx10_yaw_detection_state_struct* detection_state, struct cx10_corner_exploration_struct* corner_exploration);
void explore_corner(struct cx10_yaw_detection_state_struct* detection_state, struct cx10_corner_exploration_struct* corner_exploration,
					struct Int32Eulers* commands, float left_reading, float front_reading, float right_reading,
                    struct cx10_pid_data* pid_left, struct cx10_pid_data* pid_front, struct cx10_pid_data* right, bool wall_locked);

#endif