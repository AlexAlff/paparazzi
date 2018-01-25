#ifndef YAW_DETECTION_CX10
#define YAW_DETECTION_CX10

// Structs
struct cx10_yaw_detection_state_struct
{
	float desired_heading;
	bool yaw_detection_flag;
	bool wall_detected_flag;
	float final_heading;
};
extern void yaw_detection(struct cx10_yaw_detection_state_struct* detection_state, struct Int32Eulers* commands, float left_reading, float front_reading);
#endif