#ifndef CX10_WA_HELPER_FUNCTIONS
#define CX10_WA_HELPER_FUNCTIONS
extern float heading_mod_2pi(float a);
extern float GetHeadingError(float initial_heading, float final_heading);
extern float max_angle_limiting_factor(float des_pitch, float des_roll, float max_angle);
#endif