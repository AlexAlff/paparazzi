/*
 * @file modules/wall_avoidance/WA_control.h
 * @brief Laser range finder wall avoider
 *
 */

// Drone doesn't respond as quickly on second corner, need to tune this mode
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
#define NO_READING 2.4 // Any value above this is considered a NaN
#define GUIDANCE_FREQ 512.0
#define TO_RAD_MULTI 0.0174533 // Probably already exists in Papa
#define CX10_SAFETY_ANGLE (6.0 * TO_RAD_MULTI) // 6.5 degrees is gewd
#define CX10_MAX_ANGLE (40.0 * TO_RAD_MULTI) // Anything above 35 is dangerous
#define G 9.81
#define READING_FILTER_TAU (0.057875)

// Cornering macros
#define CX10_SAFETY_SPEED ((14.41 * CX10_SAFETY_ANGLE) + 0.4) // Max speed at safety angle from drag estimator
#define PSI_DOT (M_PI_2 + (30 * TO_RAD_MULTI)) // Max yaw rate
#define D_PSI_CORNERING (M_PI_2) // Cornering angle
#define MIN_VEL_ESTIMATE_COUNT 43
#define PSI_DOT_CORNERING (D_PSI_CORNERING / GUIDANCE_FREQ) //Yaw increments
#define YAW_INPUT_DELAY 0.3 // Time taken to yaw

#define DES_DIST 0.7 // Desired distance d from wall during stabilization and cornering
#define REQUIRED_APPROACH_SPEED (D_PSI_CORNERING * (LASER_RANGE - DES_DIST) / (1 + D_PSI_CORNERING * YAW_INPUT_DELAY))//works but wrong
#define DECELERATION_PITCH (25 * TO_RAD_MULTI)
#define R_CORNERING (LASER_RANGE - DES_DIST - 0.3) // Radius of curvature during cornering decrease last term for narrower corner
#define CORNERING_FINAL_COUNT (D_PSI_CORNERING / PSI_DOT) * GUIDANCE_FREQ //Ticks required to yaw by D_PSI_CORNERING

// Structs
struct cx10_pid_data
{
	// Angles in degrees, error in meters
	float last;// = LASER_RANGE; // Should equate to laser range when LR is small
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

struct cx10_cornering_state
{
	float desired_heading;
	float final_heading;
	float desired_roll;
	float desired_pitch;
	int tick_count;
	int final_tick_count;
	bool ctrl_init_flag;
	bool inited;
	// Deceleration phase
	bool deceleration_phase;
	int deceleration_ticks;
};

// Prototypes
float max_angle_limiting_factor(float des_pitch, float des_roll);
float heading_mod_2pi(float a);
float GetHeadingError(float initial, float final);
float pid_cx10(float reading, float desired_distance, float current_distance,
		struct cx10_pid_data*, Butterworth2LowPass*);
void ctrl_module_init(struct cx10_pid_data* pid_left, struct cx10_pid_data* pid_front, struct cx10_pid_data* pid_right);
void ctrl_module_run(bool in_flight, struct Int32Eulers*, float left_reading, float front_reading, float right_reading);
void cx10_cornering_init(bool reset_state, struct cx10_cornering_state*, float front_reading);
void cx10_cornering(struct cx10_cornering_state*, struct Int32Eulers*, float left_reading,
					float front_reading, float right_reading);

// Variable declarations
int tick_count = 0; // DELETE AFTER YAW TEST
float desired_heading; // in rad
bool wall_locked_flag = FALSE;
bool ctrl_init_flag;
float cornering_roll;
float CX10_RANGE = LASER_RANGE;
int printing_counter = 0;
// PID
float CX10_desired_dist_left = DES_DIST;
float CX10_desired_dist_front = DES_DIST;
float CX10_desired_dist_right = 0.0;
float CX10_KP = 0.24;
float CX10_KD; // depends on frequency -> initialized in init function
float CX10_KD_before = 0.18;
float CX10_KI = 0.002;
float integ_windup_band_error = 0.2; // error when integral term kicks in
float integ_windup_band_d_error = 0.0003; // differentiated error when integral term kicks in (times freq for vel in m/s)

// Struct inits
struct cx10_pid_data cx10_pid_left={0.,0.};
struct cx10_pid_data cx10_pid_front={0.,0.};
struct cx10_pid_data cx10_pid_right={0.,0.};

struct cx10_cornering_state cornering_state;

Butterworth2LowPass butter_left_reading;
Butterworth2LowPass butter_front_reading;
Butterworth2LowPass butter_right_reading;

// tau : f_c/ f_s * 2 * pi
void ctrl_module_init(struct cx10_pid_data* pid_left, struct cx10_pid_data* pid_front,
		struct cx10_pid_data* pid_right)
{
//	rc_commands.rc_x = 0.;
//	rc_commands.rc_y = 0.;
//	rc_commands.rc_z = 0.;
//	rc_commands.rc_t = 0.;
	wall_locked_flag = FALSE;
	desired_heading = stateGetNedToBodyEulers_f() -> psi; //heading
	CX10_KD = CX10_KD_before * GUIDANCE_FREQ; // Times frequency
	// Filter: tau =  0.2617 works good

	float left_init_reading = laser_telemetry[0] > LASER_RANGE ? LASER_RANGE : laser_telemetry[0];
	float front_init_reading = laser_telemetry[1] > LASER_RANGE ? LASER_RANGE : laser_telemetry[1];
	float right_init_reading = laser_telemetry[2] > LASER_RANGE ? LASER_RANGE : laser_telemetry[2];

	float left_init_distance = left_init_reading * cos(stateGetNedToBodyEulers_f()-> phi);
	float front_init_distance = front_init_reading * cos(stateGetNedToBodyEulers_f()-> theta);
	float right_init_distance = right_init_reading * cos(stateGetNedToBodyEulers_f()-> phi);
	init_butterworth_2_low_pass(&butter_left_reading, READING_FILTER_TAU,
			1 / GUIDANCE_FREQ, left_init_distance);
	init_butterworth_2_low_pass(&butter_front_reading, READING_FILTER_TAU,
			1 / GUIDANCE_FREQ, front_init_distance);
	init_butterworth_2_low_pass(&butter_right_reading, READING_FILTER_TAU,
			1 / GUIDANCE_FREQ, right_init_distance);
	pid_left->wall_locked_flag = FALSE;
	pid_front->wall_locked_flag = FALSE;
	pid_right->wall_locked_flag = FALSE;
	pid_left->last = 0.;
	pid_front->last = 0.;
	pid_right->last = 0.;
	pid_left->pid_error_sum = 0.;
	pid_front->pid_error_sum = 0.;
	pid_right->pid_error_sum = 0.;
	pid_left->reading_lost_tick_count = 0.;
	pid_front->reading_lost_tick_count = 0.;
	pid_right->reading_lost_tick_count = 0.;
}

void ctrl_module_run(bool in_flight, struct Int32Eulers* commands, float left_reading, float front_reading, float right_reading)
// Takes laser readings as input and will stabilize/'hover' drone certain distance from wall saving its outputs into sp_cmd_i
{
	float front_distance = front_reading * cos(stateGetNedToBodyEulers_f()-> theta);
	float left_distance = left_reading * cos(stateGetNedToBodyEulers_f()-> phi);

	float desired_pitch = 0;//pid_cx10(front_reading, CX10_desired_dist_front, front_distance,
			//&cx10_pid_front, &butter_front_reading);

	float desired_roll = pid_cx10(left_reading, CX10_desired_dist_left, left_distance,
			&cx10_pid_left, &butter_left_reading);
	if(cx10_pid_left.wall_locked_flag == TRUE)// && cx10_pid_front.wall_locked_flag == TRUE)
	{
		wall_locked_flag = TRUE;
		//printf("Wall locked in ctrl module run\n");
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
	ctrl_module_init(&cx10_pid_left, &cx10_pid_front, &cx10_pid_left);
	cx10_cornering_init(TRUE, &cornering_state, 0);
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

	float left_reading = laser_telemetry[0];
	float front_reading = laser_telemetry[1];
	float right_reading = laser_telemetry[2];
    if((wall_locked_flag == TRUE) && (front_reading < NO_READING) && (cornering_state.inited == FALSE))
    {
    	ctrl_init_flag = FALSE;
    	cx10_cornering_init(FALSE, &cornering_state, front_reading);
    	//printf("corner inited\n");
    }
    if(cornering_state.inited == TRUE)
    {
    	ctrl_init_flag = FALSE;
    	cx10_cornering(&cornering_state, &sp_cmd_i, left_reading, front_reading, right_reading);
    	if(printing_counter % 20 == 0){//printf("Cornering\n");
    	}
    }

    else // if wallLockedFlag == False and front_reading >= NO_READING:
    {
    	if(ctrl_init_flag == FALSE)
    	{
        	ctrl_init_flag = TRUE;
        	guidance_h_module_init();

    	}

    	ctrl_module_run(in_flight, &sp_cmd_i, left_reading, front_reading, right_reading);
    	if(printing_counter % 20 == 0)
    	{
    		//printf("controlling-> lock: %i, front: %f, left: %f, cornering_state: %i\n", wall_locked_flag, front_reading, left_reading, cornering_state.inited);
    	}
    	if(wall_locked_flag)
    	{ // When locked on, pitch forward, if imu_theta greater than safety, decrease pitch amount
    		float cruising_pitch = - CX10_SAFETY_ANGLE;
    		if(stateGetNedToBodyEulers_f() -> theta < - CX10_SAFETY_ANGLE)
    		{
    			cruising_pitch += 5 * TO_RAD_MULTI;
    			printf("cruising pitch desired: %f, actual: %f\n", cruising_pitch / TO_RAD_MULTI, stateGetNedToBodyEulers_f() -> theta / TO_RAD_MULTI);
    		}
    		sp_cmd_i.theta = ANGLE_BFP_OF_REAL(cruising_pitch);  // pitch
    	}
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
    stabilization_attitude_set_rpy_setpoint_i(&sp_cmd_i);
    stabilization_attitude_run(in_flight);
    if(printing_counter >= 100){printing_counter = 0;}
}

//void guidance_v_module_init(void)
//{
//  // initialization of your custom vertical controller goes here
//}
//
//// Implement own Vertical loops
//void guidance_v_module_enter(void)
//{
//  // your code that should be executed when entering this vertical mode goes here
//}
//
//void guidance_v_module_run(UNUSED bool in_flight)
//{
//  // your vertical controller goes here
//}


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
		if(fabs(pid_error) < integ_windup_band_error && fabs(d_input) < integ_windup_band_d_error)
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
		if((d_input > 0.0) && (pid_error < 0.0)) // when approaching wall before passing desired distance
		{
			KP_factor *= 0.7; // Doesn't fly towards wall as fast on initial approach
			KD_factor *= 1.3; // More breaking when approaching wall, before passing desired distance
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

void cx10_cornering_init(bool reset_state, struct cx10_cornering_state* cornering, float front_reading)
// Sets the initial values for cornering. After velocity estimation is complete, roll based on velocity calculation.
{
	if(reset_state)
	{
		cornering->inited = FALSE;
	}
	else
	{
		cornering->inited = TRUE;
		float initial_heading = stateGetNedToBodyEulers_f() -> psi;
		cornering->desired_heading = initial_heading;
		cornering->final_heading = initial_heading + D_PSI_CORNERING;
		cornering->tick_count = 0;
		cornering->final_tick_count = CORNERING_FINAL_COUNT;

		//Deceleration phase:
		cornering->deceleration_phase = TRUE;
		cornering->desired_roll = atan(powf(REQUIRED_APPROACH_SPEED, 2) / (G * R_CORNERING));;
		cornering->desired_pitch = DECELERATION_PITCH;
		cornering->deceleration_ticks = round(GUIDANCE_FREQ * (CX10_SAFETY_SPEED - REQUIRED_APPROACH_SPEED) / (G * tan(DECELERATION_PITCH)));
//		printf("Cornering deceleration ticks: %i,\nR_cornering: %f\n, final speed: %f\n cornering roll: %f\n",
//				cornering->deceleration_ticks, R_CORNERING, REQUIRED_APPROACH_SPEED,
//				cornering->desired_roll / TO_RAD_MULTI);
//		printf("\n\ninit cornering state heading: %f\n", stateGetNedToBodyEulers_f() -> psi);
//		//Speed estimation:
//		cornering->speed_estimation = TRUE;
//		cornering->initial_speed = CX10_SAFETY_SPEED;
//		cornering->desired_roll = atan(powf(CX10_SAFETY_SPEED, 2) / (G * (LASER_RANGE - DES_DIST)));
//		cornering->desired_pitch = 0;//- (CX10_SAFETY_ANGLE / 2);
//		cornering->first_dist = front_reading * cos(stateGetNedToBodyEulers_f()-> theta);
//		printf("cornering final count: %f\nRadius: %f\nDesired distance: %f\n", CORNERING_FINAL_COUNT, R_CORNERING, DES_DIST);
	}
}

void cx10_cornering(struct cx10_cornering_state* cornering, struct Int32Eulers* commands, float left_reading,
		float front_reading, float right_reading)
// Gets speed from front laser, turns, then initializes and runs ctrl_module_run
{
	float current_heading = stateGetNedToBodyEulers_f() -> psi;
	float heading_error = heading_mod_2pi(GetHeadingError(current_heading, cornering->final_heading));
	//printf("ticks: %i\n", cornering->tick_count);

	if(cornering->deceleration_phase)
	{
		if(cornering->tick_count > cornering->deceleration_ticks)
		{
			//printf("braking done, now cornering\n");
			cornering->deceleration_phase = FALSE;
			cornering->desired_heading += (PSI_DOT / GUIDANCE_FREQ);
			cornering->desired_pitch = 0;//- CX10_SAFETY_ANGLE / 2; // should be pitching down... this way it brakes!
		}

//	if(cornering->speed_estimation)
//	{
//		if(cornering->tick_count > MIN_VEL_ESTIMATE_COUNT && front_reading < NO_READING)
//		{
//			cornering->speed_estimation = FALSE;
//			cornering->initial_speed = (cornering->first_dist - front_reading/ cos(stateGetNedToBodyEulers_f()-> theta)) / cornering->tick_count * GUIDANCE_FREQ;
//			cornering->desired_roll = atan(powf(cornering->initial_speed, 2) / (G * (LASER_RANGE - DES_DIST)));
//			cornering->desired_heading += (PSI_DOT / GUIDANCE_FREQ);
//			cornering->desired_pitch = CX10_SAFETY_ANGLE / 2; // should be pitching down... this way it brakes!
//			printf("After measuring: first reading: %f, last reading: %f, tick_count: %i, phi: %f, initial corner speed: %f\n", cornering->first_dist,
//					front_reading, cornering->tick_count, cornering->desired_roll / TO_RAD_MULTI, cornering->initial_speed);
//
//		}
		else
		{
			cornering->desired_heading += (PSI_DOT / GUIDANCE_FREQ);
		}
		commands->psi = ANGLE_BFP_OF_REAL(cornering->desired_heading);
		commands->theta = ANGLE_BFP_OF_REAL(cornering->desired_pitch); // pitch
		commands->phi = ANGLE_BFP_OF_REAL(cornering->desired_roll);
	}
	else
	{
		if(heading_error > 1 * TO_RAD_MULTI)
		{
			if(cornering->tick_count < cornering->final_tick_count) //maintain psi until end of cornering
			{
				cornering->desired_heading += (PSI_DOT / GUIDANCE_FREQ);
				commands->theta = ANGLE_BFP_OF_REAL(cornering->desired_pitch); // pitch
				commands->phi = ANGLE_BFP_OF_REAL(cornering->desired_roll);
				commands->psi = ANGLE_BFP_OF_REAL(cornering->desired_heading); // Test whether you need to input it during every loop
			}
			else
			{
				commands->theta = ANGLE_BFP_OF_REAL(cornering->desired_pitch); // pitch
								commands->phi = ANGLE_BFP_OF_REAL(cornering->desired_roll);
								commands->psi = ANGLE_BFP_OF_REAL(cornering->desired_heading); // Test whether you need to input it during every loop
//				if(cornering->ctrl_init_flag == FALSE)
//					{
//						ctrl_module_init(&cx10_pid_left, &cx10_pid_front, &cx10_pid_right);
//						desired_heading = cornering->desired_heading; //bypass init function's heading set
//						cornering->ctrl_init_flag = TRUE;
//					}
//				ctrl_module_run(TRUE, commands, left_reading, front_reading, right_reading);
				}
		}
		else
		{
			cornering->inited = FALSE;
			//printf("\n\n\n\n\nDONE TURNING\n\n\n\n");
		}
	}
	cornering->tick_count += 1;
	//printf("cornering state heading: %i, actual heading: %f\n", commands->psi, stateGetNedToBodyEulers_f() -> psi);
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

float heading_mod_2pi(float a)
{
	// Returns negative float modulus
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
