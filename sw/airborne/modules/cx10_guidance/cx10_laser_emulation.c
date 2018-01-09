/*
 * Copyright (C) AAlff
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/wall_avoidance/wall_avoidance.c"
 * @author AAlff
 * Virtual L, R, F, and U laser range finders. Module uses East and CCW as positive, but uses Ned,
 * sooo that's why heading is multiplied by -1 and a 90 deg offset has been introduced. I also
 * didn't know how to properly use floats at first so I split a line segment ((x1, y1), (x2, y2))
 * into (x1, y1, x2, y2) in most cases... would have been faster to pass pointers to array, could be a TO DO!

 */



#include "../cx10_guidance/cx10_laser_emulation.h"

#include "state.h"
#include <stdio.h>
#include <math.h>
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

// Wall coordinates
#define WALL_PT_A
#define WALL_PT_B
#define WALL_PT_C
#define WALL_PT_D

const float OFFSET = M_PI_2; //-1.0036; // for heading, in rad, here 0 deg is east
const float CEILING = 2; //ceiling height

void cx10_laser_emulation_init(void) {};
void cx10_laser_emulation_start(void) {};
void cx10_laser_emulation_stop(void) {};

void orient(float *coords, float heading, float offset, float rel_size, float position_x, float position_y);
float distance_2_pts(float x1, float y1, float x2, float y2);
int ccw(float x1, float y1, float x2, float y2, float x3, float y3);
float det(float x1, float y1, float x2, float y2);
float laser_read(float A, float B, float C, float D, float E, float F, float G, float H);
void wall_detection(float position_x, float position_y, float *left_pt, float *forward_pt, float *right_pt, float height, float phi, float theta);

// float wall_detection(float position, float left_pt, float forward_pt, float right_pt);

float opti_North;
float opti_East;
float opti_z;
float opti_theta;
float opti_phi;
float opti_psi;

//float LEFT_WALL_CZ[] = {-4.998, 1.104, -1.053, -5.184}; // x1, y1, x2, y2
//float RIGHT_WALL_CZ[] = {1.143, 5.119, 5.264, -1.133};
//float TOP_WALL_CZ[] = {1.143, 5.119, -4.998, 1.104};
//float BOTTOM_WALL_CZ[] = {-1.053, -5.184, 5.264, -1.133};

float LEFT_WALL_CZ[] = {-5.273, 1.348, -1.289, -5.27}; // x1, y1, x2, y2
float RIGHT_WALL_CZ[] = {1.289, 5.54, 5.52, -1.21};
float TOP_WALL_CZ[] = {1.289, 5.54, -5.273, 1.348};
float BOTTOM_WALL_CZ[] = {-1.289, -5.27, 5.52, -1.21};

float laser_telemetry[4];



void orient(float *coords, float heading, float offset, float rel_size, float position_x, float position_y)
{
// Gives position of a point, given another point's heading, required angle offset, and relative distance to first point
	coords[0] = position_x + cos(heading + offset) * rel_size;
	coords[1] = position_y + sin(heading + offset) * rel_size;
}

float distance_2_pts(float x1, float y1, float x2, float y2)
{
    return sqrt(powf(y2 - y1, 2) + powf(x2 - x1, 2));
}

int ccw(float x1, float y1, float x2, float y2, float x3, float y3)
{
    // Boe Bryce's algorithm for checking whether 2 line segments cross
	// printf("%i\n", (y3 - y1) * (x2 - x1) > (y2 - y1) * (x3 - x1));
    return (y3 - y1) * (x2 - x1) > (y2 - y1) * (x3 - x1);
}

float det(float x1, float y1, float x2, float y2)
{  // For 2D only
    return x1 * y2 - y1 * x2;
}

float laser_read(float A, float B, float C, float D, float E, float F, float G, float H){
    // Detects walls
//     Finds where laser and wall cross and returns distance
//     The two segments can't meet at end points... floats so who cars really
//     AB-CD is laser segment with (A,B) being the origin (drone, or position)

    // Test if segments cross
    if (!(ccw(A, B, E, F, G, H) != ccw(C, D, E, F, G, H) && ccw(A, B, C, D, E, F) != ccw(A, B, C, D, G, H)))
    {
        // printf("tick\n");
        return 999.;
    }


    float xdiff[] = {A - C, E - G};
    float ydiff[] = {B - D, F - H};

    float div_matrix = det(xdiff[0], xdiff[1], ydiff[0], ydiff[1]);
    if (div_matrix == 0)
    {
        return 999.;
    }

    float d[] = {det(A,B,C,D), det(E,F,G,H)};
    float x = det(d[0], d[1], xdiff[0], xdiff[1]) / div_matrix;
    float y = det(d[0], d[1], ydiff[0], ydiff[1]) / div_matrix;

    return distance_2_pts(A, B, x, y);
}

void wall_detection(float position_x, float position_y, float *left_pt, float *forward_pt, float *right_pt, float height, float phi, float theta)
{
//     Simulates laser range data in m, using left, right, forward lasers' points
//     Generate segments from points
//
//     3 lasers, 2 points per laser, 2 coordinates per point. Order: left, forward, right
    float lasers[] = {
    		position_x, position_y, left_pt[0], left_pt[1],
			position_x, position_y, forward_pt[0], forward_pt[1],
			position_x, position_y, right_pt[0], right_pt[1]
    };

    float walls[] = {
    		LEFT_WALL_CZ[0], LEFT_WALL_CZ[1], LEFT_WALL_CZ[2], LEFT_WALL_CZ[3],
    		RIGHT_WALL_CZ[0], RIGHT_WALL_CZ[1], RIGHT_WALL_CZ[2], RIGHT_WALL_CZ[3],
			TOP_WALL_CZ[0], TOP_WALL_CZ[1], TOP_WALL_CZ[2], TOP_WALL_CZ[3],
			BOTTOM_WALL_CZ[0], BOTTOM_WALL_CZ[1], BOTTOM_WALL_CZ[2], BOTTOM_WALL_CZ[3]
    };

    laser_telemetry[0] = 999.; //left
    laser_telemetry[1] = 999.; //front
    laser_telemetry[2] = 999.; //right
	laser_telemetry[3] = 999.; //up

	// Horizontal readings
    for (int l_idx = 0; l_idx < 12; l_idx += 4)
    {  // Matches lasers' direction with idx for segment and read lists
        for (int w_idx = 0; w_idx < 16; w_idx += 4)
        {
            float reading= laser_read(lasers[l_idx], lasers[l_idx + 1], lasers[l_idx + 2], lasers[l_idx + 3],
            						   walls[w_idx], walls[w_idx + 1], walls[w_idx + 2], walls[w_idx + 3]);
            if ((reading < laser_telemetry[l_idx / 4]) && ((l_idx == 0)|| (l_idx == 8))) // adjust left and right for roll
            {
            	laser_telemetry[l_idx / 4] = reading / cos(phi);
            }
            if ((reading < laser_telemetry[l_idx / 4]) && (l_idx == 4)) // adjust forward for pitch
            {
            	laser_telemetry[l_idx / 4] = reading / cos(theta);
            }
        }
    }


    // Vertical reading (adjusted for pitch and roll):
    float top_laser = (CEILING - height) / (cos(theta)*cos(phi));
    if (top_laser < LASER_RANGE)
    {
    	laser_telemetry[3] = top_laser;
    }
}

void cx10_laser_emulation_periodic(void)
{
	 float l5;

	 opti_North = stateGetPositionNed_f()->x; // North
	 opti_East = stateGetPositionNed_f()->y; // East
	 //printf("%f,%f\n", opti_North, opti_East);
	 opti_z =  - stateGetPositionNed_f()->z; // making up positive
	 opti_theta = stateGetNedToBodyEulers_f()-> theta; // pitch
	 opti_phi = stateGetNedToBodyEulers_f() -> phi; // roll
	 opti_psi = stateGetNedToBodyEulers_f() -> psi * -1 + OFFSET; // Yaw or Heading +/- pi rad wrt East
	 printf("x, y: %f, %f\n", opti_North, opti_East);
	 float coords_left[2];
	 float coords_forward[2];
	 float coords_right[2];

	 orient(coords_left, opti_psi, M_PI_2, LASER_RANGE, opti_East, opti_North);
	 orient(coords_forward, opti_psi, 0, LASER_RANGE, opti_East, opti_North);
	 orient(coords_right, opti_psi, - M_PI_2, LASER_RANGE, opti_East, opti_North);

	 wall_detection(opti_East, opti_North, coords_left, coords_forward,coords_right, opti_z, opti_phi, opti_theta);

	 // printf("%f, %f, %f\n", opti_theta, opti_phi, opti_z);
	 // printf("left: %f, forward: %f, right: %f, top: %f\n", laser_telemetry[0], laser_telemetry[1], laser_telemetry[2], laser_telemetry[3]);

	 DOWNLINK_SEND_IR_SENSORS(DefaultChannel, DefaultDevice,
			&laser_telemetry[0], &laser_telemetry[1], &laser_telemetry[2],&laser_telemetry[3],&opti_theta );
}

void cx10_laser_emulation_event(void) {};
//void wall_avoidance_datalink_callback() {};


