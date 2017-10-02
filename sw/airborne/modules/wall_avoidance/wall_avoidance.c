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



#include "modules/wall_avoidance/wall_avoidance.h"

#include "state.h"
#include <stdio.h>
#include <math.h>
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

const float LASER_RANGE = 2;
const float OFFSET = M_PI_2; //-1.0036; // for heading, in rad
const float CEILING = 2; //ceiling height

void wall_avoidance_init(void) {};
void wall_avoider_start(void) {};
void wall_avoider_stop(void) {};

// float wall_detection(float position, float left_pt, float forward_pt, float right_pt);

float opti_North;
float opti_East;
float opti_z;
float opti_theta;
float opti_phi;
float opti_psi;

float LEFT_WALL_CZ[] = {-4.998, 1.104, -1.053, -5.184}; // x1, y1, x2, y2
float RIGHT_WALL_CZ[] = {1.143, 5.119, 5.264, -1.133};
float TOP_WALL_CZ[] = {1.143, 5.119, -4.998, 1.104};
float BOTTOM_WALL_CZ[] = {-1.053, -5.184, 5.264, -1.133};

float *orient(float heading, float offset, float rel_size, float position_x, float position_y)
{
// Gives position of a point, given another point's heading, required angle offset, and relative distance to first point
	float *coords = malloc(2 * sizeof(float));
	if(!coords)
	{
			return NULL;
	}
	coords[0] = position_x + cos(heading + offset) * rel_size;
	coords[1] = position_y + sin(heading + offset) * rel_size;
	return coords;
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

    float div = det(xdiff[0], xdiff[1], ydiff[0], ydiff[1]);
    if (div == 0)
    {
        return 999.;
    }

    float d[] = {det(A,B,C,D), det(E,F,G,H)};
    float x = det(d[0], d[1], xdiff[0], xdiff[1]) / div;
    float y = det(d[0], d[1], ydiff[0], ydiff[1]) / div;

    return distance_2_pts(A, B, x, y);
}

float *wall_detection(float position_x, float position_y, float left_pt_x,
					  float left_pt_y, float forward_pt_x, float forward_pt_y,
					  float right_pt_x, float right_pt_y, float height,
					  float phi, float theta)
{
//     Simulates laser range data, using left, right, forward lasers' points
//     Generate segments from points
//
//     3 lasers, 2 points per laser, 2 coordinates per point. Order: left, forward, right
    float lasers[] = {
    		position_x, position_y, left_pt_x, left_pt_y,
			position_x, position_y, forward_pt_x, forward_pt_y,
			position_x, position_y, right_pt_x, right_pt_y
    };

    float walls[] = {
    		LEFT_WALL_CZ[0], LEFT_WALL_CZ[1], LEFT_WALL_CZ[2], LEFT_WALL_CZ[3],
    		RIGHT_WALL_CZ[0], RIGHT_WALL_CZ[1], RIGHT_WALL_CZ[2], RIGHT_WALL_CZ[3],
			TOP_WALL_CZ[0], TOP_WALL_CZ[1], TOP_WALL_CZ[2], TOP_WALL_CZ[3],
			BOTTOM_WALL_CZ[0], BOTTOM_WALL_CZ[1], BOTTOM_WALL_CZ[2], BOTTOM_WALL_CZ[3]
    };

    float *laser_readings = malloc(4 * sizeof(float));  // left, forward, right, top;
	if(!laser_readings)
	{
			return NULL;
	}
	laser_readings[0] = 999.;
	laser_readings[1] = 999.;
	laser_readings[2] = 999.;
	laser_readings[3] = 999.;

	// Horizontal readings
    for (int l_idx = 0; l_idx < 12; l_idx += 4)
    {  // Matches lasers' direction with idx for segment and read lists
        for (int w_idx = 0; w_idx < 16; w_idx += 4)
        {
            float reading= laser_read(lasers[l_idx], lasers[l_idx + 1], lasers[l_idx + 2], lasers[l_idx + 3],
            						   walls[w_idx], walls[w_idx + 1], walls[w_idx + 2], walls[w_idx + 3]);
            if ((reading < laser_readings[l_idx / 4]) && ((l_idx == 0)|| (l_idx == 8))) // adjust left and right for roll
            {
            	laser_readings[l_idx / 4] = reading / cos(phi);
            }
            if ((reading < laser_readings[l_idx / 4]) && (l_idx == 4)) // adjust forward for pitch
            {
            	laser_readings[l_idx / 4] = reading / cos(theta);
            }
        }
    }


    // Vertical reading (adjusted for pitch and roll):
    float top_laser = (CEILING - height) / (cos(theta)*cos(phi));
    if (top_laser < LASER_RANGE)
    {
    	laser_readings[3] = top_laser;
    }
    return laser_readings;
}

void wall_avoider_periodic(void)
{
	 float l5;

	 opti_North = stateGetPositionNed_f()->x; // North
	 opti_East = stateGetPositionNed_f()->y; // East
	 opti_z =  - stateGetPositionNed_f()->z; // making up positive
	 opti_theta = stateGetNedToBodyEulers_f()-> theta; // pitch
	 opti_phi = stateGetNedToBodyEulers_f() -> phi; // roll
	 opti_psi = stateGetNedToBodyEulers_f() -> psi * -1 + OFFSET; // Yaw or Heading +/- pi rad wrt East

	 float *left_laser_pt = orient(opti_psi, M_PI_2, LASER_RANGE, opti_East, opti_North);
	 float *forward_laser_pt = orient(opti_psi, 0, LASER_RANGE, opti_East, opti_North);
	 float *right_laser_pt = orient(opti_psi, - M_PI_2, LASER_RANGE, opti_East, opti_North);

	 float *laser_telemetry = wall_detection(opti_East, opti_North, left_laser_pt[0],
							  left_laser_pt[1], forward_laser_pt[0], forward_laser_pt[1],
							  right_laser_pt[0], right_laser_pt[1], opti_z,
							  opti_phi, opti_theta);

	 // printf("%f, %f, %f\n", opti_theta, opti_phi, opti_z);
	 printf("left: %f, forward: %f, right: %f, top: %f\n", laser_telemetry[0], laser_telemetry[1], laser_telemetry[2], laser_telemetry[3]);



	 DOWNLINK_SEND_IR_SENSORS(DefaultChannel, DefaultDevice,
			&laser_telemetry[0], &laser_telemetry[1], &laser_telemetry[2],&laser_telemetry[3],&l5 );

	 free(left_laser_pt);
	 free(forward_laser_pt);
	 free(right_laser_pt);
	 free(laser_telemetry);
}

void wall_avoidance_event(void) {};
//void wall_avoidance_datalink_callback() {};


