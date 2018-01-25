/*
 * Copyright (C) AAlff
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/wall_avoidance/wall_avoidance.c"
 * @author AAlff
 * Virtual L, R, F, and U laser range finders.
 * First C program, could have been more clear if I had used structs and points.
 * Module uses East and CCW as positive, but uses using state's Ned -> heading is multiplied by
 *  -1 and a 90 deg offset has been introduced.
 * Walls treated as line segments ((x1, y1), (x2, y2)) and saved to array walls[] in format: (x1, y1, x2, y2, ...)
 */



#include "cx10_laser_emulation.h"
#include "state.h"
#include <stdio.h>
#include <math.h>
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"


////////////////////////////////////////////////////////////////
// Uncomment desired test, comment all for CyberZoo walls only//
////////////////////////////////////////////////////////////////

#define ARC_TEST
// #define GROOVE_TEST
// #define CABINET_TEST
// #define PARALLELOGRAM_TEST

////////////////////////////////////////////////////////////////

// CyberZoo Wall coordinates

// B-------C
// |       |
// A-------D
// <-Desks->

#define WALL_PT_A -1.60, -4.05
#define WALL_PT_B -3.92,  1.68
#define WALL_PT_C  1.25,  4.06
#define WALL_PT_D  4.18, -1.29

#define LEFT_WALL_CZ   WALL_PT_A, WALL_PT_B  // x1, y1, x2, y2
#define RIGHT_WALL_CZ  WALL_PT_C, WALL_PT_D
#define TOP_WALL_CZ    WALL_PT_B, WALL_PT_C
#define BOTTOM_WALL_CZ WALL_PT_A, WALL_PT_D

#define WALLS_CZ LEFT_WALL_CZ, RIGHT_WALL_CZ,TOP_WALL_CZ, BOTTOM_WALL_CZ

// Define arc
#ifdef ARC_TEST
#define PT_A -1.40, -3.90
#define PT_B -1.78, -2.83
#define PT_C -1.93, -1.39
#define PT_D -1.60,  0.51
#define PT_E -0.68,  2.22
#define PT_F  0.43,  3.34
#define PT_G  1.30,  3.90

#define TEST_WALL1 PT_A, PT_B
#define TEST_WALL2 PT_B, PT_C
#define TEST_WALL3 PT_C, PT_D
#define TEST_WALL4 PT_D, PT_E
#define TEST_WALL5 PT_E, PT_F
#define TEST_WALL6 PT_F, PT_G

#define TEST_WALLS TEST_WALL1, TEST_WALL2, TEST_WALL3, TEST_WALL4, TEST_WALL5, TEST_WALL6

float walls[] = {TEST_WALLS, WALLS_CZ};

// Define groove
#elif defined GROOVE_TEST
#define PT_A -0.20, -3.40
#define PT_B -0.83, -2.01
#define PT_C -1.40, -2.30
#define PT_D -2.20, -0.40
#define PT_E -1.61, -0.16
#define PT_F -2.60,  2.20

#define TEST_WALL1 PT_A, PT_B
#define TEST_WALL2 PT_B, PT_C
#define TEST_WALL3 PT_C, PT_D
#define TEST_WALL4 PT_D, PT_E
#define TEST_WALL5 PT_E, PT_F

#define TEST_WALLS TEST_WALL1, TEST_WALL2, TEST_WALL3, TEST_WALL4, TEST_WALL5

float walls[] = {TEST_WALLS, WALLS_CZ};

// Define cabinet
#elif defined CABINET_TEST
#define PT_A -0.20, -3.40
#define PT_B -1.2, -1.10
#define PT_C -0.60, -0.80
#define PT_D -1.40, 1.00
#define PT_E -1.95, 0.7
#define PT_F -2.60, 2.20

#define TEST_WALL1 PT_A, PT_B
#define TEST_WALL2 PT_B, PT_C
#define TEST_WALL3 PT_C, PT_D
#define TEST_WALL4 PT_D, PT_E
#define TEST_WALL5 PT_E, PT_F

#define TEST_WALLS TEST_WALL1, TEST_WALL2, TEST_WALL3, TEST_WALL4, TEST_WALL5

float walls[] = {TEST_WALLS, WALLS_CZ};

// Define parallelogram
#elif defined PARALLELOGRAM_TEST
#define PT_A -1.18, -3.88
#define PT_B  1.50, -2.59
#define PT_C  1.25,  4.10
#define PT_D -1.40,  2.82

#define TEST_WALL1 PT_A, PT_B
#define TEST_WALL2 PT_B, PT_C
#define TEST_WALL3 PT_C, PT_D
#define TEST_WALL4 PT_D, PT_A

#define TEST_WALLS TEST_WALL1, TEST_WALL2, TEST_WALL3, TEST_WALL4

float walls[] = {TEST_WALLS, WALLS_CZ};

// Only CyberZoo walls
#else
float walls[] = {WALLS_CZ};
#endif

///////////////////////////////////////////////////////////////////////////////////////////

const float OFFSET = M_PI_2; //-1.0036; // for heading, in rad, here 0 deg is east
const float CEILING = 2; //ceiling height

void orient(float *coords, float heading, float offset, float rel_size, float position_x, float position_y);
float distance_2_pts(float x1, float y1, float x2, float y2);
int ccw(float x1, float y1, float x2, float y2, float x3, float y3);
float det(float x1, float y1, float x2, float y2);
float laser_read(float A, float B, float C, float D, float E, float F, float G, float H);
void wall_detection(float position_x, float position_y, float *left_pt, float *forward_pt, float *right_pt, float height, float cx10_phi, float cx10_theta);

// float wall_detection(float position, float left_pt, float forward_pt, float right_pt);

float opti_North;
float opti_East;
float opti_z;
float cx10_theta;
float cx10_phi;
float cx10_psi;

float laser_telemetry[4];
int print_tick = 0;


void orient(float *coords, float heading, float offset, float rel_size, float position_x, float position_y)
{
// Gives position of a point, given another point's heading, required angle offset, and relative distance to first point
	coords[0] = position_x + cos(heading + offset) * rel_size;
	coords[1] = position_y + sin(heading + offset) * rel_size;
}

float distance_2_pts(float xx1, float yy1, float xx2, float yy2)
{
    return sqrt(powf(yy2 - yy1, 2) + powf(xx2 - xx1, 2));
}

int ccw(float xx1, float yy1, float xx2, float yy2, float xx3, float yy3)
{
    // Boe Bryce's algorithm for checking whether 2 line segments cross
    return (yy3 - yy1) * (xx2 - xx1) > (yy2 - yy1) * (xx3 - xx1);
}

float det(float xx1, float yy1, float xx2, float yy2)
{  // For 2D only
    return xx1 * yy2 - yy1 * xx2;
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
    float xx = det(d[0], d[1], xdiff[0], xdiff[1]) / div_matrix;
    float yy = det(d[0], d[1], ydiff[0], ydiff[1]) / div_matrix;

    return distance_2_pts(A, B, xx, yy);
}

void wall_detection(float position_x, float position_y, float *left_pt, float *forward_pt, float *right_pt, float height, float ref_phi, float ref_theta)
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

    laser_telemetry[0] = 999.; //left
    laser_telemetry[1] = 999.; //front
    laser_telemetry[2] = 999.; //right
	laser_telemetry[3] = 999.; //up

	// Horizontal readings
    for (int l_idx = 0; l_idx < 12; l_idx += 4)
    {  // Matches lasers' direction with idx for segment and read lists
        for (unsigned w_idx = 0; w_idx < sizeof(walls)/sizeof(float); w_idx += 4)
        {
            float reading= laser_read(lasers[l_idx], lasers[l_idx + 1], lasers[l_idx + 2], lasers[l_idx + 3],
            						   walls[w_idx], walls[w_idx + 1], walls[w_idx + 2], walls[w_idx + 3]);
            if ((reading < laser_telemetry[l_idx / 4]) && ((l_idx == 0)|| (l_idx == 8))) // adjust left and right for roll
            {
            	laser_telemetry[l_idx / 4] = reading / cos(ref_phi);
            }
            if ((reading < laser_telemetry[l_idx / 4]) && (l_idx == 4)) // adjust forward for pitch
            {
            	laser_telemetry[l_idx / 4] = reading / cos(ref_theta);
            }
        }
    }


    // Vertical reading (adjusted for pitch and roll):
    float top_laser = (CEILING - height) / (cos(ref_theta)*cos(ref_phi));
    if (top_laser < LASER_RANGE)
    {
    	laser_telemetry[3] = top_laser;
    }
}

void cx10_laser_emulation_periodic(void)
{
	 float l5;
     print_tick += 1;
	 opti_North = stateGetPositionNed_f()->x; // North
	 opti_East = stateGetPositionNed_f()->y; // East
	 printf("%f,%f\n", opti_East, opti_North);
	 opti_z =  - stateGetPositionNed_f()->z; // making up positive
	 cx10_theta = stateGetNedToBodyEulers_f()-> theta; // pitch
	 cx10_phi = stateGetNedToBodyEulers_f() -> phi; // roll
	 cx10_psi = stateGetNedToBodyEulers_f() -> psi * -1 + OFFSET; // Yaw or Heading +/- pi rad wrt East
	 if(print_tick >= 100)
     {
         //printf("x, y: %f, %f\n", opti_North, opti_East);
         print_tick = 0;
     }
	 float coords_left[2];
	 float coords_forward[2];
	 float coords_right[2];

	 orient(coords_left, cx10_psi, M_PI_2, LASER_RANGE, opti_East, opti_North);
	 orient(coords_forward, cx10_psi, 0, LASER_RANGE, opti_East, opti_North);
	 orient(coords_right, cx10_psi, - M_PI_2, LASER_RANGE, opti_East, opti_North);

	 wall_detection(opti_East, opti_North, coords_left, coords_forward,coords_right, opti_z, cx10_phi, cx10_theta);

	 // printf("%f, %f, %f\n", cx10_theta, cx10_phi, opti_z);
	 // printf("left: %f, forward: %f, right: %f, top: %f\n", laser_telemetry[0], laser_telemetry[1], laser_telemetry[2], laser_telemetry[3]);

	 DOWNLINK_SEND_IR_SENSORS(DefaultChannel, DefaultDevice,
			&laser_telemetry[0], &laser_telemetry[1], &laser_telemetry[2],&laser_telemetry[3], &opti_East, &opti_North, &l5 );
    
}
