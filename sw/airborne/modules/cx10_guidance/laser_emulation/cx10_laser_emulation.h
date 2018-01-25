/*
 * Copyright (C) AAlff
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/wall_avoidance/cx10_laser_emulation.h"
 * @author AAlff
 * Emulates what a Left, Forward, Right, and Top laser range finder would read using 3D tracking and IMU data
 */

#ifndef CX10_LASER_EMULATION_H
#define CX10_LASER_EMULATION_H

extern void cx10_laser_emulation_periodic(void);

extern float laser_telemetry[4];

#define LASER_RANGE 1.8

#endif
