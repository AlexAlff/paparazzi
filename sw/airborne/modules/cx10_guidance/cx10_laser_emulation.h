/*
 * Copyright (C) AAlff
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/wall_avoidance/cx10_laser_emulation.h"
 * @author AAlff
 * Don't hit walls
 */

#ifndef CX10_LASER_EMULATION_H
#define CX10_LASER_EMULATION_H

//extern void air_data_init(void);

extern void cx10_laser_emulation_init(void);
extern void cx10_laser_emulation_start(void);
extern void cx10_laser_emulation_stop(void);
extern void cx10_laser_emulation_periodic(void);
extern void cx10_laser_emulation_event(void);
//extern void cx10_laser_emulation_datalink_callback();

extern float laser_telemetry[4];

#define LASER_RANGE 1.8

#endif
