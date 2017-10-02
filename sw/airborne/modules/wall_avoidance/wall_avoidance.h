/*
 * Copyright (C) AAlff
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/wall_avoidance/wall_avoidance.h"
 * @author AAlff
 * Don't hit walls
 */

#ifndef WALL_AVOIDANCE_H
#define WALL_AVOIDANCE_H

//extern void air_data_init(void);

extern void wall_avoidance_init(void);
extern void wall_avoider_start(void);
extern void wall_avoider_stop(void);
extern void wall_avoider_periodic(void);
extern void wall_avoidance_event(void);
//extern void wall_avoidance_datalink_callback();


extern float laser1;

#endif

