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

extern void wall_avoidance_init();
extern void wall_avoider_start();
extern void wall_avoider_stop();
extern void wall_avoider_periodic();
extern void wall_avoidance_event();
//extern void wall_avoidance_datalink_callback();

#endif

