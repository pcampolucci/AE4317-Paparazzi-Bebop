/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.h"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */


#ifndef TRAJECTORY_OPTIMIZER
#define TRAJECTORY_OPTIMIZER

// struct
extern struct EnuCoor_i start_trajectory;
extern struct Obstacle obmap;

// settings
extern void optimize_trajectory(struct Obstacle *obmap, struct EnuCoor_i *start_trajectory);

#endif