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

#ifndef ORANGE_AVOIDER_H
#define ORANGE_AVOIDER_H

// settings
// struct point {};

// void build_trajectory(void);
// build empty trajectory of x,y coordinates

struct point {
  float x;
  float y;
};

#define TRAJECTORY_LENGTH 3

struct point trajectory[TRAJECTORY_LENGTH];

void build_trajectory(void);

// settings
extern float oa_color_count_frac;

// functions
extern void orange_avoider_init(void);
extern void orange_avoider_periodic(void);

#endif

