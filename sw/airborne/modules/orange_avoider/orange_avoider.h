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

#include "state.h"

// Local constants
#define OUTER_TRAJECTORY_LENGTH 3
#define INNER_TRAJECTORY_LENGTH 5
#define INNER_TRAJECTORY_SPACE 25
#define MAX_OBSTACLES_IN_MAP 25
#define MAX_OBSTACLES_IN_MSG 5
#define MAX_OPTIMIZER_SIZE 20

// Obstacle struct used for optimization and avoidance
struct Obstacle {
  struct EnuCoor_i loc;
  double width;
};

struct Obstacle2 {
  double distance;
  double left_heading;
  double right_heading;
};

struct ObstacleMsg {
  uint8_t size;
  struct Obstacle2 *obstacles;
};

// motion increment point
struct DPoint {
  int dx;
  int dy;
};

// trajectory object
struct Trajectory {
  double *x;
  double *y;
  int size;
};

// whole potential map with size
struct PotentialMap {
  double **pmap;              
  int size_x;
  int size_y;
  double minx;
  double miny;
};

struct OptimizedTrajectory {
  int size; 
  struct EnuCoor_i buf[MAX_OPTIMIZER_SIZE];
};

// trajectory list
struct TrajectoryList {
  struct EnuCoor_i inner_trajectory[INNER_TRAJECTORY_SPACE];
  uint8_t size;
};

// functions
extern void orange_avoider_init(void);
extern void orange_avoider_periodic(void);

#endif

