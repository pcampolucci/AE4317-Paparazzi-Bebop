/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the navigation mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 */

#include "modules/orange_avoider/orange_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <time.h>
#include <stdio.h>
#include <math.h>

#define ORANGE_AVOIDER_VERBOSE TRUE

#define KP 5
#define ETA 100
#define AREA_WIDTH 30
#define OSCILLATION_DETECTION_LENGTH 6

#ifdef INFINITY
/* INFINITY is supported */
#endif

#define PRINT(string,...) fprintf(stderr, "[trajectory_optimizer->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static double calc_attractive_potential(double x, double y, double gx, double gy);

// obstacle struct used for obstacle classification
struct Obstacle {
  struct EnuCoor_i loc;
  double width;
  double heading;
  double depth;
};

struct dpoint {
  int dx;
  int dy;
}

struct dpoint motion[8];

// define settings
float grid_size = 0.5;     // potential grid size [m]
float robot_radius = 2.0;  // robot radius [m]
double minx;
double miny;

/*
 * Do the actual magic
 */
void optimize_trajectory(struct Obstacle obstacle_map, struct EnuCoor_i start_trajectory) {
  VERBOSE_PRINT("Optimizes!!!!!!!!!");
}

/*
 * Calculates attractive potential
 */
double calc_attractive_potential(double x, double y, double gx, double gy) {
  return 0.5 * KP * hypot(x - gx, y - gy);
}

/*
 * Calculates repulsive potential
 */
double calc_repulsive_potential(double x, double y, double ox, double oy, double rr) {
  int minid = -1;
  float dmin = INFINITY;
  double ox_len = sizeof(ox);

  for (int i = 0; i < ox_len; i++) {
    double d = hypot(x - ox[i], y - oy[i]);
    if (dmin >= d) {
      dmin = d;
      minid = i;
    }
  }

  // calc repulsive potential
  double dq = hypot(x - ox[minid], y - oy[minid]);

  if (dq <= rr) {
    if (dq <= 0.1) {
      dq = 0.1;
    }
    return 0.5 * ETA * pow(1.0 / dq = 1.0 / rr, 2);
  
  else:
    return 0.0;
  }
}

/*
 * Oscillation detection
 */
uint8_t oscillations_detection(previous_ids, ix, iy) {
  // previous_ids.append((ix, iy))

  // if (len(previous_ids) > OSCILLATIONS_DETECTION_LENGTH):
  //     previous_ids.popleft()

  // // check if contains any duplicates by copying into a set
  // previous_ids_set = set()
  // for index in previous_ids:
  //     if index in previous_ids_set:
  //         return 1
  //     else:
  //         previous_ids_set.add(index)
  // return 0
  return 0;
}

/*
 * Clculate potential field
 */
double calc_potential_field(double gx, double gy, double ox, double oy, double reso, double rr, double sx, double sy)
{
  // get map contour
  minx = min(min(ox), sx, gx) - AREA_WIDTH / 2.0;
  miny = min(min(oy), sy, gy) - AREA_WIDTH / 2.0;
  double maxx = max(max(ox), sx, gx) + AREA_WIDTH / 2.0;
  double maxy = max(max(oy), sy, gy) + AREA_WIDTH / 2.0;

  // get step resolution in the map
  uint32_t xw = round((maxx - minx) / reso));
  uint32_t yw = round((maxy - miny) / reso));

  // build a matrix representring the map
  double pmap[xw][yw];

  // populate map with values regarding potential
  for (int ix = 0; ix < xw; ix++) {
    double x = ix * reso + minx;

    for (int iy = 0; iy < yw; iy++) {
      double y = iy * reso + miny;
      double ug = calc_attractive_potential(x, y, gx, gy);
      double uo = calc_repulsive_potential(x, y, ox, oy, rr);
      double uf = ug + uo;
      pmap[ix][iy] = uf;
    }
  }

  return pmap;
}

/*
 * Get fixed motion model
 */
void get_motion_model() {

  int dx_list[8] = {1, 0, -1, 0, -1, -1, 1, 1};
  int dy_list[8] = {0, 1, 0, -1, -1, 1, -1, 1};

  for (int i; i < 8; i++) {
    motion[i].dx = dx_list[i];
    motion[i].dy = dy_list[i]; 
  }
}

/*
 * Updates the path 
 */
double potential_field_planning(double sx, double sy, double gx, double gy, double ox, double oy, double reso, double rr) {

  double pmap = calc_potential_field(gx, gy, ox, oy, reso, rr, sx, sy);

  // search path
  double d = hypot(sx - gx, sy - gy);
  double ix = round((sx - minx) / reso);
  double iy = round((sy - miny) / reso);
  double gix = round((gx - minx) / reso);
  double giy = round((gy - miny) / reso);
  double p;
  double minp;
  double minix;
  double miniy;

  double rx = sx;
  double ry = sy;
  motion = get_motion_model();
  // previous_ids = deque()

  while (d >= reso) {
    double minp = INFINITY;
    double minix, miniy = -1;

    double motion_len = sizeof(motion);

    for (int i; i < motion_len; i++) {
      int inx = ix + motion[i][0];
      int iny = iy + motion[i][1];

      if (inx >= sizeof(pmap) || iny >= sizeof(pmap[0]) || inx < 0 || iny < 0) {
        p = INFINITY;
        VERBOSE_PRINT("Outside potential \n");
      } else {
        p = pmap[inx][iny];
      }

      if (minp > p) {
        minp = p;
        minix = inx;
        miniy = iny;
      }
    }

    ix = minix;
    iy = miniy;
    double xp = ix * reso + minx;
    double yp = iy * reso + miny;
    d = hypot(gx - xp, gy - yp);
    // rx.append(xp);
    // ry.append(yp);

    // if (oscillations_detection(previous_ids, ix, iy)):
    //     print("Oscillation detected at ({},{})!".format(ix, iy))
    //     break
  }
}
