/**
 * @file "modules/orange_avoider/trajectory_optimizer.c"
 * @author Pietro Campolucci
 * Trajectory optimized with the potential field method
 */

#include "modules/orange_avoider/orange_avoider.h"
#include "modules/orange_avoider/trajectory_optimizer.h"  // with this we ensure that the header is self contained
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
#define INNER_TRAJECTORY_LENGTH 5
#define OBSTACLES_IN_MAP 3

#ifdef INFINITY
/* INFINITY is supported */
#endif

#define PRINT(string,...) fprintf(stderr, "[trajectory_optimizer->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

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
};

struct Trajectory {
  double *x;
  double *y;
  int size;
};

struct PotentialMap {
  double **pmap;    // potential 2d map
  int size_x;
  int size_y;
  double minx;
  double miny;
};

// define settings
float grid_size = 0.1;     // potential grid size [m]
float robot_radius = 0.5;  // robot radius [m]
double minx;
double miny;
struct dpoint motion[8];
struct Trajectory resulting_trajectory;
struct PotentialMap potential;


// define objects to specify


// declare functions 
// static void optimize_trajectory(struct Obstacle obstacle_map, struct EnuCoor_i start_trajectory);
// static double potential_field_planning(double sx, double sy, double gx, double gy, double ox, double oy, double reso, double rr);
// static double calc_potential_field(double gx, double gy, double ox, double oy, double reso, double rr, double sx, double sy);
// static double calc_attractive_potential(double x, double y, double gx, double gy);
// static double calc_repulsive_potential(double x, double y, double ox, double oy, double rr);
// static void get_motion_model();
// static double oscillations_detection(double previous_ids, double ix, double iy);

// helpers

// return max of the array 
int MaxArray(double *array, int n) 
{ 
    int mx = INT8_MIN; 
    for (int i = 0; i < n; i++) { 
        int temp = array[i];
        mx = Max(mx, temp); 
    } 
    return mx; 
}

// return min of the array 
int MinArray(double *array, int n) 
{ 
    int mx = INT8_MIN; 
    for (int i = 0; i < n; i++) { 
        int temp = array[i];
        mx = Min(mx, temp); 
    } 
    return mx; 
}

/*
 * Do the actual magic
 */
void optimize_trajectory(struct Obstacle *obstacle_map, struct EnuCoor_i *start_trajectory) {
  VERBOSE_PRINT("Trajectory Optimisation Started \n");

  // setup initial conditions required for the optimisation
  double sx = POS_FLOAT_OF_BFP(start_trajectory[0].x);                        // starting point in x [m]
  double sy = POS_FLOAT_OF_BFP(start_trajectory[0].y);                        // starting point in y [m]
  double gx = POS_FLOAT_OF_BFP(start_trajectory[INNER_TRAJECTORY_LENGTH-1].x);      // final point in x [m]
  double gy = POS_FLOAT_OF_BFP(start_trajectory[INNER_TRAJECTORY_LENGTH-1].y);      // final point in y [m]

  VERBOSE_PRINT("1\n");

  // get list of obstacles that will be inserted in the optimisation
  double *ox = malloc(sizeof(double*) * OBSTACLES_IN_MAP);
  double *oy = malloc(sizeof(double*) * OBSTACLES_IN_MAP);

  VERBOSE_PRINT("2\n");

  for (int i = 0; i < OBSTACLES_IN_MAP; i++) {
    ox[i] = obstacle_map[i].loc.x;
    oy[i] = obstacle_map[i].loc.y;
  }

  VERBOSE_PRINT("Received Trajectory of length: %d\n", INNER_TRAJECTORY_LENGTH);
  VERBOSE_PRINT("Initial Trajectory of length: %d\n", resulting_trajectory.size);
  VERBOSE_PRINT("Sending (%f/%f) as starting point\n", sx, sy);
  VERBOSE_PRINT("Sending (%f/%f) as goal point\n", gx, gy);
  VERBOSE_PRINT("Sending (%f) as grid size\n", grid_size);
  VERBOSE_PRINT("Sending (%f) as drone radius\n", robot_radius);

  // run the optimisation and return a new Trajectory
  potential_field_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_radius);

  VERBOSE_PRINT("Post Computed Trajectory of length: %d\n", resulting_trajectory.size);

  VERBOSE_PRINT("4\n");

  // once this is done rx and ry should be ready to be sent to the Trajectory
  // first we reallocate once the Trajectory to the new length
  //start_trajectory = (struct EnuCoor_i *)realloc(start_trajectory, sizeof(struct EnuCoor_i) * resulting_trajectory.size);

  VERBOSE_PRINT("5\n");
  // then we go through the whole new Trajectory to assign the new values
  for (int i = 0; i < resulting_trajectory.size; i++) {
    start_trajectory[i].x = resulting_trajectory.x[i];
    start_trajectory[i].y = resulting_trajectory.y[i];
  }

  VERBOSE_PRINT("6\n");

}

/*
 * Updates the path 
 */
void potential_field_planning(double sx, double sy, double gx, double gy, double *ox, double *oy, double reso, double rr) {

  VERBOSE_PRINT("Potential Field Calculation started\n");

  // update old field information
  calc_potential_field(gx, gy, ox, oy, reso, rr, sx, sy);

  // get new potential field
  double **pmap = potential.pmap;

  // search path
  double d = hypot(sx - gx, sy - gy);
  int ix = round((sx - minx) / reso);
  int iy = round((sy - miny) / reso);
  int minix = 0;
  int miniy = 0;
  double p;

  // construct solution Trajectory used to update the pointer
  // build two arrays of new solution coordinates
  size_t trajectory_size = 1;
  double *rx = malloc(sizeof(double*) * trajectory_size);
  double *ry = malloc(sizeof(double*) * trajectory_size);

  rx[0] = sx;
  ry[0] = sy;

  get_motion_model();
  // previous_ids = deque()

  while (d >= reso) {
    double minp = INFINITY;

    double motion_len = sizeof(motion);

    for (int i = 0; i < motion_len; i++) {
      int inx = ix + motion[i].dx;
      int iny = iy + motion[i].dy;

      int num_rows = potential.size_x;
      int num_cols = potential.size_y;

      if (inx >= num_rows || iny >= num_cols || inx < 0 || iny < 0) {
        p = INFINITY;
        // VERBOSE_PRINT("Outside potential \n");
      } else {
        p = pmap[inx][iny];
        // VERBOSE_PRINT("Inside potential \n");
      }

      if (minp > p) {
        minp = p;
        minix = inx;
        miniy = iny;
      }
    }

    ix = minix;
    iy = miniy;
    double xp = ix * reso + potential.minx;
    double yp = iy * reso + potential.miny;
    d = hypot(gx - xp, gy - yp);

    // now we can expand the solution space and add a new point
    trajectory_size += 1;
    rx = realloc(rx, sizeof(double*) * trajectory_size);
    ry = realloc(ry, sizeof(double*) * trajectory_size);
    rx[trajectory_size-1] = xp;
    ry[trajectory_size-1] = yp;
    VERBOSE_PRINT("Found (%f/%f) as new optimal point\n", xp, yp);

    // if (oscillations_detection(previous_ids, ix, iy)):
    //     print("Oscillation detected at ({},{})!".format(ix, iy))
    //     break
  }

  // update trajectory message 
  resulting_trajectory.x = rx;
  resulting_trajectory.y = ry;
  resulting_trajectory.size = trajectory_size;

}

/*
 * Calculate potential field
 */
void calc_potential_field(double gx, double gy, double *ox, double *oy, double reso, double rr, double sx, double sy)
{

  VERBOSE_PRINT("Calculating Potential Map \n");

  int ox_len = sizeof(ox);
  int oy_len = sizeof(oy);
  // get map contour
  minx = Min(Min(MinArray(ox, ox_len), sx), gx) - AREA_WIDTH / 2.0;
  miny = Min(Min(MinArray(oy, oy_len), sy), gy) - AREA_WIDTH / 2.0;
  double maxx = Max(Max(MaxArray(ox, ox_len), sx), gx) + AREA_WIDTH / 2.0;
  double maxy = Max(Max(MaxArray(oy, oy_len), sy), gy) + AREA_WIDTH / 2.0;

  // get step resolution in the map
  uint32_t xw = round((maxx - minx) / reso);
  uint32_t yw = round((maxy - miny) / reso);

  // build an array representring the map
  double **pmap = malloc(sizeof(double*) * xw);

  for(uint32_t i = 0; i < xw; i++) {
        pmap[i] = malloc(sizeof(double*) * yw);
    }

  // populate map with values regarding potential
  for (uint32_t ix = 0; ix < xw; ix++) {
    double x = ix * reso + minx;

    for (uint32_t iy = 0; iy < yw; iy++) {
      double y = iy * reso + miny;
      double ug = calc_attractive_potential(x, y, gx, gy);
      double uo = calc_repulsive_potential(x, y, ox, oy, rr);
      double uf = ug + uo;
      pmap[ix][iy] = uf;
    }
  }

  // update potential map
  potential.pmap = pmap;
  potential.size_x = xw;
  potential.size_y = yw;
  potential.minx = minx;
  potential.miny = miny;

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
double calc_repulsive_potential(double x, double y, double *ox, double *oy, double rr) {
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
    return 0.5 * ETA * pow(1.0 / dq - 1.0 / rr, 2);
  
  } else {
    return 0.0;
  }
}

/*
 * Get fixed motion model
 */
void get_motion_model() {

  int dx_list[8] = {1, 0, -1, 0, -1, -1, 1, 1};
  int dy_list[8] = {0, 1, 0, -1, -1, 1, -1, 1};

  for (int i = 0; i < 8; i++) {
    motion[i].dx = dx_list[i];
    motion[i].dy = dy_list[i]; 
  }
}

/*
 * Oscillation detection
 */
double oscillations_detection(double previous_ids, double ix, double iy) {
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

