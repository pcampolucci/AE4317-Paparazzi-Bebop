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

// Local constants
#define KP 5                              // attractive potential gain
#define ETA 100                           // repulsive potential gain
#define AREA_WIDTH 30                     // potential area width
#define OSCILLATION_DETECTION_LENGTH 6    // the number of previous positions used to check oscillations
#define GRID_SIZE 0.5                     // size of the motion grid
#define DRONE_RADIUS 0.3                  // circular space occupied by the drone in motion
#define MAX_LENGTH_NEW_TRAJECTORY 15      // maximum accepted length for new computed trajectory

#ifdef INFINITY
/* INFINITY is supported */
#endif

#define PRINT(string,...) fprintf(stderr, "[trajectory_optimizer->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// Initialize objects and variables
struct DPoint motion[8];                        // motion choices for trajectory
struct Trajectory resulting_trajectory;         // final optimized trajectory object
struct PotentialMap potential = {0,0,0,0,0};    // potential map used to find the optimal
bool out_of_bounds = false;                     // check if trajectory length exceeds the limits

/*
 * Do the actual magic
 */
struct OptimizedTrajectory optimize_trajectory(struct Obstacle *obstacle_map_opt, struct EnuCoor_i *start_trajectory_opt, uint8_t *current_length, uint8_t obstacles) {
  VERBOSE_PRINT("------------------------------------------------------------------------------------ \n");

  // setup initial conditions required for the optimisation
  double sx = POS_FLOAT_OF_BFP(start_trajectory_opt[0].x);                      // starting point in x [m]
  double sy = POS_FLOAT_OF_BFP(start_trajectory_opt[0].y);                      // starting point in y [m]
  double gx = POS_FLOAT_OF_BFP(start_trajectory_opt[*current_length-1].x);      // final point in x [m]
  double gy = POS_FLOAT_OF_BFP(start_trajectory_opt[*current_length-1].y);      // final point in y [m]

  // get list of obstacles that will be inserted in the optimisation
  double ox[obstacles];
  double oy[obstacles];

  // get first and last point to append to the resulting trajectory
  int last_point_index = *current_length;
  struct EnuCoor_i first_point = start_trajectory_opt[0];
  struct EnuCoor_i last_point = start_trajectory_opt[last_point_index-1];

  for (int i = 0; i < obstacles; i++) {
    ox[i] = POS_FLOAT_OF_BFP(obstacle_map_opt[i].loc.x);
    oy[i] = POS_FLOAT_OF_BFP(obstacle_map_opt[i].loc.y);
    VERBOSE_PRINT("[OPTIMIZER] Obstacle %d in Map (%f/%f)\n", i+1, ox[i], oy[i]);
  }

  VERBOSE_PRINT("[OPTIMIZER] Received Trajectory of length: %d\n", *current_length);
  VERBOSE_PRINT("[OPTIMIZER] Sending (%f/%f) as starting point\n", sx, sy);
  VERBOSE_PRINT("[OPTIMIZER] Sending (%f/%f) as goal point\n", gx, gy);

  // run the optimisation and return a new Trajectory
  potential_field_planning(sx, sy, gx, gy, ox, oy, GRID_SIZE, DRONE_RADIUS);

  // check if we managed to update the trajectory, otherwise we use the old one
  if(out_of_bounds){

    VERBOSE_PRINT("[OPTIMIZER] Optimisation Failed! Keeping the old trajectory\n");

    struct OptimizedTrajectory optimized_trajectory;
    optimized_trajectory.size = *current_length;

    // then we go through the whole new Trajectory to assign the new values
    for (int i = 0; i < *current_length; i++) {
      optimized_trajectory.buf[i].x = start_trajectory_opt[i].x;
      optimized_trajectory.buf[i].y = start_trajectory_opt[i].y;
    }

    out_of_bounds = false;

    VERBOSE_PRINT("------------------------------------------------------------------------------------ \n");

    return optimized_trajectory;
  }
  

  VERBOSE_PRINT("[OPTIMIZER] Post Computed Trajectory of length: %d\n", resulting_trajectory.size);
  *current_length = resulting_trajectory.size;

  // once this is done rx and ry should be ready to be sent to the Trajectory
  // first we reallocate once the Trajectory to the new length
    struct OptimizedTrajectory optimized_trajectory;
    optimized_trajectory.size = *current_length;

  // then we go through the whole new Trajectory to assign the new values
  for (int i = 1; i < resulting_trajectory.size-1; i++) {
    optimized_trajectory.buf[i].x = POS_BFP_OF_REAL(resulting_trajectory.x[i+1]);
    optimized_trajectory.buf[i].y = POS_BFP_OF_REAL(resulting_trajectory.y[i+1]);
  }

  // append first and last point
  optimized_trajectory.buf[0] = first_point;
  optimized_trajectory.buf[resulting_trajectory.size-1] = last_point;

  for (int i = 0; i < resulting_trajectory.size; i++) {
    VERBOSE_PRINT("[OPTIMIZER] Adding point %f/%f\n", POS_FLOAT_OF_BFP(optimized_trajectory.buf[i].x), POS_FLOAT_OF_BFP(optimized_trajectory.buf[i].y));
  }

  VERBOSE_PRINT("------------------------------------------------------------------------------------ \n");

  return optimized_trajectory;
}

/*
 * Updates the path 
 */
void potential_field_planning(double sx, double sy, double gx, double gy, double *ox, double *oy, double reso, double rr) {

  //VERBOSE_PRINT("Potential Field Calculation started\n");

  // update old field information 
  calc_potential_field(gx, gy, ox, oy, reso, rr, sx, sy);

  // get new potential field
  double **pmap = potential.pmap;

  // search path
  double d = hypot(sx - gx, sy - gy);
  int ix = round((sx - potential.minx) / reso);
  int iy = round((sy - potential.miny) / reso);
  int minix = 0;
  int miniy = 0;
  double p;

  // construct solution Trajectory used to update the pointer
  // build two arrays of new solution coordinates
  size_t trajectory_size = 1;
  double rx[MAX_LENGTH_NEW_TRAJECTORY];
  double ry[MAX_LENGTH_NEW_TRAJECTORY];

  rx[0] = sx;
  ry[0] = sy;

  get_motion_model();

  while (d >= reso) {
    double minp = INFINITY;

    double motion_len = sizeof(motion) / sizeof(motion[0]);

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

    // check if we are going out of bounds
    if(trajectory_size >= MAX_LENGTH_NEW_TRAJECTORY-1) {
      out_of_bounds = true;
      break;
    } else {
      trajectory_size += 1;
    }

    // now we can expand the solution space and add a new point
    rx[trajectory_size-1] = xp;
    ry[trajectory_size-1] = yp;
    // VERBOSE_PRINT("Found (%f/%f) as new optimal point at index %d\n", rx[trajectory_size-1], ry[trajectory_size-1], trajectory_size);

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

  //VERBOSE_PRINT("Calculating Potential Map \n");

  int ox_len = round(sizeof(ox) / sizeof(ox[0]));
  int oy_len = round(sizeof(oy) / sizeof(oy[0]));
  // get map contour
  potential.minx = Min(Min(MinArray(ox, ox_len), sx), gx) - AREA_WIDTH / 2.0;
  potential.miny = Min(Min(MinArray(oy, oy_len), sy), gy) - AREA_WIDTH / 2.0;
  double maxx = Max(Max(MaxArray(ox, ox_len), sx), gx) + AREA_WIDTH / 2.0;
  double maxy = Max(Max(MaxArray(oy, oy_len), sy), gy) + AREA_WIDTH / 2.0;

  // get step resolution in the map
  uint32_t xw = round((maxx - potential.minx) / reso);
  uint32_t yw = round((maxy - potential.miny) / reso);

  // build an array representring the map
  if(potential.pmap != 0){
    for(uint32_t i = 0; i < xw; i++) {
      free(potential.pmap[i]);
    }
    free(potential.pmap);
  }

  // build an array representring the map
  double **pmap = malloc(sizeof(double*) * xw);
  for(uint32_t i = 0; i < xw; i++) {
        pmap[i] = malloc(sizeof(double*) * yw);
    }

  potential.pmap = pmap;
  potential.size_x = xw;
  potential.size_y = yw;

  // populate map with values regarding potential
  for (uint32_t ix = 0; ix < xw; ix++) {
    double x = ix * reso + potential.minx;

    for (uint32_t iy = 0; iy < yw; iy++) {
      double y = iy * reso + potential.miny;
      double ug = calc_attractive_potential(x, y, gx, gy);
      double uo = calc_repulsive_potential(x, y, ox, oy, rr);
      double uf = ug + uo;
      pmap[ix][iy] = uf;
    }
  }
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
// double oscillations_detection(double previous_ids, double ix, double iy) {
//   previous_ids.append((ix, iy))
//   if (len(previous_ids) > OSCILLATIONS_DETECTION_LENGTH):
//       previous_ids.popleft()
//   // check if contains any duplicates by copying into a set
//   previous_ids_set = set()
//   for index in previous_ids:
//       if index in previous_ids_set:
//           return 1
//       else:
//           previous_ids_set.add(index)
//   return 0
//   return 0;
// }

/*
 * Helper 1: estimate max value in the array
 */
int MaxArray(double *array, int n) 
{ 
    int mx = 0; 
    for (int i = 0; i < n-1; i++) { 
        int temp = array[i];
        mx = Max(mx, temp); 
    } 
    return mx; 
}

/*
 * Helper 2: estimate min value in the array
 */
int MinArray(double *array, int n) 
{ 
    int mx = 10e5; 
    for (int i = 0; i < n-1; i++) { 
        int temp = array[i];
        mx = Min(mx, temp); 
    } 
    return mx; 
}

