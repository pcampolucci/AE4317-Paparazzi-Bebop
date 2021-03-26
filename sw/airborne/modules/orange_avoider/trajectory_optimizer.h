/**
 * @file "modules/orange_avoider/trajectory_optimizer.h"
 * @author Pietro Campolucci
 * Trajectory optimized with the potential field method
 */


#ifndef TRAJECTORY_OPTIMIZER
#define TRAJECTORY_OPTIMIZER

#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <time.h>
#include <stdio.h>
#include <math.h>

// struct
extern struct EnuCoor_i *start_trajectory;
extern struct Obstacle *obmap;

// functions declarations
extern struct EnuCoor_i *optimize_trajectory(struct Obstacle *obmap, struct EnuCoor_i *start_trajectory, uint8_t *current_length, uint8_t obstacles);
extern void potential_field_planning(double sx, double sy, double gx, double gy, double *ox, double *oy, double reso, double rr);
extern void calc_potential_field(double gx, double gy, double *ox, double *oy, double reso, double rr, double sx, double sy);
extern double calc_attractive_potential(double x, double y, double gx, double gy);
extern double calc_repulsive_potential(double x, double y, double *ox, double *oy, double rr);
extern void get_motion_model(void);
extern double oscillations_detection(double previous_ids, double ix, double iy);

// helpers declarations
extern int MaxArray(double *array, int n);
extern int MinArray(double *array, int n);

#endif /* TRAJECTORY_OPTIMIZER */