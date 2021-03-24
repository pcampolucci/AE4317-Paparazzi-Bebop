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
#include "modules/orange_avoider/trajectory_optimizer.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <time.h>
#include <stdio.h>
#include <stdbool.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// Declare functions used in the code
static void moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static void buildOuterTrajectory(void);
static void buildInnerTrajectory(uint8_t currentOuterTrajIndex);
static void moveWaypointNext(uint8_t waypoint, struct EnuCoor_i *trajectory, uint8_t index_current_waypoint);
static void checkWaypointArrival(uint8_t waypoint_target, double *mseVar);
static bool update_trajectory(struct Obstacle *obstacle_map, struct EnuCoor_i *start_trajectory, uint8_t *size);

// define and initialise global variables
double mse_outer;                              // mean squared error to check if we reached the outer target waypoint
double mse_inner;                              // mean squared error to check if we reached the inner target waypoint
uint8_t outer_index = 0;                       // index of the outer waypoint the drone is moving towards
uint8_t inner_index = 0;                       // index of the inner waypoint the drone is moving towards
uint8_t subtraj_index = 0;                     // index of the subtrajectory the drone is using to fly
bool trajectory_updated = false;               // check if it is safe to use the incoming trajectory
uint8_t n_obstacles = OBSTACLES_IN_MAP;        // indicates the number of msg present in the map      

// build variables for trajectories
struct EnuCoor_i *outer_trajectory;
struct EnuCoor_i *inner_trajectory;
struct TrajectoryList *full_trajectory; 
struct Obstacle *obstacle_map;

/*
 * This next section defines an ABI messaging event (http://wiki.paparazziuav.org/wiki/ABI), necessary
 * any time data calculated in another module needs to be accessed. Including the file where this external
 * data is defined is not enough, since modules are executed parallel to each other, at different frequencies,
 * in different threads. The ABI event is triggered every time new data is sent out, and as such the function
 * defined in this file does not need to be explicitly called, only bound in the init function
 */
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event color_detection_ev;

/* Update Obstacle Map based on Obstacle Detector Info */
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id, struct ObstacleMsg *msg)
{
  VERBOSE_PRINT("Received a message of size %d\n", msg->size);

  for (int i=0; i < msg->size; i++){
      // if we are getting a zero value, we just ignore it
      if (msg->obstacles[i].distance == 0) {
        msg->obstacles[i].distance = 1;
      }
    float origin_mse = sqrt(pow(msg->obstacles[i].distance,2)+pow(msg->obstacles[i].left_heading,2)+pow(msg->obstacles[i].right_heading,2));
    if (origin_mse >= 0.0) {
      VERBOSE_PRINT("Received valid obstacle message %f, %f, %f\n", msg->obstacles[i].distance, msg->obstacles[i].left_heading, msg->obstacles[i].right_heading);
      struct EnuCoor_i absolute_position;
      double heading = RadOfDeg((msg->obstacles[i].left_heading + msg->obstacles[i].right_heading)/2);
      absolute_position.x = POS_BFP_OF_REAL(GetPosX() + sin(heading + stateGetNedToBodyEulers_f()->psi) * msg->obstacles[i].distance);
      absolute_position.y = POS_BFP_OF_REAL(GetPosY() + cos(heading + stateGetNedToBodyEulers_f()->psi) * msg->obstacles[i].distance);
      VERBOSE_PRINT("drone state (psi, x, y, z): %f %f %f %f\n", stateGetNedToBodyEulers_f()->psi, GetPosX(), GetPosY(), GetPosAlt());
      VERBOSE_PRINT("obstacle absolute : %f %f\n", POS_FLOAT_OF_BFP(absolute_position.x), POS_FLOAT_OF_BFP(absolute_position.y));
    }
  }
}

/*
 * Initialisation function, setting the colour filter, random seed and heading_increment
 */
void orange_avoider_init(void)
{

  // Build arrays (trajectory and obstacle) with initial memory alloaction
  outer_trajectory = malloc(sizeof(struct EnuCoor_i) * OUTER_TRAJECTORY_LENGTH);
  obstacle_map = malloc(sizeof(struct Obstacle) * OBSTACLES_IN_MAP);

  // Populate the outer trajectory with inner trajectories (one for index)
  full_trajectory = malloc(sizeof(struct TrajectoryList) * OUTER_TRAJECTORY_LENGTH);

  for (int i = 0; i < OUTER_TRAJECTORY_LENGTH; i++) {
    full_trajectory[i].inner_trajectory = malloc(sizeof(struct EnuCoor_i) * INNER_TRAJECTORY_LENGTH);
    full_trajectory[i].size = INNER_TRAJECTORY_LENGTH;
  }

  // Build outer trajectory based on few sparse waypoints
  buildOuterTrajectory();

  // For each space between outer waypoints build an editable pointwise inner trajectory
  for (int i = 0; i < OUTER_TRAJECTORY_LENGTH; i++)
  {
    buildInnerTrajectory(i);
  }

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgOBSTACLE_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
}

/*
 * Function that checks it reached a waypoint, and then updates the new ones 
 */
void orange_avoider_periodic(void)
{
  // Starting subtrajectory gets modified based on the presence of msg in the map
  clock_t t_periodic; 
  t_periodic = clock();

  // only evaluate our state machine if we are flying
  // if(!autopilot_in_flight()){
  //   return;
  // }

  // Check how close we are from the targets
  checkWaypointArrival(WP_OUTER, &mse_outer);      // Calculate how close it is from the next outer waypoint
  checkWaypointArrival(WP_INNER, &mse_inner);      // Calculate how close it is from the next inner waypoint

  if (mse_inner < 0.15 && trajectory_updated){

    VERBOSE_PRINT("[INNER TRAJECTORY] Setting new Waypoint at %d, going to : (%f/%f) \n", \
    inner_index, \
    POS_FLOAT_OF_BFP(full_trajectory[subtraj_index].inner_trajectory[inner_index].x), \
    POS_FLOAT_OF_BFP(full_trajectory[subtraj_index].inner_trajectory[inner_index].y));

    moveWaypointNext(WP_INNER, full_trajectory[subtraj_index].inner_trajectory, inner_index);

    if (inner_index < full_trajectory[subtraj_index].size-1) {
      inner_index += 1;
    }
  
  } if (mse_outer < 0.15) {

    if (outer_index < OUTER_TRAJECTORY_LENGTH-1) {
      outer_index += 1;
      subtraj_index = outer_index - 1;
    } else {
      outer_index = 0;
      subtraj_index = OUTER_TRAJECTORY_LENGTH-1;
    }

    inner_index = 0;

    VERBOSE_PRINT("[OUTER TRAJECTORY] Setting new Waypoint at %d, going to : (%f/%f) \n", \
    outer_index, \
    POS_FLOAT_OF_BFP(outer_trajectory[outer_index].x), \
    POS_FLOAT_OF_BFP(outer_trajectory[outer_index].y));

    // move to the next waypoint
    moveWaypointNext(WP_OUTER, outer_trajectory, outer_index);

    trajectory_updated = update_trajectory(obstacle_map, full_trajectory[subtraj_index].inner_trajectory, &full_trajectory[subtraj_index].size);

  }

  NavGotoWaypointHeading(WP_INNER);

  t_periodic = clock() - t_periodic; 
  double time_taken_trajectory = 1000 * ((double)t_periodic)/CLOCKS_PER_SEC; // in milliseconds 
  //VERBOSE_PRINT("Time Taken for Periodic : %f ms\n", time_taken_trajectory);
  return;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
bool update_trajectory(struct Obstacle *obstacle_map, struct EnuCoor_i *start_trajectory, uint8_t *size) {
  clock_t t_trajectory; 
  t_trajectory = clock();
  struct EnuCoor_i *new_inner = optimize_trajectory(obstacle_map, start_trajectory, size);
  full_trajectory[subtraj_index].inner_trajectory = realloc(full_trajectory[subtraj_index].inner_trajectory, sizeof(struct EnuCoor_i) * *size);
  full_trajectory[subtraj_index].inner_trajectory = new_inner;
  t_trajectory = clock() - t_trajectory; 
  double time_taken_trajectory = 1000 * ((double)t_trajectory)/CLOCKS_PER_SEC; // in milliseconds 
  VERBOSE_PRINT("Time Taken for Trajectory Optimization : %f ms\n", time_taken_trajectory);
  return true;
  free(new_inner);
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
void moveWaypointNext(uint8_t waypoint, struct EnuCoor_i *trajectory, uint8_t index_current_waypoint)
{ 
  moveWaypoint(waypoint, &trajectory[index_current_waypoint]);
}

/*
 * Checks if WP_GOAL is very close to WP_TARGET, then change the waypoint
 */
void checkWaypointArrival(uint8_t waypoint_target, double *mseVar)
{
  double error_x = GetPosX() - WaypointX(waypoint_target);
  double error_y = GetPosY() - WaypointY(waypoint_target);
  *mseVar = sqrt(pow(error_x,2)+pow(error_y,2));
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
void moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
}

/*
 * Builds the trajectory in the contour by random values
 */
void buildOuterTrajectory(void) {

  // set outer trajectory points
  double rx_list[5] = {0, 2, 2};
  double ry_list[5] = {0, 2, -2};

  VERBOSE_PRINT("------------------------------------------------------------------------------------ \n");

  // populate outer_tajectory struct
  for (int i = 0; i < OUTER_TRAJECTORY_LENGTH; i++) {
      outer_trajectory[i].x = POS_BFP_OF_REAL(rx_list[i]);
      outer_trajectory[i].y = POS_BFP_OF_REAL(ry_list[i]);
      VERBOSE_PRINT("[OUTER TRAJECTORY] Point added: (%f/%f) \n", POS_FLOAT_OF_BFP(outer_trajectory[i].x), POS_FLOAT_OF_BFP(outer_trajectory[i].y));
  }

  VERBOSE_PRINT("------------------------------------------------------------------------------------ \n");
}

/*
 * Creates an 'inner' trajectory between the trajectory[i] and trajectory[i+1]
 */
void buildInnerTrajectory(uint8_t outer_index){

  uint8_t actual_index = outer_index + 1;

  if (outer_index == OUTER_TRAJECTORY_LENGTH-1) {
    actual_index = 0;
  }

  float x_diff = POS_FLOAT_OF_BFP(outer_trajectory[actual_index].x) - POS_FLOAT_OF_BFP(outer_trajectory[outer_index].x);
  float y_diff = POS_FLOAT_OF_BFP(outer_trajectory[actual_index].y) - POS_FLOAT_OF_BFP(outer_trajectory[outer_index].y);
  float increment_x =  x_diff/(INNER_TRAJECTORY_LENGTH);
  float increment_y = y_diff/(INNER_TRAJECTORY_LENGTH);

  VERBOSE_PRINT("------------------------------------------------------------------------------------ \n");

  for (int i = 0; i < INNER_TRAJECTORY_LENGTH; i++){

    // Create set of points between current position and the desired waypoint (initialized as straight line)
    full_trajectory[outer_index].inner_trajectory[i].x = POS_BFP_OF_REAL((i+1)*increment_x + POS_FLOAT_OF_BFP(outer_trajectory[outer_index].x));
    full_trajectory[outer_index].inner_trajectory[i].y = POS_BFP_OF_REAL((i+1)*increment_y + POS_FLOAT_OF_BFP(outer_trajectory[outer_index].y));

    VERBOSE_PRINT("[INNER TRAJECTORY] Point added: (%f/%f) \n", POS_FLOAT_OF_BFP(full_trajectory[outer_index].inner_trajectory[i].x), POS_FLOAT_OF_BFP(full_trajectory[outer_index].inner_trajectory[i].y));
  }

  VERBOSE_PRINT("------------------------------------------------------------------------------------ \n");
}

