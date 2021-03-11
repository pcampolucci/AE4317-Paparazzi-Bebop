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


#define OUTER_TRAJECTORY_LENGTH 3
#define INNER_TRAJECTORY_LENGTH 5
#define OBSTACLES_IN_MAP 3

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
static uint8_t chooseRandomIncrementAvoidance(void);
static uint8_t buildTrajectory(void);
static uint8_t buildInnerTrajectory(uint8_t currentOuterTrajIndex);
static uint8_t moveWaypointNext(uint8_t waypoint, struct EnuCoor_i *trajectory, uint8_t index_current_waypoint, uint8_t trajectory_length);
static double checkWaypointArrival(uint8_t waypoint_goal, uint8_t waypoint_target, double mseVar);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS
};

// obstacle struct used for obstacle classification
struct Obstacle {
  struct EnuCoor_i loc;
  double width;
  double heading;
  double depth;
};

// define settings
float oa_color_count_frac = 0.18f;

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;
int32_t color_count = 0;                // orange color count from color filter for obstacle detection
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
float heading_increment = 5.f;          // heading angle increment [deg]
float maxDistance = .75;               // max waypoint displacement [m]
uint8_t current_waypoint_outer = 0;     // index of the outer waypoint the drone is going to
uint8_t current_waypoint_inner = 0;     // index of the inner waypoint the drone is going to
uint8_t current_waypoint_inner_glob = 0; 
double mse_outer = 10;                  // mean squared error to check if we reached the outer target waypoint
double mse_inner = 10;                  // mean squared error to check if we reached the inner target waypoint

struct EnuCoor_i outer_trajectory[OUTER_TRAJECTORY_LENGTH];
struct EnuCoor_i inner_trajectory[INNER_TRAJECTORY_LENGTH];
struct EnuCoor_i inner_trajectory_total[INNER_TRAJECTORY_LENGTH*OUTER_TRAJECTORY_LENGTH]; 
bool trajectory_creation_complete = false; 
struct Obstacle obstacle_map[OBSTACLES_IN_MAP];

const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free

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
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  color_count = quality;
}

/*
 * Initialisation function, setting the colour filter, random seed and heading_increment
 */
void orange_avoider_init(void)
{
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // set up trajectory
  buildTrajectory();
  for(int i=0; i<= OUTER_TRAJECTORY_LENGTH-1; i++)
  {
    buildInnerTrajectory(i);
  }

  waypoint_move_xy_i(WP_NEXT_SUBTARGET, inner_trajectory_total[0].x, inner_trajectory_total[0].y);
  current_waypoint_inner+= 1; 
  current_waypoint_outer += 1; 
  VERBOSE_PRINT("First element subtrajectory: (%f/%f) \n", POS_FLOAT_OF_BFP(inner_trajectory_total[0].x), POS_FLOAT_OF_BFP(inner_trajectory_total[0].y));

  // test: I feed a known obstacle map + the current trajectory sub. The trajectory should get updated. 
  optimize_trajectory(obstacle_map, inner_trajectory);

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
}

/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void orange_avoider_periodic(void)
{
  // only evaluate our state machine if we are flying
  // if(!autopilot_in_flight()){
  //   return;
  // }

  // compute current color thresholds
  int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;

  //VERBOSE_PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state);

  // update our safe confidence using color threshold
  if(color_count < color_count_threshold){
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }

  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  float moveDistance = fminf(maxDistance, 0.2f * obstacle_free_confidence);

  switch (navigation_state){
    case SAFE:
      // Move waypoint forward
      moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);    // Checks 1.5m in front if it reaches the bounds
      mse_outer = checkWaypointArrival(WP_GOAL, WP_NEXT_TARGET, mse_outer);   // Calculate how close it is from the next outer waypoint
      mse_inner = checkWaypointArrival(WP_GOAL, WP_NEXT_SUBTARGET, mse_inner);   // Calculate how close it is from the next inner waypoint
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        navigation_state = OUT_OF_BOUNDS;
        VERBOSE_PRINT("I am in navigation state out of bounds\n");
      } else if (obstacle_free_confidence == 0){
        navigation_state = OBSTACLE_FOUND;
        VERBOSE_PRINT("I am in else if obstacle free confidence\n");
      // Reaches outer waypoint
      } else if (mse_outer < 0.15){
        VERBOSE_PRINT("I am in else if mse outer\n"); 
        // Moves to the next waypoint in the outer trajectory list
        current_waypoint_outer = moveWaypointNext(WP_NEXT_TARGET, outer_trajectory, current_waypoint_outer, OUTER_TRAJECTORY_LENGTH);
        //buildInnerTrajectory(WP_GOAL, WP_NEXT_TARGET);

      // Reaches inner waypoint
      } else if (mse_inner < 0.15){
        VERBOSE_PRINT("I am in else if mse inner\n"); 
        current_waypoint_inner_glob = moveWaypointNext(WP_NEXT_SUBTARGET, inner_trajectory_total, current_waypoint_inner_glob, INNER_TRAJECTORY_LENGTH*OUTER_TRAJECTORY_LENGTH);
      } else {
        VERBOSE_PRINT("I am in else\n"); 
        NavGotoWaypointHeading(WP_NEXT_SUBTARGET);
        //NavGotoWaypoint(WP_NEXT_SUBTARGET);
        moveWaypointForward(WP_GOAL, moveDistance);
      }

      break;
    case OBSTACLE_FOUND:
      // stop
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);

      // randomly select new search direction
      chooseRandomIncrementAvoidance();

      navigation_state = SEARCH_FOR_SAFE_HEADING;

      break;
    case SEARCH_FOR_SAFE_HEADING:
      increase_nav_heading(heading_increment);

      // make sure we have a couple of good readings before declaring the way safe
      if (obstacle_free_confidence >= 2){
        navigation_state = SAFE;
      }
      break;
    case OUT_OF_BOUNDS:
      increase_nav_heading(heading_increment);
      moveWaypointForward(WP_TRAJECTORY, 1.5f);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        // add offset to head back into arena
        increase_nav_heading(heading_increment);

        // reset safe counter
        obstacle_free_confidence = 0;

        // ensure direction is safe before continuing
        navigation_state = SEARCH_FOR_SAFE_HEADING;
      }
      break;
    default:
      break;
  }
  return;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading, declared in firmwares/rotorcraft/navigation.h
  // for performance reasons the navigation variables are stored and processed in Binary Fixed-Point format
  nav_heading = ANGLE_BFP_OF_REAL(new_heading);

  //VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  //VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,
  //POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
  //stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Sets the variable 'heading_increment' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    heading_increment = 5.f;
    //VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  } else {
    heading_increment = -5.f;
    //VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  }
  return false;
}
/*
 * Creates an 'inner' trajectory between the trajectory[i] and trajectory[i+1]
 */
uint8_t buildInnerTrajectory(uint8_t currentOuterTrajIndex){
  double x_diff = POS_FLOAT_OF_BFP(outer_trajectory[currentOuterTrajIndex+1].x) - POS_FLOAT_OF_BFP(outer_trajectory[currentOuterTrajIndex].x);
  double y_diff = POS_FLOAT_OF_BFP(outer_trajectory[currentOuterTrajIndex+1].y) - POS_FLOAT_OF_BFP(outer_trajectory[currentOuterTrajIndex].y);
  float increment_x =  x_diff/(INNER_TRAJECTORY_LENGTH);
  float increment_y = y_diff/(INNER_TRAJECTORY_LENGTH);
  for (int i = 0; i < INNER_TRAJECTORY_LENGTH; i++){
    // Create set of points between current position and the desired waypoint
    // Currently a straight line
    inner_trajectory[i].x = POS_BFP_OF_REAL((i+1)*increment_x + POS_FLOAT_OF_BFP(outer_trajectory[currentOuterTrajIndex].x));
    inner_trajectory[i].y = POS_BFP_OF_REAL((i+1)*increment_y + POS_FLOAT_OF_BFP(outer_trajectory[currentOuterTrajIndex].y));
    inner_trajectory_total[INNER_TRAJECTORY_LENGTH*currentOuterTrajIndex + i].x = inner_trajectory[i].x;
    inner_trajectory_total[INNER_TRAJECTORY_LENGTH*currentOuterTrajIndex + i].y = inner_trajectory[i].y;

    //VERBOSE_PRINT("Subtrajectory point added to list: (%f/%f) \n", POS_FLOAT_OF_BFP(inner_trajectory[i].x), POS_FLOAT_OF_BFP(inner_trajectory[i].y));
    VERBOSE_PRINT("Subtrajectory point added to the big list: (%f/%f) \n", POS_FLOAT_OF_BFP(inner_trajectory_total[INNER_TRAJECTORY_LENGTH*currentOuterTrajIndex + i].x), POS_FLOAT_OF_BFP(inner_trajectory_total[INNER_TRAJECTORY_LENGTH*currentOuterTrajIndex + i].y));
    //VERBOSE_PRINT("THE INDEX IS %i \n", current_waypoint_outer);
  }



  return false; 
}

/*
 * Builds the trajectory in the contour by random values
 */
uint8_t buildTrajectory(void) {
  /*
  double rx_list[3] = {1, -1, -1};
  double ry_list[3] = {1, 1, -1};
  for (int i = 0; i < OUTER_TRAJECTORY_LENGTH; i++) {
      // deviate from centroid of the cyberzoo by a bit
      // double r_x = (rand() % 500) - 200;
      // double r_y = (rand() % 500) - 200;
      // outer_trajectory[i].x = POS_BFP_OF_REAL(r_x/100);
      // outer_trajectory[i].y = POS_BFP_OF_REAL(r_y/100);
      outer_trajectory[i].x = POS_BFP_OF_REAL(rx_list[i]);
      outer_trajectory[i].y = POS_BFP_OF_REAL(ry_list[i]);
      VERBOSE_PRINT("Trajectory point added to list: (%f/%f) \n", POS_FLOAT_OF_BFP(outer_trajectory[i].x), POS_FLOAT_OF_BFP(outer_trajectory[i].y));
  }
  */
  outer_trajectory[0].x = POS_BFP_OF_REAL(0);
  outer_trajectory[0].y = POS_BFP_OF_REAL(0);
  VERBOSE_PRINT("Trajectory point added to list: (%f/%f) \n", POS_FLOAT_OF_BFP(outer_trajectory[0].x), POS_FLOAT_OF_BFP(outer_trajectory[0].y));
  outer_trajectory[1].x = POS_BFP_OF_REAL(2);
  outer_trajectory[1].y = POS_BFP_OF_REAL(2);
  VERBOSE_PRINT("Trajectory point added to list: (%f/%f) \n", POS_FLOAT_OF_BFP(outer_trajectory[1].x), POS_FLOAT_OF_BFP(outer_trajectory[1].y));
  outer_trajectory[2].x = POS_BFP_OF_REAL(2);
  outer_trajectory[2].y = POS_BFP_OF_REAL(-2);
  VERBOSE_PRINT("Trajectory point added to list: (%f/%f) \n", POS_FLOAT_OF_BFP(outer_trajectory[2].x), POS_FLOAT_OF_BFP(outer_trajectory[2].y));
  outer_trajectory[3].x = POS_BFP_OF_REAL(-2);
  outer_trajectory[3].y = POS_BFP_OF_REAL(0);
  VERBOSE_PRINT("Trajectory point added to list: (%f/%f) \n", POS_FLOAT_OF_BFP(outer_trajectory[3].x), POS_FLOAT_OF_BFP(outer_trajectory[3].y));
  outer_trajectory[4].x = POS_BFP_OF_REAL(0);
  outer_trajectory[4].y = POS_BFP_OF_REAL(0);
  VERBOSE_PRINT("Trajectory point added to list: (%f/%f) \n", POS_FLOAT_OF_BFP(outer_trajectory[4].x), POS_FLOAT_OF_BFP(outer_trajectory[4].y));

  waypoint_move_xy_i(WP_NEXT_TARGET, outer_trajectory[0].x, outer_trajectory[0].y);
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointNext(uint8_t waypoint, struct EnuCoor_i *trajectory, uint8_t index_current_waypoint, uint8_t trajectory_length)
{
  VERBOSE_PRINT("index_current_waypoint is %d \n", index_current_waypoint);
  VERBOSE_PRINT("Setting new Waypoint: (%f/%f) \n", POS_FLOAT_OF_BFP(trajectory[index_current_waypoint].x), POS_FLOAT_OF_BFP(trajectory[index_current_waypoint].y));
  
  moveWaypoint(waypoint, &trajectory[index_current_waypoint]);

  // update the waypoint index
  if (index_current_waypoint == trajectory_length - 1) {
    index_current_waypoint = 0;
  } 
  else {
    index_current_waypoint += 1;
  }
  return index_current_waypoint;
}

/*
 * Checks if WP_GOAL is very close to WP_TARGET, then change the waypoint
 */
double checkWaypointArrival(uint8_t waypoint_goal, uint8_t waypoint_target, double mseVar)
{
  double error_x = WaypointX(waypoint_goal) - WaypointX(waypoint_target);
  double error_y = WaypointY(waypoint_goal) - WaypointY(waypoint_target);
  mseVar = sqrt(pow(error_x,2)+pow(error_y,2));
  //VERBOSE_PRINT("Proximity to target (mse): %f, \n", mseVar);
  return mseVar;
}
