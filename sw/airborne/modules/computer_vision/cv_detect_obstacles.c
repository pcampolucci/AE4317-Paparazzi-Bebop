/*
 * Copyright (C) 2019 Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/cv_detect_object.h
 * Assumes the object consists of a continuous color and checks
 * if you are over the defined object or not
 */

// Own header
#include "modules/computer_vision/cv_detect_obstacles.h"
#include "modules/orange_avoider/orange_avoider.h"
#include "modules/computer_vision/cv.h"
#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/abi.h"
#include "std.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"
#include <time.h>

// The variables are expressed in a landscape picture
#define MASK_IT_VERBOSE TRUE
#define WIDTH_PIXELS 520  // size of the width of the image in terms of pixels
#define HEIGHT_PIXELS 240  // size of the height of the image in terms of pixels
#define SECTOR_HEIGHT 5  // height of a sector in pixels 
#define SECTOR_WIDTH 5  // width of a sector in pixels
#define AMOUNT_OF_ROWS 48  // amount of rows in terms of sectors
#define AMOUNT_OF_COLUMNS 104  // amount of columns in terms of sectors
#define MAX_AMOUNT_OF_OBSTACLES_BE_DETECTED 10  // 

#define SIZE_MASK_ARRAY 124800  // size is 520x240 = 124800
#define SIZE_BLACK_ARRAY 4992  // size is 48x104 = 4992
#define SIZE_OBST_ARRAY 30  // size is 10x3 = 30
#define SIZE_OUTPUT_ARRAY 30  // size is the same as obst array

#define FOV_HORIZONTAL 1.850049 // [rad], equivalent to 106 degrees
#define FOV_VERTICAL 0.912849  // [rad] equivalent to 52.3024 degrees 


#define COL_OBST 3
#define ROW_OUT 20
#define COL_OUT 3
#define N_OBST 4 //ALE CHANGED 
#define pi 3.1415
#define obstaclerows 5000


#define PRINT(string,...) fprintf(stderr, "[mask_it->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if MASK_IT_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;

#ifndef OBSTACLE_DETECTOR_FPS
#define OBSTACLE_DETECTOR_FPS 0 ///< Default FPS (zero means run at camera fps)
#endif

// Filter Settings

uint8_t cod_lum_min = 0;
uint8_t cod_lum_max = 0;
uint8_t cod_cb_min = 0;
uint8_t cod_cb_max = 0;
uint8_t cod_cr_min = 0;
uint8_t cod_cr_max = 0;

bool cod_draw = false;

float e = 2.71828;  // Euler's number

//New section, specify set dimensions

// define global variables
struct color_object_t {
  int32_t x_c;
  int32_t y_c;
  uint32_t color_count;
  bool updated;
};
struct color_object_t global_filters[2];

// struct obstacle_message_t {
//   float distance;
//   float left_heading;
//   float right_heading;
// };

// struct obstacle_message_t global_obstacle_msg;

int npix_dist_global = 0;  // Used to log the data 
int npix_headl_global = 0;  // Used to log the data 
int npix_headr_global = 0;  // Used to log the data 

struct ObstacleMsg global_obstacle_msg;

// Define the arrays globally
uint8_t masked_frame_f[SIZE_MASK_ARRAY];
uint8_t black_array[SIZE_BLACK_ARRAY];
uint16_t obstacle_array[SIZE_OBST_ARRAY]; 
float output_array[SIZE_OUTPUT_ARRAY];
//float output_array_real[ROW_OUT*ROW_OUT];


// Function declaration
uint32_t mask_it(struct image_t *img, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max, uint8_t *masked_frame2);

void getBlackArray(float threshold, uint8_t *maskie, uint8_t *blackie);
uint8_t getObstacles(uint8_t *black_array, uint16_t *obs_2);
void headingCalc(int l_sec, int r_sec, float *head_array);
double distCalc(int nsectors);
void distAndHead(uint8_t n_obstacles, uint16_t *obstacle_array, float *input_array);
int getRealValues(float *array); //ALE: Changed from void to uint8_t to int
static struct image_t *object_detector(struct image_t *img);

void obstacle_detector_init(void)
{
  memset(global_filters, 0, 2*sizeof(struct color_object_t));
  pthread_mutex_init(&mutex, NULL);
  //VERBOSE_PRINT("Obstacle detector initialized\n");     ALE: UNCOMMENT LATER!!!!!!!!!!!!!!!!!!!!

  // allocate memory for obstacle message (5 is the expected amount of obstacles to be sent in one go)
  global_obstacle_msg.obstacles = malloc(sizeof(struct Obstacle2) * 10);

  #ifdef OBSTACLE_DETECTOR_CAMERA
    #ifdef OBSTACLE_DETECTOR_LUM_MIN
      cod_lum_min = OBSTACLE_DETECTOR_LUM_MIN;
      cod_lum_max = OBSTACLE_DETECTOR_LUM_MAX;
      cod_cb_min = OBSTACLE_DETECTOR_CB_MIN;
      cod_cb_max = OBSTACLE_DETECTOR_CB_MAX;
      cod_cr_min = OBSTACLE_DETECTOR_CR_MIN;
      cod_cr_max = OBSTACLE_DETECTOR_CR_MAX;
    #endif
    #ifdef OBSTACLE_DETECTOR_DRAW
      cod_draw = OBSTACLE_DETECTOR_DRAW;
    #endif
    //cv_add_to_device(&OBSTACLE_DETECTOR_CAMERA, object_detector, OBSTACLE_DETECTOR_FPS);
    cv_add_to_device(&OBSTACLE_DETECTOR_CAMERA, object_detector, OBSTACLE_DETECTOR_FPS);
  #endif
}

void obstacle_detector_periodic(void)
{
  static struct color_object_t local_filters[2];
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, 2*sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);
  AbiSendMsgOBSTACLE_DETECTION(OBSTACLE_DETECTION_ID, &global_obstacle_msg);
}

/*
 * Function description
 */
static struct image_t *object_detector(struct image_t *img)
{
  uint8_t lum_min, lum_max;
  uint8_t cb_min, cb_max;
  uint8_t cr_min, cr_max;
  bool draw;
  lum_min = cod_lum_min;
  lum_max = cod_lum_max;
  cb_min = cod_cb_min;
  cb_max = cod_cb_max;
  cr_min = cod_cr_min;
  cr_max = cod_cr_max;
  draw = cod_draw;
  uint8_t n_obst = 0; 

  // Make sure the arrays are clean
  memset(masked_frame_f, 0, SIZE_MASK_ARRAY*sizeof(uint8_t));
  memset(masked_frame_f, 0, SIZE_BLACK_ARRAY*sizeof(uint8_t));
  memset(obstacle_array, 0, SIZE_OBST_ARRAY*sizeof(uint16_t));
  memset(output_array, 0, SIZE_OUTPUT_ARRAY*sizeof(float));
  
  //Filter and find centroid
  //clock_t maskit_1 = clock();
  uint32_t count = mask_it(img, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max, masked_frame_f);
  //clock_t maskit_2 = clock();
  //clock_t blackArray_1 = clock();
  getBlackArray(0.8, masked_frame_f, black_array);  // Make threshold slider
  //clock_t blackArray_2 = clock();
  //clock_t getObstacles_1 = clock();
  n_obst = getObstacles(black_array, obstacle_array);
  //clock_t getObstacles_2 = clock();
  // //clock_t distAndHead_1 = clock();
  // VERBOSE_PRINT("AMOUNT OF OBSTACLES IS EQUAL TOO %i \n", n_obst);
  // for (int i = 0; i<n_obst; i++){
  //    VERBOSE_PRINT("OBSTACLE OUTPUT %i with value : %i, %i, %i \n",i, obstacle_array[i*3],obstacle_array[i*3 +1],obstacle_array[i*3+2]);
  // }
  //VERBOSE_PRINT("AMOUNT OF OBSTACLES DETECTED IS %i", n_obst);
  distAndHead(n_obst, obstacle_array, output_array);

  // for (int i = 0; i<n_obst; i++){
  //    VERBOSE_PRINT("FINAL OUTPUT %i with value : %f, %f, %f \n",i, output_array[i*3],output_array[i*3 +1],output_array[i*3+2]);
  // }

  // n_obstReal = getRealValues(output_array_real,&process_variables);
  

  // //ALE DATA ANALYSIS
  // if (n_obstReal == 1)
  // {
  //   VERBOSE_PRINT("OBSTACLE DETECTOR OUTPUT %i, %i, %i, %i, %i, %i \n", obstacle_array[0], obstacle_array[1], obstacle_array[2], obstacle_array[3], obstacle_array[4], obstacle_array[5]);
  //   VERBOSE_PRINT("COMPARISON %f, %f, %i, %i, %i, %f, %f, %f, %f \n", process_variables.altitude, process_variables.pitch, npix_dist_global, npix_headl_global, npix_headr_global, output_array_real[0], output_array_real[1], output_array_real[2], output_array[0]);
  // }
  // //{0, 20, 30, 2, 15, 20}
  
  // // update the obstacle message 

  // // update message size
  global_obstacle_msg.size = n_obst;

  // start to override the previous message
  for (int i=0; i < n_obst; i++) {
    global_obstacle_msg.obstacles[i].distance = output_array[3*i];
    global_obstacle_msg.obstacles[i].left_heading = output_array[3*i+1];
    global_obstacle_msg.obstacles[i].right_heading = output_array[3*i+2];
  }

  // if we have more then expected, we can reallocate memory and expand the message
  // TODO

  pthread_mutex_lock(&mutex);
  global_filters[0].color_count =count;
  global_filters[0].x_c = 0 ;//x_c;
  global_filters[0].y_c = 0 ;//y_c;
  global_filters[0].updated = true;
  pthread_mutex_unlock(&mutex);

  return img;
}

/*
 * Function description
 */
void getBlackArray(float threshold, uint8_t *maskie, uint8_t *blackie){
    float sum_sec_line = 0; //changed from int
    float sum_sec_tot = 0; //changed from int
    float average = 0; 
    int countie = 0; 
    //Loops through the cols (so in the rotated pic from up to down)
    for (int i = 0; i < AMOUNT_OF_COLUMNS; i++){       
        //loops through the rows (so in the rotated pic from left to right)
        for (int g = 0; g < AMOUNT_OF_ROWS; g++){
            sum_sec_tot = 0; 
            average = 0;
            // Loops through the amount of cols of 1 sector (so in the rotated pic from up to down)            
            for (int j=0; j < SECTOR_WIDTH; j++){
                sum_sec_line = 0; 
                // Loops through the amount of rows of 1 sector (so in the rotated pic from left to right)
                for (int k=0; k < SECTOR_HEIGHT; k++){
                   //                                   |RIGHT START              |TRANSLATION WITHIN SECTOR
                   sum_sec_line = sum_sec_line + maskie[g*SECTOR_HEIGHT+HEIGHT_PIXELS*i*SECTOR_WIDTH+k+(j*HEIGHT_PIXELS)];
                }
                sum_sec_tot = sum_sec_tot + sum_sec_line;
            }

            // Get full sector
            average = sum_sec_tot/(SECTOR_HEIGHT*SECTOR_WIDTH);

            if (average<threshold){
                blackie[AMOUNT_OF_COLUMNS*(AMOUNT_OF_ROWS-1-g)+i] = 0;
            }
            else{
                blackie[AMOUNT_OF_COLUMNS*(AMOUNT_OF_ROWS-1-g)+i] = 1;
            }
            countie++; 
        } 
    }   
}

/*
 * Function description
 */
uint8_t getObstacles(uint8_t *black_array, uint16_t *obs_2){
  // Declare local variables
  int nsectrow = AMOUNT_OF_COLUMNS; 
  int nsectcol = AMOUNT_OF_ROWS;
  
  int obs_1[obstaclerows][3]  ={0};
  int obstacle_line[AMOUNT_OF_COLUMNS]; 
  int rewriter = 0,rewriter2  = 0;
  int p=0, pnew=0, count1=0, io=0;
  int minl=0, maxr=0, cr=0;
  int black_counter_row = 0;

  // Find all white to black/ black to white steps in the line
  for (int i=0; i<AMOUNT_OF_ROWS;i++){  // Loops through the amount of rows  
    bool begin_obstacle_detected = false;
    bool end_obstacle_detected = false; 
    int row_begin_obstacle = 0; 
    int column_begin_obstacle = 0; 
    int column_end_obstacle = 0;  
    int black_counter_col = 0; 
 
    for (int j=0; j<AMOUNT_OF_COLUMNS-1;j++){ // Loops through the amount of columns 
      p       = black_array[i*AMOUNT_OF_COLUMNS+j];  //check-value of current sector
      pnew    = black_array[i*AMOUNT_OF_COLUMNS+j+1];  //check-value of following sector
      if (p + pnew == 2){  // Detect black lines
        black_counter_col++; 
      }     
      else if (p - pnew < 0 && !begin_obstacle_detected){  //activate on white-to-black step, so begin of obstacle  
        row_begin_obstacle = i;  // sets row
        column_begin_obstacle = j;  // set index begin obstacle
        begin_obstacle_detected = true;
      }
      else if (begin_obstacle_detected && p - pnew > 0){ // detect black-to-white step, so end of obstacle
         column_end_obstacle = j+1;  // set index end obstacle
         begin_obstacle_detected = false;
         end_obstacle_detected = true; 
      }      
      if (end_obstacle_detected)
      {
        obs_1[count1][0]=row_begin_obstacle;  // add row number 
        obs_1[count1][1]=column_begin_obstacle;  // add column number begin
        obs_1[count1][2]=column_end_obstacle;  // add column number end
        count1 +=1;
        end_obstacle_detected = false; 
      }
    }
    if (black_counter_col >= AMOUNT_OF_COLUMNS-1){
      black_counter_row++; 
    }          
  }
  // VERBOSE_PRINT("LOOP 1: ROWS 0 %i, %i, %i \n", obs_1[0][0], obs_1[0][1], obs_1[0][2]);
  // VERBOSE_PRINT("LOOP 1: ROWS 1 %i, %i, %i \n", obs_1[1][0], obs_1[1][1], obs_1[1][2]);
  // VERBOSE_PRINT("LOOP 1: ROWS 2 %i, %i, %i \n", obs_1[2][0], obs_1[2][1], obs_1[2][2]);
  // VERBOSE_PRINT("LOOP 1: ROWS 3 %i, %i, %i \n", obs_1[3][0], obs_1[3][1], obs_1[3][2]);

  // Creates array which contains values which show the contour of the obstacles
  for (int i=0; i<AMOUNT_OF_COLUMNS; i++){  // i is the number of columns
    int current_max_value = 0; 
    int potential_max_value = 0; 
    bool part_of_obstacle = false; 
    for (int j = 0; j<count1; j++){ 
      if (i > obs_1[j][1] && i < obs_1[j][2] && obs_1[j][0] > black_counter_row){   // if in the interval
        potential_max_value = obs_1[j][0];  // sets the current height equal to the variable
        if (potential_max_value > current_max_value){  // checks if the potential max value is higher than the one already found
          current_max_value = potential_max_value; 
          part_of_obstacle = true; 
        }
      }
    }
    if (part_of_obstacle){
      obstacle_line[i] = current_max_value;
    }
    else{
      obstacle_line[i] = 0;
    }
  }

  // for (int i = 0; i<AMOUNT_OF_COLUMNS; i++){
  //   VERBOSE_PRINT("OBSTACLE LINE NUMBER %i with value : %i, \n",i, obstacle_line[i]);
  // }

  // Convert the contour to obstacles
  int total_amount_of_obst = 0; 
  bool start_blue = false;
  bool end_blue = false;
  int begin = 0;
  int end = 0;
  int dist = 0;
  for(int i=0;i<AMOUNT_OF_COLUMNS;i++){
    if(obstacle_line[i]!=0 && !start_blue){
      begin = i;
      dist = obstacle_line[i];
      start_blue = true;
    }
    if(start_blue && !end_blue){
      if(obstacle_line[i]>dist){
        dist = obstacle_line[i];
      }
      if(obstacle_line[i]==0){
        end = i;
        end_blue = true;
      }
    }
    if(end_blue){
      obs_2[total_amount_of_obst*3 + 0] = dist;
      obs_2[total_amount_of_obst*3 + 1] = begin;
      obs_2[total_amount_of_obst*3 + 2] = end;
      total_amount_of_obst += 1;
      start_blue = false;
      end_blue = false;
    }
  }
  return total_amount_of_obst; 
  // for (int i = 0; i<total_amount_of_obst; i++){
  //   VERBOSE_PRINT("OBSTACLE LINE NUMBER %i with value : %i, %i, %i \n",i, obs_2[i*3],obs_2[i*3 +1],obs_2[i*3+2]);
  // }
}


/*
 * Function description
 */
void headingCalc(int l_sec, int r_sec, float *head_array){  
    int l_pixels = (l_sec+1)*SECTOR_WIDTH;
    int r_pixels = r_sec * SECTOR_HEIGHT;
    float factor = 0.2023*pi/180;
    float heading_l =0; 
    float heading_r =0; 

    if (l_pixels < (WIDTH_PIXELS/2)) {
        heading_l = factor * (l_pixels - (WIDTH_PIXELS / 2));
        head_array[0] = heading_l;
    }
    else if (l_pixels >= (WIDTH_PIXELS/2)){
        heading_l = factor * (l_pixels - (WIDTH_PIXELS / 2));
        head_array[0] = heading_l;
    }
    if (r_pixels < (WIDTH_PIXELS/2)){
        heading_r = factor * (r_pixels - (WIDTH_PIXELS / 2));
        head_array[1] = heading_r;
    }
    else if (r_pixels >= (WIDTH_PIXELS/2)){
        heading_r = factor * (r_pixels - (WIDTH_PIXELS / 2));
        head_array[1] = heading_r;
    }
    npix_headl_global = l_pixels;
    npix_headr_global = r_pixels;
}

/*
 * Function description
 */
double distCalc(int nsectors){
    float altitude = GetPosAlt();
    int npixels = (AMOUNT_OF_ROWS - 1 - nsectors)*SECTOR_HEIGHT;
    double dist = 0; 
    double pitch = stateGetNedToBodyEulers_f()->theta; 
    double pitch_pix = (pitch/((FOV_VERTICAL)/57.2958))*SECTOR_HEIGHT*AMOUNT_OF_ROWS;
    double p00 =  -4.045e+05;
    double p10 = 156.6;
    double p01 = 2.232e+06;
    double p20 = -0.03603;
    double p11 = -574.2;
    double p02 = -4.105e+06;
    double p21 = 0.06744;
    double p12 = 526.3 ;
    double p03 = 2.517e+06;
    double x = 0;
    double y = 0;
    npixels = npixels + round(pitch_pix);
    npix_dist_global = npixels; 
    x = npixels;
    y = altitude;

    if (npixels <= 1){
        dist = 0;
    }
    // else if (npixels < 10000){
    //     dist = (1/(pow(0.45,(npixels/60))))-1 + altitude/tan((FOV_vertical/2)/57.2958);
    // }
    else if (npixels < 300){
        //dist = 0.01894959 - (-0.01608105/-0.02331507)*(1 - pow(e,(0.02331507*npixels))) + altitude/tan((FOV_vertical/2)/57.2958);
        //dist = p00 + p10*x + p01*y + p20*pow(x,2) + p11*x*y + p02*pow(y,2) + p21*pow(x,2)*y + p12*x*pow(y,2) + p03*pow(y,3) ;
        dist = altitude*2.2*(0.01894959 - (-0.01608105/-0.02331507)*(1 - pow(e,(0.02331507*npixels)))) + altitude/tan((FOV_VERTICAL/2));
        //VERBOSE_PRINT("THE DISTANCE IN HERE IS %lf ", dist);
    }
    if (dist > 10){
        dist = 0; 
        VERBOSE_PRINT("DIST IS LARGER THAN 10");
    }
    return dist; 
}

/*
 * Function description
 */
void distAndHead(uint8_t n_obstacles, uint16_t *obstacle_array, float *input_array){
    int input_dist = 0;
    int input_headl = 0; 
    int input_headr = 0; 
    float heading_array[2] = {};
    uint8_t obst_counter = 0; 
    for(int i=0; i < n_obstacles*3; i=i+3){
        // check for zeros 
        input_dist = obstacle_array[i];
        input_headl = obstacle_array[i+1];
        input_headr = obstacle_array[i+2]; 
        if (i > SIZE_OUTPUT_ARRAY){
          VERBOSE_PRINT("YOWWW OUTSIDE BOUNDS CHECK THIS SHIT");
          break; 
        }
        else{
          headingCalc(input_headl, input_headr, heading_array);
          input_array[i] = distCalc(input_dist);
          input_array[i+1] = heading_array[0];
          input_array[i+2] = heading_array[1]; 
        }
        obst_counter++; 
    }
} 


// int getRealValues(float *array, struct process_variables_t *var){ //Ale Changed
//   // Get the coordinates
//   //float pole_array_tot[N_OBST*2] = {-0.05, -1.9, 3.6, -0.15, 0.4, 0.3, -3.3, 0.2};  // Format is: {x_location_pole1, y_location_pole1, x_location_pole2, ...}
//   float pole_array_tot[N_OBST*2] = {-1.8, -3.4, -1.8, 0.5, 1.5, -2.5, 2.8, 2.5};  // Format is: {x_location_pole1, y_location_pole1, x_location_pole2, ...}
//   //float pole_array_tot[N_OBST*2] = {-1.8, 0.5};
//   float poles_in_view[N_OBST*2]; 
//   float drone_posx = GetPosX();  //NO LONGER FLIPPED! // Flipped because of logger //Possibly try stateGetPositionNed_i()->y
//   float drone_posy = GetPosY();  //NO LONGER FLIPPED! // Flipped because of logger //Possibly try stateGetPositionNed_i()->x
//   float drone_posz = GetPosAlt(); 
//   float drone_yaw = stateGetNedToBodyEulers_f()->psi; 
//   float heading_old = 0; 
//   float heading_sign = 0;
//   float heading = 0;
//   float FOV_hor = var->FOV_horizontal; 
//   int count = 0; //changed to int
//   float width_pole = 0.2; 
//   float distance = 0; 
//   float angle = 0;

//   float ax = 0;
//   float ay = 0;
//   float bx = 0;
//   float by = 0;
  
//   // Get the poles in view
//   for (int i=0; i<N_OBST*2; i=i+2){
//     //Replaced heading function
//     //                pole_x             drone_x      pole_y              drone_y           yaw
//     heading_sign  = atan((pole_array_tot[i]-drone_posx)/(pole_array_tot[i+1]-drone_posy)) + (- drone_yaw); //this is used to calcuated sign of the angle (heading2/fabs(heading2)) gives 1 or -1
//     distance = sqrt(pow((drone_posx-pole_array_tot[i]),2) + pow((drone_posy-pole_array_tot[i+1]),2));  // Real Distance
//     //VERBOSE_PRINT("IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII is equal TOOOOOOOO %i", i);
//     //Formula for angle between two vectors
//     //nominator = (distance*sin(drone_yaw)-drone_posx)*(pole_array_tot[i]-drone_posx)+(distance*cos(drone_yaw)-drone_posy)*(pole_array_tot[i+1]-drone_posy)
//     //denominator = sqrt(pow((distance*sin(drone_yaw)-drone_posx),2)+pow((distance*cos(drone_yaw)-drone_posy),2))*sqrt(pow((pole_array_tot[i]-drone_posx),2)+pow((pole_array_tot[i+1]-drone_posy),2))
//     heading_old = acos(((distance*sin(drone_yaw)-drone_posx)*(pole_array_tot[i]-drone_posx)+(distance*cos(drone_yaw)-drone_posy)*(pole_array_tot[i+1]-drone_posy))/(sqrt(pow((distance*sin(drone_yaw)-drone_posx),2)+pow((distance*cos(drone_yaw)-drone_posy),2))*sqrt(pow((pole_array_tot[i]-drone_posx),2)+pow((pole_array_tot[i+1]-drone_posy),2))));
//     //         ^ this bracket stuff gives negative if to the left of the drone
//     ax = (distance*sin(drone_yaw)-drone_posx);
//     by = (pole_array_tot[i+1]-drone_posy);
//     ay =(distance*cos(drone_yaw)-drone_posy);
//     bx = (pole_array_tot[i]-drone_posx);
//     angle = -atan2( ax*by - ay*bx, ax*bx + ay*by );
    
//     if (heading<(pi/2)){
//       heading = heading_old*(heading_sign/fabs(heading_sign));
//     }
//     else{
//       heading = -heading_old*(heading_sign/fabs(heading_sign));
//     }

//     //VERBOSE_PRINT("HEADING NEW %i, %f \n", i, (angle*180/pi));
//     //VERBOSE_PRINT("HEADING %i, %f \n",i, (heading*180/pi));
//     //VERBOSE_PRINT("IF STATEMENT %i, %f \n", (i+1), (fabs(heading)+atan((width_pole/2)/distance)));
//     if (fabs(angle)+atan((width_pole/2)/distance) < FOV_hor/2){
//       poles_in_view[count] = pole_array_tot[i];
//       poles_in_view[count] = pole_array_tot[i+1];
//       array[count] = distance;                                   // Real Distance
//       array[count+1] = angle - atan((width_pole/2)/distance);  // Real Heading left
//       array[count+2] = angle + atan((width_pole/2)/distance);  // Real Heading right
//       count = count + 3; 
//       //VERBOSE_PRINT("WE ARE INNNNNNNNNNNNNNNNNNNN \n");
//     }
//   }
//   return (count/3); //removed /3
// }


/*
 * Function description
 */
uint32_t mask_it(struct image_t *img, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max, uint8_t *masked_frame2)
{
  uint32_t cnt = 0;
  uint32_t len = img->h*img->w;
  //uint32_t masked_frame[len] ;
  uint32_t tot_x = 0;
  uint32_t tot_y = 0;
  uint8_t *buffer = img->buf;
  //int mysize = img->buf_size;//sizeof(img->buf) / sizeof(uint8_t);
  // VERBOSE_PRINT("size of buffer = %d\n",mysize);
  // VERBOSE_PRINT("Image height = %d\n",img->h);
  // VERBOSE_PRINT("Image width = %d\n",img->w);
  // Go through all the pixels
  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = 0; x < img->w; x++) {
      // Check if the color is inside the specified values
      uint8_t *yp, *up, *vp;
      if (x % 2 == 0) {
        // Even x
        up = &buffer[y * 2 * img->w + 2 * x];      // U
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
        vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
        //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
      } else {
        // Uneven x
        up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
        //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
        vp = &buffer[y * 2 * img->w + 2 * x];      // V
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
      }
      if ( (*yp >= lum_min) && (*yp <= lum_max) &&
           (*up >= cb_min ) && (*up <= cb_max ) &&
           (*vp >= cr_min ) && (*vp <= cr_max )) {
        cnt ++;
        tot_x += x;
        tot_y += y;
        int idx = y * img->w +x;
        //masked_frame[idx] = 1;
        masked_frame2[idx] = 0;  // Changed to zero (white)
        if (draw){
          *yp = 255;  // make pixel brighter in image
          *up = 128;
          *vp = 128;
        }
              
      }else {
           int idx = y * img->w +x;
           //masked_frame[idx] = 0;
           masked_frame2[idx] = 1;  // Changed to one (black)
          if (draw){
            *yp = 0;
            *up = 128;
            *vp = 128;
          }
      }
      //fprintf("%s", masked_frame[y * img->w +x]);
    }
  }

  // VERBOSE_PRINT("cnt = %d\n",cnt);
  uint32_t summy = 0;
  for (int ica = len-1; ica >= 0; ica--) {
    summy += masked_frame2[ica];
  }
  //VERBOSE_PRINT("%d/%d/%d\n", masked_frame[50000], masked_frame[100000], masked_frame[len]);
  //double percentage;
  //double percentage2;
  //percentage = 100.0*summy/len;
  //percentage = 100.0*cnt/len;
  //percentage2 = 100.0*summy/len;
  // VERBOSE_PRINT("summy = %d\n",summy);
  // VERBOSE_PRINT("len = %d\n",len);
  // VERBOSE_PRINT("Positive mask = %f \n", percentage);
  // VERBOSE_PRINT("Positive mask 2 = %f \n", percentage2);
  //printf("%s\n ",masked_frame);
  //fflush(stdout);
  // VERBOSE_PRINT("MASK HAS BEEN CALCULATED!\n");
  // VERBOSE_PRINT("\n"); 
  //return *masked_frame;
  return cnt;
}

