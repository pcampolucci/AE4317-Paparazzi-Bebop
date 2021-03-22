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
#include "modules/computer_vision/cv.h"
#include "subsystems/abi.h"
#include "std.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"



#define MASK_IT_VERBOSE TRUE
#define ROW_OBST 100
#define COL_OBST 3
#define ROW_OUT 20
#define COL_OUT 3

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

// define global variables
struct color_object_t {
  int32_t x_c;
  int32_t y_c;
  uint32_t color_count;
  bool updated;
};
struct color_object_t global_filters[2];

struct process_variables_t {
  float FOV_horizontal;
  float FOV_vertical; 
  int npixh; 
  int npixv; 
  int height_pic;
  int width_pic;
  int nsectcol;
  int nsectrow;
};

struct process_variables_t process_variables; 

struct obstacle_message_t {
  float distance;
  float left_heading;
  float right_heading;
};

struct obstacle_message_t global_obstacle_msg;


int altitude = 1.09100; //meters
float e = 2.71828;  // the constant



// Function declaration
uint32_t mask_it(struct image_t *img, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max, uint8_t *masked_frame2);

void getBlackArray(float threshold, uint8_t *maskie, uint8_t *blackie, struct process_variables_t *var);
void getObstacles(uint8_t *black_array, uint16_t *obs_2, struct process_variables_t *var);
void headingCalc(int l_sec, int r_sec, float *head_array, struct process_variables_t *var);
float distCalc(int nsectors, struct process_variables_t *var);
void distAndHead(uint16_t *obstacle_array, float *input_array, struct process_variables_t *var);
static struct image_t *object_detector(struct image_t *img);

void obstacle_detector_init(void)
{
  memset(global_filters, 0, 2*sizeof(struct color_object_t));
  pthread_mutex_init(&mutex, NULL);
  VERBOSE_PRINT("Obstacle detector initialized\n");

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
  AbiSendMsgOBSTACLE_DETECTION(OBSTACLE_DETECTION_ID, global_obstacle_msg.distance,
                                                      global_obstacle_msg.left_heading, 
                                                      global_obstacle_msg.right_heading);
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
  //return img;
  
  
  // Define constants
  uint32_t img_w = img->w;
  uint32_t img_h = img->h; 

  uint32_t len_pic = img_w*img_h;
  // int32_t x_c, y_c;

  // Populate struct
  process_variables.height_pic = img_w;
  process_variables.width_pic = img_h; 
  process_variables.npixh = 5;
  process_variables.npixv = 5;
  process_variables.nsectcol = img_h/(process_variables.npixv);
  process_variables.nsectrow = img_w/(process_variables.npixh);
  process_variables.FOV_horizontal = 110;
  process_variables.FOV_vertical = 52.3024;

  // Define arrays needed for the processing
  uint8_t masked_frame_f[len_pic];
  memset(masked_frame_f, 0, len_pic*sizeof(uint8_t));
  uint8_t black_array[len_pic];
  uint16_t obstacle_array[ROW_OBST*COL_OBST]; 
  memset(obstacle_array, 0, ROW_OBST*COL_OBST*sizeof(uint16_t));
  float output_array[ROW_OUT*ROW_OUT];
  memset(output_array, 0, ROW_OUT*COL_OUT*sizeof(float));

  //VERBOSE_PRINT("check me bitch 1= %d\n", masked_frame_f[50000]);

  // Filter and find centroid
  uint32_t count = mask_it(img, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max, masked_frame_f);
  // VERBOSE_PRINT("Color count %d: %u, threshold %u, x_c %d, y_c %d\n", camera, object_count, count_threshold, x_c, y_c);
  // VERBOSE_PRINT("centroid %d: (%d, %d) r: %4.2f a: %4.2f\n", camera, x_c, y_c,
  //       hypotf(x_c, y_c) / hypotf(img->w * 0.5, img->h * 0.5), RadOfDeg(atan2f(y_c, x_c)));
  //VERBOSE_PRINT("check me bitch 2= %d\n", masked_frame_f[50000]);
  //VERBOSE_PRINT("Test if mask frame works = %d", count );
  // VERBOSE_PRINT("IM GOING INTO BLACKIE \n");
  // for (int iii=0;iii<520;iii++){
  //   for(int iv=0;iv<240;iv++){
  //      VERBOSE_PRINT("%i ",masked_frame_f[iii*240 + iv]);
  //    }
  //    VERBOSE_PRINT("\n");
  //  }
  getBlackArray(0.8, masked_frame_f, black_array, &process_variables);  // Make threshold slider
  
  // for (int iii=0;iii<process_variables.nsectrow;iii++){
  //   for(int iv=0;iv<process_variables.nsectcol;iv++){
  //     VERBOSE_PRINT("%i ",black_array[iii*process_variables.nsectcol + iv]);
  //   }
  //   VERBOSE_PRINT("\n");
  // }
  getObstacles(black_array, obstacle_array, &process_variables);
  VERBOSE_PRINT("OBSTACLE 1 IS %i, %i, %i \n", obstacle_array[0], obstacle_array[1], obstacle_array[2]);  // Entry 0: distance, Entry 1: headingleft, Entry 2: headingright
  VERBOSE_PRINT("OBSTACLE 2 IS %i, %i, %i \n", obstacle_array[3], obstacle_array[4], obstacle_array[5]);
  //VERBOSE_PRINT("OBSTACLES IS %i, %i, %i \n", obstacle_array[0][0], obstacle_array[0][1], obstacle_array[0][2]);
  distAndHead(obstacle_array, output_array, &process_variables);
  VERBOSE_PRINT("OUTPUT 1 IS %f, %f, %f \n", output_array[0], output_array[1], output_array[2]);  // Entry 0: distance, Entry 1: headingleft, Entry 2: headingright
  VERBOSE_PRINT("OUTPUT 2 IS %f, %f, %f \n", output_array[3], output_array[4], output_array[5]);
  // update the obstacle message 
  global_obstacle_msg.distance = output_array[0];
  global_obstacle_msg.left_heading = output_array[1];
  global_obstacle_msg.right_heading = output_array[2];

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
void getBlackArray(float threshold, uint8_t *maskie, uint8_t *blackie, struct process_variables_t *var){
    int nsectrow = var->nsectrow; 
    int nsectcol = var->nsectcol;
    int npixh = var->npixh;
    int npixv = var->npixv; 
    int height_pic = var->height_pic;
    //int width_pic = var->width_pic;
    float sum_sec_line = 0; //changed from int
    float sum_sec_tot = 0; //changed from int
    float average = 0; 
    int countie = 0; 
    //Loops through the cols (so in the rotated pic from up to down)
    for (int i = 0; i < nsectcol; i++){       
        //loops through the rows (so in the rotated pic from left to right)
        for (int g = 0; g < nsectrow; g++){
            sum_sec_tot = 0; 
            average = 0;
            // Loops through the amount of cols of 1 sector (so in the rotated pic from up to down)            
            for (int j=0; j < npixh; j++){
                sum_sec_line = 0; 
                // Loops through the amount of rows of 1 sector (so in the rotated pic from left to right)
                for (int k=0; k < npixv; k++){
                   //                                   |RIGHT START              |TRANSLATION WITHIN SECTOR
                   sum_sec_line = sum_sec_line + maskie[g*npixv+height_pic*i*npixh+k+(j*height_pic)];
                }
                sum_sec_tot = sum_sec_tot + sum_sec_line;
            }

            // Get full sector
            average = sum_sec_tot/(npixv*npixh);
            //VERBOSE_PRINT("AVERAGE IS %f \n", average);
            //VERBOSE_PRINT("COUNTIE IS %i \n", countie);
            //VERBOSE_PRINT("%f \n", threshold);
            if (average<threshold){
                blackie[nsectcol*(nsectrow-1-g)+i] = 0;
                //VERBOSE_PRINT("HALLO I AM ZERO ");
                //VERBOSE_PRINT("%i \n", nsectcol*(nsectrow-1-g)+i);
                //VERBOSE_PRINT("BLACKIE VALUE %i \n",blackie[nsectcol*(nsectrow-1-g)+i] );
            }
            else{
                blackie[nsectcol*(nsectrow-1-g)+i] = 1;
                //VERBOSE_PRINT("HALLO I AM ONEEEE ");
                //VERBOSE_PRINT("%i \n", nsectcol*(nsectrow-1-g)+i);
                //VERBOSE_PRINT("BLACKIE VALUE %i \n",blackie[nsectcol*(nsectrow-1-g)+i] );
            }
            countie++; 
        } 
    }   
}

/*
 * Function description
 */
void getObstacles(uint8_t *black_array, uint16_t *obs_2, struct process_variables_t *var)
{
  int nsectrow = var->nsectcol; 
  int nsectcol = var->nsectrow;
  // int npixh = var->npixh;
  // int npixv = var->npixv; 
  // int height_pic = var->height_pic;
  // int width_pic = var->width_pic;
  // int obs_counter         = 0;
  int obs_1[50][3]        ={0};
  int rewriter =0,rewriter2  = 0;
  int p,pnew,count1       =0;
  int minl,maxr,cr        = 0;

  for(int i=0;i<50;i++)
    {   
    obs_2[i*3+0]=0  ;
    obs_2[i*3+1]=0  ;
    obs_2[i*3+2]=0  ;
    }  
 
  for (int i=0; i<nsectcol;i++)
  { 
      for (int j=0; j<nsectrow;j++)
      {     
        p       = black_array[i*nsectrow+j];        //check-value of current sector
        pnew    = black_array[i*nsectrow+j+1];      //cehck-value of following sector
        if (p - pnew < 0 && j<nsectrow-1){          //activate on white-to-black step, accounting for counter  

          obs_1[count1][1]=j+1;
          obs_1[count1][0]=i;
          count1 +=1;   
        } 
        if (p - pnew == 1 && j<nsectrow-1 && obs_1[count1-1][1]<=j+1 && obs_1[count1-1][1]!=0){
          obs_1[count1-1][2]=j+1; 
        }
        // VERBOSE_PRINT("Adding detection \n");  
      }
      
  }

  count1 = 0;

  // for(int i=0;i<15;i++)
  //   {   
  //       printf("obstacle %d %d %d \n",obs_1[i][0],obs_1[i][1],obs_1[i][2]);
  //   }  


  for(int i=0;i<50;i++)
  {
  
      if (obs_1[i][0]==0 || obs_1[i][1]==0 || obs_1[i][2]==0){
        obs_1[i][0]=0;
        obs_1[i][1]=0;
        obs_1[i][2]=0;
      }
      else{                      
        obs_1[rewriter][0] =  obs_1[i][0];
        obs_1[rewriter][1] =  obs_1[i][1];
        obs_1[rewriter][2] =  obs_1[i][2];
        obs_1[i][0]=0;
        obs_1[i][1]=0;
        obs_1[i][2]=0;
        rewriter +=1;
      }
  } 

  rewriter = 0;  
  for(int i=0;i<50;i++)
  {            
      if(obs_1[i][0]!=0)
      {   
          
          for(int j=0+i;j<50;j++)
          {
              if(obs_1[j][0]>obs_1[i][0])
              {
                  if (obs_1[j][1] >= obs_1[i][1]  && obs_1[j][1] <= obs_1[i][2]){
                      if(obs_1[j][2] > obs_1[i][2]){
                        maxr = obs_1[j][2];
                        minl = obs_1[i][1];
                      }
                      else{
                        maxr = obs_1[i][2];
                        minl = obs_1[i][1];
                      }
                      cr = obs_1[j][0];   
                  }
                  if (obs_1[j][2] >= obs_1[i][1]  && obs_1[j][2] <= obs_1[i][2]){
                      if(obs_1[j][1] < obs_1[i][1]){
                        maxr = obs_1[i][2];
                        minl = obs_1[j][1];
                      }
                      else{
                        maxr = obs_1[i][2];
                        minl = obs_1[i][1];
                      }   
                      cr = obs_1[j][0];   
                  }            
              }    
          }
          if(cr==0){}
          else{
              if(obs_2[(rewriter2-1)*3+0]== cr){
                  if(obs_2[(rewriter2-1)*3+1]==minl && obs_2[(rewriter2-1)*3+2]==maxr){
                    cr=0;
                    minl=0;
                    maxr=0; 
                  }
                  if(obs_2[(rewriter2-1)*3+1]<minl || obs_2[(rewriter2-1)*3+2]>maxr){
                    cr=0;
                    minl=0;
                    maxr=0; 
                  }
              }
              else{
                obs_2[rewriter2*3+0]=cr;
                obs_2[rewriter2*3+1]=minl;
                obs_2[rewriter2*3+2]=maxr;
                rewriter2 +=1;
                cr=0;
                minl=0;
                maxr=0; 
              }                
          }
      }
  }
  rewriter2 = 0;

  // this one is causing problems
  // for(int i=0;i<15;i++)
  //   {   
  //       printf("obstacle %d %d %d \n",obs_2[i*3+0],obs_2[i*3+1],obs_2[i*3+2]);
  //   }  
}

/*
 * Function description
 */
void headingCalc(int l_sec, int r_sec, float *head_array, struct process_variables_t *var){  
    // int nsectrow = var->nsectrow; 
    // int nsectcol = var->nsectcol;
    int npixh = var->npixh;
    // int npixv = var->npixv; 
    // int height_pic = var->height_pic;
    int width_pic = var->width_pic;
    //convert sectors into pixels
    int l_pixels = (l_sec+1)*npixh;
    int r_pixels = r_sec * npixh;
    float factor = 0.2023;
    float heading_l =0; 
    float heading_r =0; 

    if (l_pixels < (width_pic/2)) {
        heading_l = -factor * (l_pixels - (width_pic / 2));
        head_array[0] = heading_l;
    }
    else if (l_pixels >= (width_pic/2)){
        heading_l = factor * (l_pixels - (width_pic / 2));
        head_array[0] = heading_l;
    }
    if (r_pixels < (width_pic/2)){
        heading_r = -factor * (r_pixels - (width_pic / 2));
        head_array[1] = heading_r;
    }
    else if (r_pixels >= (width_pic/2)){
        heading_r = factor * (r_pixels - (width_pic / 2));
        head_array[1] = heading_r;
    }
    //return 0; //QUESTION: why return 0??
}

/*
 * Function description
 */
float distCalc(int nsectors, struct process_variables_t *var){
    int nsectrow = var->nsectrow; 
    // int nsectcol = var->nsectcol;
    // int npixh = var->npixh;
    int npixv = var->npixv; 
    // int height_pic = var->height_pic;
    // int width_pic = var->width_pic;
    float FOV_vertical = var->FOV_vertical; 
    int npixels = (nsectrow - 1 - nsectors)*npixv;
    float dist = 0; 
    if (npixels <= 1){
        dist = 0;
    }
    else if (npixels < 10){
        dist = (1/(pow(0.45,(npixels/43))))-1 + altitude/tan((FOV_vertical/2)/57.2958);
    }
    else if (npixels < 200){
        dist = 0.01894959 - (-0.01608105/-0.02331507)*(1 - pow(e,(0.02331507*npixels))) + altitude/tan((FOV_vertical/2)/57.2958);
    }
    else{
        dist = 1000; 
    }
    return dist; 
}

/*
 * Function description
 */
void distAndHead(uint16_t *obstacle_array, float *input_array, struct process_variables_t *var){
    // {{0, 0, 0}, {43, 29, 31}, {47, 8, 11}, {0, 0, 0}}
    //int nsectrow = var->nsectrow; 
    //int nsectcol = var->nsectcol;
    //int npixh = var->npixh;
    //int npixv = var->npixv; 
    //int height_pic = var->height_pic;
    //int width_pic = var->width_pic;
    int input_dist = 0;
    int input_headl = 0; 
    int input_headr = 0; 
    float heading_array[2] = {};

    for(int i=0; i < COL_OBST*ROW_OBST; i=i+3){
        // check for zeros 
        input_dist = obstacle_array[i];
        input_headl = obstacle_array[i+1];
        input_headr = obstacle_array[i+2]; 
        int sum = input_dist + input_headl + input_headr;
        if (sum == 0){
            break; 
        }
        else{
            headingCalc(input_headl, input_headr, heading_array, var);
            input_array[i] = distCalc(input_dist, var);
            input_array[i+1] = heading_array[0];
            input_array[i+2] = heading_array[1]; 
            // VERBOSE_PRINT("output 1: %f \n", input_array[i]);
            // VERBOSE_PRINT("output 2: %f \n", input_array[i+1]);
            // VERBOSE_PRINT("output 3: %f \n", input_array[i+2]);
        }
    }
}

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

