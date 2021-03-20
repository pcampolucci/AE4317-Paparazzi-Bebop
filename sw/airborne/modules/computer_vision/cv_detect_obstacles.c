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

// Function
uint32_t mask_it(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max);



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


  int32_t x_c, y_c;

  // Filter and find centroid
  uint32_t count = mask_it(img, &x_c, &y_c, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max);
  // VERBOSE_PRINT("Color count %d: %u, threshold %u, x_c %d, y_c %d\n", camera, object_count, count_threshold, x_c, y_c);
  // VERBOSE_PRINT("centroid %d: (%d, %d) r: %4.2f a: %4.2f\n", camera, x_c, y_c,
  //       hypotf(x_c, y_c) / hypotf(img->w * 0.5, img->h * 0.5), RadOfDeg(atan2f(y_c, x_c)));
  VERBOSE_PRINT("Test if mask frame works = %d", count );
  pthread_mutex_lock(&mutex);
  global_filters[0].color_count =count;
  global_filters[0].x_c = 0 ;//x_c;
  global_filters[0].y_c = 0 ;//y_c;
  global_filters[0].updated = true;
  pthread_mutex_unlock(&mutex);

  return img;
}

// struct image_t *object_detector1(struct image_t *img);
// struct image_t *object_detector1(struct image_t *img)
// {
//   return object_detector(img);
// }

//cancelled call for object dector 1 and 2 as we do not have a switch anymore




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




uint32_t mask_it(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max)
{
  uint32_t cnt = 0;
  uint32_t len = img->h*img->w;
  uint32_t masked_frame[len] ;
  uint32_t tot_x = 0;
  uint32_t tot_y = 0;
  uint8_t *buffer = img->buf;
  int mysize = img->buf_size;//sizeof(img->buf) / sizeof(uint8_t);
  VERBOSE_PRINT("size of buffer = %d\n",mysize);
  VERBOSE_PRINT("Image height = %d\n",img->h);
  VERBOSE_PRINT("Image width = %d\n",img->w);
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
        masked_frame[idx] = 1;
        if (draw){
          *yp = 255;  // make pixel brighter in image
          *up = 128;
          *vp = 128;
        }
              
      }else {
           int idx = y * img->w +x;
           masked_frame[idx] = 0;
          if (draw){
            *yp = 0;
            *up = 128;
            *vp = 128;
          }
      }
      //fprintf("%s", masked_frame[y * img->w +x]);
    }
  }

  VERBOSE_PRINT("cnt = %d\n",cnt);
  uint32_t summy = 0;
  for (int ica = len-1; ica >= 0; ica--) {
    summy += masked_frame[ica];
  }
  //VERBOSE_PRINT("%d/%d/%d\n", masked_frame[50000], masked_frame[100000], masked_frame[len]);
  double percentage;
  double percentage2;
  //percentage = 100.0*summy/len;
  percentage = 100.0*cnt/len;
  percentage2 = 100.0*summy/len;
  VERBOSE_PRINT("summy = %d\n",summy);
  VERBOSE_PRINT("len = %d\n",len);
  VERBOSE_PRINT("Positive mask = %f \n", percentage);
  VERBOSE_PRINT("Positive mask 2 = %f \n", percentage2);
  //printf("%s\n ",masked_frame);
  //fflush(stdout);
  VERBOSE_PRINT("MASK HAS BEEN CALCULATED!\n");
  VERBOSE_PRINT("\n"); 
  //return *masked_frame;
  return cnt;
}




void obstacle_detector_periodic(void)
{
  static struct color_object_t local_filters[2];
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, 2*sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);
  VERBOSE_PRINT("Sending Visual detection\n");
  AbiSendMsgVISUAL_DETECTION(OBSTACLE_DETECTION_ID, local_filters[0].x_c, local_filters[0].y_c, 0, 0, 4, 0);
}
