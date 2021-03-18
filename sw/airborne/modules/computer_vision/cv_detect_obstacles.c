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

// define global variables
struct color_object_t {
  int32_t x_c;
  int32_t y_c;
  uint32_t color_count;
  bool updated;
};
struct color_object_t global_filters[2];

/*
 * object_detector
 * @param img - input image to process
 * @param filter - which detection filter to process
 * @return img
 */




/*
static struct image_t *object_detector(struct image_t *img)
{
  Put mask here
  
  VERBOSE_PRINT("Image received!?\n");
  return img;
}
*/

uint32_t mask_it(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max)
{
  uint32_t cnt = 0;
  int len = img->h*img->w;
  int masked_frame[len] ;
  uint32_t tot_x = 0;
  uint32_t tot_y = 0;
  uint8_t *buffer = img->buf;

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
        masked_frame[y * img->w +x] = 1;
        if (draw){
          *yp = 255;  // make pixel brighter in image
        }
        
      }
      //fprintf("%s", masked_frame[y * img->w +x]);
      //VERBOSE_PRINT("cazzo %s",  masked_frame[y * img->w +x]* );
    }
  }
  
  double len2 = img->h*img->w;
  int summy = 0;
  VERBOSE_PRINT("summy = %d\n",summy);
  VERBOSE_PRINT("summy = %zu\n",cnt);
  for (int i = 0; i < len; i++) {
    //printf("%s", masked_frame[i]);
    //VERBOSE_PRINT("%d", masked_frame[i]);
    //VERBOSE_PRINT("culo\n");
    summy += masked_frame[i];
  }
  VERBOSE_PRINT("%d/%d/%d", masked_frame[0], masked_frame[7], masked_frame[100]);
  double percentage;
  //percentage = 100.0*summy/len;
  percentage = 100.0*cnt/len;
  VERBOSE_PRINT("summy = %d\n",summy);
  VERBOSE_PRINT("len = %d\n",len);
  VERBOSE_PRINT("len2 = %f\n",len2);
  VERBOSE_PRINT("Positive mask = %f \n", percentage);
  //printf("%s\n ",masked_frame);
  //fflush(stdout); 
  VERBOSE_PRINT("MASK HAS BEEN CALCULATED!\n");
  return masked_frame;
}



void obstacle_detector_init(void)
{
  memset(global_filters, 0, 2*sizeof(struct color_object_t));
  pthread_mutex_init(&mutex, NULL);
  VERBOSE_PRINT("Obstacle detector initialized\n");

  #ifdef OBSTACLE_DETECTOR_CAMERA
    //cv_add_to_device(&OBSTACLE_DETECTOR_CAMERA, object_detector, OBSTACLE_DETECTOR_FPS);
    cv_add_to_device(&OBSTACLE_DETECTOR_CAMERA, mask_it, OBSTACLE_DETECTOR_FPS);
  #endif
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
