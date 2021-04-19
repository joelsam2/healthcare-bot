/*
 * gps.h
 *
 *  Created on: Apr 17, 2021
 *      Author: joels
 */


#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

typedef struct {
  float latitude;
  float longitude;
} gps_coordinates_t;

gps_coordinates_t current_gps_coordinates;


void gps_extract(void);
