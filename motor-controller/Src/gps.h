/*
 * gps.h
 *
 *  Created on: Apr 17, 2021
 *      Author: joels
 * certain code & algorithms were referenced from previous project done in course CMPE243.
 */


#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

typedef struct {
  float latitude;
  float longitude;
} gps_coordinates_t;

extern gps_coordinates_t current_gps_coordinates; // gps values updates in this struct.


void process_latitude_longitude(gps_coordinates_t *t, float latitude, char lat_direction, float longitude, char long_direction);
void gps_extract(void);
