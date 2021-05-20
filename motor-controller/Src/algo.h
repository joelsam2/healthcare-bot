/*
 * algo.h
 *
 *  Created on: May 9, 2021
 *      Author: joels
 * certain code & algorithms were referenced from previous project done in course CMPE243.
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "compass.h"
#include "gps.h"
#include "dc_motor.h"

/* certain code & algorithms were referenced from projects done in course CMPE243.
 */

#define PI 3.14159f
#define MIN_DISTANCE 5
static const float max_allowed_distance_in_meters = 2000.0;
static const float minimum_checkpoint_arrival_distance = 3.0;

extern gps_coordinates_t destination_gps_c;

typedef struct{
	char lat_dir;
	char long_dir;
}direction_t;


void core_algo();
void algo_navigation();
void test_init();
int next_check_point(gps_coordinates_t current, gps_coordinates_t final_destination);

static int8_t TOLERANCE_DIRECTION_POSITIVE = 20;
static int8_t TOLERANCE_DIRECTION_NEGATIVE = -20;
static const uint8_t MINIMUM_DISTACE_RANGE = 5;

