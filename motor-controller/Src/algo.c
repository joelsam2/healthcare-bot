/*
 * algo.c
 *
 *  Created on: May 9, 2021
 *      Author: joels
 */

#include "algo.h"
#include "compass.h"
#include "cmsis_os.h"
#include "stm32f7xx_hal.h"
#include <stm32f767xx.h>
#include <stm32f7xx_hal_i2c.h>
#include <task.h>
#include <stdbool.h>


int check_point_index;
extern gps_coordinates_t final_destination_gps_c;
static uint8_t next_check_point_index = 0;
gps_coordinates_t check_pts[2] = {
    {3720.4350, 12153.9490}, // 1
    {3720.4226, 12153.9503} // 2 -> Final destination
};

direction_t  dir_pts[2] = {
		{'N', 'W'},
		{'N', 'W'}  // 5 -> Final destination
};

bool isReached[2] = {false};

extern UART_HandleTypeDef huart3;
extern void motor_forward();
extern void motor_reverse();
extern void motor_stop();
extern void servo_hard_left();
extern void servo_hard_right();
extern void servo_soft_left();
extern void servo_soft_right();
extern void servo_forward();

void convert_to_radian(gps_coordinates_t *current_c, gps_coordinates_t *destination_c)
{
  current_c->latitude = (current_c->latitude * PI) / 180;
  current_c->longitude = (-current_c->longitude * PI) / 180;
  destination_c->latitude = (destination_c->latitude * PI) / 180;
  destination_c->longitude = (-destination_c->longitude * PI) / 180;
}

float round_upto_2_decimal(float bearing) {
  uint64_t value = (uint64_t)(bearing * 100 + 0.5f);
  return (float)value / 100;
}


float calculate_destination_angle(gps_coordinates_t curr_c, gps_coordinates_t dest_c) {
  gps_coordinates_t current_c = curr_c;
  gps_coordinates_t destination_c = dest_c;
  float bearing = 0.0f;
  float longitude_difference = 0.0f;

    convert_to_radian(&current_c, &destination_c);
    longitude_difference = (current_c.longitude - (destination_c.longitude));

    bearing = atan2(
        (sin(longitude_difference) * cos(destination_c.latitude)),
        ((cos(current_c.latitude) * sin(destination_c.latitude)) -
         (sin(current_c.latitude) * cos(destination_c.latitude) *
          cos(longitude_difference))));
    bearing = fmodf(((bearing * 180) / PI) + 360, 360);
  
  bearing = round_upto_2_decimal(bearing);


  static int count = 0;
  char temp[50];

  count++;

  if(count > 200){
	int temp_int = bearing;
	float temp_float = bearing - temp_int;
	int temp_float_to_int = temp_float * 100;

	sprintf(temp, "B:%d.%d\r\n", temp_int,temp_float_to_int );
	HAL_UART_Transmit(&huart3, temp, strlen(temp), HAL_MAX_DELAY);
	count = 0;
  }


  return bearing;
}

float calculate_destination_distance(gps_coordinates_t curr_c, gps_coordinates_t dest_c) {
    gps_coordinates_t current_c = curr_c;
    gps_coordinates_t destination_c = dest_c;
    float distance = 0.0f;

    convert_to_radian(&current_c, &destination_c);

    float latitude_diff = destination_c.latitude - current_c.latitude;
    float longitude_diff = destination_c.longitude - current_c.longitude;
    float a = pow(sin(latitude_diff / 2), 2) + cos(current_c.latitude) * cos(destination_c.latitude) * pow(sin(longitude_diff / 2), 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    distance = 6371 * 1000 * c;

    distance = round_upto_2_decimal(distance);


    static int count = 0;
      char temp[50];

      count++;

      if(count > 200){
    	int temp_int = distance;
    	float temp_float = distance - temp_int;
    	int temp_float_to_int = temp_float * 100;

    	sprintf(temp, "D:%d.%d\r\n", temp_int,temp_float_to_int );
    	HAL_UART_Transmit(&huart3, temp, strlen(temp), HAL_MAX_DELAY);
    	count = 0;
      }

    return distance;
}

bool algo_is_destination_reached(gps_coordinates_t current_gps_coordinates, gps_coordinates_t destination_gps_c)
{
    if(calculate_destination_distance(current_gps_coordinates, destination_gps_c) < MIN_DISTANCE)
        return true;

    return false;
}


float algo_compute_deflection(float *compass_current_heading, float *compass_destination_heading ) {
  float deflection = *compass_destination_heading - *compass_current_heading;

  if (deflection > 180) {
    deflection -= 360;
  } else if (deflection < -180) {
    deflection += 360;
  }
  return deflection;
}

static void algo_get_gps_heading_direction(float compass_current_heading, float compass_destination_heading ) {
  static float final_def = 0.0f;
  final_def = algo_compute_deflection(&compass_current_heading, &compass_destination_heading);

  static int count = 0;
        char temp[50];

        count++;

        if(count > 200){
      	int temp_int = final_def;
      	float temp_float = final_def - temp_int;
      	int temp_float_to_int = temp_float * 100;

      	sprintf(temp, "DF:%d.%d\r\n", temp_int,temp_float_to_int );
      	HAL_UART_Transmit(&huart3, temp, strlen(temp), HAL_MAX_DELAY);
      	count = 0;
        }

  if (final_def > TOLERANCE_DIRECTION_POSITIVE) {
	  (final_def < 45) ? servo_soft_right() : servo_hard_right();
  } else if (final_def < TOLERANCE_DIRECTION_NEGATIVE) {
	 (final_def > -45) ? servo_soft_left() : servo_hard_left();
  } else {
    servo_forward();
  }
}


#if 0
void algo_navigation()
{
	// current gps value and current compass gps value are constantly updated in background by different FreeRTOS tasks.
    gps_coordinates_t current_gps_c = current_gps_coordinates;
    float compass_current_heading = compass_degree;


    /**************** CHECKPOINT CODE  **************/

    // check distance of each checkpoint from current coordinates.
    // Go to nearest checkpoint and mark as reached.
    // init function should update distnace to 0 and is reached to NO.


    //calculate_min_distance -> fills the distance array with min distance.
    //check_min -> function checks min distance from all of them and marks it as reached.


    // TO-DO -> need to fix core_alog -> isdestination reached function...!!!
    // need to change logic again -> because once we calculate the min distance again anad again we are calucalting the min distnace when 
    // the car is moving to the first checkpoint. which is not right. so nned to change 
    // destination_gps co-ordinate logic and entire logic. 

    calculate_distance();
    int min_index = check_min_distance();

    //process_latitude_longitude(&destination_gps_c, check_pts[min_index].latitude, dir_pts[min_index].lat_dir, check_pts[min_index].longitude,  dir_pts[min_index].long_dir );
    process_latitude_longitude(&destination_gps_c, check_pts[4].latitude, dir_pts[4].lat_dir, check_pts[4].longitude,  dir_pts[4].long_dir );

    // to calculate destination HEADING angle we use current lat,long (from gps module) and final destination lat,long (static).
    float compass_destination_heading = calculate_destination_angle(current_gps_c, destination_gps_c);
    algo_get_gps_heading_direction(compass_current_heading, compass_destination_heading);
}
#endif


#if 1
void algo_navigation()
{
  	// current gps value and current compass gps value are constantly updated in background by different FreeRTOS tasks.
    gps_coordinates_t current_gps_c = current_gps_coordinates;
    float compass_current_heading = compass_degree;


    //to calculate destination HEADING angle we use current lat,long (from gps module) and final destination lat,long (static).
    float compass_destination_heading = calculate_destination_angle(current_gps_c, destination_gps_c);
    algo_get_gps_heading_direction(compass_current_heading, compass_destination_heading);
}
#endif

/*
void test_init()
{
  float destination_lat = 3720.4503f;
  float destination_long = 12153.9412f;
  char destination_lat_dir = 'N';
  char destination_long_dir = 'W';
  process_latitude_longitude(&destination_gps_c, destination_lat, destination_lat_dir, destination_long, destination_long_dir);
}
*/


int next_check_point(gps_coordinates_t current, gps_coordinates_t final_destination) {

    float distance_from_origin_to_checkpoint = 0;
    float distance_from_checkpoint_to_destination = 0;
    float nearest_checkpoint_distance = max_allowed_distance_in_meters;
    float nearest_destination_distance = max_allowed_distance_in_meters;
    bool check_point_found = false;

    float distance_from_origin_to_destination = calculate_destination_distance(current, final_destination);
    uint8_t size_of_checkpoints = 2;


    for (int i = 0; i < size_of_checkpoints; i++) {
      distance_from_origin_to_checkpoint = calculate_destination_distance(current, check_pts[i]);
      distance_from_checkpoint_to_destination = calculate_destination_distance(check_pts[i], final_destination);


      char temp[10];
      int temp_int = distance_from_origin_to_checkpoint;
            	float temp_float = distance_from_origin_to_checkpoint - temp_int;
            	int temp_float_to_int = temp_float * 100;

            	sprintf(temp, "%d,OC:%d.%d\r\n", i, temp_int,temp_float_to_int );
            	HAL_UART_Transmit(&huart3, temp, strlen(temp), HAL_MAX_DELAY);

      if ((distance_from_origin_to_checkpoint < nearest_checkpoint_distance) && (distance_from_origin_to_checkpoint > minimum_checkpoint_arrival_distance) && // add code to check if checkpoint selected is 5 which is destination in our case. if yes detination reached, stop car.
          (distance_from_checkpoint_to_destination <= distance_from_origin_to_destination) ) {
        nearest_checkpoint_distance = distance_from_origin_to_checkpoint;
        nearest_destination_distance = distance_from_checkpoint_to_destination;



        next_check_point_index = i;
        check_point_found = true;
      }

    }

    //update destination_gps_c with checkpoint
    char c[10];
    sprintf(c, "CHECK_POINT_P:%d\r\n", next_check_point_index);
    HAL_UART_Transmit(&huart3, c, strlen(c), 30);


    process_latitude_longitude(&destination_gps_c, check_pts[next_check_point_index].latitude, dir_pts[next_check_point_index].lat_dir, check_pts[next_check_point_index].longitude,  dir_pts[next_check_point_index].long_dir );
    return next_check_point_index;
}

void core_algo()
{
	static int count = 0;
    if (algo_is_destination_reached(current_gps_coordinates, destination_gps_c)) {
    	servo_forward();
    	motor_stop();
     if(next_check_point_index == 1)
      {
        servo_forward();
        motor_stop();
        HAL_UART_Transmit(&huart3, "FINAL DEST!!!!\r\n", 15, 200);
      }else
      {
        isReached[next_check_point_index] = true;
        HAL_Delay(2000);
        check_point_index = next_check_point(current_gps_coordinates, final_destination_gps_c);
        motor_forward();
      }

    } else {
      //print_ultrasonic_values();
      if (obstacle_avoidance_required()) {
        decide_motor_state();
      } else {
        algo_navigation();
      }
    }
}
