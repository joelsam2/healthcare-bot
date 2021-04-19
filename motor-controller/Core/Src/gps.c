/*
 * gps.c
 *
 *  Created on: Apr 17, 2021
 *      Author: joels
 */
#include "gps.h"
#include "circular_queue.h"



bool check_line() {
  bool has_line = false;
  int read_index = read_index2;
  int count = queue_count2();

  for (int chars = 0; chars < count; chars++) {
    if ( queue2[read_index]== '\n') {
      has_line = true;
      break;
    }
    read_index = (1 + read_index) % QUEUE_SIZE;
  }
  return has_line;
}

double convert_to_degree(double value) {
  double degree_value = (double)((int)(value / 100));
  double decimal_value = (value - degree_value * 100) / 60;
  return degree_value + decimal_value;
}

static void process_latitude_longitude(gps_coordinates_t *t, float latitude,
                                                     char lat_direction, float longitude, char long_direction) {
  t->longitude = convert_to_degree(longitude);
  t->latitude = convert_to_degree(latitude);
  if (long_direction == 'W' || long_direction == 'w') {
    t->longitude *= -1;
  }
  if (lat_direction == 'S' || lat_direction == 's') {
    t->latitude *= -1;
  }

  char buff[30];
  sprintf(buff, "%d,%c,%d,%c\r\n", (int)t->latitude, lat_direction, (int)t->longitude, long_direction);
  print_stuff(buff, strlen(buff));
}


bool extract_gpgga_string(char *buff, int buff_size) {
  bool return_value = false;
  int i = 0;
  if (check_line()) {
    for (int i = 0; i < buff_size; i++) {
      if (queue_count2() > 0) {

    	  	  	char t = dequeue2();

    	  		while(t != '$' && isEmpty2() == false)
    	  			t = dequeue2();

    	  		while(t != '\n' && isEmpty2() == false)
    	  		{
    	  			buff[i] = t;
    	  			t = dequeue2();
    	  			i++;
    	  		}

    	  		//buff[i] = '\n';
    	  		//buff[++i] = '\0';
    	  		buff[i] = '\0';
    	  		i = 0;

    	  return_value = true;
    	  break;
      } else {
        return_value = false;
        break;
      }
    }
  }
  return return_value;
}


//$GPGGA,203940.000,3720.4195,N,12153.9105,W,2,06,1.72,104.1,M,-25.6,M,0000,0000*5D


void gps_extract(void) {
  static char gps_value[200];
  static double latitude;
  static double longitude;
  static char lat_direction;
  static char long_direction;
  int valid_gps;
  static int token_count = 1;

  if (extract_gpgga_string(gps_value, 200)) {
    if (gps_value[0] == '$' && gps_value[1] == 'G' && gps_value[2] == 'P' && gps_value[3] == 'G' && gps_value[4] == 'G' && gps_value[5] == 'A') {
       
        char * token = strtok(gps_value, ",");

        while( token != NULL ) {

          if(token_count >= 8)
          {
            token_count = 1;
            break;
          }
          
          if(token_count >=3 && token_count <=7)
          {
            char s[10];
            switch (token_count)
            {
            case 3:
                latitude = atof(token);
                /* sprintf(s, "%d", (int)latitude);
                print_stuff(s, strlen(s));
                int x = (latitude - ((int)latitude))*100;
                sprintf(s, "%d", (int)x);
                print_stuff(s, strlen(s)); */
              break;
            case 4:
                lat_direction = token[0];
                // print_stuff(&lat_direction, 1);
              break;              
            case 5:
                longitude = atof(token);
                /* sprintf(s, "%d", (int)longitude);
                print_stuff(s, strlen(s)); */
              break;
            case 6:
                long_direction = token[0];
                // print_stuff(&long_direction, 1);
              break;
            case 7:
                valid_gps = atoi(token);
                /* sprintf(s, "%d", valid_gps);
                print_stuff(s, strlen(s)); */
              break;                                       
            default:
              break;
            }
          }

          //print_stuff(token, strlen(token));
          token_count++;
          token = strtok(NULL, ",");
		  }

      process_latitude_longitude(&current_gps_coordinates, latitude, lat_direction, longitude, long_direction);

    }
  }
}



