/*
 * dc_motor.h
 *
 *  Created on: Mar 31, 2021
 *      Author: joels
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stm32f767xx.h>

enum dc_motor_state{ forward = 0, reverse, stop, h_left, h_right, s_left, s_right};
extern enum dc_motor_state dc_motor_s;
volatile int ultrasonic[5];
void get_ultrasonic_value();
void decide_motor_state();
void motor_init();



void control_dc_motor(enum dc_motor_state x);
