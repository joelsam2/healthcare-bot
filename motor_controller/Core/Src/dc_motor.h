/*
 * dc_motor.h
 *
 *  Created on: Mar 31, 2021
 *      Author: joels
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stm32f767xx.h>

enum dc_motor_state{ forward = 0, reverse, stop};
extern enum dc_motor_state dc_motor_s;


void control_dc_motor(enum dc_motor_state x);
