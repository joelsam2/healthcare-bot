/*
 * servo_motor.c
 *
 *  Created on: Mar 31, 2021
 *      Author: joel
 */


#include "servo_motor.h"

enum servo_motor_state servo_motor_s;


// Control servo Motor PWM
void control_servo_motor(enum servo_motor_state x)
{

	switch(x)
	{

	case straight:

		break;

	case hard_left:

		break;

	case soft_left:

		break;

	case hard_right:

		break;

	case soft_right:

		break;

	default:
		// straight

		break;
	}

}
