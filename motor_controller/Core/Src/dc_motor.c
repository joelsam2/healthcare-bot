#include "dc_motor.h"

enum dc_motor_state dc_motor_s = forward;

// Control DC Motor PWM
void control_dc_motor(enum dc_motor_state x)
{

	switch(x)
	{

	case forward:
		break;

	case reverse:

			break;

	case stop:

			break;

	default:
		// stop

		break;
	}

}
