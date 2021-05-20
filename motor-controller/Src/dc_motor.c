#include "dc_motor.h"
#include "circular_queue.h"
#include "stm32f7xx_hal.h"

enum dc_motor_state dc_motor_s = forward;
enum dc_motor_state servo_motor_s = forward;
char s[20];

extern void print_stuff();
extern void motor_forward();
extern void motor_reverse();
extern void motor_stop();
extern void servo_hard_left();
extern void servo_hard_right();
extern void servo_soft_left();
extern void servo_soft_right();
extern void servo_forward();
extern UART_HandleTypeDef huart3;

static int count1 = 0;
static int count2 = 0;
static int count3 = 0;
static int count4 = 0;


static int avg1 = 0;
static int avg2 = 0;
static int avg3 = 0;
static int avg4 = 0;


volatile static int v1;
volatile static int v2;
volatile static int v3;
volatile static int v4;


void fill_ultrasonic_value(char* p, int len)
{

	if(len < 3)
		return;

	int x = p[0] - '0';
	char g[20];
	int value = atoi(&p[2]);


	switch(x)
	{
		case 1:

			if(count1 < 10)
			{
				avg1 = avg1 + value;
				count1++;
			}else{
				ultrasonic[1] = avg1/10;
				avg1 = 0;
				count1 = 0;
			}

			//sprintf(g, "1:%d\r\n", value);
			//print_stuff(g, strlen(g));
			break;

		case 2:
			if(count2 < 10)
			{
				avg2 = avg2 + value;
				count2++;
			}else{
				ultrasonic[2] = avg2/10;
				avg2 = 0;
				count2 = 0;
			}
			//sprintf(g, "2:%d\r\n", value);
			//print_stuff(g, strlen(g));
			break;

		case 3:
			if(count3 < 10)
			{
				avg3 = avg3 + value;
				count3++;
			}else{
				ultrasonic[3] = avg3/10;
				avg3 = 0;
				count3 = 0;
			}
			//sprintf(g, "3:%d\r\n", value);
			//print_stuff(g, strlen(g));
			break;

		case 4:
			if(count4 < 10)
			{
				avg4 = avg4 + value;
				count4++;
			}else{
				ultrasonic[4] = avg4/10;
				avg4 = 0;
				count4 = 0;
			}
			//sprintf(g, "4:%d\r\n", value);
			//print_stuff(g, strlen(g));
			break;
	}
}

void motor_init()
{
	dc_motor_s = forward;
	servo_forward();
	motor_stop();
}


void get_ultrasonic_value()
{
	static int i = 0;
	char t;
	char local_buff[20];

	if(isEmpty() == false)
	{
		t = dequeue();

		while(t != 'S' && isEmpty() == false)
			t = dequeue();

		while(t != 'E' && isEmpty() == false)
		{
			t = dequeue();
			local_buff[i] = t;
			i++;
		}

		local_buff[i] = '\0';
		i = 0;
		fill_ultrasonic_value(local_buff, strlen(local_buff));

		v1 = ultrasonic[1];
		v2 = ultrasonic[2];
		v3 = ultrasonic[3];
		v4 = ultrasonic[4];

	}
}


void print_ultrasonic_values()
{

	 char b[100];
	 static int count = 0;

		if(count > 5){
		  sprintf(b, "1:%d - 2:%d - 3:%d - 4:%d\r\n",v1, v2, v3, v4);
		  HAL_UART_Transmit(&huart3, b, strlen(b), 50);
		  count = 0;
	    }

		count++;
}


// check ultrasonic values and assign control signals.
void decide_motor_state()
{

	char b[50];

	//sprintf(b, "1:%d - 2:%d - 3:%d - 4:%d\r\n",v1, v2, v3, v4);
	//print_stuff(b, strlen(b));

	switch(dc_motor_s)
		{
			case forward:
				if(v1 <= 30)
				{
					dc_motor_s = stop;
					motor_stop();
				}
				if(v1 > 30 && v2 > 45 && v3 > 45)
				{
					servo_forward();
					motor_forward();
					dc_motor_s = forward;

				}
				if(v1 > 30 && (v2 <= 45 || v3 <=45))
				{
					if(v2 <= 45)
					{
						servo_hard_left();
					}else if(v3 <= 45)
					{
						servo_hard_right();
					}
				}


				break;

			case stop:

				if(v1 <= 30 && v4 > 20)
				{
					dc_motor_s = reverse;
					motor_reverse();

					if(v2 > 45)
						servo_hard_left();
					else
						servo_hard_right();
				}
				else if(v1 > 30)
				{
					dc_motor_s = forward;
					motor_forward();
				}else if(v1 <= 30 && v4 <= 20)
				{
					dc_motor_s = stop;
					motor_stop();
				}

				break;

			case reverse:

				if(v4 <= 10)
				{
					dc_motor_s = stop;
					motor_stop();
				}
				else
					dc_motor_s = stop;
				break;

			default:
				dc_motor_s = stop;
				motor_stop();
				break;
		}
}

/*
void control_servo_motor(enum dc_motor_state x)
{
	switch(x)
	{
		case h_left:
			motor_hard_left();
			break;

		case h_right:
			motor_hard_right();
			break;

		case s_left:
			motor_soft_left();
			break;

		case s_right:
			motor_soft_right();
			break;
	}
}
*/

// Control DC Motor PWM
void control_dc_motor(enum dc_motor_state x)
{

	switch(x)
	{

	case forward:
		motor_forward();
		break;

	case stop:
		motor_stop();
		break;

	case reverse:
		break;

	default:
		motor_stop();
		break;
	}

}

bool obstacle_avoidance_required()
{

	if(v1 < 30 || v3 <= 45 || v2 <= 45 || v4 < 10)
		return true;
	
	return false;
}
