#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stm32f767xx.h>

enum servo_motor_state{ straight = 0, hard_left, soft_left, hard_right, soft_right};
extern enum servo_motor_state servo_motor_s;
