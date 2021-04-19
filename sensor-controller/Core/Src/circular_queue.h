/*
 * circular_queue.h
 *
 *  Created on: Apr 8, 2021
 *      Author: joels
 */

#define QUEUE_SIZE 450
#include <stdint.h>
#include <stdbool.h>

int write_index;
int read_index;
char queue[QUEUE_SIZE];

void enqueue(char x);
char dequeue(void);
void queue_init();
bool isEmpty();
