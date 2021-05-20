/*
 * circular_queue.h
 *
 *  Created on: Apr 9, 2021
 *      Author: joels
 */

#define QUEUE_SIZE 10000
#include <stdint.h>
#include <stdbool.h>

int write_index;
int read_index;
char queue[QUEUE_SIZE];

void enqueue(char x);
char dequeue(void);
void queue_init();
bool isEmpty();
int queue_count();


/*******************   QUEUE 2 ******************/

int write_index2;
int read_index2;
char queue2[QUEUE_SIZE];

void enqueue2(char x);
char dequeue2(void);
void queue_init2();
bool isEmpty2();
int queue_count2();

