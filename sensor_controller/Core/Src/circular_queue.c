#include "circular_queue.h"


void queue_init()
{
	write_index = -1;
	read_index = -1;
}

bool isEmpty()
{
	if(write_index == -1 && read_index == -1)
		return true;
	
	return false;
}


void enqueue(char x)
{
	if((write_index + 1)%QUEUE_SIZE == read_index)
		return;
	else if(write_index == -1 && read_index == -1)
		write_index = read_index = 0;
	else
		write_index = (write_index+1)%QUEUE_SIZE;

	queue[write_index] = x;
}


char dequeue(void)
{
	if(write_index == -1 && read_index == -1)
		return '-1'; //error code
	else if(write_index == read_index)
	{
		char temp = queue[read_index];
		write_index = read_index = -1;
		return temp;
	}
	else
	{
		char t = queue[read_index];
		read_index = (read_index + 1) % QUEUE_SIZE;
		return t;
	}
}
