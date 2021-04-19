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

int queue_count()
{
	int count = (QUEUE_SIZE+write_index - read_index)%QUEUE_SIZE + 1;
	return count;
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
		return 0;
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



/*******************   QUEUE 2 ******************/

void queue_init2()
{
	write_index2 = -1;
	read_index2 = -1;
}

bool isEmpty2()
{
	if(write_index2 == -1 && read_index2 == -1)
		return true;

	return false;
}

int queue_count2()
{
	int count = (QUEUE_SIZE + write_index2 - read_index2) % QUEUE_SIZE + 1;
	return count;
}


void enqueue2(char x)
{
	if((write_index2 + 1)%QUEUE_SIZE == read_index2)
		return;
	else if(write_index2 == -1 && read_index2 == -1)
		write_index2 = read_index2 = 0;
	else
		write_index2 = (write_index2+1)%QUEUE_SIZE;

	queue2[write_index2] = x;
}


char dequeue2(void)
{
	if(write_index2 == -1 && read_index2 == -1)
		return 0;
	else if(write_index2 == read_index2)
	{
		char temp = queue2[read_index2];
		write_index2 = read_index2 = -1;
		return temp;
	}
	else
	{
		char t = queue2[read_index2];
		read_index2 = (read_index2 + 1) % QUEUE_SIZE;
		return t;
	}
}
