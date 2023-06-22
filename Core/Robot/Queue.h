/*
 * Queue.h
 *
 *  Created on: Apr 19, 2021
 *      Author: Troll
 */

#ifndef ROBOT_QUEUE_H_
#define ROBOT_QUEUE_H_

#define QUEUE_MAX 100

typedef struct{
	float array[QUEUE_MAX];
	int front;
	int rear;
	int itemCount;
}QUEUE_Queue;



float QUEUE_peek(QUEUE_Queue *q);

int QUEUE_isEmpty(QUEUE_Queue *q);

int QUEUE_isFull(QUEUE_Queue *q);

int QUEUE_size(QUEUE_Queue *q);

void QUEUE_insert(float data, QUEUE_Queue *q);

float QUEUE_removeData(QUEUE_Queue *q);


#endif /* ROBOT_QUEUE_H_ */
