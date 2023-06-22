#include "Queue.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>



float QUEUE_peek(QUEUE_Queue *q) {
   return q->array[q->front];
}

int QUEUE_isEmpty(QUEUE_Queue *q) {
   return q->itemCount == 0;
}

int QUEUE_isFull(QUEUE_Queue *q) {
   return q->itemCount == QUEUE_MAX;
}

int QUEUE_size(QUEUE_Queue *q) {
   return q->itemCount;
}

void QUEUE_insert(float data, QUEUE_Queue *q) {

   if(!QUEUE_isFull(q)) {

      if(q->rear == QUEUE_MAX-1) {
         q->rear = -1;
      }

      q->array[++q->rear] = data;
      q->itemCount++;
   }
}

float QUEUE_removeData(QUEUE_Queue *q) {
   int data = q->array[q->front++];

   if(q->front == QUEUE_MAX) {
      q->front = 0;
   }

   q->itemCount--;
   return data;
}
