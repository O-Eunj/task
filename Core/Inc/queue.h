#ifndef QUEUE_H
#define QUEUE_H

#include <stdbool.h>

#define MAX_QUEUE 5

typedef struct {
    int data[MAX_QUEUE];
    int tail;
    int head;
    int size;
} Queue;

void init_queue(Queue* q);
bool empty_queue(Queue* q);
bool full_queue(Queue* q);
void push_queue(Queue* q, int value);
bool pop_queue(Queue* q, int* value);

#endif