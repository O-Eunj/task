#include "queue.h"

void init_queue(Queue* q) {
    q->head = q->tail = q->size = 0;
}

bool empty_queue(Queue* q) {
    return (q->size == 0);
}

bool full_queue(Queue* q) {
    return (q->size == MAX_QUEUE);
}

void push_queue(Queue* q, int value) {
    if (!full_queue(q)) {
        q->data[q->tail] = value;
        q->tail = (q->tail + 1) % MAX_QUEUE;
        q->size++;
    }
}

bool pop_queue(Queue* q, int* value) {
    if (empty_queue(q)) return false;
    *value = q->data[q->head];
    q->head = (q->head + 1) % MAX_QUEUE;
    q->size--;
    return true;
}