#ifndef PROJC_QUEUE_H
#define PROJC_QUEUE_H

// C program for array implementation of queue
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

// A structure to represent a queue
struct Queue {
    int front, rear, size;
    unsigned capacity;
    int *array;
};

// function to create a queue of given capacity.
// It initializes size of queue as 0
struct Queue *createQueue(unsigned capacity) {
    struct Queue *queue = (struct Queue *) malloc(sizeof(struct Queue));
    queue->capacity = capacity;
    queue->front = queue->size = 0;
    queue->rear = capacity - 1;  // This is important, see the enqueue
    queue->array = (int *) malloc(queue->capacity * sizeof(int));
    return queue;
}

// Queue is full when size becomes equal to the capacity
int isFull(struct Queue *queue) { return (queue->size == queue->capacity); }

// Queue is empty when size is 0
int isEmpty(struct Queue *queue) { return (queue->size == 0); }

// Function to add an item to the queue.
// It changes rear and size
void enqueue(struct Queue *queue, int item) {
    if (isFull(queue))
        return;
    queue->rear = (queue->rear + 1) % queue->capacity;
    queue->array[queue->rear] = item;
    queue->size = queue->size + 1;
}

// Function to remove an item from queue.
// It changes front and size
int dequeue(struct Queue *queue) {
    if (isEmpty(queue))
        return INT_MIN;
    int item = queue->array[queue->front];
    queue->front = (queue->front + 1) % queue->capacity;
    queue->size = queue->size - 1;
    return item;
}

// Function to get front of queue
int front(struct Queue *queue) {
    if (isEmpty(queue))
        return INT_MIN;
    return queue->array[queue->front];
}

// Function to get rear of queue
int rear(struct Queue *queue) {
    if (isEmpty(queue))
        return INT_MIN;
    return queue->array[queue->rear];
}

void get_queue_maxmin(struct Queue *queue, int *pMax, int *pMin) {
    *pMax = INT_MIN;
    *pMin = INT_MAX;

    for (int i = queue->front; i <= queue->front + queue->size; i = (i + 1) % queue->capacity){
        if (queue->array[i] >= *pMax) *pMax = queue->array[i];
        if (queue->array[i] <= *pMin) *pMin = queue->array[i];
    }
}

void release_queue(struct Queue *queue){
    free(queue->array);
    free(queue);
}

void print_queue(struct Queue *queue){
    int item = 0;
    printf("[");
    if (!isEmpty(queue)){
        item = dequeue(queue);
        printf("%d", item);
    }
    while (!isEmpty(queue)){
        item = dequeue(queue);
        printf(", %d", item);
    }
    printf("]\n");
}

void save_queue_to_file(struct Queue *queue, FILE *fp){
    int item = 0;
//    printf("[");
    fprintf(fp, "[");
    if (!isEmpty(queue)){
        item = dequeue(queue);
//        printf("%d", item);
        fprintf(fp, "%d", item);
    }
    while (!isEmpty(queue)){
        item = dequeue(queue);
//        printf(", %d", item);
        fprintf(fp, ", %d", item);
    }
//    printf("]\n");
    fprintf(fp, "]\n");
}
#endif //PROJC_QUEUE_H
