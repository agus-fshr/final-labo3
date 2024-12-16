#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

typedef struct CircularBuffer
{
    long *buffer;
    size_t head;
    size_t tail;
    size_t size;
    size_t max_size;
    bool overflow;
} CircularBuffer_t;

CircularBuffer_t *circular_buffer_init(size_t max_size);
void circular_buffer_free(CircularBuffer_t *buffer);

size_t circular_buffer_used(CircularBuffer_t *buffer);
bool circular_buffer_empty(CircularBuffer_t *buffer);
bool circular_buffer_full(CircularBuffer_t *buffer);

void circular_buffer_push(CircularBuffer_t *buffer, long data);
long circular_buffer_pop(CircularBuffer_t *buffer);

#endif // CIRCULAR_BUFFER_H