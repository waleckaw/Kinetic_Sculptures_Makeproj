
#ifndef RING_BUF_H
#define RING_BUF_H

#include <stdint.h>
#include "CustomUserFunctions.h"

typedef struct {
    analogPosnArray vals[10];
    uint8_t numFilled;
    int size;
    int head;
    int tail;
} ring_buffer;

void printRingBuf(ring_buffer *buf);
void resetRingBuf(ring_buffer *buf);
void addToRingBuf(ring_buffer *buf, analogPosnArray val);
analogPosnArray getRingBufXRecent(ring_buffer *buf, int x);

#endif //RING_BUF_H