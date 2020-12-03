#include "ring_buf.h"
#include <stdint.h>
#include <stdio.h>
#include "CustomUserFunctions.h"

void resetRingBuf(ring_buffer *buf) {
    buf->numFilled = 0;
    buf->head = 0;
    buf->tail = 0;
}

 void addToRingBuf(ring_buffer *buf, analogPosnArray val) {
    if (buf->numFilled == buf->size) {
        buf->tail++;
        buf->tail = buf->tail % buf->size;
    } else {
        buf->numFilled++;
        if (buf->numFilled == 1) {
            buf->vals[0] = val;
            return;
        }
    }

    buf->head++;
    buf->head = buf->head % buf->size;

    buf->vals[buf->head] = val;
}

//0 = most recent (head)
analogPosnArray getRingBufXRecent(ring_buffer *buf, int x) {

    if (x < 0) {
        printf("cannot return value from the future\r\n");
        return buf->vals[buf->head];
    } else if (x > buf->size-1) { //starts at 0, so x=4 is really 5th most recent
        printf("x exceeds size of buf\r\n");
        return buf->vals[buf->tail]; //whats the best thing to actually do here? return oldest val?
    } else {
        int ind = buf->head - x;
        if (ind < 0) {
            ind = buf->size + ind;
        }
        return buf->vals[ind];
    }
}

//prints most recent first
void printRingBuf(ring_buffer *buf) {
    printf("head = %d, tail = %d, numFilled = %d\r\n", buf->head, buf->tail, buf->numFilled);
    printf("cbuf: ");
    for (int i=0; i<buf->numFilled; i++) {
        analogPosnArray posns = getRingBufXRecent(buf, i);
        for (int i=3; i<NUM_SERVOS; i++) {
            printf("%d, ", posns.servoVals[i]);
        }

        printf("\r\n");
    }
    printf("-----------------------\r\n");
}