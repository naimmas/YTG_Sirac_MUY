#ifndef RINGBUF_H_
#define RINGBUF_H_

#define RINGBUF_SIZE    20

typedef struct RINGBUF_ {
   volatile int head;
   volatile float buffer[RINGBUF_SIZE];
} RINGBUF;

void ringbuf_init(RINGBUF *RingBuf);
void ringbuf_addSample(float sample, RINGBUF *RingBuf);
float ringbuf_averageOldestSamples(int numSamples, RINGBUF *RingBuf);
float ringbuf_averageNewestSamples(int numSamples, RINGBUF *RingBuf);

#endif
