#include "RingBuffer/ringbuf.h"
#include "string.h"

void ringbuf_init(RINGBUF *RingBuf) {
   memset(RingBuf->buffer,0,RINGBUF_SIZE*sizeof(float));
   RingBuf->head = RINGBUF_SIZE-1;
   }

void ringbuf_addSample(float sample, RINGBUF *RingBuf) {
   RingBuf->head++;
   if (RingBuf->head >= RINGBUF_SIZE) RingBuf->head = 0;
   RingBuf->buffer[RingBuf->head] = sample;
   }


float ringbuf_averageOldestSamples(int numSamples, RINGBUF *RingBuf) {
   int index = RingBuf->head+1; // oldest Sample
   float accum = 0.0f;
   for (int count = 0; count < numSamples; count++) {
      if (index >= RINGBUF_SIZE) index = 0;
      accum += RingBuf->buffer[index];
      index++;
      }
   return accum/numSamples;
   }   

float ringbuf_averageNewestSamples(int numSamples, RINGBUF *RingBuf) {
   int index = RingBuf->head; // newest Sample
   float accum = 0.0f;
   for (int count = 0; count < numSamples; count++) {
      if (index < 0) index = RINGBUF_SIZE - 1;
      accum += RingBuf->buffer[index];
      index--;
      }
   return accum/numSamples;
   }
