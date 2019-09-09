#ifndef __CIRCULARBUFFER_H
#define __CIRCULARBUFFER_H
#ifdef __cplusplus
extern "C" {
#endif

typedef struct tCircularBuffer
{
	float* buf;

	uint16_t size; // make SURE this is a power of 2
	uint16_t read;
	uint16_t write;
} tCircularBuffer;

#endif
