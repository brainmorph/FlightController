#ifndef __CIRCULARBUFFER_H
#define __CIRCULARBUFFER_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct tCircularBuffer
{
	float* buf;

	uint16_t size; // make SURE this is a power of 2 to make calculations quicker
	uint16_t read;
	uint16_t write;
} tCircularBuffer;

uint16_t cbDataLength(tCircularBuffer* cb);
int cbWrite(tCircularBuffer* cb, float data, uint8_t overwrite);
int cbRead(tCircularBuffer* cb, float* data);

#endif
