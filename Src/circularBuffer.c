/*
 * circularBuffer.c
 *
 *  Created on: Dec 6, 2019
 *      Author: DC
 */

#include "circularBuffer.h"

// -- Module Function Definitions -- // ----------------------------------------------

/*
 * Counts number of data elements currently in buffer
 *
 * Input: circular buffer
 *
 * Output: number of data elements currently in buffer
 */
uint16_t cbDataLength(tCircularBuffer* cb)
{
	int32_t length = cb->write - cb->read;
	if(length > 0)
		return length;

	//otherwise the buffer has rolled over and you need to read the length this way
	length = cb->write + (length - cb->read);
	return length;
}

/*
 * Force new element into circular buffer and overwrite the oldest value if full
 *
 * Input: pointer to circular buffer, value to insert
 */
static void cbBumpNewDataIn(tCircularBuffer* cb, float val)
{
	cb->read = (cb->read + 1) & (cb->size - 1);
	cb->buf[cb->write] = val;
	cb->write = (cb->write + 1) & (cb->size -1);
}

/*
 * Add element to circular buffer.
 * If buffer is already full and if force != 0 then overwrite the oldest element
 *
 * Input:	circular buffer, data to be inserted, overwrite flag
 *
 * Output:	-1 if buffer is full
 */
int cbWrite(tCircularBuffer* cb, float data, uint8_t force)
{
	if(cbDataLength(cb) == (cb->size - 1)) //check if buffer is full
	{
		if(force !=0)
			cbBumpNewDataIn(cb, data);

		return -1;
	}

	//otherwise insert new data
	cb->buf[cb->write] = data;
	cb->write = (cb->write + 1) & (cb->size - 1); //this is possible because size is a power of 2

	return 0;
}

/*
 *
 */
int cbRead(tCircularBuffer* cb, float* data)
{
	if(cbDataLength(cb) == 0)
		return -1; //empty buffer

	//otherwise return oldest data
	*data = cb->buf[cb->read];
	cb->read = (cb->read + 1) & (cb->size - 1); //this is possible because size is a power of 2
}
