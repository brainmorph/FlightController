/*
 * morph_stopwatch.h
 *
 *  Created on: Nov 24, 2019
 *      Author: DC
 *
 * Enables easy creation of multiple stop-watches
 */

#ifndef MORPH_STOPWATCH_H_
#define MORPH_STOPWATCH_H_

uint32_t NOW_MS = 0;

typedef struct morph_stopwatch
{
	uint32_t EPOCH;
	uint32_t CURRENT_MS;
}morph_stopwatch;

void morphStopWatch_start(morph_stopwatch s) // restarts the stopwatch if already started
{
	s.EPOCH = NOW_MS;
	s.CURRENT_MS = 0;
}

uint32_t morphStopWatch_ms(morph_stopwatch id) // returns time since specified stop-watch started (in milliseconds)
{
	return NOW_MS - id.EPOCH;
}

#endif /* MORPH_STOPWATCH_H_ */
