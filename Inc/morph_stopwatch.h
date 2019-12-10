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

#include <stdio.h>
extern uint32_t NOW_MS;

typedef struct morph_stopwatch
{
	uint32_t EPOCH;
}morph_stopwatch;

void morphStopWatch_start(morph_stopwatch *s);

uint32_t morphStopWatch_ms(morph_stopwatch *s);

#endif /* MORPH_STOPWATCH_H_ */
