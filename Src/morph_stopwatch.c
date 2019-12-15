/*
 * morph_stopwatch.c
 *
 *  Created on: Dec 10, 2019
 *      Author: DC
 */

#include "morph_stopwatch.h"

uint32_t NOW_MS;

void morphStopWatch_start(morph_stopwatch *s) // restarts the stopwatch if already started
{
	(*s).EPOCH = NOW_MS;
}

uint32_t morphStopWatch_ms(morph_stopwatch *s) // returns time since specified stop-watch started (in milliseconds)
{
	return NOW_MS - (*s).EPOCH;
}


