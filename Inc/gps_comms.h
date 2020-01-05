/*
 * gps_comms.h
 *
 *  Created on: Dec 19, 2019
 *      Author: DC
 */

#ifndef GPS_COMMS_H_
#define GPS_COMMS_H_

#define AVAILABLE_GPS_RADIOS 1 // must correspond to number of available GPS receivers

#include <stdint.h>

typedef struct gpsCoordinates
{
	float x;
	float y;
	float z;
} GpsCoordinates;


GpsCoordinates readGPScoordinates(uint8_t buffer[], uint16_t size);

#endif /* GPS_COMMS_H_ */
