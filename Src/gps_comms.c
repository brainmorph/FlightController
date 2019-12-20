/*
 * gps_comms.c
 *
 *  Created on: Dec 19, 2019
 *      Author: DC
 */


/*
 * This module is intended for high order instructions of the GPS subsystem
 */
#include "gps_comms.h"
#include "usart.h"





GpsCoordinates readGPScoordinates()
{
	// return the latest UART reception consisting of coordinates

	uint8_t uartReceive[300] = {0};
	HAL_UART_Receive(&huart5, uartReceive, 300, 1000); // TODO: narrow down number of bytes required to read

	// TODO: parse uartReceive buffer
	GpsCoordinates c = {0};

	//c.x = uartReceive[...]
	//c.y = uartReceive[...]
	return c;
}
