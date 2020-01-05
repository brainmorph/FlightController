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
#include "stm32f4xx_hal_uart.h"




GpsCoordinates readGPScoordinates(uint8_t buffer[], uint16_t size)
{
	// return the latest UART reception consisting of coordinates
	if(__HAL_UART_GET_FLAG(&huart5, UART_FLAG_RXNE)) // check if data is available
		HAL_UART_Receive(&huart5, buffer, size, 300); // TODO: narrow down number of bytes required to read

	// TODO: parse uartReceive buffer
	GpsCoordinates c = {0};
	c.x = 1.0;
	c.y = 2.0;
	c.z = 3.0;

	//c.x = uartReceive[...]
	//c.y = uartReceive[...]

	return c;
}
