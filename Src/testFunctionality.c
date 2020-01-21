/*
 * testFunctionality.c
 *
 *  Created on: Jan 11, 2020
 *      Author: DC
 */

#include <stdio.h>
#include "bme280.h"
#include "MY_NRF24.h"
#include "spi.h"
#include "usart.h"

void testI2C_BME()
{
  //----TEST I2C----- // TODO: remove
  volatile uint8_t registerVal = bme280ReadReg(0xD0);
  bme280WriteReg(0xF4, 0x07); // wake the BME280 sensor

  int32_t tRaw = 0;
  int32_t pRaw = 0;
  int32_t hRaw = 0;

  BME280_Read_Calibration();
  while(1)
  {
	  bme280ReadAllRaw(&tRaw, &pRaw, &hRaw);

	  uint32_t temperature = BME280_CalcT(tRaw);
	  uint32_t paPressure = BME280_CalcP(pRaw);
	  volatile float pascalFloat = ((float)paPressure)/256.0;
	  uint32_t dummy = pRaw;
	  float dummy2 = pascalFloat;


	  // Human-readable temperature, pressure and humidity value
	  uint32_t pressure;
	  uint32_t humidity;

	  // Human-readable altitude value
	  volatile int32_t altitude;

	  // Pressure in Q24.8 format
	  uint32_t press_q24_8;
	  press_q24_8 = BME280_CalcP(pRaw);
	  //			press_q24_8 = 24674867; // test Q24.8 value, output must be 96386.199
	  // Convert Q24.8 value to integer
	  // e.g. Q24.8 value '24674867' will be converted to '96386199' what represents 96386.199
	  // Fractional part computed as (frac / 2^8)
	  pressure = ((press_q24_8 >> 8) * 1000) + (((press_q24_8 & 0xff) * 390625) / 100000);

	  altitude = BME280_Pa_to_Alt(pressure / 1000);
	  volatile int32_t dummy99 = altitude;
  }
  //---------------------------------
}

void spiNRF24()
{
  //----TEST SPI----- // TODO: remove
  NRF24_begin(GPIOB, SPI3_CS_Pin, SPI3_CE_Pin, hspi3);
  nrf24_DebugUART_Init(huart2);

  printRadioSettings();

  // Transmit and don't wait for ACK
  NRF24_stopListening(); // just in case
  uint64_t TxpipeAddrs = 0x11223344AA;
  uint64_t RxpipeAddrs = 0x11223344AA;
  NRF24_openReadingPipe(1, RxpipeAddrs);
  NRF24_setAutoAck(false); // turn off auto ACK
  NRF24_setChannel(52);

  NRF24_startListening(); // start buffering data that comes in over radio

  uint8_t counter = 0;
  uint8_t myRxData[50];
  while(1)
  {
	  if(NRF24_available())
	  {
		  NRF24_read(myRxData, 32);

		  myRxData[32] = '\r'; myRxData[32+1] = '\n';
		  HAL_UART_Transmit(&huart2, (uint8_t *)myRxData, 32+2, 10);
	  }

	  uint8_t gpsData[300] = {0};
	  readGPScoordinates(gpsData, 300); // I know this pointer points to a buffer of 300 elements  TODO: automate with creating structure of 2 elements consisting of array pointer and size pointed to
	  HAL_UART_Transmit(&huart2, gpsData, 300, 100);

	  // print counter
	  char counterBuf[30] = {0};
	  snprintf(counterBuf, 30, "count = %d\r\n", counter);
	  HAL_UART_Transmit(&huart2, (uint8_t*)counterBuf, 30, 10);
	  counter++;
  }

  // Transmit data over nRF
//  char myTxData[32] = "Hello World!"; // data to transmit over nRF
//  while(1)
//  {
//	  if(NRF24_write(myTxData, 32)) // maximum data length is 32
//		  HAL_UART_Transmit(&huart2, (uint8_t *)"Transmitted Successfully\r\n", strlen("Transmitted Successfully\r\n"), 10);
//
//  	  HAL_Delay(1000);
//  }
  //-----------------REMOVE THIS
}
