/*
 * bme280.h
 *
 *  Created on: Jan 7, 2020
 *      Author: DC
 */

#ifndef BME280_H_
#define BME280_H_

uint8_t bme280ReadReg(uint8_t reg);
void bme280WriteReg(uint8_t reg, uint8_t value);
uint16_t bme280ReadPressure();

#endif /* BME280_H_ */
