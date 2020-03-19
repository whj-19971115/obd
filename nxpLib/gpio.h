/*
 * gpio.h
 *
 *  Created on: 2019Äê5ÔÂ14ÈÕ
 *      Author: Administrator
 */

#ifndef NXPLIB_GPIO_H_
#define NXPLIB_GPIO_H_
#include "stdint.h"
void GpioInit();
void GpioSetDirection(uint8_t portCate, uint8_t portNum, uint8_t dir);
void GpioSetVal(uint8_t portCate, uint8_t portNum, uint8_t portVal);
void GpioGetVal(uint8_t portCate, uint8_t portNum, uint8_t *portVal);

#endif /* NXPLIB_GPIO_H_ */
