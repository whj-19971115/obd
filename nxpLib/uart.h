/*
 * uart.h
 *
 *  Created on: 2019Äê5ÔÂ15ÈÕ
 *      Author: Administrator
 */

#ifndef NXPLIB_UART_H_
#define NXPLIB_UART_H_
#include "stdint.h"

int uartInit(uint8_t port, uint32_t baudrate);
int uartIrqInit(uint8_t port);
// must be unsigned char*, do not cast other char*, otherwise the receiver will receive garble code!!!!!!!!!!
int uartSendBlock(uint8_t port, unsigned char *wData, uint32_t len, uint32_t timeOut);
int uartReadBlock(uint8_t port, unsigned char *rData, uint32_t len, uint32_t timeOut);
int uartReadIrq(uint8_t port,  void *rData, int32_t len, int32_t* rLenNow);
int uartSendIrq(uint8_t port,  unsigned char *wData, int32_t len, int32_t* sLenNow);
void UART_Reset(int port);
void UART_pin_settings_reset(int port);
int uartSendStatusGet(int port);

#endif /* NXPLIB_UART_H_ */
