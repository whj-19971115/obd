/*
 * usertype.h
 *
 *  Created on: 2019Äê5ÔÂ15ÈÕ
 *      Author: Administrator
 */

#ifndef NXPLIB_USERTYPE_H_
#define NXPLIB_USERTYPE_H_

//--------gpio----------------
#define PORT_A	1
#define PORT_B	2
#define PORT_C	3
#define PORT_D	4
#define PORT_E	5

#define PORT_IN		0
#define PORT_OUT	1
#define PORT_HIGH	1
#define PORT_LOW	0

//---------uart---------------
#define UART0	0
#define UART1	1
#define UART2	2
#define UART_READ_MAX		1024

#define UART0_READ_ING		0
#define UART0_READ_OVER		1

#define UART1_READ_ING		0
#define UART1_READ_OVER		1

#define UART0_SEND_ING		0
#define UART0_SEND_OVER		1
#define UART0_SEND_NO			2

#define UART1_SEND_ING		0
#define UART1_SEND_OVER		1
#define UART1_SEND_NO			2
//----------canbus---------------
#define CAN_EXTERN_MODE		1
#define CAN_STANDARD_MODE	2

//----------spi-----------------------
#define SPI0	0
#define SPI1	1

#define SPI_READ_ING		0
#define SPI_READ_OVER		1

#define SPI_SEND_ING		0
#define SPI_SEND_OVER		1

//-----------timer--------------
#define TIMER_MAX	10
#define TIMER_OUT	1

#endif /* NXPLIB_USERTYPE_H_ */
