#ifndef NXPLIB_SPI_H_
#define NXPLIB_SPI_H_
#include "stdint.h"
#include <stdbool.h>

bool SpiInitMaster(uint8_t transBits);
uint32_t SpiSend (uint8_t *wData, int32_t len, uint32_t timeOut_ms);
uint32_t SpiRecv (uint8_t *rData, int32_t len, uint32_t timeOut_ms);
int spiIrqInit(int port);
int spiReadIrq(unsigned char *rData, int32_t len);
int SpiSendIrq (uint8_t *wData, int32_t len);

#endif
