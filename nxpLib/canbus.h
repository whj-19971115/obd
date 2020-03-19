#ifndef NXPLIB_CANBUS_H_
#define NXPLIB_CANBUS_H_

int Canbus0Init(uint32_t baudrate);
void Canbus0IrqInit();
int Canbus0Send(uint32_t id, uint8_t *wdata, uint8_t len, uint8_t mode);
void Canbus0Recv(uint32_t *id, uint8_t *rdata, uint8_t *len);
void canbus0ReadIrq(int *id, uint8_t *rData, int32_t *queueLen);


#endif
