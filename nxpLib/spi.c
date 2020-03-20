#include "device_registers.h" /* include peripheral declarations S32K144 */
#include "stdint.h"
#include "string.h"
#include "usertype.h"
//#include "null.h"
#include "nxpQueue.h"

/*
 * ivcu only spi0 connect out, so only consider spi0
 * below spi all spi0
 * baudrate is 1MHZ
 * */

static queue str_recvQueue;
static queue str_sendQueue;

static int queueMaxSize = 100;
static int spi_sendStatus = 0, spi_readStatus;
static uint32_t spi_sendNum = 0, spi_needSendNum = 0;
static uint32_t spi_readNum = 0, spi_needReadNum = 0;
unsigned char spi_buf[1024] = {0}, spi_recv[1024] = {0};
static uint8_t _transBits = 8;
//transBits is 8,16 or 32.
bool SpiInitMaster(uint8_t transBits)
{
	if(!((transBits==8) || (transBits==16) || (transBits==32))){
		return false;
	}
	_transBits = transBits;
	initQueue(&str_recvQueue, queueMaxSize);
	initQueue(&str_sendQueue, queueMaxSize);
	spi_readStatus = SPI_READ_OVER;
	spi_sendStatus = SPI_SEND_OVER;

	PCC->PCCn[PCC_PORTB_INDEX ]|=PCC_PCCn_CGC_MASK; /* Enable clock for PORTB */
	  PORTB->PCR[2]|=PORT_PCR_MUX(3); /* Port B14: MUX = ALT3, LPSPI0_SCK */
	  PORTB->PCR[3]|=PORT_PCR_MUX(3); /* Port B15: MUX = ALT3, LPSPI0_SIN */
	  PORTB->PCR[4]|=PORT_PCR_MUX(3); /* Port B16: MUX = ALT3, LPSPI0_SOUT */
	  PORTB->PCR[5]|=PORT_PCR_MUX(3); /* Port B17: MUX = ALT3, LPSPI0_PCS3 */

	/*!
	 * SPI0 Clocking:
	 * ===================================================
	 */
	PCC->PCCn[PCC_LPSPI0_INDEX] = 0;          		/* Disable clocks to modify PCS ( default) 	*/
	PCC->PCCn[PCC_LPSPI0_INDEX] = PCC_PCCn_PR_MASK	/* (default) Peripheral is present.			*/
								 |PCC_PCCn_CGC_MASK	/* Enable PCS=SPLL_DIV2 (40 MHz func'l clock) 	*/
								 |PCC_PCCn_PCS(6);
	/*!
	 * LPSPI0 Initialization:
	 * ===================================================
	 */
  LPSPI0->CR    = 0x00000000;   			/* Disable module for configuration 			*/
  LPSPI0->IER   =  LPSPI_IER_RDIE_MASK
				  | LPSPI_IER_TEIE_MASK
				  | LPSPI_IER_FCIE_MASK;   			/* Interrupts not used 						*/
//  LPSPI0->IER   = LPSPI_IER_TDIE_MASK;

  LPSPI0->DER   = 0x00000000;   			/* DMA not used 								*/
  LPSPI0->CFGR0 = 0x00000000;   			/* Defaults: 									*/
                                			/* RDM0=0: rec'd data to FIFO as normal 		*/
                                			/* CIRFIFO=0; Circular FIFO is disabled 		*/
                                			/* HRSEL, HRPOL, HREN=0: Host request disabled */

  LPSPI0->CFGR1 = LPSPI_CFGR1_MASTER_MASK  /* Configurations: master mode									*/
				  | LPSPI_CFGR1_NOSTALL_MASK;						/* PCSCFG=0: PCS[3:2] are enabled 								*/
											/* OUTCFG=0: Output data retains last value when CS negated	*/
											/* PINCFG=0: SIN is input, SOUT is output 						*/
											/* MATCFG=0: Match disabled 									*/
											/* PCSPOL=0: PCS is active low 								*/
											/* NOSTALL=0: Stall if Tx FIFO empty or Rx FIFO full 			*/
											/* AUTOPCS=0: does not apply for master mode 					*/
											/* SAMPLE=0: input data sampled on SCK edge 					*/
											/* MASTER=1: Master mode 										*/
  LPSPI0->TCR = 0;
  LPSPI0->TCR   = 0
				  |LPSPI_TCR_PRESCALE(2)
				  |LPSPI_TCR_CONTC(0)  //
				  |LPSPI_TCR_CONT(0)   //
					|LPSPI_TCR_CPOL(0)
					&(~LPSPI_TCR_CPHA(1))
				  |LPSPI_TCR_PCS(1)
				  |LPSPI_TCR_FRAMESZ(transBits - 1);   /* Transmit cmd: PCS3, 16 bits, prescale func'l clk by 4, etc	*/
											/* CPOL=0: SCK inactive state is low 							*/
											/* CPHA=1: Change data on SCK lead'g, capture on trail'g edge	*/
											/* PRESCALE=2: Functional clock divided by 2**2 = 4 			*/
											/* PCS=3: Transfer using PCS3 									*/
											/* LSBF=0: Data is transfered MSB first 						*/
											/* BYSW=0: Byte swap disabled 									*/
											/* CONT, CONTC=0: Continuous transfer disabled 				*/
											/* RXMSK=0: Normal transfer: rx data stored in rx FIFO 		*/
											/* TXMSK=0: Normal transfer: data loaded from tx FIFO 			*/
											/* WIDTH=0: Single bit transfer 								*/
											/* FRAMESZ=31: # bits in frame = 31+1=32 						*/

  LPSPI0->CCR   = LPSPI_CCR_SCKPCS(4)
				  |LPSPI_CCR_PCSSCK(4)
				  |LPSPI_CCR_DBT(8)
				  |LPSPI_CCR_SCKDIV(8);   	/* Clock dividers based on prescaled func'l clk of 100 nsec 	*/
											/* SCKPCS=4: SCK to PCS delay = 4+1 = 5 (500 nsec) 			*/
											/* PCSSCK=4: PCS to SCK delay = 9+1 = 10 (1 usec) 				*/
											/* DBT=8: Delay between Transfers = 8+2 = 10 (1 usec) 			*/
											/* SCKDIV=8: SCK divider =8+2 = 10 (1 usec: 1 MHz baud rate) 	*/

  LPSPI0->FCR   = LPSPI_FCR_TXWATER(3);   	/* RXWATER=0: Rx flags set when Rx FIFO >0 	*/
                                			/* TXWATER=3: Tx flags set when Tx FIFO <= 3 	*/

  LPSPI0->CR    = LPSPI_CR_MEN_MASK;
		  	  	  //|LPSPI_CR_DBGEN_MASK;   	/* Enable module for operation 			*/
											/* DBGEN=1: module enabled in debug mode 	*/
											/* DOZEN=0: module enabled in Doze mode 	*/
											/* RST=0: Master logic not reset 			*/
											/* MEN=1: Module is enabled 				*/
																				
	return true;
}

int spiIrqInit(int port){

	  if(port != SPI0 && port != SPI1)	return -1;
	
		S32_NVIC->ICPR[(LPSPI0_IRQn + port*2) >> 5] = 1 << ((LPSPI0_IRQn + port) % 32);  /* clr any pending IRQ*/
		S32_NVIC->ISER[(LPSPI0_IRQn + port*2) >> 5] = 1 << ((LPSPI0_IRQn + port) % 32);  /* enable IRQ */
		S32_NVIC->IP[LPSPI0_IRQn + port] = 0xA;              /* priority 10 of 0-15*/

}
//-------------------------------------------------------------------------------------------
//-------How many bits are sent and received here is related to LPSPI_TCR_FRAMESZ----yxl-----

void LPSPI0_transmit_32bits (uint32_t send)
{
  while((LPSPI0->SR & LPSPI_SR_TDF_MASK)>>LPSPI_SR_TDF_SHIFT==0);
                                   /* Wait for Tx FIFO available 	*/
  LPSPI0->TDR = send;              /* Transmit data 				*/
  LPSPI0->SR |= LPSPI_SR_TDF_MASK; /* Clear TDF flag 				*/
}

uint8_t LPSPI0_receive_32bits (void)
{
  uint32_t recieve = 0;

  while((LPSPI0->SR & LPSPI_SR_RDF_MASK)>>LPSPI_SR_RDF_SHIFT==0);
                                   /* Wait at least one RxFIFO entry 	*/
  recieve= LPSPI0->RDR;            /* Read received data 				*/
  LPSPI0->SR |= LPSPI_SR_RDF_MASK; /* Clear RDF flag 					*/
  return recieve;                  /* Return received data 			*/
}

void LPSPI0_transmit_16bits (uint16_t send)
{
  while((LPSPI0->SR & LPSPI_SR_TDF_MASK)>>LPSPI_SR_TDF_SHIFT==0);
                                   /* Wait for Tx FIFO available 	*/
  LPSPI0->TDR = send;              /* Transmit data 				*/
  LPSPI0->SR |= LPSPI_SR_TDF_MASK; /* Clear TDF flag 				*/
}

uint16_t LPSPI0_receive_16bits (void)
{
  uint16_t recieve = 0;

  while((LPSPI0->SR & LPSPI_SR_RDF_MASK)>>LPSPI_SR_RDF_SHIFT==0);
                                   /* Wait at least one RxFIFO entry 	*/
  recieve= LPSPI0->RDR;            /* Read received data 				*/
  LPSPI0->SR |= LPSPI_SR_RDF_MASK; /* Clear RDF flag 					*/
  return recieve;                  /* Return received data 			*/
}

void LPSPI0_transmit_8bits (uint8_t send)
{
  while((LPSPI0->SR & LPSPI_SR_TDF_MASK)>>LPSPI_SR_TDF_SHIFT==0);
                                   /* Wait for Tx FIFO available 	*/
  LPSPI0->TDR = send;              /* Transmit data 				*/
  LPSPI0->SR |= LPSPI_SR_TDF_MASK; /* Clear TDF flag 				*/
}

uint8_t LPSPI0_receive_8bits (void)
{
  uint8_t recieve = 0;

  while((LPSPI0->SR & LPSPI_SR_RDF_MASK)>>LPSPI_SR_RDF_SHIFT==0);
                                   /* Wait at least one RxFIFO entry 	*/
  recieve= LPSPI0->RDR;            /* Read received data 				*/
  LPSPI0->SR |= LPSPI_SR_RDF_MASK; /* Clear RDF flag 					*/
  return recieve;                  /* Return received data 			*/
}
//--------------------------------------------------------------------
//--------------------------------------------------------------------
uint32_t SpiSend (uint8_t *wData, int32_t len, uint32_t timeOut_ms)
{
	int i = 0;
	uint32_t timeOutTemp = 0, sendNum = 0;
	uint32_t val = 0;
	if(len < 0 || wData == NULL)	return 0;
	timeOut_ms *= 8000;
	uint8_t transOnce = _transBits/8;
	uint8_t tmp[100] = 0, index = 0;
	
	if(len >= transOnce){
		for(i=0; i<len - len%transOnce; i=i+transOnce){
			  while(((LPSPI0->SR & LPSPI_SR_TDF_MASK)>>LPSPI_SR_TDF_SHIFT) == 0){
				  //timeOutTemp++;
				  if(timeOutTemp >= timeOut_ms){
					  return sendNum;
				  }
			  }
			  sendNum += transOnce;
				/* Wait for Tx FIFO available 	*/
				for(int j=0;j<transOnce; j++){
					val |= (wData[i+j] << (8 * j));
				}
				tmp[index++] = val;
			  LPSPI0->TDR = val;              /* Transmit data 				*/
				val = 0;
			  LPSPI0->SR |= LPSPI_SR_TDF_MASK; /* Clear TDF flag 				*/
			  if(sendNum == len){	
					return sendNum;
				}
		}
	}
	if(sendNum < len){
		while(((LPSPI0->SR & LPSPI_SR_TDF_MASK)>>LPSPI_SR_TDF_SHIFT) == 0){
		  //timeOutTemp++;
		  if(timeOutTemp >= timeOut_ms){
			  return sendNum;
		  }
    }
		for(i=len-len%transOnce; i<len; i++){
			val |= (wData[i] << (8 * i));
		}
		sendNum = len;
		LPSPI0->TDR = val;
		LPSPI0->SR |= LPSPI_SR_TDF_MASK; /* Clear TDF flag 				*/
	}
	return sendNum;
}

/*
 * len % 4 == 0
 * */
uint32_t SpiRecv (uint8_t *rData, int32_t len, uint32_t timeOut_ms)
{
	  uint32_t recieve = 0;
	  uint32_t timeOutTemp = 0,timeOutSendTemp =0, readNum = 0;
	  timeOut_ms *= (8000*10);
	  uint8_t transOnce = _transBits/8;
	  int rxCount = 0;
	  uint8_t tmp[100] = 0;

	  while(len > readNum){
			//--------------------------------------------------------------------------------
//			while(((LPSPI0->SR & LPSPI_SR_TDF_MASK)>>LPSPI_SR_TDF_SHIFT) == 0){
//				timeOutSendTemp = timeOutSendTemp;
//      }
			
//			LPSPI0->TDR = 0;
		  LPSPI0->SR |= LPSPI_SR_TDF_MASK; /* Clear TDF flag 				*/
			//----------------------------------------------------------------------------------
		  while(((LPSPI0->SR & LPSPI_SR_RDF_MASK)>>LPSPI_SR_RDF_SHIFT==0) || (rxCount == 0)){
				rxCount = (LPSPI0->FSR >> 16) & 0x07;
				if(rxCount > 0){
					rxCount = rxCount;
				}
			  timeOutTemp++;
			  if(timeOutTemp >= timeOut_ms){
				  return readNum;
			  }
		  }
										   /* Wait at least one RxFIFO entry 	*/
		  recieve= LPSPI0->RDR;            /* Read received data 				*/
			if(recieve != 0){
				recieve = recieve;
			}
		  memcpy(rData + readNum, &recieve, transOnce);
		  LPSPI0->SR |= LPSPI_SR_RDF_MASK; /* Clear RDF flag 					*/
		  readNum += transOnce;

		  if(readNum == len)	{
				memcpy(tmp, rData, len);
				return readNum;
			}
	  }
	  return readNum;                  /* Return received data 	length		*/
}

int spiReadIrq(unsigned char *rData, int32_t len)
{
	int ret = -1;
	if(len > queueMaxSize)	return ret;

	if(spi_needReadNum == 0){
		spi_needReadNum = len;
		spi_readStatus = SPI_READ_ING;
	}
	else if(spi_readNum == spi_needReadNum){
		memcpy(rData, spi_recv, len);
		spi_readNum = spi_needReadNum = 0;
		spi_readStatus = SPI_READ_OVER;
	}
	ret = spi_readStatus;

	return ret;
}
int SpiSendIrq (uint8_t *wData, int32_t len)
{
	int ret = -1, i = 0;
	ElemType  _elemIn;
	if(len > queueMaxSize)	return ret;
	if(spi_needSendNum == 0){
		spi_needSendNum = len;
		spi_sendStatus = SPI_SEND_ING;
		for(i=0; i<len; i++){
			_elemIn.data = wData[i];
			enQueue(&str_sendQueue, _elemIn);
		}
		LPSPI0->IER |= LPSPI_IER_TDIE_MASK;
	}
	else if(spi_needSendNum == spi_sendNum){
		spi_sendNum = spi_needSendNum = 0;
		spi_sendStatus = SPI_SEND_OVER;
	}
	ret = spi_sendStatus;
	return ret;
}

void LPSPI0_IRQHandler(uint32_t instance)
{
	uint32_t recvVal = 0, sendVal = 0;
	uint16_t txCount, rxCount, sendFlag = 0;
	ElemType _elemType, _elemOut, _elemIn;
	int i = 0;
//	txCount = LPSPI0->FSR & 0x07;
	rxCount = (LPSPI0->FSR >> 16) & 0x07;

	/* RECEIVE IRQ handler: Check read buffer only if there are remaining bytes to read. */
	if((LPSPI0->SR & LPSPI_SR_RDF_MASK)>>LPSPI_SR_RDF_SHIFT==1 && (rxCount != (uint16_t)0) /*&& (LPSPI0->SR & LPSPI_SR_REF_MASK)>>LPSPI_SR_RDF_SHIFT==1*/)
	{
		 recvVal = LPSPI0->RDR;
			LPSPI0->SR |= LPSPI_SR_RDF_MASK;//
		for(i = 0;i< rxCount; i++){
			_elemType.data = (recvVal >> (8*i)) & 0xff;
			enQueue(&str_recvQueue,_elemType);
		}
		if(spi_needReadNum > 0){
			while(getQueueLen(&str_recvQueue) > 0 && spi_readNum<spi_needReadNum){
				deQueue(&str_recvQueue, &_elemOut);
				spi_recv[spi_readNum++] = _elemOut.data;
			}
		}
	}
	/* Transmit data */
	if ((LPSPI0->SR & LPSPI_SR_TDF_MASK)>>LPSPI_SR_TDF_SHIFT==1 /*&& (txCount != (uint16_t)0)*/)
	{
		for(i=0; i< 4; i++){
			if(getQueueLen(&str_sendQueue) > 0){
				deQueue(&str_sendQueue, &_elemIn);
				sendVal |=  ((uint32_t)_elemIn.data << (8*i));
				spi_sendNum++;
				sendFlag = 1;
			}
			else{
				break;
			}
		}
		if(sendFlag){
			LPSPI0->TDR = sendVal;
			LPSPI0->SR |= LPSPI_SR_TDF_MASK; /* Clear TDF flag 				*/
		}
		if(getQueueLen(&str_sendQueue) <= 0){
			LPSPI0->IER &= (~LPSPI_IER_TDIE_MASK);
		}
	}
}

