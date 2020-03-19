#include "device_registers.h" /* include peripheral declarations S32K144 */
#include "stdint.h"
#include "usertype.h"
#include "nxpQueue.h"
#include "string.h"

uint32_t  RxCODE;              /*< Received message buffer code 			*/
uint32_t  RxID;                /*< Received message ID 					*/
uint32_t  RxLENGTH;            /*< Recieved message number of data bytes 	*/
uint32_t  RxDATA[2];           /*< Received message data (2 words) 		*/
uint32_t  RxTIMESTAMP;         /*< Received message time 					*/

static uint32_t canbusBaudrate[5] = {50000, 100000, 125000, 250000, 500000};
typedef struct BaudParameter{
	uint8_t presdiv[5];
	uint8_t pseg2[5];
	uint8_t pseg1[5];
	uint8_t propseg[5];
	uint8_t rjw[5];
	uint8_t smp[5];
}_BaudParameter;


_BaudParameter str_baudParameter;
static ElemTypeCan str_elemType0;
static queueCan str_canbusQueue0;

static int queueMaxSize = 200;

void canbusParaInit(){
	int i = 0;
	str_baudParameter.presdiv[0] = 9;
	str_baudParameter.presdiv[1] = 4;
	str_baudParameter.presdiv[2] = 3;
	str_baudParameter.presdiv[3] = 1;
	str_baudParameter.presdiv[4] = 0;

	for(i=0; i< 5; i++){
		str_baudParameter.pseg1[i] = 3;
		str_baudParameter.pseg2[i] = 3;
		str_baudParameter.propseg[i] = 6;
		str_baudParameter.rjw[i] = 3;
		str_baudParameter.smp[i] = 1;
	}
	initQueueCan(&str_canbusQueue0, queueMaxSize);
}

int Canbus0Init(uint32_t baudrate)
{
#define MSG_BUF_SIZE  4		/* Msg Buffer Size. (CAN 2.0AB: 2 hdr +  2 data= 4 words) */
  uint32_t   i=0, baudIndex = 0;

  for(i=0; i< sizeof(canbusBaudrate); i++){
	  if(baudrate == canbusBaudrate[i]){
		  baudIndex = i;
		  break;
	  }
	  else if(i == sizeof(canbusBaudrate) - 1){
		  return -1;
	  }

  }
  canbusParaInit();
  PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK;	/* Enable clock for PORTE */
   PORTB->PCR[0] |= PORT_PCR_MUX(5);	/* Port E4: MUX = ALT5, CAN0_RX */
   PORTB->PCR[1] |= PORT_PCR_MUX(5); /* Port E5: MUX = ALT5, CAN0_TX */

  PCC->PCCn[PCC_FlexCAN0_INDEX] |= PCC_PCCn_CGC_MASK; /* CGC=1: enable clock to FlexCAN0 */

  CAN0->MCR |= CAN_MCR_MDIS_MASK;         /* MDIS=1: Disable module before selecting clock 	*/
  CAN0->CTRL1 &= ~CAN_CTRL1_CLKSRC_MASK;  /* CLKSRC=0: Clock Source = SOSCDIV2				*/
  CAN0->MCR &= ~CAN_MCR_MDIS_MASK;        /* MDIS=0; Enable module config. (Sets FRZ, HALT)	*/
//  CAN0->MCR |= CAN_MCR_RFEN_MASK;//YXL

  while (!((CAN0->MCR & CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT))  {}
	/*!
	 * Good practice:
	 * ===================================================
	 * wait for FRZACK=1 on freeze mode entry/exit
	 */
  CAN0->CTRL1 = 0
		  |CAN_CTRL1_PRESDIV(str_baudParameter.presdiv[baudIndex])
		  |CAN_CTRL1_PSEG2(str_baudParameter.pseg2[baudIndex])	    /* Configure for 500 KHz bit time 								*/
		  |CAN_CTRL1_PSEG1(str_baudParameter.pseg1[baudIndex]) 		/* Time quanta freq = 16 time quanta x 500 KHz bit time= 8MHz 	*/
		  |CAN_CTRL1_PROPSEG(str_baudParameter.propseg[baudIndex])		/* PRESDIV+1 = Fclksrc/Ftq = 8 MHz/8 MHz = 1 					*/
		  |CAN_CTRL1_RJW(str_baudParameter.rjw[baudIndex])			/*    so PRESDIV = 0 											*/
		  |CAN_CTRL1_SMP(str_baudParameter.smp[baudIndex]); 		/* PSEG2 = Phase_Seg2 - 1 = 4 - 1 = 3 							*/
								            /* PSEG1 = PSEG2 = 3 											*/
									/* PROPSEG= Prop_Seg - 1 = 7 - 1 = 6 							*/
									/* RJW: since Phase_Seg2 >=4, RJW+1=4 so RJW=3. 				*/
									/* SMP = 1: use 3 bits per CAN sample 							*/
									/* CLKSRC=0 (unchanged): Fcanclk= Fosc= 8 MHz 					*/

  for(i=0; i<128; i++ )
  {   					/* CAN0: clear 32 msg bufs x 4 words/msg buf = 128 words */
    CAN0->RAMn[i] = 0;  /* Clear msg buf word */
  }
  for(i=0; i<16; i++ )
  {          						/* In FRZ mode, init CAN0 16 msg buf filters */
    CAN0->RXIMR[i] = 0X000;  	/* Check all ID bits for incoming messages */
  }
  CAN0->RXMGMASK = 0x00000000;//0x1FFFFFFF;  				/* Global acceptance mask: check all ID bits 	*/
  CAN0->RAMn[ 4*MSG_BUF_SIZE + 0] = 0x04000000; /* Msg Buf 4, word 0: Enable for reception 	*/
                                                /* EDL,BRS,ESI=0: CANFD not used 				*/
                                                /* CODE=4: MB set to RX inactive 				*/
                                                /* IDE=0: Standard ID 							*/
                                                /* SRR, RTR, TIME STAMP = 0: not applicable 	*/
#ifdef NODE_A                                   /* Node A receives msg with std ID 0x511 		*/
  CAN0->RAMn[ 4*MSG_BUF_SIZE + 1] = 0x10000000;//0x14440000; /* Msg Buf 4, word 1: Standard ID = 0x111 		*/
#else                                           /* Node B to receive msg with std ID 0x555 	*/
  CAN0->RAMn[ 4*MSG_BUF_SIZE + 1] = 0x15540000; /* Msg Buf 4, word 1: Standard ID = 0x555 		*/
#endif
  //CAN0->RAMn[ 3*MSG_BUF_SIZE + 0] = 0x04000000;
  //CAN0->RAMn[ 3*MSG_BUF_SIZE + 1] = 0x14440000;
                                /* PRIO = 0: CANFD not used */
  CAN0->MCR = 0x0000001F;       /* Negate FlexCAN 1 halt state for 32 MBs */

//  MB0_ID = 0x100
//  RXIMR0 = 0x700  // lower 8 bit are don't care


  while ((CAN0->MCR && CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT)  {}
  /* Good practice: wait for FRZACK to clear (not in freeze mode) */

  while ((CAN0->MCR && CAN_MCR_NOTRDY_MASK) >> CAN_MCR_NOTRDY_SHIFT)  {}
  /* Good practice: wait for NOTRDY to clear (module ready) */
}

void Canbus0IrqInit()//
{
//	S32_NVIC->ICPR[1] = 1 << (CAN0_ORed_0_15_MB_IRQn % 32);  /* clr any pending IRQ*/
//	S32_NVIC->ISER[1] = 1 << (CAN0_ORed_0_15_MB_IRQn % 32);  /* enable IRQ */
	S32_NVIC->ISER[(uint32_t)(CAN0_ORed_0_15_MB_IRQn) >> 5U] = (uint32_t)(1U << ((uint32_t)(CAN0_ORed_0_15_MB_IRQn) & (uint32_t)0x1FU));
	S32_NVIC->ICPR[(uint32_t)(CAN0_ORed_0_15_MB_IRQn) >> 5U] = (uint32_t)(1U << ((uint32_t)(CAN0_ORed_0_15_MB_IRQn) & (uint32_t)0x1FU));
	S32_NVIC->IP[CAN0_ORed_0_15_MB_IRQn] = 0xA;              /* priority 10 of 0-15*/

	CAN0->IMASK1 = 1<<4 | 1<< 3;

}

void CAN0_ORed_0_15_MB_IRQHandler(uint8_t instance)
{
	uint32_t id; uint8_t rdata[8]; uint8_t len;
	Canbus0Recv(&id, rdata, &len);
}


void CAN0_ORed_16_31_MB_IRQHandler(uint8_t instance)
{
	uint32_t id; uint8_t rdata[8]; uint8_t len;
	Canbus0Recv(&id, rdata, &len);
}

int Canbus0Send(uint32_t id, uint8_t *wdata, uint8_t len, uint8_t mode)
{
	/*! Assumption:
	 * =================================
	 * Message buffer CODE is INACTIVE
	 */
	uint32_t data[2] = {0};
	if(len > 8 || len <=0)
		return -1;
	CAN0->IFLAG1 = 0x00000001;	/* Clear CAN 0 MB 0 flag without clearing others*/

//	memcpy(&data[0], wdata, 4);
//	memcpy(&data[1], &wdata[4], 4);
	data[0] = wdata[3] | (wdata[2] << 8)| (wdata[1] << 16)| (wdata[0] << 24);
	data[1] = wdata[7] | (wdata[6] << 8)| (wdata[5] << 16)| (wdata[4] << 24);
	CAN0->RAMn[ 0*MSG_BUF_SIZE + 2] = data[0];	/* MB0 word 2: data word 0 */
	CAN0->RAMn[ 0*MSG_BUF_SIZE + 3] = data[1]; /* MB0 word 3: data word 1 */

	if(mode == CAN_STANDARD_MODE){
		CAN0->RAMn[ 0*MSG_BUF_SIZE + 1] = (id << 18) & 0x1ffc0000; /* MB0 word 1: Tx msg with STD ID 0x511 */

		CAN0->RAMn[ 0*MSG_BUF_SIZE + 0] = 0x0C400000 | (len << CAN_WMBn_CS_DLC_SHIFT);
													/* MB0 word 0: 								*/
													/* EDL,BRS,ESI=0: CANFD not used 				*/
													/* CODE=0xC: Activate msg buf to transmit 		*/
													/* IDE=0: Standard ID 							*/
													/* SRR=1 Tx frame (not req'd for std ID) 		*/
													/* RTR = 0: data, not remote tx request frame	*/
													/* DLC = 8 bytes 								*/
	}
	else if(mode == CAN_EXTERN_MODE){
		CAN0->RAMn[ 0*MSG_BUF_SIZE + 1] = (id << 0) & 0x1fffffff; /* MB0 word 1: Tx msg with STD ID 0x511 */

		CAN0->RAMn[ 0*MSG_BUF_SIZE + 0] = 0x0C420000 | (len << CAN_WMBn_CS_DLC_SHIFT);
	}
	return 0;
}

void Canbus0Recv(uint32_t *id, uint8_t *rdata, uint8_t *len)
{
/*! Receive msg from ID 0x556 using msg buffer 4
 * =============================================
 */
  uint8_t j = 0, RxSRR = 0;
  if((CAN0->IFLAG1 >> 4) & 1 ){
	  RxCODE   = (CAN0->RAMn[ 4*MSG_BUF_SIZE + 0] & 0x07000000) >> 24;	/* Read CODE field */
	  RxSRR = (CAN0->RAMn[ 4*MSG_BUF_SIZE + 0] >> 22) & 0x01;
	  RxID  = (CAN0->RAMn[ 4*MSG_BUF_SIZE + 1] & CAN_WMBn_ID_ID_MASK)  >> CAN_WMBn_ID_ID_SHIFT;	/* Read ID 			*/
	  if(!RxSRR)
		  *id = RxID >> 18;
	  else
		  *id = RxID;

	  *len = RxLENGTH = (CAN0->RAMn[ 4*MSG_BUF_SIZE + 0] & CAN_WMBn_CS_DLC_MASK) >> CAN_WMBn_CS_DLC_SHIFT;	/* Read Message Length */

	  for (j=0; j<2; j++)
	  {  /* Read two words of data (8 bytes) */
		RxDATA[j] = CAN0->RAMn[ 4*MSG_BUF_SIZE + 2 + j];
	  }
	  memcpy(rdata, RxDATA, 4);
	  memcpy(rdata + 4, &RxDATA[1], 4);

	  str_elemType0.canbusId = *id;
	  memcpy(str_elemType0.canbusData, rdata, 8);
	  enQueueCan(&str_canbusQueue0, str_elemType0);

	  RxTIMESTAMP = (CAN0->RAMn[ 0*MSG_BUF_SIZE + 0] & 0x000FFFF);
	  uint32_t dummy = CAN0->TIMER;             /* Read TIMER to unlock message buffers */
	  CAN0->IFLAG1 = (1<<4);       /* Clear CAN 0 MB 4 flag without clearing others*/
  }
  else if((CAN0->IFLAG1 >> 3) & 1 ){
	  RxCODE   = (CAN0->RAMn[ 3*MSG_BUF_SIZE + 0] & 0x07000000) >> 24;	/* Read CODE field */
	  *id = RxID     = (CAN0->RAMn[ 3*MSG_BUF_SIZE + 1] & CAN_WMBn_ID_ID_MASK)  >> CAN_WMBn_ID_ID_SHIFT;	/* Read ID 			*/
	  *len = RxLENGTH = (CAN0->RAMn[ 3*MSG_BUF_SIZE + 0] & CAN_WMBn_CS_DLC_MASK) >> CAN_WMBn_CS_DLC_SHIFT;	/* Read Message Length */

	 	  for (j=0; j<2; j++)
	 	  {  /* Read two words of data (8 bytes) */
	 		RxDATA[j] = CAN0->RAMn[ 3*MSG_BUF_SIZE + 2 + j];
	 	  }
	 	  memcpy(rdata, RxDATA, 4);
	 	  memcpy(rdata + 4, &RxDATA[1], 4);

	 	 str_elemType0.canbusId = *id;
	 	 memcpy(str_elemType0.canbusData, rdata, 8);
	 	 enQueueCan(&str_canbusQueue0, str_elemType0);

	 	  RxTIMESTAMP = (CAN0->RAMn[ 0*MSG_BUF_SIZE + 0] & 0x000FFFF);
	 	  uint32_t dummy = CAN0->TIMER;             /* Read TIMER to unlock message buffers */
	 	  CAN0->IFLAG1 = (1<<3);       /* Clear CAN 0 MB 4 flag without clearing others*/
  }
}

int canbus0ReadIrq(int *id, uint8_t *rData, int32_t *queueLen)
{
	ElemTypeCan _elemType0;
	int ret = deQueueCan(&str_canbusQueue0, &_elemType0);
	if(ret == FALSE){
		return -1;
	}
	*id = _elemType0.canbusId;
	memcpy(rData, _elemType0.canbusData, sizeof(_elemType0.canbusData));
	*queueLen = getQueueLenCan(&str_canbusQueue0);
	return 0;
}

