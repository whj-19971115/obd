/*
 * uart.c
 *
 *  Created on: 2019Äê5ÔÂ14ÈÕ
 *      Author: yxl
 */
#include "timer.h"
#include "device_registers.h" /* include peripheral declarations S32K144 */
#include "usertype.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define TIMER_MAX	10
#define TIMER_OUT	1
static int timerNum = 0, timerMax = 0, heartbeat = 0;
static int timer[TIMER_MAX] = {0}, timerFlag[TIMER_MAX] = {0};
static int timeInitFlag = 0;

void LPTMR_init(void)//2ms
{
	// 64
    PCC->PCCn[PCC_LPTMR0_INDEX] = PCC_PCCn_CGC_MASK;	/* Enable LPTMR Clock 		*/
    LPTMR0->PSR |= LPTMR_PSR_PCS(1)                  /* LPTMR clk src: 1KHz LPO  0b01*/
                  |LPTMR_PSR_PBYP_MASK;               	/* Bypass Prescaler 		*/
    LPTMR0->CMR = 1;//time_ms - 1;                                  /* 500 ms interrupt 		*/
    LPTMR0->CSR |= LPTMR_CSR_TIE_MASK; 					/* Timer interrupt enabled */
    LPTMR0->CSR |= LPTMR_CSR_TEN_MASK;                  /* Enable Timer 			*/
	
		S32_NVIC->ISER[(uint32_t)(LPTMR0_IRQn) >> 5U] = (uint32_t)(1U << ((uint32_t)(LPTMR0_IRQn) & (uint32_t)0x1FU));
		S32_NVIC->ICPR[(uint32_t)(LPTMR0_IRQn) >> 5U] = (uint32_t)(1U << ((uint32_t)(LPTMR0_IRQn) & (uint32_t)0x1FU));
		timeInitFlag = 1;
}

void LPTMR_noIrq_init(void)//2ms
{
	// 64
    PCC->PCCn[PCC_LPTMR0_INDEX] = PCC_PCCn_CGC_MASK;	/* Enable LPTMR Clock 		*/
    LPTMR0->PSR |= LPTMR_PSR_PCS(1)                  /* LPTMR clk src: 1KHz LPO  0b01*/
                  |LPTMR_PSR_PBYP_MASK;               	/* Bypass Prescaler 		*/
    LPTMR0->CMR = 24;//time_ms - 1;                                  /* 500 ms interrupt 		*/
    LPTMR0->CSR |= LPTMR_CSR_TIE_MASK; 					/* Timer interrupt enabled */
    LPTMR0->CSR |= LPTMR_CSR_TEN_MASK;                  /* Enable Timer 			*/
}

int LPTMR_COUNT()
{
	int ret = 0;
	if(0!=(LPTMR0->CSR & LPTMR_CSR_TCF_MASK))
	{
		//modeTemp = SMC->PMSTAT;
		/* Check if TCF flag is set */
		LPTMR0->CSR |= LPTMR_CSR_TCF_MASK;
		ret = 1;
	}
	return ret;
}

void LPTMR_Reset()
{

		//PCC->PCCn[PCC_LPTMR0_INDEX] = ~PCC_PCCn_CGC_MASK;	/* Enable LPTMR Clock 		*/ 
    LPTMR0->PSR |= ~LPTMR_PSR_PCS(1)                  /* LPTMR clk src: 1KHz LPO  0b01*/
                  |~LPTMR_PSR_PBYP_MASK;               	/* Bypass Prescaler 		*/	/* 500 ms interrupt 		*/
    LPTMR0->CSR |= ~LPTMR_CSR_TIE_MASK; 					/* Timer interrupt enabled */
    LPTMR0->CSR |= ~LPTMR_CSR_TEN_MASK;                  /* Enable Timer 			*/
		//S32_NVIC->ICER[(uint32_t)(LPTMR0_IRQn) >> 5U] = (uint32_t)(1U << ((uint32_t)(LPTMR0_IRQn) & (uint32_t)0x1FU));
		timeInitFlag = 0;               /* disable Timer 			*/
			S32_NVIC->ICER[(uint32_t)(LPTMR0_IRQn) >> 5U] = (uint32_t)(1U << ((uint32_t)(LPTMR0_IRQn) & (uint32_t)0x1FU));
}

int  TimerCreate(unsigned int time_2ms){
	int i = 0, temp = 0;
	if(timeInitFlag != 1)
		return -3;
	for(i=0; i < timerNum; i++){
		if(timer[i] == time_2ms)
			return -2;
	}
	if(timerNum < TIMER_MAX)
		timer[timerNum++] = time_2ms;
	else
		return -1;
	for(i=0; i< timerNum; i++){
		if(i>0){
			if(timer[i] > timer[i-1] && timer[i] > temp){
				temp = timer[i];
				timerMax = i;
			}
		}
	}
	return 0;
}

int  TimerDelete(unsigned int time_2ms){
	int i = 0;
	for(i=0; i< timerNum; i++){
		if(timer[i] == time_2ms){
			memcpy(timer+i, timer+i+1, sizeof(int)*(timerNum - i - 1));
			memcpy(timerFlag+i, timerFlag+i+1, sizeof(int)*(timerNum - i - 1));
			timerNum = timerNum - 1;
			return 0;
		}
	}
	return -1;
}
int TimerOutGet(unsigned int time_2ms)
{
	int ret = -1, i = 0;
	for(i=0; i < timerNum; i++){
		if(timer[i] == time_2ms){
			ret = timerFlag[i];
			timerFlag[i] = 0;
			return ret;
		}
	}
	return ret;
}
int modeTemp = 0;
void LPTMR0_IRQHandler (void)
{
	if(0!=(LPTMR0->CSR & LPTMR_CSR_TCF_MASK))
	{
		modeTemp = SMC->PMSTAT;
		/* Check if TCF flag is set */
		LPTMR0->CSR |= LPTMR_CSR_TCF_MASK;	/*	Clear TCF flag by writting a logic one */
		heartbeat++;
		int i = 0;
		for(i=0; i< timerNum; i++){
			if(heartbeat == timer[i]){
				timerFlag[i] = TIMER_OUT;
			}
		}
		if(heartbeat == timer[timerMax])
			heartbeat = 0;
	}
}

//-------------------------------------
static int timerLpitNum = 0, timerLpitMax = 0, heartbeatLpit = 0;
static int timerLpit[TIMER_MAX] = {0}, timerLpitFlag[TIMER_MAX] = {0};
static int timeLpitInitFlag = 0;
//--------------------------------------
//if want to change the timer value, change TVAL
void LPIT0_init (void) // 1ms
{
	PCC->PCCn[PCC_LPIT_INDEX] = PCC_PCCn_PCS(6);    // LPIT0?? Clock Src = 6 (SPLL2_DIV2_CLK=160MHZ/4 = 40MHZ)
	PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC_MASK; //??LPIT0??
	LPIT0->MCR = 0x00000001;    // DBG_EN-0: ??????????????
                                // DOZE_EN=0: ?????????????? 
                                // SW_RST=0: ????????????
                                // M_CEN=1: ?????????
	LPIT0->MIER = 0x00000001;   // TIE0=1: ???LPIT ??0????
	LPIT0->TMR[0].TVAL = 40000;     // ???????: 4M clocks
	LPIT0->TMR[0].TCTRL = 0x00000001; // T_EN=1: ?????
	
	S32_NVIC->ISER[(uint32_t)(LPIT0_Ch0_IRQn) >> 5U] = (uint32_t)(1U << ((uint32_t)(LPIT0_Ch0_IRQn) & (uint32_t)0x1FU));
  S32_NVIC->ISER[(uint32_t)(LPIT0_Ch0_IRQn) >> 5U] = (uint32_t)(1UL << ((uint32_t)(LPIT0_Ch0_IRQn) & (uint32_t)0x1FU));
  S32_NVIC->IP[LPIT0_Ch0_IRQn] = 0x8;              /* IRQ48-LPIT0 ch0: priority 8 of 0-15*/
	timeInitFlag = 1;
}

int  TimerLpitCreate(unsigned int time_ms){
	int i = 0, temp = 0;
	if(timeLpitInitFlag != 1)
		return -3;
	for(i=0; i < timerLpitNum; i++){
		if(timerLpit[i] == time_ms)
			return -2;
	}
	if(timerLpitNum < TIMER_MAX)
		timerLpit[timerLpitNum++] = time_ms;
	else
		return -1;
	for(i=0; i< timerLpitNum; i++){
		if(i>0){
			if(timerLpit[i] > timerLpit[i-1] && timerLpit[i] > temp){
				temp = timerLpit[i];
				timerLpitMax = i;
			}
		}
	}
	return 0;
}

int  TimerLpitDelete(unsigned int time_ms){
	int i = 0;
	for(i=0; i< timerLpitNum; i++){
		if(timerLpit[i] == time_ms){
			memcpy(timerLpit+i, timerLpit+i+1, sizeof(int)*(timerLpitNum - i - 1));
			memcpy(timerLpitFlag+i, timerLpitFlag+i+1, sizeof(int)*(timerLpitNum - i - 1));
			timerLpitNum = timerLpitNum - 1;
			return 0;
		}
	}
	return -1;
}
int TimerLpitOutGet(unsigned int time_ms)
{
	int ret = -1, i = 0;
	for(i=0; i < timerLpitNum; i++){
		if(timerLpit[i] == time_ms){
			ret = timerLpitFlag[i];
			timerLpitFlag[i] = 0;
			return ret;
		}
	}
	return ret;
}

void LPIT0_Ch0_IRQHandler (void)
{
	if(0!=(LPIT0->MSR & LPIT_MSR_TIF0_MASK)){
		LPIT0->MSR |= LPIT_MSR_TIF0_MASK; /* ??????*/
		heartbeatLpit++;
		int i = 0;
		for(i=0; i< timerLpitNum; i++){
			if(heartbeatLpit == timerLpit[i]){
				timerLpitFlag[i] = TIMER_OUT;
			}
		}
		if(heartbeatLpit == timerLpit[timerMax])
			heartbeatLpit = 0;
	}
}

void LPIT0_noIrq_init (void)//10ms
{
	/*!
	 * LPIT Clocking:
	 * ==============================
	 */
  PCC->PCCn[PCC_LPIT_INDEX] = PCC_PCCn_PCS(6);    /* Clock Src = 6 (SPLL2_DIV2_CLK)*/
  PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC_MASK; /* Enable clk to LPIT0 regs 		*/

  /*!
   * LPIT Initialization:
   */
  LPIT0->MCR |= LPIT_MCR_M_CEN_MASK;  /* DBG_EN-0: Timer chans stop in Debug mode */
                              	  	  /* DOZE_EN=0: Timer chans are stopped in DOZE mode */
                              	  	  /* SW_RST=0: SW reset does not reset timer chans, regs */
                              	  	  /* M_CEN=1: enable module clk (allows writing other LPIT0 regs) */
	LPIT0->TMR[0].TVAL = 400000 * 5; /* Chan 0 Timeout period: 40M clocks */

  LPIT0->TMR[0].TCTRL |= LPIT_TMR_TCTRL_T_EN_MASK;
  	  	  	  	  	  	  	  /* T_EN=1: Timer channel is enabled */
                              /* CHAIN=0: channel chaining is disabled */
                              /* MODE=0: 32 periodic counter mode */
                              /* TSOT=0: Timer decrements immediately based on restart */
                              /* TSOI=0: Timer does not stop after timeout */
                              /* TROT=0 Timer will not reload on trigger */
                              /* TRG_SRC=0: External trigger soruce */
                              /* TRG_SEL=0: Timer chan 0 trigger source is selected*/
}

void LPIT0_noIrq_uninit (void){
  //PCC->PCCn[PCC_LPIT_INDEX] = PCC_PCCn_PCS(6);    /* Clock Src = 6 (SPLL2_DIV2_CLK)*/
  //PCC->PCCn[PCC_LPIT_INDEX] &= ~PCC_PCCn_CGC_MASK; /* Enable clk to LPIT0 regs   */

  /*!
   * LPIT Initialization:
   */
  //LPIT0->MCR |= (1 << 0);  /* DBG_EN-0: Timer chans stop in Debug mode */
                                    /* DOZE_EN=0: Timer chans are stopped in DOZE mode */
                                    /* SW_RST=0: SW reset does not reset timer chans, regs */
                                    /* M_CEN=1: enable module clk (allows writing other LPIT0 regs) */
	
	LPIT0->TMR[0].TVAL = 0;
  LPIT0->TMR[0].TCTRL |= ~LPIT_TMR_TCTRL_T_EN_MASK;
	//LPIT0->MCR &= ~(1 << 0);
  LPIT0->CLRTEN |= 1;

  S32_NVIC->ICER[(uint32_t)(LPIT0_Ch0_IRQn) >> 5U] = (uint32_t)(1U << ((uint32_t)(LPIT0_Ch0_IRQn) & (uint32_t)0x1FU));
}

int LPIT0_COUNT()
{
  	int ret = 0;// = LPIT0->TMR[0].TVAL;
//	unsigned int count = LPIT0->TMR[0].CVAL;
//	count = count;
	if (1 == (LPIT0->MSR & LPIT_MSR_TIF0_MASK)) { /* Wait for LPIT0 CH0 Flag */
		LPIT0->MSR |= LPIT_MSR_TIF0_MASK; /* Clear LPIT0 timer flag 0 */
		ret = 1;
	}
	return ret;
}