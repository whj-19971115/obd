/*
 * gpio.c
 *
 *  Created on: 2019Äê5ÔÂ14ÈÕ
 *      Author: Administrator
 */
#include "device_registers.h" /* include peripheral declarations S32K144 */
#include "stdint.h"
#include "usertype.h"


void GpioInit()
{
	PCC->PCCn[PCC_PORTA_INDEX] |= PCC_PCCn_CGC_MASK;	/* Enable clock for PORTE */
	PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK;
	PCC->PCCn[PCC_PORTC_INDEX] |= PCC_PCCn_CGC_MASK;
	PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK;
	PCC->PCCn[PCC_PORTE_INDEX] |= PCC_PCCn_CGC_MASK;
}

/*
 * portCate: PORTA~PORTE
 * portNum
 * dir: 0--in   1--out
 * */
void GpioSetDirection(uint8_t portCate, uint8_t portNum, uint8_t dir)
{
	switch(portCate){
	case PORT_A:
		PORTA->PCR[portNum] = PORT_PCR_MUX(1);/* Port : MUX = GPIO 				*/
		if(dir == PORT_OUT)
			PTA->PDDR |= 1<<portNum;            	 /* Port: Data direction  */
		else if(dir == PORT_IN)
			PTA->PDDR &= ~(1<<portNum);
		break;
	case PORT_B:
		PORTB->PCR[portNum] = PORT_PCR_MUX(1);
		if(dir == PORT_OUT)
			PTB->PDDR |= 1<<portNum;
		else if(dir == PORT_IN)
			PTB->PDDR &= ~(1<<portNum);
		break;
	case PORT_C:
		PORTC->PCR[portNum] = PORT_PCR_MUX(1);
		if(dir == PORT_OUT)
			PTC->PDDR |= 1<<portNum;
		else if(dir == PORT_IN)
			PTC->PDDR &= ~(1<<portNum);
		break;
	case PORT_D:
		PORTD->PCR[portNum] = PORT_PCR_MUX(1);
		if(dir == PORT_OUT)
			PTD->PDDR |= 1<<portNum;
		else if(dir == PORT_IN)
			PTD->PDDR &= ~(1<<portNum);
		break;
	case PORT_E:
		PORTE->PCR[portNum] = PORT_PCR_MUX(1);
		if(dir == PORT_OUT)
			PTE->PDDR |= 1<<portNum;
		else if(dir == PORT_IN)
			PTE->PDDR &= ~(1<<portNum);
		break;
	default:
		break;
	}
}

void GpioSetVal(uint8_t portCate, uint8_t portNum, uint8_t portVal)
{
	switch(portCate){
		case PORT_A:
			if(portVal == 1)
				PTA->PSOR |= 1<<portNum;
			else if(portVal == 0)
				PTA->PCOR |= 1<<portNum;
			break;
		case PORT_B:
			if(portVal == 1)
				PTB->PSOR |= 1<<portNum;
			else if(portVal == 0)
				PTB->PCOR |= 1<<portNum;
			break;
		case PORT_C:
			if(portVal == 1)
				PTC->PSOR |= 1<<portNum;
			else if(portVal == 0)
				PTC->PCOR |= 1<<portNum;
			break;
		case PORT_D:
			if(portVal == 1)
				PTD->PSOR |= 1<<portNum;
			else if(portVal == 0)
				PTD->PCOR |= 1<<portNum;
			break;
		case PORT_E:
			if(portVal == 1)
				PTE->PSOR |= 1<<portNum;
			else if(portVal == 0)
				PTE->PCOR |= 1<<portNum;
			break;
		default:
			break;
		}
}

void GpioGetVal(uint8_t portCate, uint8_t portNum, uint8_t *portVal)
{
	switch(portCate){
	case PORT_A:
		if(((PTA->PDDR >> portNum) & 0X01) == PORT_OUT)
			*portVal = (PTA->PDOR >> portNum) & 0X01;
		else
			*portVal = (PTA->PDIR >> portNum) & 0X01;
		break;
	case PORT_B:
		if(((PTB->PDDR >> portNum) & 0X01) == PORT_OUT)
			*portVal = (PTB->PDOR >> portNum) & 0X01;
		else
			*portVal = (PTB->PDIR >> portNum) & 0X01;
		break;
	case PORT_C:
		if(((PTC->PDDR >> portNum) & 0X01) == PORT_OUT)
			*portVal = (PTC->PDOR >> portNum) & 0X01;
		else
			*portVal = (PTC->PDIR >> portNum) & 0X01;
		break;
	case PORT_D:
		if(((PTD->PDDR >> portNum) & 0X01) == PORT_OUT)
			*portVal = (PTD->PDOR >> portNum) & 0X01;
		else
			*portVal = (PTD->PDIR >> portNum) & 0X01;
		break;
	case PORT_E:
		if(((PTE->PDDR >> portNum) & 0X01) == PORT_OUT)
			*portVal = (PTE->PDOR >> portNum) & 0X01;
		else
			*portVal = (PTE->PDIR >> portNum) & 0X01;
		break;
	default:
		break;
	}
}
