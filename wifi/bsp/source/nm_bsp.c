/**
 *
 * \file
 *
 * \brief This module contains SAM4S BSP APIs implementation.
 *
 * Copyright (c) 2016-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"
#include "conf_winc.h"
#include "stdint.h"
#include "stdio.h"
#include "stdlib.h"
#include "gpio.h"
#include "usertype.h"
#include "timer.h"
#include "spi.h"
#include "s32k142.h"
#include "conf_winc.h"
#include "nmspi.h"
#include "driver/source/m2m_hif.h"
static tpfNmBspIsr gpfIsr;
#ifndef IRQ_PORTCATE
#define EN_PORTCATE		PORT_C
#define EN_PORTNUM		16

#define RESET_PORTCATE		PORT_C
#define RESET_PORTNUM		17

#define CE_PORTCATE		PORT_B
#define CE_PORTNUM		5

#define WAKE_PORTCATE		PORT_B
#define WAKE_PORTNUM		0

#define IRQ_PORTCATE PORT_B
#define IRQ_PORTNUM	1
#endif

static void chip_isr(void)
{
	#if 0
	if (gpfIsr) {
		gpfIsr();
	}
	#endif
}

/**
 *	@fn		nm_bsp_reset
 *	@brief	Reset NMC1500 SoC by setting CHIP_EN and RESET_N signals low,
 *           CHIP_EN high then RESET_N high
 */
void nm_bsp_reset(void)
{
	GpioSetVal(EN_PORTCATE, EN_PORTNUM, PORT_LOW);//pio_set_pin_low(CONF_WINC_PIN_CHIP_ENABLE);
	GpioSetVal(RESET_PORTCATE, RESET_PORTNUM, PORT_LOW);
	nm_bsp_sleep(2);//1
	GpioSetVal(EN_PORTCATE, EN_PORTNUM, PORT_HIGH);
	nm_bsp_sleep(10);//5
	GpioSetVal(RESET_PORTCATE, RESET_PORTNUM, PORT_HIGH);
	nm_bsp_sleep(15);
}


/*
 *	@fn		init_chip_pins
 *	@brief	Initialize reset, chip enable and wake pin
 */
static void init_chip_pins(void)
{
#ifdef __SAM4SD32C__
	pio_configure_pin(CONF_WINC_PIN_RESET, PIO_TYPE_PIO_OUTPUT_0|PIO_PULLUP);
	pio_configure_pin(CONF_WINC_PIN_CHIP_ENABLE, PIO_TYPE_PIO_OUTPUT_0|PIO_PULLUP);
	pio_configure_pin(CONF_WINC_PIN_WAKE, PIO_TYPE_PIO_OUTPUT_0|PIO_PULLUP);
	pio_configure_pin(CONF_WINC_SPI_CS_GPIO, PIO_DEFAULT|PIO_PULLUP);
	pio_set_pin_high(CONF_WINC_SPI_CS_GPIO);
#else
	GpioInit();
	GpioSetDirection(EN_PORTCATE, EN_PORTNUM, PORT_OUT);
	GpioSetDirection(RESET_PORTCATE, RESET_PORTNUM, PORT_OUT);
	GpioSetDirection(WAKE_PORTCATE, WAKE_PORTNUM, PORT_OUT);
  GpioSetDirection(IRQ_PORTCATE, IRQ_PORTNUM, PORT_IN);	
	GpioSetVal(WAKE_PORTCATE, WAKE_PORTNUM, PORT_HIGH);
#endif
}

/*
*	@fn		nm_bsp_init
*	@brief	Initialize BSP
*	@return	0 in case of success and -1 in case of failure
*/
sint8 nm_bsp_init(void)
{
	gpfIsr = NULL;

	/* Initialize chip IOs. */
	init_chip_pins();

    /* Make sure a 1ms Systick is configured. */
//    if (!(SysTick->CTRL & SysTick_CTRL_ENABLE_Msk && SysTick->CTRL & SysTick_CTRL_TICKINT_Msk)) {
//	    delay_init();
//    }
		LPIT0_init();
	/* Perform chip reset. */
	nm_bsp_reset();

	return 0;
}

/**
*   @fn      nm_bsp_deinit
*   @brief   De-iInitialize BSP
*   @return  0 in case of success and -1 in case of failure
*/
sint8 nm_bsp_deinit(void)
{
	GpioSetVal(EN_PORTCATE, EN_PORTNUM, PORT_LOW);//pio_set_pin_low(CONF_WINC_PIN_CHIP_ENABLE);
	GpioSetVal(RESET_PORTCATE, RESET_PORTNUM, PORT_LOW);
	return M2M_SUCCESS;
}

/*
*	@fn		nm_bsp_sleep
*	@brief	Sleep in units of mSec
*	@param[IN]	u32TimeMsec
*				Time in milliseconds
*/
void nm_bsp_sleep(uint32 u32TimeMsec)
{
	int ms = 8000 * u32TimeMsec;
	while (ms--){
	}
	
}

/*
*	@fn		nm_bsp_register_isr
*	@brief	Register interrupt service routine
*	@param[IN]	pfIsr
*				Pointer to ISR handler
*/


/*
*	@fn		nm_bsp_interrupt_ctrl
*	@brief	Enable/Disable interrupts
*	@param[IN]	u8Enable
*				'0' disable interrupts. '1' enable interrupts
*/
void m2mStub_EintEnable(void){
	PORTB->PCR[1] = 0x000a0100; 
	S32_NVIC->ICPR[(PORTB_IRQn) >> 5] = 1 << ((PORTB_IRQn ) % 32);  /* clr any pending IRQ*/
	S32_NVIC->ISER[(PORTB_IRQn) >> 5] = 1 << ((PORTB_IRQn) % 32);  /* enable IRQ */
	S32_NVIC->IP[PORTB_IRQn] = 0xA;              /* priority 10 of 0-15*/
}
void m2mStub_EintDisable(void){
		S32_NVIC->ICER[(PORTB_IRQn) >> 5] = 1 << ((PORTB_IRQn) % 32);  /* disable IRQ */
}
void nm_bsp_register_isr(tpfNmBspIsr pfIsr)
{
		gpfIsr = pfIsr;
	m2mStub_EintEnable();
	#if 0


	ext_irq_register(CONF_WINC_EXT_INT_PIN, chip_isr);
	#endif
}
void nm_bsp_interrupt_ctrl(uint8 u8Enable)
{
	if(u8Enable == 1){
		m2mStub_EintEnable();
	}
	else if(u8Enable == 0){
		m2mStub_EintDisable();
	}
}
void PORTB_IRQHandler(void) 
{
	if((PTB->PDIR & (1<<1))==0x00)
	{
		m2m_EintHandler();
	}
	PORTB->ISFR |= 0x4000;
}
