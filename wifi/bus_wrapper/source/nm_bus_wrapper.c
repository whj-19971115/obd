/**
 *
 * \file
 *
 * \brief This module contains NMC1000 bus wrapper APIs implementation.
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

#include <stdio.h>
#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"
#include "bus_wrapper/include/nm_bus_wrapper.h"
#include "conf_winc.h"
#include 	<stdint.h>
#include "gpio.h"
#include "spi.h"
#include "usertype.h"
#include <stdlib.h>
#include <string.h>

#define printf nxpPrintf

#define xSPI_ASSERT_CS()				{GpioSetVal(PORT_B,4,0);}
#define xSPI_DEASSERT_CS()				{GpioSetVal(PORT_B,4,1);}
	
#define SPI_ASSERT_CS()					{GpioSetVal(PORT_B,4,0);}
#define SPI_DEASSERT_CS()				{GpioSetVal(PORT_B,4,1);}
#ifndef IRQ_PORTCATE

#endif

static tpfNmBspIsr gpfIsr;
#ifdef CONF_WINC_USE_SPI
/** Pointer to Pdc data structure. */
//static Pdc *g_p_spim_pdc;
#endif

#define NM_BUS_MAX_TRX_SZ	4096

tstrNmBusCapabilities egstrNmBusCapabilities =
{
	NM_BUS_MAX_TRX_SZ
};

#ifdef CONF_WINC_USE_I2C
#define SLAVE_ADDRESS 0x60

/** Number of times to try to send packet if failed. */
#define I2C_TIMEOUT 100

static sint8 nm_i2c_write(uint8 *b, uint16 sz)
{
	sint8 result = M2M_ERR_BUS_FAIL;
	return result;
}

static sint8 nm_i2c_read(uint8 *rb, uint16 sz)
{
	sint8 result = M2M_ERR_BUS_FAIL;
	return result;
}

static sint8 nm_i2c_write_special(uint8 *wb1, uint16 sz1, uint8 *wb2, uint16 sz2)
{
	static uint8 tmp[NM_BUS_MAX_TRX_SZ];
	m2m_memcpy(tmp, wb1, sz1);
	m2m_memcpy(&tmp[sz1], wb2, sz2);
	return nm_i2c_write(tmp, sz1+sz2);
}
#endif

#ifdef CONF_WINC_USE_SPI
/** PIO instance used by CS. */
//Pio *p_pio;

extern uint32_t g_winc_logstart = 1;
 uint8_t spi_txbuf[1024];
uint8 pu8Mosibuf[100]={0};
uint8 pu8Misobuf[100]={0};
static sint8 spi_rw(uint8* pu8Mosi, uint8* pu8Miso, uint16 u16Sz)
{
	int b = u16Sz;
	int c = 0;
while(b){
	pu8Mosibuf[c] = *(pu8Mosi+c);
	c++;
	b--;
}
	
#ifdef DMA_SPI
	pdc_packet_t pdc_spi_tx_packet,pdc_spi_rx_packet;

	p_pio = (Pio *)((uint32_t)PIOA + (PIO_DELTA * (CONF_WINC_SPI_CS_GPIO >> 5)));

	pdc_spi_tx_packet.ul_addr = NULL;
	pdc_spi_rx_packet.ul_addr = NULL;
	pdc_spi_tx_packet.ul_size = u16Sz;
	pdc_spi_rx_packet.ul_size = u16Sz;
		
	if (pu8Mosi) {
		pdc_spi_tx_packet.ul_addr = (uint32_t)pu8Mosi;
	}

	if(pu8Miso) {
		pdc_spi_rx_packet.ul_addr = (uint32_t)pu8Miso;
	}

	pdc_tx_init(g_p_spim_pdc, &pdc_spi_tx_packet, NULL);
	pdc_rx_init(g_p_spim_pdc, &pdc_spi_rx_packet, NULL);

	/*Assert CS*/
	SPI_ASSERT_CS();
	/* Enable the RX and TX PDC transfer requests */
	g_p_spim_pdc->PERIPH_PTCR = PERIPH_PTCR_RXTEN|PERIPH_PTCR_TXTEN;

	/* Waiting transfer done*/
	while((CONF_WINC_SPI->SPI_SR & SPI_SR_RXBUFF) == 0);

	/*DEASSERT CS*/
	SPI_DEASSERT_CS();
	
	/* Disable the RX and TX PDC transfer requests */
	g_p_spim_pdc->PERIPH_PTCR = PERIPH_PTCR_TXTDIS|PERIPH_PTCR_RXTDIS;
	
	
	return M2M_SUCCESS;
		
#elif (defined LOW_DELAY)


uint8 u8Dummy = 0;
uint8 u8SkipMosi = 0, u8SkipMiso = 0;
uint8_t uc_pcs = 0;

p_pio = (Pio *)((uint32_t)PIOA + (PIO_DELTA * (CONF_WINC_SPI_CS_GPIO >> 5)));

if (!pu8Mosi) {
	pu8Mosi = &u8Dummy;
	u8SkipMosi = 1;
}
else if(!pu8Miso) {
	/*RX must be zero*/
	pu8Miso = &u8Dummy;
	u8SkipMiso = 1;
}
else {
	return M2M_ERR_BUS_FAIL;
}

/*TX path*/
if(!u8SkipMosi)
{
	SPI_ASSERT_CS();
	while (u16Sz--)
	{
		CONF_WINC_SPI->SPI_TDR = SPI_TDR_TD((uint16)*pu8Mosi);
		while (!(CONF_WINC_SPI->SPI_SR & SPI_SR_TDRE));
		pu8Mosi++;
	}
	SPI_DEASSERT_CS();
}
/*RX path*/
if(!u8SkipMiso)
{
	uc_pcs = 0;
	SPI_ASSERT_CS();
	while (u16Sz--)
	{
		
		CONF_WINC_SPI->SPI_TDR = SPI_TDR_TD((uint16)*pu8Mosi);
		while (!(CONF_WINC_SPI->SPI_SR & SPI_SR_TDRE));
		while (!(CONF_WINC_SPI->SPI_SR & (SPI_SR_RDRF)));
		*pu8Miso = (uint16_t) ((CONF_WINC_SPI->SPI_RDR) & SPI_RDR_RD_Msk);
		pu8Miso++;
	}
	SPI_DEASSERT_CS();
}
return M2M_SUCCESS;

#else
 uint8_t spi_rxbuf[1024];
	uint8 u8Dummy    = 0;
	uint8 u8SkipMosi = 0, u8SkipMiso = 0;
	uint16_t txd_data = 0;
	uint16_t rxd_data = 0;
	uint8_t uc_pcs = 0;
	volatile uint32_t dat_idx = 0;

	if (!pu8Mosi) {
		pu8Mosi    = &u8Dummy;
		u8SkipMosi = 1;
	} else if (!pu8Miso) {
		pu8Miso    = &u8Dummy;
		u8SkipMiso = 1;
	} else {
		return M2M_ERR_BUS_FAIL;
	}

int dat_idxtx = 0;
int dat_idxrx = 0;
	GpioSetVal(PORT_B, 5, 0);
	if (!u8SkipMiso) {
		dat_idxrx = SpiRecv(pu8Miso,u16Sz,10);
//		io_read(io, pu8Miso, u16Sz);
	}
	if (!u8SkipMosi) {
	dat_idxtx = SpiSend(pu8Mosi,u16Sz,10);
//		io_write(io, pu8Mosi, u16Sz);
	}
	if (g_winc_logstart) {
		printf("spi tx package =%d\r\n",dat_idxtx);
	
		for (uint32_t i = 0; i < dat_idxtx; i++) {
			printf("0x%02X ", pu8Mosi[i]);
		}
		printf("\r\n");
	
		printf("spi rx package =%d\r\n",dat_idxrx);
	
		for (uint32_t j = 0; j < dat_idxrx; j++) {
			printf("0x%02X ", pu8Miso[j]);
		}
		printf("\r\n");
	}	
	GpioSetVal(PORT_B, 5, 1);
	
//	if (g_winc_logstart) {
//		printf("spi tx package =%d\r\n",dat_idx);
//	
//		for (uint32_t i = 0; i < dat_idx; i++) {
//			printf("0x%02X ", pu8Mosi[i]);
//		}
//		printf("\r\n");
//	
//		printf("spi rx package =%d\r\n",dat_idx);
//	
//		for (uint32_t j = 0; j < dat_idx; j++) {
//			printf("0x%02X ", spi_rxbuf[j]);
//		}
//		printf("\r\n");
//	}



	return M2M_SUCCESS;
	#endif

}
#endif

/*
*	@fn		nm_bus_init
*	@brief	Initialize the bus wrapper
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_bus_init(void *pvinit)
{
	sint8 result = M2M_SUCCESS;
#if 0

	spi_m_sync_get_io_descriptor(spi_instance, &io);
	spi_m_sync_enable(spi_instance);

	nm_bsp_reset();
	nm_bsp_sleep(1);
#endif
	return result;
}

/*
*	@fn		nm_bus_ioctl
*	@brief	send/receive from the bus
*	@param[IN]	u8Cmd
*					IOCTL command for the operation
*	@param[IN]	pvParameter
*					Arbitrary parameter depending on IOCTL
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@note	For SPI only, it's important to be able to send/receive at the same time
*/
sint8 nm_bus_ioctl(uint8 u8Cmd, void* pvParameter)
{
	sint8 s8Ret = 0;
	switch(u8Cmd)
	{
#ifdef CONF_WINC_USE_I2C
		case NM_BUS_IOCTL_R: {
			tstrNmI2cDefault *pstrParam = (tstrNmI2cDefault *)pvParameter;
			s8Ret = nm_i2c_read(pstrParam->pu8Buf, pstrParam->u16Sz);
		}
		break;
		case NM_BUS_IOCTL_W: {
			tstrNmI2cDefault *pstrParam = (tstrNmI2cDefault *)pvParameter;
			s8Ret = nm_i2c_write(pstrParam->pu8Buf, pstrParam->u16Sz);
		}
		break;
		case NM_BUS_IOCTL_W_SPECIAL: {
			tstrNmI2cSpecial *pstrParam = (tstrNmI2cSpecial *)pvParameter;
			s8Ret = nm_i2c_write_special(pstrParam->pu8Buf1, pstrParam->u16Sz1, pstrParam->pu8Buf2, pstrParam->u16Sz2);
		}
		break;
#elif defined CONF_WINC_USE_SPI
		case NM_BUS_IOCTL_RW: {
			tstrNmSpiRw *pstrParam = (tstrNmSpiRw *)pvParameter;
			s8Ret = spi_rw(pstrParam->pu8InBuf, pstrParam->pu8OutBuf, pstrParam->u16Sz);
		}
		break;
#endif
		default:
			s8Ret = -1;
		M2M_ERR("invalid ioclt cmd\n");
			break;
	}

	return s8Ret;
}

/*
*	@fn		nm_bus_deinit
*	@brief	De-initialize the bus wrapper
*/
sint8 nm_bus_deinit(void)
{
	return M2M_SUCCESS;
}
