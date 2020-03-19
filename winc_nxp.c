#include "winc1500_api.h"
#include "device_registers.h"
#include "stdio.h"
#include "stdlib.h"
#include "gpio.h"
#include "usertype.h"
#include "timer.h"
#include "spi.h"

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

void m2mStub_Pin_init(void){
	GpioSetDirection(EN_PORTCATE, EN_PORTNUM, PORT_OUT);
	GpioSetDirection(RESET_PORTCATE, RESET_PORTNUM, PORT_OUT);
	//GpioSetDirection(WAKE_PORTCATE, WAKE_PORTNUM, PORT_OUT);
  GpioSetDirection(IRQ_PORTCATE, IRQ_PORTNUM, PORT_IN);	
}

void m2mStub_time_init(void){
	LPIT0_init();
}
void nm_bsp_sleep(int ms){
	int timeOut = 8000*ms;
	while(timeOut--);
}
/**
 *	@fn		nm_bsp_reset
 *	@brief	Reset NMC1500 SoC by setting CHIP_EN and RESET_N signals low,
 *           CHIP_EN high then RESET_N high
 */

void delayCeshi(){
	GpioSetVal(RESET_PORTCATE, RESET_PORTNUM, PORT_LOW);
	nm_bsp_sleep(10);
	GpioSetVal(RESET_PORTCATE, RESET_PORTNUM, PORT_HIGH);
		nm_bsp_sleep(10);

}
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

/*
*	@fn		nm_bsp_init
*	@brief	Initialize BSP
*	@return	0 in case of success and -1 in case of failure
*/
int nm_bsp_init(void)
{
	/* Initialize chip IOs. */
	m2mStub_Pin_init();
//    /* Make sure a 1ms Systick is configured. */
//    if (!(SysTick->CTRL & SysTick_CTRL_ENABLE_Msk && SysTick->CTRL & SysTick_CTRL_TICKINT_Msk)) {
//	    delay_init();
//    }

	/* Perform chip reset. */
	nm_bsp_reset();

	return 0;
}
/*******************************************************************************
  Function:
    void m2mStub_PinSet_CE(t_m2mWifiPinAction action)

  Summary:
    Sets WINC1500 CHIP_EN pin high or low.

  Description:
    The WINC1500 driver will call this function to set the WINC1500 CHIP_EN pin
    high or low.  This is a host output GPIO.

  Parameters:
    action -- M2M_WIFI_PIN_LOW or M2M_WIFI_PIN_HIGH

  Returns:
    None
 *****************************************************************************/
void m2mStub_PinSet_CE(t_m2mWifiPinAction action){
		GpioSetVal(EN_PORTCATE, EN_PORTNUM, action);
}

/*******************************************************************************
  Function:
    void m2mStub_PinSet_RESET(t_m2mWifiPinAction action)

  Summary:
    Sets WINC1500 RESET_N pin high or low.

  Description:
    The WINC1500 driver will call this function to set the WINC1500 RESET_N pin
    high or low.  This is a host output GPIO.

  Parameters:
    action -- M2M_WIFI_PIN_LOW or M2M_WIFI_PIN_HIGH

  Returns:
    None
 *****************************************************************************/
void m2mStub_PinSet_RESET(t_m2mWifiPinAction action){
		GpioSetVal(RESET_PORTCATE, RESET_PORTNUM, action);
}

/*******************************************************************************
  Function:
    void m2mStub_PinSet_SPI_SS(t_m2mWifiPinAction action)

  Summary:
    Sets WINC1500 SPI_SSN pin high or low.

  Description:
    The WINC1500 driver will call this function to set the WINC1500 SPI_SSN pin
    high or low.  This is a host output GPIO.

  Parameters:
    action -- M2M_WIFI_PIN_LOW or M2M_WIFI_PIN_HIGH

  Returns:
    None
 *****************************************************************************/
void m2mStub_PinSet_SPI_SS(t_m2mWifiPinAction action){
	//SS IS AUTO HIGH AND LOW, THIE method is not use here
		//GpioSetVal(CE_PORTCATE, CE_PORTNUM, action);
}

/*******************************************************************************
  Function:
    uint32_t m2mStub_GetOneMsTimer(void)

  Summary:
    Reads 1 millisecond counter value

  Description:
    The WINC1500 driver will call this function to read the 1ms counter.

  Parameters:
    None

  Returns:
    One-millisecond counter value
 *****************************************************************************/
uint32_t m2mStub_GetOneMsTimer(void){
	uint32_t ms;
	timeLpitMsGet(&ms);
	return ms;
}

/*******************************************************************************
  Function:
    void m2mStub_EintEnable(void)

  Summary:
    Enables the WINC1500 interrupt

  Description:
    The WINC1500 driver will call this function to enable the WINC1500 interrupt.
    When the interrupt is initially configured it should be in a disabled state.
    The interrupt handler should call m2m_EintHandler() and clear the interrupt.

  Parameters:
    None

  Returns:
    None
 *****************************************************************************/
void m2mStub_EintEnable(void){
	PORTB->PCR[1] = 0x000a0100; 
	S32_NVIC->ICPR[(PORTB_IRQn) >> 5] = 1 << ((PORTB_IRQn ) % 32);  /* clr any pending IRQ*/
	S32_NVIC->ISER[(PORTB_IRQn) >> 5] = 1 << ((PORTB_IRQn) % 32);  /* enable IRQ */
	S32_NVIC->IP[PORTB_IRQn] = 0xA;              /* priority 10 of 0-15*/
}

/*******************************************************************************
  Function:
    void m2mStub_EintDisable(void)

  Summary:
    Disables the WINC1500 interrupt

  Description:
    The WINC1500 driver will call this function to disable the WINC1500 interrupt.

  Parameters:
    None

  Returns:
    None
 *****************************************************************************/
void m2mStub_EintDisable(void){
		S32_NVIC->ICER[(PORTB_IRQn) >> 5] = 1 << ((PORTB_IRQn) % 32);  /* disable IRQ */
}

/*******************************************************************************
  Function:
    void m2mStub_SpiTxRx(uint8_t *p_txBuf, uint16_t txLen, uint8_t *p_rxBuf, uint16_t rxLen)

  Summary:
    Writes and reads bytes from the WINC1500 via the SPI interface

  Description:
    If txLen > rxLen then:
        Throw away the extra read bytes.  Do NOT write the garbage read bytes to p_rxBuf

    If rxLen is > txLen then:
        Write out filler bytes of 0x00 in order to get all the read bytes

  Parameters:
    p_txBuf -- Pointer to tx data (data being clocked out to the WINC1500).
               This will be NULL if txLen is 0.
    txLen   -- Number of Tx bytes to clock out.  This will be 0 if only a read is
               occurring.
    p_rxBuf -- Pointer to rx data (data being clocked in from the WINC1500).
               This will be NULL if rxLen is 0.
    rxLen   -- Number of bytes to read.  This will be 0 if only a write is occurring.

  Returns:
    None
 *****************************************************************************/
void m2mStub_SpiTxRx(uint8_t *p_txBuf, uint16_t txLen, uint8_t *p_rxBuf, uint16_t rxLen){
	if(txLen > 0){
		SpiSend(p_txBuf, txLen, 10);
	}
	if(rxLen > 0){
		SpiRecv(p_rxBuf, rxLen, 10);
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
