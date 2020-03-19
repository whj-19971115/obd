#include "S32K142.h" /* include peripheral declarations S32K144 */
#include "clocks_and_modes.h"
#include "gpio.h"
#include "usertype.h"
#include "uart.h"
#include "timer.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "socket.h"
#include "wifi/driver/include/m2m_wifi.h"
#include "wifi/driver/source/nmasic.h"
#include "spi.h"
#include "main.h"
#include "printf.h"
/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] u8MsgType type of Wi-Fi notification. Possible types are:
 *  - [M2M_WIFI_RESP_CON_STATE_CHANGED](@ref M2M_WIFI_RESP_CON_STATE_CHANGED)
 *  - [M2M_WIFI_REQ_DHCP_CONF](@ref M2M_WIFI_REQ_DHCP_CONF)
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters
 * (if any). It should be casted to the correct data type corresponding to the
 * notification type.
 */
static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED: {
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
//			nxpPrintf("Station disconnected\r\n");
		}

		break;
	}

	case M2M_WIFI_REQ_DHCP_CONF: {
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
//		nxpPrintf("Station connected\r\n");
//		nxpPrintf("Station IP is %u.%u.%u.%u\r\n", pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
		break;
	}

	default: {
		break;
	}
	}
}
#if 1
int main(void)
{
	  int ret = 0, slen = 0, rlen = 0;
		int timeNum = 0, num;
		unsigned char wData[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
		SOSC_init_8MHz();       /* Initialize system oscilator for 8 MHz xtal */
	  SPLL_init_160MHz();     /* Initialize SPLL to 160 MHz with 8 MHz SOSC */
	  NormalRUNmode_80MHz();  /* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash */
	  SpiInitMaster(8);
		
		tstrWifiInitParam param;
		tstrM2MAPConfig   strM2MAPConfig;
		nm_bsp_init();
		/* Initialize Wi-Fi parameters structure. */
		memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));
		uartInit(UART0, 9600);
		
		/* Initialize Wi-Fi driver with data and status callbacks. */
		param.pfAppWifiCb = wifi_cb;
		m2m_wifi_init(&param);

		/* Initialize AP mode parameters structure with SSID, channel and OPEN security type. */
		memset(&strM2MAPConfig, 0x00, sizeof(tstrM2MAPConfig));
		strcpy((char *)&strM2MAPConfig.au8SSID, MAIN_WLAN_SSID);
		strM2MAPConfig.u8ListenChannel = MAIN_WLAN_CHANNEL;
		strM2MAPConfig.u8SecType       = MAIN_WLAN_AUTH;
		strM2MAPConfig.u8KeySz = 10;
		strM2MAPConfig.au8DHCPServerIP[0] = 192;
		strM2MAPConfig.au8DHCPServerIP[1] = 168;
		strM2MAPConfig.au8DHCPServerIP[2] = 1;
		strM2MAPConfig.au8DHCPServerIP[3] = 1;

	#if USE_WEP
		strcpy((char *)&strM2MAPConfig.au8WepKey, MAIN_WLAN_WEP_KEY);
		strM2MAPConfig.u8KeySz   = strlen(MAIN_WLAN_WEP_KEY);
		strM2MAPConfig.u8KeyIndx = MAIN_WLAN_WEP_KEY_INDEX;
	#endif

		/* Bring up AP mode with parameters structure. */
		ret = m2m_wifi_enable_ap(&strM2MAPConfig);
		if (M2M_SUCCESS != ret) {
			nxpPrintf("main: m2m_wifi_enable_ap call error!\r\n");
			while (1) {
			}
		}

		nxpPrintf("AP mode started. You can connect to %s.\r\n", (char *)MAIN_WLAN_SSID);

		while (1) {
			/* Handle pending events from network controller. */
			while (m2m_wifi_handle_events(NULL) != M2M_SUCCESS) {
			}
		}

		return 0;
}
#else
int main(void)
{
	  int ret = 0, slen = 0, rlen = 0;
		int timeNum = 0, num;
		unsigned char wData[10] = {0x11, 0x12, 0x22, 0x23, 0x32, 0x33, 0x42, 0x43, 0x52, 0x53};
		SOSC_init_8MHz();       /* Initialize system oscilator for 8 MHz xtal */
	  SPLL_init_160MHz();     /* Initialize SPLL to 160 MHz with 8 MHz SOSC */
	  NormalRUNmode_80MHz();  /* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash */
		SpiInitMaster(8);
		
		while(1){
				SpiSend (wData, sizeof(wData), 10);
			  int timeOut = 800000;
			  while(timeOut--);
		}
}
#endif