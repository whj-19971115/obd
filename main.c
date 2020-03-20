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
#define printf  nxpPrintf
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
typedef struct s_msg_wifi_product {
	uint8_t name[9];
} t_msg_wifi_product;

typedef struct s_msg_wifi_product_main {
	uint8_t name[9];
} t_msg_wifi_product_main;

/** Message format declarations. */
static t_msg_wifi_product msg_wifi_product = {
	.name = MAIN_WIFI_M2M_PRODUCT_NAME,
};

static t_msg_wifi_product_main msg_wifi_product_main = {
	.name = MAIN_WIFI_M2M_PRODUCT_NAME,
};
static uint8_t packetCnt = 0;

/** Socket for Rx */
char tx_socket = -1;
char rx_socket = -1;
static uint8_t wifi_connected;
/** Test buffer */
static uint8_t gau8SocketTestBuffer[MAIN_WIFI_M2M_BUFFER_SIZE] = {0};
static char tcp_server_socket = -1;
static char tcp_client_socket = -1;
int aceptflag = 0;
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	switch (u8Msg) {
	/* Socket bind */
	case SOCKET_MSG_BIND:
	{
		tstrSocketBindMsg *pstrBind = (tstrSocketBindMsg *)pvMsg;
		if (pstrBind && pstrBind->status == 0) {
			printf("socket_cb: bind success!\r\n");
			listen(tcp_server_socket, 0);
		} else {
			printf("socket_cb: bind error!\r\n");
			close(tcp_server_socket);
			tcp_server_socket = -1;
		}
	}
	break;

	/* Socket listen */
	case SOCKET_MSG_LISTEN:
	{
		tstrSocketListenMsg *pstrListen = (tstrSocketListenMsg *)pvMsg;
		if (pstrListen && pstrListen->status == 0) {
			printf("socket_cb: listen success!\r\n");
			accept(tcp_server_socket, NULL, NULL);
		} else {
			printf("socket_cb: listen error!\r\n");
			close(tcp_server_socket);
			tcp_server_socket = -1;
		}
	}
	break;

	/* Connect accept */
	case SOCKET_MSG_ACCEPT:
	{
		tstrSocketAcceptMsg *pstrAccept = (tstrSocketAcceptMsg *)pvMsg;
		if (pstrAccept) {
			printf("socket_cb: accept success!\r\n");
			accept(tcp_server_socket, NULL, NULL);
			tcp_client_socket = pstrAccept->sock;
			aceptflag = 1;
			recv(tcp_client_socket, gau8SocketTestBuffer, sizeof(gau8SocketTestBuffer), 0);
		} else {
			printf("socket_cb: accept error!\r\n");
			close(tcp_server_socket);
			tcp_server_socket = -1;
		}
	}
	break;

	/* Message send */
	case SOCKET_MSG_SEND:
	{

		printf("socket_cb: send success!\r\n");
		printf("TCP Server Test Complete!\r\n");
		recv(tcp_client_socket, gau8SocketTestBuffer, sizeof(gau8SocketTestBuffer), 0);
//		printf("close socket\n");
//		close(tcp_client_socket);
//		close(tcp_server_socket);
	}
	break;

	/* Message receive */
	case SOCKET_MSG_RECV:
	{
		tstrSocketRecvMsg *pstrRecv = (tstrSocketRecvMsg *)pvMsg;
		if (pstrRecv && pstrRecv->s16BufferSize > 0) {
			printf("socket_cb: recv success!\r\n");
			printf("recv:  %s\r\n",pstrRecv->pu8Buffer);
			memset(pstrRecv->pu8Buffer,0,pstrRecv->s16BufferSize);
			send(tcp_client_socket, &msg_wifi_product, sizeof(t_msg_wifi_product), 0);
		} else {
			printf("socket_cb: recv error!\r\n");
			close(tcp_server_socket);
			tcp_server_socket = -1;
		}
	}

	break;

	default:
		break;
	}
}

/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] u8MsgType type of Wi-Fi notification. Possible types are:
 *  - [M2M_WIFI_RESP_CURRENT_RSSI](@ref M2M_WIFI_RESP_CURRENT_RSSI)
 *  - [M2M_WIFI_RESP_CON_STATE_CHANGED](@ref M2M_WIFI_RESP_CON_STATE_CHANGED)
 *  - [M2M_WIFI_RESP_CONNTION_STATE](@ref M2M_WIFI_RESP_CONNTION_STATE)
 *  - [M2M_WIFI_RESP_SCAN_DONE](@ref M2M_WIFI_RESP_SCAN_DONE)
 *  - [M2M_WIFI_RESP_SCAN_RESULT](@ref M2M_WIFI_RESP_SCAN_RESULT)
 *  - [M2M_WIFI_REQ_WPS](@ref M2M_WIFI_REQ_WPS)
 *  - [M2M_WIFI_RESP_IP_CONFIGURED](@ref M2M_WIFI_RESP_IP_CONFIGURED)
 *  - [M2M_WIFI_RESP_IP_CONFLICT](@ref M2M_WIFI_RESP_IP_CONFLICT)
 *  - [M2M_WIFI_RESP_P2P](@ref M2M_WIFI_RESP_P2P)
 *  - [M2M_WIFI_RESP_AP](@ref M2M_WIFI_RESP_AP)
 *  - [M2M_WIFI_RESP_CLIENT_INFO](@ref M2M_WIFI_RESP_CLIENT_INFO)
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters
 * (if any). It should be casted to the correct data type corresponding to the
 * notification type. Existing types are:
 *  - tstrM2mWifiStateChanged
 *  - tstrM2MWPSInfo
 *  - tstrM2MP2pResp
 *  - tstrM2MAPResp
 *  - tstrM2mScanDone
 *  - tstrM2mWifiscanResult
 */
static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			printf("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: CONNECTED\r\n");
			m2m_wifi_request_dhcp_client();
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			printf("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: DISCONNECTED\r\n");
			wifi_connected = 0;
		}
	}
	break;

	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		wifi_connected = 1;
		printf("wifi_cb: M2M_WIFI_REQ_DHCP_CONF: IP is %u.%u.%u.%u\r\n",
				pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
	}
	break;

	default:
		break;
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
		struct sockaddr_in addr;
		volatile uint32_t send_dly = 0;
		tstrWifiInitParam param;
		tstrM2MAPConfig   strM2MAPConfig;
		nm_bsp_init();
		/* Initialize Wi-Fi parameters structure. */
		memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));
		uartInit(UART0, 9600);
		
		/* Initialize Wi-Fi driver with data and status callbacks. */
		param.pfAppWifiCb = wifi_cb;
		m2m_wifi_init(&param);
	addr.sin_family = AF_INET;
	addr.sin_port = _htons(MAIN_WIFI_M2M_SERVER_PORT);
	addr.sin_addr.s_addr = _htons(MAIN_WIFI_M2M_SERVER_IP);
		/* Initialize socket module */
		socketInit();	
		registerSocketCallback(socket_cb, NULL);
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
		strM2MAPConfig.u8KeyIndx = 1;
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
		m2m_wifi_handle_events(NULL);
		if (wifi_connected == M2M_WIFI_CONNECTED) {
			if (tcp_server_socket == 0xff ) {
				/* Open TCP server socket */
				if ((tcp_server_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
					printf("main: failed to create TCP server socket error!\r\n");
					continue;
				}

				/* Bind service*/
				bind(tcp_server_socket, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
				printf("main: bind  TCP server socket !\r\n");
			}
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