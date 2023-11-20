/*
 * wizInterface.c
 *
 *  Created on: 2020. 5. 31.
 *      Author: eziya76@gmail.com
 */

#include "wizInterface.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

extern SPI_HandleTypeDef hspi1;
#define WIZ_SPI_HANDLE	&hspi1

static bool ip_assigned = 0;
static uint8_t buff_size[] = { 2, 2, 2, 2 };

#ifdef USE_DHCP
static uint8_t dhcp_buffer[1024];
static uint16_t dhcp_retry = 0;
#endif

//network information
wiz_NetInfo netInfo = {
		.mac = { 0x00, 0x08, 0xdc, 0xab, 0xcd, 0xef },
		.ip = { 192, 168, 2, 110 },
		.sn = { 255, 255, 255, 0 },
		.gw = { 192, 168, 2, 1 } };

wiz_NetTimeout timeout = {
		.retry_cnt = 3, 		//RCR = 3
		.time_100us = 5000};    //500ms

void WIZ_SPI_Select(void)
{
	HAL_GPIO_WritePin(WIZ_SPI1_CS_GPIO_Port, WIZ_SPI1_CS_Pin, GPIO_PIN_RESET);
}

void WIZ_SPI_Deselect(void)
{
	HAL_GPIO_WritePin(WIZ_SPI1_CS_GPIO_Port, WIZ_SPI1_CS_Pin, GPIO_PIN_SET);
}

void WIZ_SPI_TxByte(uint8_t byte)
{
	HAL_SPI_Transmit(WIZ_SPI_HANDLE, &byte, 1, HAL_MAX_DELAY);
}

uint8_t WIZ_SPI_RxByte(void)
{
	uint8_t ret;
	HAL_SPI_Receive(WIZ_SPI_HANDLE, &ret, 1, HAL_MAX_DELAY);
	return ret;
}

void WIZ_SPI_TxBuffer(uint8_t *buffer, uint16_t len)
{
	HAL_SPI_Transmit(WIZ_SPI_HANDLE, buffer, len, HAL_MAX_DELAY);
}

void WIZ_SPI_RxBuffer(uint8_t *buffer, uint16_t len)
{
	HAL_SPI_Receive(WIZ_SPI_HANDLE, buffer, len, HAL_MAX_DELAY);
}

//dhcp callbacks
static void cbIPAddrAssigned(void) {
	printf("IP Address is assigned.\n");
	ip_assigned = true;
}

static void cbIPAddrConfict(void) {
	printf("IP Address is conflicted.\n");
	ip_assigned = false;
}

bool WIZ_ChipInit(void)
{
	int32_t ret;
	uint8_t tmpstr[6] = { 0, };

	//power reset arduino ethernet shield
	//HAL_GPIO_WritePin(WIZ_RESET_GPIO_Port, WIZ_RESET_Pin, GPIO_PIN_RESET);
	//HAL_Delay(500);
	//HAL_GPIO_WritePin(WIZ_RESET_GPIO_Port, WIZ_RESET_Pin, GPIO_PIN_SET);
	//HAL_Delay(500);
	
	wizchip_cris_initialize();

#if (_WIZCHIP_ == W5100)
	//register spi functions
	reg_wizchip_cs_cbfunc(WIZ_SPI_Select, WIZ_SPI_Deselect);
	reg_wizchip_spi_cbfunc(WIZ_SPI_RxByte, WIZ_SPI_TxByte);

	//set rx,tx buffer sizes
	ret = wizchip_init(buff_size, buff_size);
	if (ret < 0) {
		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		printf("wozchip_init failed.\n");
		return false;
	}

#elif (_WIZCHIP_ == W5500)
	//register spi functions
	reg_wizchip_cs_cbfunc(WIZ_SPI_Select, WIZ_SPI_Deselect);
	reg_wizchip_spi_cbfunc(WIZ_SPI_RxByte, WIZ_SPI_TxByte);
	reg_wizchip_spiburst_cbfunc(WIZ_SPI_RxBuffer, WIZ_SPI_TxBuffer);

	//check version register
	uint8_t version = getVERSIONR();
	if(version != 0x04)
	{
		printf("getVERSIONR returns wrong version!(%u)\n", version);
		return false;
	}
	else
	{
		printf("getVERSIONR(%u)\n", version);
	}

	//check PHY status
	wiz_PhyConf phyConf;
	wizphy_getphystat(&phyConf);
	printf("PHY conf.by = {%d}, conf.mode={%d}, conf.speed={%d}, conf.duplex={%d}\n",
			phyConf.by, phyConf.mode, phyConf.speed, phyConf.duplex);
#endif

	return true;
}

bool WIZ_NetworkInit(void)
{
	wiz_NetInfo tmpInfo;
	wiz_NetTimeout tmpTimeout;

#ifdef USE_DHCP
	setSHAR(netInfo.mac); //set MAC address
	DHCP_init(DHCP_SOCKET, dhcp_buffer); //init DHCP
	reg_dhcp_cbfunc(cbIPAddrAssigned, cbIPAddrAssigned, cbIPAddrConfict); //register DHCP callbacks

	//get ip from dhcp server
	dhcp_retry = 0;
	while (!ip_assigned && dhcp_retry < 0xFFFF) {//100000) {
		dhcp_retry++;
		DHCP_run();
	}
	printf("get ip from dhcp server finished...\n");
	
	//if dhcp assigned an ip address.
	if (ip_assigned) {
		getIPfromDHCP(netInfo.ip);
		getGWfromDHCP(netInfo.gw);
		getSNfromDHCP(netInfo.sn);
	}
#endif

	//set network information
	wizchip_setnetinfo(&netInfo);

	//get network information
	wizchip_getnetinfo(&tmpInfo);
	printf("IP: %03d.%03d.%03d.%03d\nGW: %03d.%03d.%03d.%03d\nNet: %03d.%03d.%03d.%03d\n",
			tmpInfo.ip[0], tmpInfo.ip[1],tmpInfo.ip[2], tmpInfo.ip[3],
			tmpInfo.gw[0], tmpInfo.gw[1], tmpInfo.gw[2], tmpInfo.gw[3],
			tmpInfo.sn[0], tmpInfo.sn[1], tmpInfo.sn[2], tmpInfo.sn[3]);

	if(tmpInfo.mac[0] != netInfo.mac[0] ||
			tmpInfo.mac[1] != netInfo.mac[1] ||
			tmpInfo.mac[2] != netInfo.mac[2] ||
			tmpInfo.mac[3] != netInfo.mac[3])
	{
		printf("wizchip_getnetinfo failed.\n");
		return false;
	}

	//set timeout
	ctlnetwork(CN_SET_TIMEOUT,(void*)&timeout);
	ctlnetwork(CN_GET_TIMEOUT, (void*)&tmpTimeout);

	if(tmpTimeout.retry_cnt != timeout.retry_cnt || tmpTimeout.time_100us != timeout.time_100us)
	{
		printf("ctlnetwork(CN_SET_TIMEOUT) failed.\n");
		return false;
	}

	return true;
}

static void wizchip_critical_section_lock(void)
{
	__disable_irq();
    //vPortEnterCritical();
}

static void wizchip_critical_section_unlock(void)
{
	__enable_irq();
    //vPortExitCritical();
}

void wizchip_cris_initialize(void)
{
    reg_wizchip_cris_cbfunc(wizchip_critical_section_lock, wizchip_critical_section_unlock);
}


