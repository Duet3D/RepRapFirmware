/**
 * \file
 *
 * \brief Ethernet management for the standalone lwIP example.
 *
 * Copyright (c) 2012 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#include <string.h>
//#include "board.h"
//#include "gpio.h"
#include "ethernet_sam.h"
//#include "emac.h"
#include "ethernet_phy.h"
#include "timer_mgt_sam.h"
//#include "sysclk.h"
/* lwIP includes */
#include "lwip/src/include/lwip/sys.h"
#include "lwip/src/include/lwip/api.h"
#include "lwip/src/include/lwip/tcp.h"
#include "lwip/src/include/lwip/tcpip.h"
#include "lwip/src/include/lwip/memp.h"
#include "lwip/src/include/lwip/dhcp.h"
#include "lwip/src/include/lwip/dns.h"
#include "lwip/src/include/lwip/stats.h"
#include "lwip/src/include/lwip/init.h"
#include "lwip/src/include/ipv4/lwip/ip_frag.h"
#if ( (LWIP_VERSION) == ((1U << 24) | (3U << 16) | (2U << 8) | (LWIP_VERSION_RC)) )
#include "lwip/src/include/netif/loopif.h"
#else
#include "lwip/src/include/ipv4/lwip/inet.h"
#include "lwip/src/include/lwip/tcp_impl.h"
#endif
#include "lwip/src/include/netif/etharp.h"
#include "lwip/src/sam/include/netif/ethernetif.h"

#if defined(HTTP_RAW_USED)
#include "httpd.h"
#endif

#include "lwip_test.h"

/* Global variable containing MAC Config (hw addr, IP, GW, ...) */
struct netif gs_net_if;

//*****************************AB
//Pass through function for interface status
//by including ethernetif.h directly and calling ethernetif_phy_link_status(); this function is not required
bool status_link_up()
{
return ethernetif_phy_link_status();
}
//*****************************AB


struct netif* GetConfiguration()
{
	return &gs_net_if;
}

/* Timer for calling lwIP tmr functions without system */
typedef struct timers_info {
	uint32_t timer;
	uint32_t timer_interval;
	void (*timer_func)(void);
} timers_info_t;

/* LwIP tmr functions list */
static timers_info_t gs_timers_table[] = {
	{0, TCP_TMR_INTERVAL, tcp_tmr},
	{0, IP_TMR_INTERVAL, ip_reass_tmr},
#if 0
	/* LWIP_TCP */
	{0, TCP_FAST_INTERVAL, tcp_fasttmr},
	{0, TCP_SLOW_INTERVAL, tcp_slowtmr},
#endif
	/* LWIP_ARP */
	{0, ARP_TMR_INTERVAL, etharp_tmr},
	/* LWIP_DHCP */
#if LWIP_DHCP
	{0, DHCP_COARSE_TIMER_SECS, dhcp_coarse_tmr},
	{0, DHCP_FINE_TIMER_MSECS, dhcp_fine_tmr},
#endif
};

/**
 * \brief Process timing functions.
 */
static void timers_update(void)
{
	static uint32_t ul_last_time;
	uint32_t ul_cur_time, ul_time_diff, ul_idx_timer;
	timers_info_t *p_tmr_inf;

	ul_cur_time = sys_get_ms();
	if (ul_cur_time >= ul_last_time) {
		ul_time_diff = ul_cur_time - ul_last_time;
	} else {
		ul_time_diff = 0xFFFFFFFF - ul_last_time + ul_cur_time;
	}

	if (ul_time_diff) {
		ul_last_time = ul_cur_time;
		for (ul_idx_timer = 0;
			 ul_idx_timer < (sizeof(gs_timers_table) / sizeof(timers_info_t));
			 ul_idx_timer++) {
			p_tmr_inf = &gs_timers_table[ul_idx_timer];
			p_tmr_inf->timer += ul_time_diff;
			if (p_tmr_inf->timer > p_tmr_inf->timer_interval) {
				if (p_tmr_inf->timer_func) {
					p_tmr_inf->timer_func();
				}

				p_tmr_inf->timer -= p_tmr_inf->timer_interval;
			}
		}
	}
}

/**
 *  \brief Set ethernet config.
 */
//err_t ethernetif_init_(struct netif *netif){return ERR_OK;};
//err_t ethernet_input_(struct pbuf *p, struct netif *netif){return ERR_OK;};

//static void ethernet_configure_interface(void)
//{
//	struct ip_addr x_ip_addr, x_net_mask, x_gateway;
//	extern err_t ethernetif_init(struct netif *netif);
//
//#if defined(DHCP_USED)
//	x_ip_addr.addr = 0;
//	x_net_mask.addr = 0;
//#else
//	/* Default ip addr */
//	IP4_ADDR(&x_ip_addr, ETHERNET_CONF_IPADDR0, ETHERNET_CONF_IPADDR1, ETHERNET_CONF_IPADDR2, ETHERNET_CONF_IPADDR3);
//
//	/* Default subnet mask */
//	IP4_ADDR(&x_net_mask, ETHERNET_CONF_NET_MASK0, ETHERNET_CONF_NET_MASK1, ETHERNET_CONF_NET_MASK2, ETHERNET_CONF_NET_MASK3);
//
//	/* Default gateway addr */
//	IP4_ADDR(&x_gateway, ETHERNET_CONF_GATEWAY_ADDR0, ETHERNET_CONF_GATEWAY_ADDR1, ETHERNET_CONF_GATEWAY_ADDR2, ETHERNET_CONF_GATEWAY_ADDR3);
//#endif
//
//	/* Add data to netif */
//	netif_add(&gs_net_if, &x_ip_addr, &x_net_mask, &x_gateway, NULL,
//			ethernetif_init, ethernet_input);
//
//	/* Make it the default interface */
//	netif_set_default(&gs_net_if);
//
//	/* Setup callback function for netif status change */
//	netif_set_status_callback(&gs_net_if, status_callback);
//
//	/* Bring it up */
//#if defined(DHCP_USED)
//	printf("LwIP: DHCP Started");
//	dhcp_start(&gs_net_if);
//#else
////	printf("LwIP: Static IP Address Assigned\r\n");
//	netif_set_up(&gs_net_if);
//#endif
//}
//
///** \brief Create ethernet task, for ethernet management.
// *
// */
//void init_ethernet(void)
//{
//	/* Initialize lwIP */
//	lwip_init();
//
//	/* Set hw and IP parameters, initialize MAC too */
//	ethernet_configure_interface();
//
//	/* Init timer service */
//	sys_init_timing();
//
//#if defined(HTTP_RAW_USED)
//	/* Bring up the web server */
//	httpd_init();
//#endif
//}


//************************************************************************************************************

// Added by AB.

static void ethernet_configure_interface(unsigned char ipAddress[], unsigned char netMask[], unsigned char gateWay[])
{
	struct ip_addr x_ip_addr, x_net_mask, x_gateway;
	extern err_t ethernetif_init(struct netif *netif);

#if defined(DHCP_USED)
	x_ip_addr.addr = 0;
	x_net_mask.addr = 0;
#else
	/* Default ip addr */
	//IP4_ADDR(&x_ip_addr, ETHERNET_CONF_IPADDR0, ETHERNET_CONF_IPADDR1, ETHERNET_CONF_IPADDR2, ETHERNET_CONF_IPADDR3);

	IP4_ADDR(&x_ip_addr, ipAddress[0], ipAddress[1], ipAddress[2], ipAddress[3]);

	/* Default subnet mask */
	//IP4_ADDR(&x_net_mask, ETHERNET_CONF_NET_MASK0, ETHERNET_CONF_NET_MASK1, ETHERNET_CONF_NET_MASK2, ETHERNET_CONF_NET_MASK3);

	IP4_ADDR(&x_net_mask, netMask[0], netMask[1], netMask[2], netMask[3]);

	/* Default gateway addr */
	//IP4_ADDR(&x_gateway, ETHERNET_CONF_GATEWAY_ADDR0, ETHERNET_CONF_GATEWAY_ADDR1, ETHERNET_CONF_GATEWAY_ADDR2, ETHERNET_CONF_GATEWAY_ADDR3);

	IP4_ADDR(&x_gateway, gateWay[0], gateWay[1], gateWay[2], gateWay[3]);
#endif

	/* Add data to netif */
	netif_add(&gs_net_if, &x_ip_addr, &x_net_mask, &x_gateway, NULL,
			ethernetif_init, ethernet_input);

	/* Make it the default interface */
	netif_set_default(&gs_net_if);

	/* Setup callback function for netif status change */
	netif_set_status_callback(&gs_net_if, status_callback);

	/* Bring it up */
#if defined(DHCP_USED)
	printf("LwIP: DHCP Started");
	dhcp_start(&gs_net_if);
#else
//	printf("LwIP: Static IP Address Assigned\r\n");
	netif_set_up(&gs_net_if);
#endif
}

/** \brief Create ethernet task, for ethernet management.
 *
 */
void init_ethernet(const unsigned char ipAddress[], const unsigned char netMask[], const unsigned char gateWay[])
{
	/* Initialize lwIP */
	lwip_init();

	/* Set hw and IP parameters, initialize MAC too */
	ethernet_configure_interface(ipAddress, netMask, gateWay);

	/* Init timer service */
	sys_init_timing();

#if defined(HTTP_RAW_USED)
	/* Bring up the web server */
	httpd_init();
#endif
}



//*************************************************************************************************************
/**
 *  \brief Status callback used to print address given by DHCP.
 *
 * \param netif Instance to network interface.
 */
void status_callback(struct netif *netif)
{
	int8_t c_mess[25];
	if (netif_is_up(netif)) {
//		printf("Network up\r\n");
//		strcpy((char*)c_mess, "IP=");
//		strcat((char*)c_mess, inet_ntoa(*(struct in_addr *)&(netif->ip_addr)));
//		printf((char const*)c_mess);
//		printf("-----------------\r\n");
		netif->flags |=NETIF_FLAG_LINK_UP;
	} else {
//		printf("Network down\r\n");
	}
}



/**0
 *  \brief Manage the Ethernet packets, if any received process them.
 *  After processing any packets, manage the lwIP timers.
 */
//int HttpSend();

void ethernet_task(void)
{
	//HttpSend();
	/* Run polling tasks */
	ethernetif_input(&gs_net_if);

	/* Run periodic tasks */
	timers_update();
}
