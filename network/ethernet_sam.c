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

#include "lwip_test.h"

extern void RepRapNetworkMessage(const char*);

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
	ul_time_diff = ul_cur_time - ul_last_time;		// we're using unsigned arithmetic, so this handles wrap around

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


//************************************************************************************************************

// Added by AB.

static void ethernet_configure_interface(unsigned char ipAddress[], unsigned char netMask[], unsigned char gateWay[])
{
	struct ip_addr x_ip_addr, x_net_mask, x_gateway;
	extern err_t ethernetif_init(struct netif *netif);

	IP4_ADDR(&x_ip_addr, ipAddress[0], ipAddress[1], ipAddress[2], ipAddress[3]);		// set IP address

	if (x_ip_addr.addr == 0)
	{
		x_net_mask.addr = 0;	// not sure this is needed, but the demo program does it
	}
	else
	{
		IP4_ADDR(&x_net_mask, netMask[0], netMask[1], netMask[2], netMask[3]);			// set network mask
	}

	IP4_ADDR(&x_gateway, gateWay[0], gateWay[1], gateWay[2], gateWay[3]);				// set gateway

	/* Add data to netif */
	netif_add(&gs_net_if, &x_ip_addr, &x_net_mask, &x_gateway, NULL, ethernetif_init, ethernet_input);

	/* Make it the default interface */
	netif_set_default(&gs_net_if);

	/* Setup callback function for netif status change */
	netif_set_status_callback(&gs_net_if, status_callback);

	/* Bring it up */
	if (x_ip_addr.addr == 0)
	{
		RepRapNetworkMessage("Starting DHCP\n");
		dhcp_start(&gs_net_if);
	}
	else
	{
		RepRapNetworkMessage("Starting network\n");
		netif_set_up(&gs_net_if);
	}
}

/** \brief Initialize the Ethernet subsystem.
 *
 */
void init_ethernet(void)
{
	lwip_init();
	ethernet_hardware_init();
}

/** \brief Try to establish a physical link at, returning true if successful.
 *
 */
bool establish_ethernet_link(void)
{
	return ethernet_establish_link();		// this is the one that takes a long time
}

/** \brief Create ethernet task, for ethernet management.
 *
 */
void start_ethernet(const unsigned char ipAddress[], const unsigned char netMask[], const unsigned char gateWay[])
{
	/* Set hw and IP parameters, initialize MAC too */
	ethernet_configure_interface(ipAddress, netMask, gateWay);

	/* Init timer service */
	sys_init_timing();
}



//*************************************************************************************************************
/**
 *  \brief Status callback used to print address given by DHCP.
 *
 * \param netif Instance to network interface.
 */
void status_callback(struct netif *netif)
{
	char c_mess[20];		// 15 for IP address, 1 for \n, 1 for null, so 3 spare
	if (netif_is_up(netif))
	{
		RepRapNetworkMessage("Network up, IP=");
		ipaddr_ntoa_r(&(netif->ip_addr), c_mess, sizeof(c_mess));
		strncat(c_mess, sizeof(c_mess) - strlen(c_mess) - 1, "\n");
		RepRapNetworkMessage(c_mess);
		netif->flags |= NETIF_FLAG_LINK_UP;
	}
	else
	{
		RepRapNetworkMessage("Network down\n");
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
