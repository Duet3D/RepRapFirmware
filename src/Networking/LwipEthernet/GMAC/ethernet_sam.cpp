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

#include "ethernet_sam.h"

#if defined(__SAME70Q20B__)
# include <Hardware/SAME70/Ethernet/GmacInterface.h>
#elif defined(__SAME54P20A__)
# include <Hardware/SAME5x/Ethernet/GmacInterface.h>
#else
# error Unsupported processor
#endif

#include "lwip/netif.h"

#include <cstring>

extern "C" {

/* lwIP includes */
#include "lwip/api.h"
#include "lwip/dns.h"
#include "lwip/init.h"
#include "lwip/memp.h"
#include "lwip/stats.h"
#include "lwip/sys.h"

/* Global variable containing MAC Config (hw addr, IP, GW, ...) */
struct netif gs_net_if;

}		// end extern "C"

//************************************************************************************************************

// This sets the static IP configuration on-the-fly
void ethernet_set_configuration(IPAddress ipAddress, IPAddress netMask, IPAddress gateWay) noexcept
{
	ip4_addr_t x_ip_addr, x_net_mask, x_gateway;
	x_ip_addr.addr = ipAddress.GetV4LittleEndian();
	x_net_mask.addr = netMask.GetV4LittleEndian();
	x_gateway.addr = gateWay.GetV4LittleEndian();

	// use static IP address
	netif_set_ipaddr(&gs_net_if, &x_ip_addr);
	netif_set_netmask(&gs_net_if, &x_net_mask);
	netif_set_gw(&gs_net_if, &x_gateway);
}

/** \brief Initialize the Ethernet subsystem.
 *
 */
void init_ethernet(IPAddress ipAddress, IPAddress netMask, IPAddress gateWay) noexcept
{
	ip4_addr_t x_ip_addr, x_net_mask, x_gateway;
	x_ip_addr.addr = ipAddress.GetV4LittleEndian();
	x_net_mask.addr = netMask.GetV4LittleEndian();
	x_gateway.addr = gateWay.GetV4LittleEndian();

	/* Initialize lwIP. */
	lwip_init();

	/* Set hw and IP parameters, initialize MAC too. */
	ethernetif_hardware_init();

	/* Add data to netif */
	netif_add(&gs_net_if, &x_ip_addr, &x_net_mask, &x_gateway, NULL, ethernetif_init, ethernet_input);

	/* Make it the default interface */
	netif_set_default(&gs_net_if);

	/* Set it up */
	netif_set_up(&gs_net_if);
}

// Terminate Ethernet and stop any interrupts, tasks etc. Used when shutting down the whole system.
void ethernet_terminate() noexcept
{
	ethernetif_terminate();
}

/** \brief Configure the Ethernet subsystem. Should be called after init_ethernet()
 *
 */
void ethernet_configure_interface(const uint8_t macAddress[], const char *hostname) noexcept
{
	ethernetif_set_mac_address(macAddress);

#if LWIP_NETIF_HOSTNAME
	netif_set_hostname(&gs_net_if, hostname);
#endif
}

/* \brief Perform ethernet auto-negotiation and establish link. Returns true when ready
 *
 */
bool ethernet_establish_link() noexcept
{
	if (ethernetif_establish_link())
	{
		netif_set_link_up(&gs_net_if);
		return true;
	}
	return false;
}

/* \brief Is the link still up? Also updates the interface status if the link has gone down
 *
 */
bool ethernet_link_established() noexcept
{
	if (!ethernetif_link_established())
	{
		netif_set_link_down(&gs_net_if);
		return false;
	}
	return true;
}

/*
 * \brief Returns the current IP address
 */
void ethernet_get_ipaddress(IPAddress& ipAddress, IPAddress& netMask, IPAddress& gateWay) noexcept
{
	ipAddress.SetV4LittleEndian(gs_net_if.ip_addr.addr);
	netMask.SetV4LittleEndian(gs_net_if.netmask.addr);
	gateWay.SetV4LittleEndian(gs_net_if.gw.addr);
}

// End

