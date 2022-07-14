/*
 * MulticastResponder.cpp
 *
 *  Created on: 12 Jul 2022
 *      Author: David
 */

#include "MulticastResponder.h"

#if SUPPORT_MULTICAST_DISCOVERY

#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include "fgmc_header.h"

extern "C" {
#include "LwipEthernet/Lwip/src/include/lwip/udp.h"
#include "LwipEthernet/Lwip/src/include/lwip/igmp.h"
extern struct netif gs_net_if;
}

static udp_pcb *ourPcb = nullptr;
static pbuf * volatile receivedPbuf = nullptr;
static volatile uint32_t receivedIpAddr;
static volatile uint16_t receivedPort;
static unsigned int messagesProcessed = 0;

static bool active = false;

static constexpr ip_addr_t ourGroup = IPADDR4_INIT_BYTES(239, 255, 2, 3);

// Receive callback function
extern "C" void rcvFunc(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) noexcept
{
	if (active && receivedPbuf == nullptr)
	{
		receivedIpAddr = addr->addr;
		receivedPort = port;
		receivedPbuf = p;
	}
	else
	{
		pbuf_free(p);
	}
}

void MulticastResponder::Init() noexcept
{
	// Nothing needed here yet
}

// Do some work, returning true if we did anything significant
void MulticastResponder::Spin() noexcept
{
	pbuf *rxPbuf = receivedPbuf;
	if (rxPbuf != nullptr)
	{
		debugPrintf("Rx UDP: addr %u.%u.%u.%u port %u data",
			(unsigned int)(receivedIpAddr & 0xFF), (unsigned int)((receivedIpAddr >> 8) & 0xFF), (unsigned int)((receivedIpAddr >> 16) & 0xFF), (unsigned int)((receivedIpAddr >> 24) & 0xFF), receivedPort);
		for (size_t i = 0; i < rxPbuf->len; ++i)
		{
			debugPrintf(" %02x", ((const uint8_t*)(rxPbuf->payload))[i]);
		}
		debugPrintf("\n");
		pbuf_free(rxPbuf);
		receivedPbuf = nullptr;
		++messagesProcessed;
	}
}

void MulticastResponder::Diagnostics(MessageType mtype) noexcept
{
	reprap.GetPlatform().MessageF(mtype, "=== Multicast handler ===\nResponder is %s, messages processed %u\n", (active) ? "active" : "inactive", messagesProcessed);
}

void MulticastResponder::Start(TcpPort port) noexcept
{
	if (ourPcb == nullptr)
	{
		ourPcb = udp_new_ip_type(IPADDR_TYPE_ANY);
		if (ourPcb == nullptr)
		{
			reprap.GetPlatform().Message(ErrorMessage, "unable to allocate a pcb\n");
		}
		else
		{
			udp_set_multicast_ttl(ourPcb, 255);
			if (igmp_joingroup_netif(&gs_net_if, ip_2_ip4(&ourGroup)) != ERR_OK)			// without this call, multicast packets to this IP address get discarded
			{
				reprap.GetPlatform().Message(ErrorMessage, "igmp_joingroup failed\n");
			}
			else if (udp_bind(ourPcb, &ourGroup, port) != ERR_OK)
			{
				reprap.GetPlatform().Message(ErrorMessage, "udp_bind call failed\n");
			}
			else
			{
				udp_recv(ourPcb, rcvFunc, nullptr);
			}
		}
	}
	active = true;
	messagesProcessed = 0;
}

void MulticastResponder::Stop() noexcept
{
	if (ourPcb != nullptr)
	{
		udp_remove(ourPcb);
		ourPcb = nullptr;
	}
	active = false;
}

#endif

// End
