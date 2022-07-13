/*
 * MulricastResponder.cpp
 *
 *  Created on: 12 Jul 2022
 *      Author: David
 */

#include "MulticastResponder.h"
#include <Platform/RepRap.h>
#include <Platform/Platform.h>

extern "C" {
#include "LwipEthernet/Lwip/src/include/lwip/udp.h"
}

#if SUPPORT_MULTICAST_DISCOVERY

static udp_pcb *ourPcb = nullptr;
static pbuf * volatile receivedPbuf = nullptr;
static volatile uint32_t receivedIpAddr;
static volatile uint16_t receivedPort;
static unsigned int messagesProcessed = 0;

static bool active = false;

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
						(unsigned int)((receivedIpAddr >> 24) & 0xFF), (unsigned int)((receivedIpAddr >> 16) & 0xFF), (unsigned int)((receivedIpAddr >> 8) & 0xFF), (unsigned int)(receivedIpAddr & 0xFF), receivedPort);
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
			if (udp_bind(ourPcb, IP_ADDR_ANY, port) != ERR_OK)
			{
				reprap.GetPlatform().Message(ErrorMessage, "udp_bind call failed\n");
			}
			else
			{
				debugPrintf("udp_bind call succeeded\n");
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
