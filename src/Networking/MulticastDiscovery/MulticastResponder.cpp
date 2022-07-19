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
#include <Hardware/ExceptionHandlers.h>

#include "fgmc_header.h"
#include "fgmc_protocol.h"

extern "C" {
#include "LwipEthernet/Lwip/src/include/lwip/udp.h"
#include "LwipEthernet/Lwip/src/include/lwip/igmp.h"
extern struct netif gs_net_if;
}

static constexpr ip_addr_t ourGroup = IPADDR4_INIT_BYTES(239, 255, 2, 3);

static udp_pcb *ourPcb = nullptr;
static pbuf * volatile receivedPbuf = nullptr;
static volatile uint32_t receivedIpAddr;
static volatile uint16_t receivedPort;
static uint16_t lastMessageReceivedPort;
static unsigned int messagesProcessed = 0;
static FGMCProtocol *fgmcHandler = nullptr;
static uint32_t ticksToReboot = 0;
static uint32_t whenRebootScheduled;

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
	if (ticksToReboot != 0)
	{
		if (millis() - whenRebootScheduled > ticksToReboot)
		{
			reprap.EmergencyStop();									// this disables heaters and drives - Duet WiFi pre-production boards need drives disabled here
			SoftwareReset(SoftwareResetReason::user);				// doesn't return
		}

	}
	else
	{
		pbuf *rxPbuf = receivedPbuf;
		if (rxPbuf != nullptr)
		{
			lastMessageReceivedPort = receivedPort;
#if 0
			debugPrintf("Rx UDP: addr %u.%u.%u.%u port %u data",
				(unsigned int)(receivedIpAddr & 0xFF), (unsigned int)((receivedIpAddr >> 8) & 0xFF), (unsigned int)((receivedIpAddr >> 16) & 0xFF), (unsigned int)((receivedIpAddr >> 24) & 0xFF), lastMessageReceivedPort);
			for (size_t i = 0; i < rxPbuf->len; ++i)
			{
				debugPrintf(" %02x", ((const uint8_t*)(rxPbuf->payload))[i]);
			}
			debugPrintf("\n");
#endif
			receivedPbuf = nullptr;
			fgmcHandler->handleStream(0, (uint8_t *)rxPbuf->payload, rxPbuf->len);
			pbuf_free(rxPbuf);
			++messagesProcessed;
		}
	}
}

void MulticastResponder::Diagnostics(MessageType mtype) noexcept
{
	reprap.GetPlatform().MessageF(mtype, "=== Multicast handler ===\nResponder is %s, messages processed %u\n", (active) ? "active" : "inactive", messagesProcessed);
}

void MulticastResponder::Start(TcpPort port) noexcept
{
	if (fgmcHandler == nullptr)
	{
		fgmcHandler = new FGMCProtocol;
		fgmcHandler->init();
	}

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

void MulticastResponder::SendResponse(uint8_t *data, size_t length) noexcept
{
#if 0
	debugPrintf("Tx UDP: port %u data", lastMessageReceivedPort);
	for (size_t i = 0; i < length; ++i)
	{
		debugPrintf(" %02x", data[i]);
	}
	debugPrintf("\n");
#endif

	pbuf * const pb = pbuf_alloc(PBUF_TRANSPORT, length, PBUF_RAM);
#if 0
	if (pbuf_take(pb, data, length) != ERR_OK)
	{
		debugPrintf("pbuf_take returned error\n");
	}
	if (udp_sendto_if(ourPcb, pb, &ourGroup, lastMessageReceivedPort, &gs_net_if) != ERR_OK)
	{
		debugPrintf("UDP send failed\n");
	}
#else
	(void)pbuf_take(pb, data, length);
	(void)udp_sendto_if(ourPcb, pb, &ourGroup, lastMessageReceivedPort, &gs_net_if);
#endif
}

// Schedule a reboot. We delay a little while to allow the response to be transmitted first.
void MulticastResponder::ScheduleReboot() noexcept
{
	whenRebootScheduled = millis();
	ticksToReboot = 1000;
}

#endif

// End
