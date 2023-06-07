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
extern netif gs_net_if;
}

extern Mutex lwipMutex;

static constexpr ip_addr_t ourGroupIpAddr = IPADDR4_INIT_BYTES(239, 255, 2, 3);

static udp_pcb *ourPcb = nullptr;
static pbuf * volatile receivedPbuf = nullptr;
static pbuf *pbufToFree = nullptr;
static volatile uint32_t receivedIpAddr;
static volatile uint16_t receivedPort;
static uint16_t lastMessageReceivedPort;
static unsigned int messagesProcessed = 0;
static unsigned int responsesSent = 0;
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
		receivedPbuf = p;				// store this one last
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
			pbufToFree = rxPbuf;

			fgmcHandler->handleStream(0, (const uint8_t *)rxPbuf->payload, rxPbuf->len);
			++messagesProcessed;

			// If processing the message didn't free the pbuf, free it now
			if (pbufToFree != nullptr)
			{
				MutexLocker lock(lwipMutex);
				pbuf_free(pbufToFree);
				pbufToFree = nullptr;
			}
		}
	}
}

void MulticastResponder::Diagnostics(MessageType mtype) noexcept
{
	reprap.GetPlatform().MessageF(mtype, "=== Multicast handler ===\nResponder is %s, messages received %u, responses %u\n",
									(active) ? "active" : "inactive", messagesProcessed, responsesSent);
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
		MutexLocker lock(lwipMutex);
		ourPcb = udp_new_ip_type(IPADDR_TYPE_ANY);
		if (ourPcb == nullptr)
		{
			lock.Release();
			reprap.GetPlatform().Message(ErrorMessage, "unable to allocate a pcb\n");
		}
		else
		{
			udp_set_multicast_ttl(ourPcb, 255);
			if (igmp_joingroup_netif(&gs_net_if, ip_2_ip4(&ourGroupIpAddr)) != ERR_OK)			// without this call, multicast packets to this IP address get discarded
			{
				lock.Release();
				reprap.GetPlatform().Message(ErrorMessage, "igmp_joingroup failed\n");
			}
			else if (udp_bind(ourPcb, &ourGroupIpAddr, port) != ERR_OK)
			{
				lock.Release();
				reprap.GetPlatform().Message(ErrorMessage, "udp_bind call failed\n");
			}
			else
			{
				udp_recv(ourPcb, rcvFunc, nullptr);
			}
		}
	}
	active = true;
	messagesProcessed = responsesSent = 0;
}

void MulticastResponder::Stop() noexcept
{
	if (ourPcb != nullptr)
	{
		MutexLocker lock(lwipMutex);
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

	MutexLocker lock(lwipMutex);

	// Release the pbuf that the message arrived in
	if (pbufToFree != nullptr)
	{
		pbuf_free(pbufToFree);
		pbufToFree = nullptr;
	}

	pbuf * const pb = pbuf_alloc(PBUF_TRANSPORT, length, PBUF_RAM);
	if (pb != nullptr)
	{
		if (pbuf_take(pb, data, length) == ERR_OK)
		{
			const err_t err = udp_sendto(ourPcb, pb, &ourGroupIpAddr, lastMessageReceivedPort);
			if (err == ERR_OK)
			{
				++responsesSent;
			}
			else
			{
				if (reprap.Debug(Module::Network))
				{
					debugPrintf("UDP send failed, err=%u\n", err);
				}
			}
		}
		else
		{
			if (reprap.Debug(Module::Network))
			{
				debugPrintf("pbuf_take returned error, length %u\n", length);
			}
		}
		pbuf_free(pb);
	}
	else if (reprap.Debug(Module::Network))
	{
		debugPrintf("pbuf_alloc failed,length=%u\n", length);
	}
}

// Schedule a reboot. We delay a little while to allow the response to be transmitted first.
void MulticastResponder::ScheduleReboot() noexcept
{
	whenRebootScheduled = millis();
	ticksToReboot = 1000;
}

#endif

// End
