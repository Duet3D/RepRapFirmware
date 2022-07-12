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

static udp_pcb *pcb = nullptr;

void MulticastResponder::Init() noexcept
{
	// TODO Auto-generated  stub
}

// Do some work, returning true if we did anything significant
void MulticastResponder::Spin() noexcept
{
	//TODO
}

void MulticastResponder::Diagnostics(MessageType mtype) noexcept
{
	//TODO
}

void MulticastResponder::Start(TcpPort port) noexcept
{
	if (pcb == nullptr)
	{
		pcb = udp_new();
		if (pcb == nullptr)
		{
			reprap.GetPlatform().Message(ErrorMessage, "unable to allocate a pcb\n");
		}
		else
		{
			udp_bind(pcb, IP_ADDR_ANY, port);
			//TODO
#if 0
			pcb = tcp_listen(pcb);
			if (pcb == nullptr)
			{
				platform.Message(ErrorMessage, "tcp_listen call failed\n");
			}
			else
			{
				listeningPcbs[protocol] = pcb;
				tcp_accept(listeningPcbs[protocol], conn_accept);
			}
#endif
		}
	}
}

void MulticastResponder::Stop() noexcept
{
	//TODO
}

#if 0
// Offer the responder a UDP packet, return true if it was accepted
bool MulticastResponder::AcceptUdp(struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, uint16_t port) noexcept
{
	debugPrintf("Rx UDP: addr %u.%u.%u.%u port %u data", (unsigned int)((addr->addr >> 24) & 0xFF));
	for (size_t i = 0; i < qq; ++i)
	{
		debugPrintf(" %02x", qq);
	}
	debugPrintf("\n");
//TODO
	return true;
}
#endif

#endif

// End
