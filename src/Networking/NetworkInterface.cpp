/*
 * NetworkInterface.cpp
 *
 *  Created on: 10 Mar 2020
 *      Author: David
 */

#include "NetworkInterface.h"
#include <Platform/RepRap.h>

void NetworkInterface::SetState(NetworkState::RawType newState) noexcept
{
	state = newState;
	reprap.NetworkUpdated();
}

// Report the protocols and ports in use
GCodeResult NetworkInterface::ReportProtocols(const StringRef& reply) const noexcept
{
	for (size_t i = 0; i < NumSelectableProtocols; ++i)
	{
#if !SUPPORT_MULTICAST_DISCOVERY
		if (i == MulticastDiscoveryProtocol) { continue; }
#endif
#if !SUPPORT_MQTT
		if (i == MqttProtocol) { continue; }
#endif
		ReportOneProtocol(i, reply);
	}
	return GCodeResult::ok;
}

void NetworkInterface::ReportOneProtocol(NetworkProtocol protocol, const StringRef& reply) const noexcept
{
	if (protocolEnabled[protocol])
	{
		reply.lcatf("%s is enabled on port %u", ProtocolNames[protocol], portNumbers[protocol]);
	}
	else
	{
		reply.lcatf("%s is disabled", ProtocolNames[protocol]);
	}
}

// End
