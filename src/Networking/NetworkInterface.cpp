/*
 * NetworkInterface.cpp
 *
 *  Created on: 10 Mar 2020
 *      Author: David
 */

#include "NetworkInterface.h"
#include <Platform/RepRap.h>

NetworkInterface::NetworkInterface() noexcept
	: state(NetworkState::disabled)
{
	for (size_t i = 0; i < NumSelectableProtocols; ++i)
	{
		ipAddresses[i] = AcceptAnyIp;
		portNumbers[i] = DefaultPortNumbers[i];
		protocolEnabled[i] = (i == HttpProtocol);
	}
}

void NetworkInterface::SetState(NetworkState::RawType newState) noexcept
{
	state = newState;
	reprap.NetworkUpdated();
}

GCodeResult NetworkInterface::EnableProtocol(NetworkProtocol protocol, int port, uint32_t ip, int secure, const StringRef& reply) noexcept
{
	if (secure > 0)
	{
		reply.copy("this firmware does not support TLS");
	}
	else
	{
		if (protocol < NumSelectableProtocols)
		{
			const TcpPort portToUse = (port < 0) ? DefaultPortNumbers[protocol] : port;
			MutexLocker lock(interfaceMutex);

			if (GetState() == NetworkState::active && (portToUse != portNumbers[protocol] || ip != ipAddresses[protocol]))
			{
				// We need to shut down and restart the protocol if it is active because the port number has changed.
				// Note that clients will probably be unable to close their connections gracefully.
				IfaceShutdownProtocol(protocol, false);
				protocolEnabled[protocol] = false;
			}
			ipAddresses[protocol] = ip;
			portNumbers[protocol] = portToUse;
			if (!protocolEnabled[protocol])
			{
				protocolEnabled[protocol] = true;
				if (GetState() == NetworkState::active)
				{
					// Only start listeners here, connection requests for clients are made in the main loop.
					IfaceStartProtocol(protocol);
					// mDNS announcement is done by the WiFi Server firmware
				}
			}
			ReportOneProtocol(protocol, reply);
			return GCodeResult::ok;
		}

		reply.copy("invalid protocol parameter");
	}
	return GCodeResult::error;
}

GCodeResult NetworkInterface::DisableProtocol(NetworkProtocol protocol, const StringRef& reply, bool shutdown) noexcept
{
	if (protocol < NumSelectableProtocols)
	{
		MutexLocker lock(interfaceMutex);

		if (shutdown && GetState() == NetworkState::active)
		{
			IfaceShutdownProtocol(protocol, true);
		}
		protocolEnabled[protocol] = false;
		ReportOneProtocol(protocol, reply);
		return GCodeResult::ok;
	}

	reply.copy("Invalid protocol parameter");
	return GCodeResult::error;
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
