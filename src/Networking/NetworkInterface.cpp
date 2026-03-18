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
	static constexpr TcpPort DefaultTlsPortNumbers[NumSelectableProtocols] =
	{
		DefaultHttpsPort, DefaultFtpsPort, DefaultTelnetsPort, 0, 0
	};
	for (size_t i = 0; i < NumSelectableProtocols; ++i)
	{
		ipAddresses[i] = AcceptAnyIp;
		portNumbers[i] = DefaultPortNumbers[i];
		tlsPortNumbers[i] = DefaultTlsPortNumbers[i];
		protocolEnabled[i] = false;
		tlsProtocolEnabled[i] = false;
	}
}

void NetworkInterface::SetState(NetworkState::RawType newState) noexcept
{
	state = newState;
	reprap.NetworkUpdated();
}

GCodeResult NetworkInterface::EnableProtocol(NetworkProtocol protocol, int port, uint32_t ip, int secure, const StringRef& reply) noexcept
{
	if (false
#if SUPPORT_HTTP
		|| protocol == HttpProtocol
#endif
#if SUPPORT_FTP
	    || protocol == FtpProtocol
#endif
#if SUPPORT_TELNET
		|| protocol == TelnetProtocol
#endif
#if SUPPORT_MULTICAST_DISCOVERY
		|| protocol == MulticastDiscoveryProtocol
#endif
#if SUPPORT_MQTT
		|| protocol == MqttProtocol
#endif
		)
	{
		if (secure > 0)
		{
			if (!SupportsTls())
			{
				reply.copy("TLS not supported by this interface");
				return GCodeResult::error;
			}
			// Enabling TLS variant: P parameter sets the secure port, plain port unchanged
			const TcpPort newTlsPort = (port >= 0) ? (TcpPort)port : tlsPortNumbers[protocol];

			MutexLocker lock(interfaceMutex);

			if (GetState() == NetworkState::active && newTlsPort != tlsPortNumbers[protocol])
			{
				// Secure port changed — shut down so StartProtocol recreates the TLS listener
				IfaceShutdownProtocol(protocol, false);
				tlsProtocolEnabled[protocol] = false;
			}
			tlsPortNumbers[protocol] = newTlsPort;
			if (!tlsProtocolEnabled[protocol])
			{
				if (!LoadTlsCertificates(reply))
				{
					return GCodeResult::error;
				}
				tlsProtocolEnabled[protocol] = true;
				if (GetState() == NetworkState::active)
				{
					IfaceStartProtocol(protocol);
				}
			}
		}
		else
		{
			const TcpPort portToUse = (port >= 0) ? (TcpPort)port : DefaultPortNumbers[protocol];

			MutexLocker lock(interfaceMutex);

			if (GetState() == NetworkState::active && (portToUse != portNumbers[protocol] || ip != ipAddresses[protocol]))
			{
				// Plain port changed — shut down and restart
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
					IfaceStartProtocol(protocol);
				}
			}
		}
		ReportOneProtocol(protocol, reply);
		return GCodeResult::ok;
	}

	reply.copy("invalid protocol parameter");
	return GCodeResult::error;
}

GCodeResult NetworkInterface::DisableProtocol(NetworkProtocol protocol, const StringRef& reply, bool shutdown) noexcept
{
	if (false
#if SUPPORT_HTTP
			|| protocol == HttpProtocol
#endif
#if SUPPORT_FTP
			|| protocol == FtpProtocol
#endif
#if SUPPORT_TELNET
			|| protocol == TelnetProtocol
#endif
#if SUPPORT_MULTICAST_DISCOVERY
			|| protocol == MulticastDiscoveryProtocol
#endif
#if SUPPORT_MQTT
			|| protocol == MqttProtocol
#endif
	)
	{
		MutexLocker lock(interfaceMutex);

		if (shutdown && GetState() == NetworkState::active)
		{
			IfaceShutdownProtocol(protocol, true);
		}
		protocolEnabled[protocol] = false;
		tlsProtocolEnabled[protocol] = false;
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
	if (protocolEnabled[protocol] && tlsProtocolEnabled[protocol])
	{
		reply.lcatf("%s is enabled on port %u, TLS on port %u", ProtocolNames[protocol], portNumbers[protocol], tlsPortNumbers[protocol]);
	}
	else if (protocolEnabled[protocol])
	{
		reply.lcatf("%s is enabled on port %u", ProtocolNames[protocol], portNumbers[protocol]);
	}
	else if (tlsProtocolEnabled[protocol])
	{
		reply.lcatf("%s TLS is enabled on port %u", ProtocolNames[protocol], tlsPortNumbers[protocol]);
	}
	else
	{
		reply.lcatf("%s is disabled", ProtocolNames[protocol]);
	}
}

// End
