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
		tlsProtocolPending[i] = false;
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
			// Enabling TLS variant: P parameter sets the secure port, plain port unchanged
			const TcpPort newTlsPort = (port >= 0) ? (TcpPort)port : tlsPortNumbers[protocol];

			MutexLocker lock(interfaceMutex);

			if (!SupportsTls())
			{
				// WiFi: M552 T1 latches the TLS probe asynchronously, so a typical config.g
				// running M586 T1 right after M552 T1 lands here. Defer the listener bind until
				// ApplyPendingTlsProtocols() is called from HandlePendingTlsRequest. Ethernet's
				// SupportsTls flips synchronously inside EnableInterface, so this branch is rare there
				tlsPortNumbers[protocol] = newTlsPort;
				tlsProtocolPending[protocol] = true;
				reply.copy("TLS not yet ready, listener will start once TLS support is up");
				return GCodeResult::ok;
			}
			tlsProtocolPending[protocol] = false;

			if (GetState() == NetworkState::active && newTlsPort != tlsPortNumbers[protocol])
			{
				// Secure port changed — shut down so StartProtocol recreates the TLS listener
				IfaceShutdownProtocol(protocol, false);
				tlsProtocolEnabled[protocol] = false;
			}
			tlsPortNumbers[protocol] = newTlsPort;
			if (!tlsProtocolEnabled[protocol])
			{
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

// Apply any M586 T1 calls that were deferred because SupportsTls() was false at the time.
// The pending bit captures intent; the port was already saved into tlsPortNumbers[]. Re-run the
// enable path here now that TLS is up. Called from WiFiInterface::HandlePendingTlsRequest once
// the asynchronous networkEnableTls probe has set supportsTls = true
void NetworkInterface::ApplyPendingTlsProtocols() noexcept
{
	if (!SupportsTls())
	{
		return;
	}
	for (NetworkProtocol protocol = 0; protocol < NumSelectableProtocols; ++protocol)
	{
		if (!tlsProtocolPending[protocol])
		{
			continue;
		}
		MutexLocker lock(interfaceMutex);
		tlsProtocolPending[protocol] = false;
		if (!tlsProtocolEnabled[protocol])
		{
			tlsProtocolEnabled[protocol] = true;
			if (GetState() == NetworkState::active)
			{
				IfaceStartProtocol(protocol);
			}
		}
	}
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
	// Only HTTP/FTP/TELNET have meaningful TLS variants here. Multicast Discovery is UDP, MQTT isn't TLS-wrapped
	if (SupportsTls() && (protocol == HttpProtocol || protocol == FtpProtocol || protocol == TelnetProtocol))
	{
		if (tlsProtocolEnabled[protocol])
		{
			reply.lcatf("%sS is enabled on port %u", ProtocolNames[protocol], tlsPortNumbers[protocol]);
		}
		else
		{
			reply.lcatf("%sS is disabled", ProtocolNames[protocol]);
		}
	}
}

// End
