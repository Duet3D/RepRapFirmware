/*
 * Network.cpp
 *
 *  Created on: 13 Dec 2016
 *      Author: David
 */

#include "W5500Interface.h"
#include "W5500Socket.h"
#include "Wiznet/Ethernet/wizchip_conf.h"
#include "Wiznet/Internet/DHCP/dhcp.h"
#include "NetworkBuffer.h"
#include "Platform.h"
#include "RepRap.h"
#include "HttpResponder.h"
#include "FtpResponder.h"
#include "TelnetResponder.h"
#include "MdnsResponder.h"
#include "General/IP4String.h"

W5500Interface::W5500Interface(Platform& p)
	: platform(p), lastTickMillis(0), ftpDataSocket(0), state(NetworkState::disabled), activated(false)
{
	// Create the sockets
	for (W5500Socket*& skt : sockets)
	{
		skt = new W5500Socket(this);
	}

	mdnsSocket = new W5500Socket(this);
	mdnsResponder = new MdnsResponder(mdnsSocket);

	for (size_t i = 0; i < NumProtocols; ++i)
	{
		portNumbers[i] = DefaultPortNumbers[i];
		protocolEnabled[i] = (i == HttpProtocol);
	}
}

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(_ret) OBJECT_MODEL_FUNC_BODY(W5500Interface, _ret)

const ObjectModelTableEntry W5500Interface::objectModelTable[] =
{
	// These entries must be in alphabetical order
	{ "gateway", OBJECT_MODEL_FUNC(&(self->gateway)), TYPE_OF(IPAddress), ObjectModelTableEntry::none },
	{ "ip", OBJECT_MODEL_FUNC(&(self->ipAddress)), TYPE_OF(IPAddress), ObjectModelTableEntry::none },
	{ "name", OBJECT_MODEL_FUNC_NOSELF("ethernet"), TYPE_OF(const char *), ObjectModelTableEntry::none },
	{ "netmask", OBJECT_MODEL_FUNC(&(self->netmask)), TYPE_OF(IPAddress), ObjectModelTableEntry::none },
};

DEFINE_GET_OBJECT_MODEL_TABLE(W5500Interface)

#endif

void W5500Interface::Init()
{
	interfaceMutex.Create("W5500");

	// Ensure that the W5500 chip is in the reset state
	pinMode(W5500ResetPin, OUTPUT_LOW);
	lastTickMillis = millis();

	SetIPAddress(DefaultIpAddress, DefaultNetMask, DefaultGateway);
	memcpy(macAddress, platform.GetDefaultMacAddress(), sizeof(macAddress));
}

GCodeResult W5500Interface::EnableProtocol(NetworkProtocol protocol, int port, int secure, const StringRef& reply)
{
	if (secure != 0 && secure != -1)
	{
		reply.copy("this firmware does not support TLS");
		return GCodeResult::error;
	}

	if (protocol < NumProtocols)
	{
		MutexLocker lock(interfaceMutex);

		const Port portToUse = (port < 0) ? DefaultPortNumbers[protocol] : port;
		if (portToUse != portNumbers[protocol] && state == NetworkState::active)
		{
			// We need to shut down and restart the protocol if it is active because the port number has changed
			protocolEnabled[protocol] = false;
			ResetSockets();
		}
		portNumbers[protocol] = portToUse;
		if (!protocolEnabled[protocol])
		{
			protocolEnabled[protocol] = true;
			if (state == NetworkState::active)
			{
				ResetSockets();
				if (state == NetworkState::active)
				{
					mdnsResponder->Announce();
				}
			}
		}
		ReportOneProtocol(protocol, reply);
		return GCodeResult::ok;
	}

	reply.copy("Invalid protocol parameter");
	return GCodeResult::error;
}

bool W5500Interface::IsProtocolEnabled(NetworkProtocol protocol)
{
	return (protocol < NumProtocols) ? protocolEnabled[protocol] : false;
}

GCodeResult W5500Interface::DisableProtocol(NetworkProtocol protocol, const StringRef& reply)
{
	if (protocol < NumProtocols)
	{
		MutexLocker lock(interfaceMutex);

		if (state == NetworkState::active)
		{
			ResetSockets();
		}
		protocolEnabled[protocol] = false;
		ReportOneProtocol(protocol, reply);
		return GCodeResult::ok;
	}

	reply.copy("Invalid protocol parameter");
	return GCodeResult::error;
}

// Report the protocols and ports in use
GCodeResult W5500Interface::ReportProtocols(const StringRef& reply) const
{
	reply.Clear();
	for (size_t i = 0; i < NumProtocols; ++i)
	{
		ReportOneProtocol(i, reply);
	}
	return GCodeResult::ok;
}

void W5500Interface::ReportOneProtocol(NetworkProtocol protocol, const StringRef& reply) const
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

// This is called at the end of config.g processing.
// Start the network if it was enabled
void W5500Interface::Activate()
{
	if (!activated)
	{
		activated = true;
		if (state == NetworkState::enabled)
		{
			Start();
		}
		else
		{
			platform.Message(NetworkInfoMessage, "Network disabled.\n");
		}
	}
}

void W5500Interface::Exit()
{
	Stop();
}

// Get the network state into the reply buffer, returning true if there is some sort of error
GCodeResult W5500Interface::GetNetworkState(const StringRef& reply)
{
	const IPAddress config_ip = platform.GetIPAddress();
	const int enableState = EnableState();
	reply.printf("Network is %s, configured IP address: %s, actual IP address: %s",
			(enableState == 0) ? "disabled" : "enabled",
					IP4String(config_ip).c_str(), IP4String(ipAddress).c_str());
	return GCodeResult::ok;
}

// Update the MAC address
void W5500Interface::SetMacAddress(const uint8_t mac[])
{
	for (size_t i = 0; i < 6; i++)
	{
		macAddress[i] = mac[i];
	}
}

// Start up the network
void W5500Interface::Start()
{
	MutexLocker lock(interfaceMutex);

	SetIPAddress(platform.GetIPAddress(), platform.NetMask(), platform.GateWay());
	pinMode(W5500ResetPin, OUTPUT_LOW);
	delayMicroseconds(550);						// W550 reset pulse must be at least 500us long
	IoPort::WriteDigital(W5500ResetPin, true);	// raise /Reset pin
	delay(55);									// W5500 needs 50ms to start up

#ifdef USE_3K_BUFFERS
	static const uint8_t bufSizes[8] = { 3, 3, 3, 3, 1, 1, 1, 1 };	// 3K buffers for http, 1K for everything else (FTP will be slow)
#else
	static const uint8_t bufSizes[8] = { 2, 2, 2, 2, 2, 2, 2, 2 };	// 2K buffers for everything
#endif

	setPHYCFGR(PHYCFGR_OPMD | PHYCFGR_OPMDC_ALLA);					// set auto negotiation and reset the PHY

	wizchip_init(bufSizes, bufSizes);
	setSHAR(macAddress);
	setSIPR(ipAddress);
	setGAR(gateway);
	setSUBR(netmask);

	setPHYCFGR(PHYCFGR_OPMD | PHYCFGR_OPMDC_ALLA | ~PHYCFGR_RST);	// remove the reset

	state = NetworkState::establishingLink;
}

// Stop the network
void W5500Interface::Stop()
{
	if (state != NetworkState::disabled)
	{
		MutexLocker lock(interfaceMutex);

		if (usingDhcp)
		{
			DHCP_stop();
		}
		digitalWrite(W5500ResetPin, false);		// put the W5500 back into reset
		state = NetworkState::disabled;
	}
}

// Main spin loop. If 'full' is true then we are being called from the main spin loop. If false then we are being called during HSMCI idle time.
void W5500Interface::Spin(bool full)
{
	switch(state)
	{
	case NetworkState::enabled:
	case NetworkState::disabled:
	default:
		// Nothing to do
		break;

	case NetworkState::establishingLink:
		{
			MutexLocker lock(interfaceMutex);

			if (full && wizphy_getphylink() == PHY_LINK_ON)
			{
				usingDhcp = ipAddress.IsNull();
				if (usingDhcp)
				{
					// IP address is all zeros, so use DHCP
//					debugPrintf("Link established, getting IP address\n");
					DHCP_init(DhcpSocketNumber, platform.Random(), reprap.GetNetwork().GetHostname());
					lastTickMillis = millis();
					state = NetworkState::obtainingIP;
				}
				else
				{
//					debugPrintf("Link established, network running\n");
					state = NetworkState::connected;
				}
			}
		}
		break;

	case NetworkState::obtainingIP:
		if (full)
		{
			MutexLocker lock(interfaceMutex);

			if (wizphy_getphylink() == PHY_LINK_ON)
			{
				const uint32_t now = millis();
				if (now - lastTickMillis >= 1000)
				{
					lastTickMillis += 1000;
					DHCP_time_handler();
				}
				const DhcpRunResult ret = DHCP_run();
				if (ret == DhcpRunResult::DHCP_IP_ASSIGN)
				{
//					debugPrintf("IP address obtained, network running\n");
					getSIPR(ipAddress);
					state = NetworkState::connected;
				}
			}
			else
			{
//				debugPrintf("Lost phy link\n");
				DHCP_stop();
				TerminateSockets();
				state = NetworkState::establishingLink;
			}
		}
		break;

	case NetworkState::connected:
		if (full)
		{
			InitSockets();
			platform.MessageF(NetworkInfoMessage, "Network running, IP address = %s\n", IP4String(ipAddress).c_str());
			mdnsResponder->Announce();
			state = NetworkState::active;
		}
		break;

	case NetworkState::active:
		{
			MutexLocker lock(interfaceMutex);

			// Check that the link is still up
			if (wizphy_getphylink() == PHY_LINK_ON)
			{
				// Maintain DHCP
				if (full && usingDhcp)
				{
					const uint32_t now = millis();
					if (now - lastTickMillis >= 1000)
					{
						lastTickMillis += 1000;
						DHCP_time_handler();
					}
					const DhcpRunResult ret = DHCP_run();
					if (ret == DhcpRunResult::DHCP_IP_CHANGED || ret == DhcpRunResult::DHCP_IP_ASSIGN)
					{
//						debugPrintf("IP address changed\n");
						getSIPR(ipAddress);
						mdnsResponder->Announce();
					}
				}

				// Poll the next TCP socket
				sockets[nextSocketToPoll]->Poll(full);

				// Keep mDNS alive
				mdnsSocket->Poll(full);
				if (full)
				{
					mdnsResponder->Spin();
				}

				// Move on to the next TCP socket for next time
				++nextSocketToPoll;
				if (nextSocketToPoll == NumW5500TcpSockets)
				{
					nextSocketToPoll = 0;
				}
			}
			else if (full)
			{
//				debugPrintf("Lost phy link\n");
				if (usingDhcp)
				{
					DHCP_stop();
				}
				TerminateSockets();
				state = NetworkState::establishingLink;
			}
		}
		break;
	}
}

void W5500Interface::Diagnostics(MessageType mtype)
{
	uint8_t phycfgr;
	{
		MutexLocker lock(interfaceMutex);
		phycfgr = getPHYCFGR();
	}
	const char * const linkSpeed = ((phycfgr & 1) == 0) ? "down" : ((phycfgr & 2) != 0) ? "100Mbps" : "10Mbps";
	const char * const linkDuplex = ((phycfgr & 1) == 0) ? "" : ((phycfgr & 4) != 0) ? " full duplex" : " half duplex";
	platform.MessageF(mtype, "Interface state %d, link %s%s\n", (int)state, linkSpeed, linkDuplex);
}

// Enable or disable the network
GCodeResult W5500Interface::EnableInterface(int mode, const StringRef& ssid, const StringRef& reply)
{
	if (!activated)
	{
		state = (mode <= 0) ? NetworkState::disabled : NetworkState::enabled;
	}
	else if (mode <= 0)
	{
		if (state != NetworkState::disabled)
		{
			Stop();
			platform.Message(NetworkInfoMessage, "Network stopped\n");
		}

	}
	else if (state == NetworkState::disabled)
	{
		state = NetworkState::enabled;
		Start();
	}
	return GCodeResult::ok;
}

int W5500Interface::EnableState() const
{
	return (state == NetworkState::disabled) ? 0 : 1;
}

void W5500Interface::UpdateHostname(const char *name) /*override*/
{
	mdnsResponder->Announce();
}

void W5500Interface::SetIPAddress(IPAddress p_ipAddress, IPAddress p_netmask, IPAddress p_gateway)
{
	ipAddress = p_ipAddress;
	netmask = p_netmask;
	gateway = p_gateway;
}

void W5500Interface::OpenDataPort(Port port)
{
	sockets[ftpDataSocket]->Init(ftpDataSocket, port, FtpDataProtocol);
}

// Close FTP data port and purge associated resources
void W5500Interface::TerminateDataPort()
{
	sockets[ftpDataSocket]->Terminate();
}

void W5500Interface::InitSockets()
{
	ResetSockets();
	nextSocketToPoll = 0;

	mdnsSocket->Init(MdnsSocketNumber, MdnsPort, MdnsProtocol);
}

// Note, the following is called to initialise the sockets as well as to reset them. Therefore it must work with sockets that have never been initialised.
void W5500Interface::ResetSockets()
{
	// See how many sockets are available
	size_t numFtpSockets = protocolEnabled[FtpProtocol] ? 2 : 0;
	size_t numTelnetSockets = protocolEnabled[TelnetProtocol] ? 1 : 0;
	size_t numHttpSockets = protocolEnabled[HttpProtocol] ? NumW5500TcpSockets - numFtpSockets - numTelnetSockets : 0;

	// Terminate every connection and reinitialize them if applicable
	for (SocketNumber skt = 0; skt < NumW5500TcpSockets; ++skt)
	{
		sockets[skt]->TerminateAndDisable();
		if (skt < numHttpSockets)
		{
			// HTTP
			sockets[skt]->Init(skt, portNumbers[HttpProtocol], HttpProtocol);
		}
		else if (skt < numHttpSockets + numFtpSockets)
		{
			if (skt == numHttpSockets)
			{
				// FTP
				sockets[skt]->Init(skt, portNumbers[FtpProtocol], FtpProtocol);
			}
			else
			{
				// FTP DATA is initialised during runtime
				ftpDataSocket = skt;
			}
		}
		else if (skt < numHttpSockets + numFtpSockets + numTelnetSockets)
		{
			// Telnet
			sockets[skt]->Init(skt, portNumbers[TelnetProtocol], TelnetProtocol);
		}
	}
}

void W5500Interface::TerminateSockets()
{
	for (SocketNumber skt = 0; skt < NumW5500TcpSockets; ++skt)
	{
		sockets[skt]->Terminate();
	}

	mdnsSocket->TerminateAndDisable();
}

// End
