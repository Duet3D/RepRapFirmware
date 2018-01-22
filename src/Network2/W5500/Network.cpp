/*
 * Network.cpp
 *
 *  Created on: 13 Dec 2016
 *      Author: David
 */

#include "Network.h"
#include "Wiznet/Ethernet/wizchip_conf.h"
#include "Wiznet/Internet/DHCP/dhcp.h"
#include "NetworkBuffer.h"
#include "Platform.h"
#include "RepRap.h"
#include "HttpResponder.h"
#include "FtpResponder.h"
#include "TelnetResponder.h"
#include "Libraries/General/IP4String.h"

const Port DefaultPortNumbers[NumProtocols] = { DefaultHttpPort, DefaultFtpPort, DefaultTelnetPort };
const char * const ProtocolNames[NumProtocols] = { "HTTP", "FTP", "TELNET" };

Network::Network(Platform& p)
	: platform(p), nextResponderToPoll(nullptr), lastTickMillis(0),
	  state(NetworkState::disabled), activated(false)
{
	for (size_t i = 0; i < NumProtocols; ++i)
	{
		portNumbers[i] = DefaultPortNumbers[i];
		protocolEnabled[i] = (i == HttpProtocol);
	}

	// Create the responders
	responders = telnetResponder = new TelnetResponder(nullptr);
	responders = ftpResponder = new FtpResponder(responders);
	for (unsigned int i = 0; i < NumHttpResponders; ++i)
	{
		responders = new HttpResponder(responders);
	}
}

void Network::Init()
{
	// Ensure that the W5500 chip is in the reset state
	pinMode(W5500ResetPin, OUTPUT_LOW);
	longWait = millis();
	lastTickMillis = millis();

	NetworkBuffer::AllocateBuffers(NetworkBufferCount);

	SetIPAddress(DefaultIpAddress, DefaultNetMask, DefaultGateway);
	strcpy(hostname, HOSTNAME);
}

void Network::EnableProtocol(int protocol, int port, int secure, StringRef& reply)
{
	if (secure != 0 && secure != -1)
	{
		reply.copy("Error: this firmware does not support TLS");
	}
	else if (protocol >= 0 && protocol < (int)NumProtocols)
	{
		const Port portToUse = (port < 0) ? DefaultPortNumbers[protocol] : port;
		if (portToUse != portNumbers[protocol] && state == NetworkState::active)
		{
			// We need to shut down and restart the protocol if it is active because the port number has changed
			ShutdownProtocol(protocol);
			protocolEnabled[protocol] = false;
		}
		portNumbers[protocol] = portToUse;
		if (!protocolEnabled[protocol])
		{
			protocolEnabled[protocol] = true;
			if (state == NetworkState::active)
			{
				StartProtocol(protocol);
#if 0	// mdns not implemented yet
				if (state == NetworkState::active)
				{
					DoMdnsAnnounce();
				}
#endif
			}
		}
		ReportOneProtocol(protocol, reply);
	}
	else
	{
		reply.copy("Invalid protocol parameter");
	}
}

void Network::DisableProtocol(int protocol, StringRef& reply)
{
	if (protocol >= 0 && protocol < (int)NumProtocols)
	{
		if (state == NetworkState::active)
		{
			ShutdownProtocol(protocol);
		}
		protocolEnabled[protocol] = false;
		ReportOneProtocol(protocol, reply);
	}
	else
	{
		reply.copy("Invalid protocol parameter");
	}
}

void Network::StartProtocol(Protocol protocol)
{
	switch(protocol)
	{
	case HttpProtocol:
		for (SocketNumber skt = 0; skt < NumHttpSockets; ++skt)
		{
			sockets[skt].Init(skt, portNumbers[protocol], protocol);
		}
		break;

	case FtpProtocol:
		sockets[FtpSocketNumber].Init(FtpSocketNumber, portNumbers[protocol], protocol);
		break;

	case TelnetProtocol:
		sockets[TelnetSocketNumber].Init(TelnetSocketNumber, portNumbers[protocol], protocol);
		break;

	default:
		break;
	}
}

void Network::ShutdownProtocol(Protocol protocol)
{
	for (NetworkResponder* r = responders; r != nullptr; r = r->GetNext())
	{
		r->Terminate(protocol);
	}

	switch(protocol)
	{
	case HttpProtocol:
		for (SocketNumber skt = 0; skt < NumHttpSockets; ++skt)
		{
			sockets[skt].TerminateAndDisable();
		}
		break;

	case FtpProtocol:
		sockets[FtpSocketNumber].TerminateAndDisable();
		sockets[FtpDataSocketNumber].TerminateAndDisable();
		break;

	case TelnetProtocol:
		sockets[TelnetSocketNumber].TerminateAndDisable();
		break;

	default:
		break;
	}
}

// Report the protocols and ports in use
void Network::ReportProtocols(StringRef& reply) const
{
	reply.Clear();
	for (size_t i = 0; i < NumProtocols; ++i)
	{
		if (i != 0)
		{
			reply.cat('\n');
		}
		ReportOneProtocol(i, reply);
	}
}

void Network::ReportOneProtocol(Protocol protocol, StringRef& reply) const
{
	if (protocolEnabled[protocol])
	{
		reply.catf("%s is enabled on port %u", ProtocolNames[protocol], portNumbers[protocol]);
	}
	else
	{
		reply.catf("%s is disabled", ProtocolNames[protocol]);
	}
}

// This is called at the end of config.g processing.
// Start the network if it was enabled
void Network::Activate()
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

void Network::Exit()
{
	Stop();
}

// Get the network state into the reply buffer, returning true if there is some sort of error
bool Network::GetNetworkState(StringRef& reply)
{
	const uint8_t * const config_ip = platform.GetIPAddress();
	const int enableState = EnableState();
	reply.printf("Network is %s, configured IP address: %s, actual IP address: %s",
			(enableState == 0) ? "disabled" : "enabled",
					IP4String(config_ip).c_str(), IP4String(ipAddress).c_str());
	return false;
}

// Start up the network
void Network::Start()
{
	SetIPAddress(platform.GetIPAddress(), platform.NetMask(), platform.GateWay());
	pinMode(W5500ResetPin, OUTPUT_LOW);
	delayMicroseconds(550);						// W550 reset pulse must be at least 500us long
	IoPort::WriteDigital(W5500ResetPin, HIGH);	// raise /Reset pin
	delay(55);									// W5500 needs 50ms to start up

#ifdef USE_3K_BUFFERS
	static const uint8_t bufSizes[8] = { 3, 3, 3, 3, 1, 1, 1, 1 };	// 3K buffers for http, 1K for everything else (FTP will be slow)
#else
	static const uint8_t bufSizes[8] = { 2, 2, 2, 2, 2, 2, 2, 2 };	// 2K buffers for everything
#endif

	wizchip_init(bufSizes, bufSizes);

	setSHAR(platform.MACAddress());
	setSIPR(ipAddress);
	setGAR(gateway);
	setSUBR(netmask);

	state = NetworkState::establishingLink;
}

// Stop the network
void Network::Stop()
{
	if (state != NetworkState::disabled)
	{
		for (NetworkResponder *r = responders; r != nullptr; r = r->GetNext())
		{
			r->Terminate(AnyProtocol);
		}
		if (usingDhcp)
		{
			DHCP_stop();
		}
		digitalWrite(W5500ResetPin, LOW);		// put the W5500 back into reset
		state = NetworkState::disabled;
	}
}

// Main spin loop. If 'full' is true then we are being called from the main spin loop. If false then we are being called during HSMCI idle time.
void Network::Spin(bool full)
{
	switch(state)
	{
	case NetworkState::enabled:
	case NetworkState::disabled:
	default:
		// Nothing to do
		break;

	case NetworkState::establishingLink:
		if (full && wizphy_getphylink() == PHY_LINK_ON)
		{
			usingDhcp = (ipAddress[0] == 0 && ipAddress[1] == 0 && ipAddress[2] == 0 && ipAddress[3] == 0);
			if (usingDhcp)
			{
				// IP address is all zeros, so use DHCP
//				debugPrintf("Link established, getting IP address\n");
				DHCP_init(DhcpSocketNumber, hostname);
				lastTickMillis = millis();
				state = NetworkState::obtainingIP;
			}
			else
			{
//				debugPrintf("Link established, network running\n");
				state = NetworkState::connected;
			}
		}
		break;

	case NetworkState::obtainingIP:
		if (full)
		{
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
					// Send mDNS announcement so that some routers can perform hostname mapping
					// if this board is connected via a non-IGMP capable WiFi bridge (like the TP-Link WR701N)
					//mdns_announce();
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
			state = NetworkState::active;
		}
		break;

	case NetworkState::active:
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
				if (ret == DhcpRunResult::DHCP_IP_CHANGED)
				{
//					debugPrintf("IP address changed\n");
					getSIPR(ipAddress);
				}
			}

			// Poll the next TCP socket
			sockets[nextSocketToPoll].Poll(full);

			// Move on to the next TCP socket for next time
			++nextSocketToPoll;
			if (nextSocketToPoll == NumTcpSockets)
			{
				nextSocketToPoll = 0;
			}

			// Poll the responders
			if (full)
			{
				NetworkResponder *nr = nextResponderToPoll;
				bool doneSomething = false;
				do
				{
					if (nr == nullptr)
					{
						nr = responders;		// 'responders' can't be null at this point
					}
					doneSomething = nr->Spin();
					nr = nr->GetNext();
				} while (!doneSomething && nr != nextResponderToPoll);
				nextResponderToPoll = nr;
			}
		}
		else if (full)
		{
//			debugPrintf("Lost phy link\n");
			if (usingDhcp)
			{
				DHCP_stop();
			}
			TerminateSockets();
			state = NetworkState::establishingLink;
		}
		break;
	}

	if (full)
	{
		platform.ClassReport(longWait);
	}
}

void Network::Diagnostics(MessageType mtype)
{
	platform.MessageF(mtype, "=== Network ===\nState: %d\n", (int)state);
	HttpResponder::CommonDiagnostics(mtype);
	platform.Message(mtype, "Responder states:");
	for (NetworkResponder *r = responders; r != nullptr; r = r->GetNext())
	{
		r->Diagnostics(mtype);
	}
	platform.Message(mtype, "\n");
}

// Enable or disable the network
void Network::Enable(int mode, StringRef& reply)
{
	if (!activated)
	{
		state = (mode == 0) ? NetworkState::disabled : NetworkState::enabled;
	}
	else if (mode == 0)
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
}

int Network::EnableState() const
{
	return (state == NetworkState::disabled) ? 0 : 1;
}

void Network::SetIPAddress(const uint8_t p_ipAddress[], const uint8_t p_netmask[], const uint8_t p_gateway[])
{
	memcpy(ipAddress, p_ipAddress, sizeof(ipAddress));
	memcpy(netmask, p_netmask, sizeof(netmask));
	memcpy(gateway, p_gateway, sizeof(gateway));
}

void Network::SetHostname(const char *name)
{
	size_t i = 0;
	while (*name && i < ARRAY_UPB(hostname))
	{
		char c = *name++;
		if (c >= 'A' && c <= 'Z')
		{
			c += 'a' - 'A';
		}

		if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || (c == '-') || (c == '_'))
		{
			hostname[i++] = c;
		}
	}

	if (i != 0)
	{
		hostname[i] = 0;
	}
	else
	{
		strcpy(hostname, HOSTNAME);
	}
}

void Network::OpenDataPort(Port port)
{
	sockets[FtpDataSocketNumber].Init(FtpDataSocketNumber, port, FtpDataProtocol);
}

// Close FTP data port and purge associated resources
void Network::TerminateDataPort()
{
	sockets[FtpDataSocketNumber].Terminate();
}

void Network::InitSockets()
{
	for (size_t i = 0; i < NumProtocols; ++i)
	{
		if (protocolEnabled[i])
		{
			StartProtocol(i);
		}
	}
	nextSocketToPoll = 0;
}

// The following is called by the FTP responder to stop listening on the FTP data port
// For the W5500 listening stop automatically when the port is terminated, so we don't need anything here
void Network::DataPortClosing()
{
	// nothing needed here
}

void Network::TerminateSockets()
{
	for (SocketNumber skt = 0; skt < NumTcpSockets; ++skt)
	{
		sockets[skt].Terminate();
	}
}

// Find a responder to process a new connection
bool Network::FindResponder(Socket *skt, Protocol protocol)
{
	for (NetworkResponder *r = responders; r != nullptr; r = r->GetNext())
	{
		if (r->Accept(skt, protocol))
		{
			return true;
		}
	}
	return false;
}

void Network::HandleHttpGCodeReply(const char *msg)
{
	HttpResponder::HandleGCodeReply(msg);
}

void Network::HandleTelnetGCodeReply(const char *msg)
{
	telnetResponder->HandleGCodeReply(msg);
}

void Network::HandleHttpGCodeReply(OutputBuffer *buf)
{
	HttpResponder::HandleGCodeReply(buf);
}

void Network::HandleTelnetGCodeReply(OutputBuffer *buf)
{
	telnetResponder->HandleGCodeReply(buf);
}

uint32_t Network::GetHttpReplySeq()
{
	return HttpResponder::GetReplySeq();
}

// End
