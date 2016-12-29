/*
 * Network.cpp
 *
 *  Created on: 13 Dec 2016
 *      Author: David
 */

#include "Network.h"
#include "NetworkTransaction.h"
#include "Platform.h"
#include "wizchip_conf.h"
#include "Wiznet/Internet/DHCP/dhcp.h"

void Network::SetIPAddress(const uint8_t p_ipAddress[], const uint8_t p_netmask[], const uint8_t p_gateway[])
{
	memcpy(ipAddress, p_ipAddress, sizeof(ipAddress));
	memcpy(netmask, p_netmask, sizeof(netmask));
	memcpy(gateway, p_gateway, sizeof(gateway));
}

Network::Network(Platform* p)
	: platform(p), lastTickMillis(0),
	  httpPort(DefaultHttpPort), state(NetworkState::disabled), activated(false)
{
}

void Network::Init()
{
	// Ensure that the W5500 chip is in the reset state
	pinMode(EspResetPin, OUTPUT_LOW);
	state = NetworkState::disabled;
	longWait = platform->Time();
	lastTickMillis = millis();

	NetworkBuffer::AllocateBuffers(NetworkBufferCount);
	NetworkTransaction::AllocateTransactions(NetworkTransactionCount);

	SetIPAddress(DefaultIpAddress, DefaultNetMask, DefaultGateway);
	strcpy(hostname, HOSTNAME);
}

// This is called at the end of config.g processing.
// Start the network if it was enabled
void Network::Activate()
{
	activated = true;
	if (state == NetworkState::enabled)
	{
		Start();
	}
}

void Network::Exit()
{
	Stop();
}

// Main spin loop. If 'full' is true then we are being called from the main spin loop. If false then we are being called during HSMCI idle time.
void Network::Spin(bool full)
{
	switch(state)
	{
	case NetworkState::enabled:
	case NetworkState::disabled:
		// Nothing to do
		break;

	case NetworkState::establishingLink:
		if (full && wizphy_getphylink() == PHY_LINK_ON)
		{
			usingDhcp = (ipAddress[0] == 0 && ipAddress[1] == 0 && ipAddress[2] == 0 && ipAddress[3] == 0);
			if (usingDhcp)
			{
				debugPrintf("Link established, getting IP address\n");
				// IP address is all zeros, so use DHCP
				DHCP_init(DhcpSocketNumber, hostname);
				lastTickMillis = millis();
				state = NetworkState::obtainingIP;
			}
			else
			{
				debugPrintf("Link established, network running\n");
				InitSockets();
				state = NetworkState::active;
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
					debugPrintf("IP address obtained, network running\n");
					getSIPR(ipAddress);
					// Send mDNS announcement so that some routers can perform hostname mapping
					// if this board is connected via a non-IGMP capable WiFi bridge (like the TP-Link WR701N)
					//mdns_announce();
					InitSockets();
					state = NetworkState::active;
				}
			}
			else
			{
				debugPrintf("Lost phy link\n");
				DHCP_stop();
				TerminateSockets();
				state = NetworkState::establishingLink;
			}
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
					debugPrintf("IP address changed\n");
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
		}
		else if (full)
		{
			debugPrintf("Lost phy link\n");
			if (usingDhcp)
			{
				DHCP_stop();
			}
			TerminateSockets();
			state = NetworkState::establishingLink;
		}
		break;
	}

	platform->ClassReport(longWait);
}

void Network::Diagnostics(MessageType mtype)
{
	platform->Message(mtype, "=== Network ===\n");
	platform->MessageF(mtype, "State: %d\n", (int)state);
}

void Network::Start()
{
	SetIPAddress(platform->GetIPAddress(), platform->NetMask(), platform->GateWay());
	pinMode(EspResetPin, OUTPUT_LOW);
	delayMicroseconds(550);						// W550 reset pulse must be at least 500us long
	Platform::WriteDigital(EspResetPin, HIGH);	// raise /Reset pin
	delay(55);									// W5500 needs 50ms to start up

#ifdef USE_3K_BUFFERS
	static const uint8_t bufSizes[8] = { 3, 3, 3, 3, 1, 1, 1, 1 };	// 3K buffers for http, 1K for everything else (FTP will be slow)
#else
	static const uint8_t bufSizes[8] = { 2, 2, 2, 2, 2, 2, 2, 2 };	// 2K buffers for everything
#endif

	wizchip_init(bufSizes, bufSizes);

	setSHAR(platform->MACAddress());
	setSIPR(ipAddress);
	setGAR(gateway);
	setSUBR(netmask);

	state = NetworkState::establishingLink;
}

void Network::Stop()
{
	if (state != NetworkState::disabled)
	{
		if (usingDhcp)
		{
			DHCP_stop();
		}
		digitalWrite(EspResetPin, LOW);	// put the ESP back into reset
		state = NetworkState::disabled;
	}
}

void Network::Enable()
{
	if (state == NetworkState::disabled)
	{
		state = NetworkState::enabled;
		if (activated)
		{
			Start();
		}
	}
}

void Network::Disable()
{
	if (activated && state != NetworkState::disabled)
	{
		Stop();
		platform->Message(GENERIC_MESSAGE, "Network stopped\n");
	}
}

bool Network::IsEnabled() const
{
	return state != NetworkState::disabled;
}

const uint8_t *Network::GetIPAddress() const
{
	return ipAddress;
}

void Network::SetHttpPort(uint16_t port)
{
	httpPort = port;
}

uint16_t Network::GetHttpPort() const
{
	return httpPort;
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

	if (i)
	{
		hostname[i] = 0;
	}
	else
	{
		strcpy(hostname, HOSTNAME);
	}
}

bool Network::Lock()
{
	return true;
}

void Network::Unlock()
{
}

bool Network::InLwip() const
{
	return false;
}

// This is called by the web server to get the next networking transaction.
//
// If conn is NoConnection, the transaction from the head of readyTransactions will be retrieved.
// If conn is not NoConnection, the first transaction with the matching connection will be returned.
//
// This method also ensures that a subsequent call with a null connection parameter will return exactly the same instance.
NetworkTransaction *Network::GetTransaction(Connection conn)
{
	if (state == NetworkState::active)
	{
		if (conn != NoConnection)
		{
			NetworkTransaction *tr = conn->GetTransaction();
			if (tr != nullptr && !tr->IsSending())
			{
				currentTransactionSocketNumber = conn->GetNumber();
				return tr;
			}
			return nullptr;
		}

		size_t socketNum = currentTransactionSocketNumber;
		do
		{
			NetworkTransaction *tr = sockets[socketNum].GetTransaction();
			if (tr != nullptr && !tr->IsSending())
			{
				currentTransactionSocketNumber = socketNum;
				return tr;
			}
			++socketNum;
			if (socketNum == NumTcpSockets)
			{
				socketNum = 0;
			}
		} while (socketNum != currentTransactionSocketNumber);
	}

	return nullptr;
}

void Network::OpenDataPort(Port port)
{
	sockets[FtpSocketNumber].Init(FtpSocketNumber, port);
	sockets[FtpSocketNumber].Poll(false);
}

Port Network::GetDataPort() const
{
	return sockets[FtpSocketNumber].GetLocalPort();
}

// Close FTP data port and purge associated PCB
void Network::CloseDataPort()
{
	sockets[FtpSocketNumber].Close();
}

bool Network::AcquireFTPTransaction()
{
	return AcquireTransaction(FtpSocketNumber);
}

bool Network::AcquireDataTransaction()
{
	return AcquireTransaction(FtpDataSocketNumber);
}

bool Network::AcquireTelnetTransaction()
{
	return AcquireTransaction(TelnetSocketNumber);
}

bool Network::AcquireTransaction(SocketNumber skt)
{
	if (   sockets[skt].GetTransaction() != nullptr
		&& sockets[skt].GetTransaction()->GetStatus() != TransactionStatus::sending
		&& sockets[skt].GetTransaction()->GetStatus() != TransactionStatus::finished
	   )
	{
		currentTransactionSocketNumber = skt;
		return true;
	}

	return false;
}

void Network::InitSockets()
{
	for (SocketNumber skt = 0; skt < NumHttpSockets; ++skt)
	{
		sockets[skt].Init(skt, httpPort);
	}
	sockets[FtpSocketNumber].Init(FtpSocketNumber, FTP_PORT);
	sockets[FtpDataSocketNumber].Init(FtpDataSocketNumber, 0);			// FTP data port is allocated dynamically
	sockets[TelnetSocketNumber].Init(TelnetSocketNumber, TELNET_PORT);
	nextSocketToPoll = currentTransactionSocketNumber = 0;
}

void Network::TerminateSockets()
{
	for (SocketNumber skt = 0; skt < NumTcpSockets; ++skt)
	{
		sockets[skt].Terminate();
	}
}

void Network::Defer(NetworkTransaction *tr)
{
	const Socket *skt = tr->GetConnection();
	if (skt != nullptr && skt->GetNumber() == currentTransactionSocketNumber)
	{
		++currentTransactionSocketNumber;
		if (currentTransactionSocketNumber == NumTcpSockets)
		{
			currentTransactionSocketNumber = 0;
		}
	}
}

/*static*/ Port Network::GetLocalPort(Connection conn) { return conn->GetLocalPort(); }
/*static*/ Port Network::GetRemotePort(Connection conn) { return conn->GetRemotePort(); }
/*static*/ uint32_t Network::GetRemoteIP(Connection conn) { return conn->GetRemoteIP(); }
/*static*/ bool Network::IsConnected(Connection conn) { return conn->IsConnected(); }
/*static*/ bool Network::IsTerminated(Connection conn) { return conn->IsTerminated(); }
/*static*/ void Network::Terminate(Connection conn) { conn->Terminate(); }

// End
