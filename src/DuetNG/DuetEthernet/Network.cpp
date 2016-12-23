/*
 * Network.cpp
 *
 *  Created on: 13 Dec 2016
 *      Author: David
 */

#include "RepRapFirmware.h"
#include "compiler.h"
#include "Pins.h"
#include "IPAddress.h"
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
	  freeTransactions(nullptr), readyTransactions(nullptr), writingTransactions(nullptr),
	  httpPort(DefaultHttpPort), state(NetworkState::disabled), activated(false)
{
	SetIPAddress(DefaultIpAddress, DefaultNetMask, DefaultGateway);
	strcpy(hostname, HOSTNAME);
}

void Network::Init()
{
	// Ensure that the chip is in the reset state
	pinMode(EspResetPin, OUTPUT_LOW);
	state = NetworkState::disabled;
	longWait = platform->Time();
	lastTickMillis = millis();
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

void Network::Spin()
{
	switch(state)
	{
	case NetworkState::enabled:
	case NetworkState::disabled:
		// Nothing to do
		break;

	case NetworkState::establishingLink:
		if (wizphy_getphylink() == PHY_LINK_ON)
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
				state = NetworkState::active;
			}
		}
		break;

	case NetworkState::obtainingIP:
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
				state = NetworkState::active;
			}
		}
		else
		{
			DHCP_stop();
			state = NetworkState::establishingLink;
		}
		break;

	case NetworkState::active:
		if (wizphy_getphylink() == PHY_LINK_ON)
		{
			if (usingDhcp)
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
					getSIPR(ipAddress);
				}
			}

			// See if we can read any packets
//			ethernet_task();

			// See if we can send anything
			NetworkTransaction *transaction = writingTransactions;
			if (transaction != nullptr /*&& sendingConnection == nullptr*/ )
			{
				if (transaction->GetNext() != nullptr)
				{
					// Data is supposed to be sent and the last packet has been acknowledged.
					// Rotate the transactions so every client is served even while multiple files are sent
					NetworkTransaction *next = transaction->GetNext();
					writingTransactions = next;
					AppendTransaction(&writingTransactions, transaction);
					transaction = next;
				}

				if (transaction->Send())
				{
					// This transaction can be released, do this here
					writingTransactions = transaction->GetNext();
					PrependTransaction(&freeTransactions, transaction);

					// If there is more data to write on this connection, do it sometime soon
					NetworkTransaction *nextWrite = transaction->GetNextWrite();
					if (nextWrite != nullptr)
					{
						PrependTransaction(&writingTransactions, nextWrite);
					}
				}
			}
		}
		else
		{
			DHCP_stop();
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

	static const uint8_t bufSizes[8] = { 2, 2, 2, 2, 2, 2, 2, 2 };
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
//TODO		Ethernet.stop();
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
		platform->Message(GENERIC_MESSAGE, "WiFi server stopped\n");
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
	//TODO
	return true;
}

void Network::Unlock()
{
	//TODO
}

bool Network::InLwip() const
{
	//TODO
	return false;
}

// This is called by the web server to get the next networking transaction.
//
// If cs is NULL, the transaction from the head of readyTransactions will be retrieved.
// If cs is not NULL, the first transaction with the matching connection will be returned.
//
// This method also ensures that the retrieved transaction is moved to the first item of
// readyTransactions, so that a subsequent call with a NULL cs parameter will return exactly
// the same instance.
NetworkTransaction *Network::GetTransaction(const ConnectionState *cs)
{
#if 1
	return nullptr;
#else
	// See if there is any transaction at all
	NetworkTransaction *transaction = readyTransactions;
	if (transaction == nullptr)
	{
		return nullptr;
	}

	// If no specific connection is specified or if the first item already matches the
	// connection we are looking for, just return it
	if (cs == nullptr || transaction->GetConnection() == cs)
	{
		return transaction;
	}

	// We are looking for a specific transaction, but it's not the first item.
	// Search for it and move it to the head of readyTransactions
	NetworkTransaction *previous = transaction;
	for(NetworkTransaction *item = transaction->next; item != nullptr; item = item->next)
	{
		if (item->GetConnection() == cs)
		{
			previous->next = item->next;
			item->next = readyTransactions;
			readyTransactions = item;
			return item;
		}
		previous = item;
	}

	// We failed to find a valid transaction for the given connection
	return nullptr;
#endif
}

void Network::AppendTransaction(NetworkTransaction* * list, NetworkTransaction *r)
{
	r->next = nullptr;
	while (*list != nullptr)
	{
		list = &((*list)->next);
	}
	*list = r;
}

void Network::PrependTransaction(NetworkTransaction* * list, NetworkTransaction *r)
{
	r->next = *list;
	*list = r;
}

void Network::OpenDataPort(uint16_t port)
{
	//TODO
#if 0
	closingDataPort = false;
	tcp_pcb* pcb = tcp_new();
	tcp_bind(pcb, IP_ADDR_ANY, port);
	ftp_pasv_pcb = tcp_listen(pcb);
	tcp_accept(ftp_pasv_pcb, conn_accept);
#endif
}

uint16_t Network::GetDataPort() const
{
#if 1
	return 0;	//TODO
#else
	return (closingDataPort || (ftp_pasv_pcb == nullptr) ? 0 : ftp_pasv_pcb->local_port);
#endif
}

// Close FTP data port and purge associated PCB
void Network::CloseDataPort()
{
	//TODO
#if 0
	// See if it's already being closed
	if (closingDataPort)
	{
		return;
	}
	closingDataPort = true;

	// Close remote connection of our data port or do it as soon as the last packet has been sent
	if (dataCs != nullptr)
	{
		NetworkTransaction *mySendingTransaction = dataCs->sendingTransaction;
		if (mySendingTransaction != nullptr)
		{
			mySendingTransaction->Close();
			return;
		}
	}

	// We can close it now, so do it here
	if (ftp_pasv_pcb != nullptr)
	{
		tcp_accept(ftp_pasv_pcb, nullptr);
		tcp_close(ftp_pasv_pcb);
		ftp_pasv_pcb = nullptr;
	}
	closingDataPort = false;
#endif
}

// These methods keep track of our connections in case we need to send to one of them
void Network::SaveDataConnection()
{
	//TODO
#if 0
	dataCs = readyTransactions->cs;
#endif
}

void Network::SaveFTPConnection()
{
	//TODO
#if 0
	ftpCs = readyTransactions->cs;
#endif
}

void Network::SaveTelnetConnection()
{
	//TODO
#if 0
	telnetCs = readyTransactions->cs;
#endif
}

bool Network::AcquireFTPTransaction()
{
#if 1
	return false;	//TODO
#else
	return AcquireTransaction(ftpCs);
#endif
}

bool Network::AcquireDataTransaction()
{
#if 1
	return false;	//TODO
#else
	return AcquireTransaction(dataCs);
#endif
}

bool Network::AcquireTelnetTransaction()
{
#if 1
	return false;	//TODO
#else
	return AcquireTransaction(telnetCs);
#endif
}

// End
