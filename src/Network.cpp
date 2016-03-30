/****************************************************************************************************

 RepRapFirmware - Network: RepRapPro Ormerod with Arduino Due controller

  2014-04-05 Created from portions taken out of Platform.cpp by dc42
  2014-04-07 Added portions of httpd.c. These portions are subject to the following copyright notice:

 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 * (end httpd.c copyright notice)

 ****************************************************************************************************/

#include "RepRapFirmware.h"

extern "C"
{
#include "ethernet_sam.h"

#include "lwipopts.h"

#ifdef LWIP_STATS
#include "lwip/src/include/lwip/stats.h"
#endif

#include "lwip/src/include/lwip/tcp.h"
#include "lwip/src/include/lwip/tcp_impl.h"

#include "contrib/apps/netbios/netbios.h"
#include "contrib/apps/mdns/mdns_responder.h"
}

static volatile bool lwipLocked = false;
static bool ethernetStarted = false;

static tcp_pcb *http_pcb = nullptr;
static tcp_pcb *ftp_main_pcb = nullptr;
static tcp_pcb *ftp_pasv_pcb = nullptr;
static tcp_pcb *telnet_pcb = nullptr;

static struct mdns_service mdns_services[] = {
	{
		.name = "\x05_echo\x04_tcp\x05local",
		.port = 7,
	},
	{
		.name = "\x05_http\x04_tcp\x05local",
		.port = DEFAULT_HTTP_PORT,
	},
	{
		.name = "\x04_ftp\x04_tcp\x05local",
		.port = FTP_PORT
	},
	{
		.name = "\x07_telnet\x04_tcp\x05local",
		.port = TELNET_PORT
	}
};

const size_t MDNS_HTTP_SERVICE_INDEX = 1;	// Index of the mDNS HTTP service above

static const char *mdns_txt_records[] = {
	"product=" NAME,
	"version=" VERSION,
	NULL
};

static bool closingDataPort = false;

static ConnectionState *sendingConnection = nullptr;

static uint32_t sendingWindow32[(TCP_WND + 3)/4];						// should be 32-bit aligned for efficiency
static char * const sendingWindow = reinterpret_cast<char *>(sendingWindow32);
static uint16_t sendingWindowSize, sentDataOutstanding;
static uint8_t sendingRetries;
static err_t writeResult, outputResult;

static uint16_t httpPort = DEFAULT_HTTP_PORT;

/*-----------------------------------------------------------------------------------*/

extern "C"
{

// Lock functions for LWIP (LWIP generally isn't thread-safe)
bool LockLWIP()
{
	if (lwipLocked)
		return false;

	lwipLocked = true;
	return true;
}

void UnlockLWIP()
{
	lwipLocked = false;
}

// Callback functions for the EMAC driver and for LwIP

// Callback to report when the network interface has gone up or down.
// Note that this is only a rough indicator and may not be called when
// the IP address is changed on-the-fly!
static void ethernet_status_callback(struct netif *netif)
{
	if (netif_is_up(netif))
	{
		char ip[16];
		ipaddr_ntoa_r(&(netif->ip_addr), ip, sizeof(ip));
		reprap.GetPlatform()->MessageF(HOST_MESSAGE, "Network up, IP=%s\n", ip);
	}
	else
	{
		reprap.GetPlatform()->Message(HOST_MESSAGE, "Network down\n");
	}
}

// Called from ISR
static void ethernet_rx_callback(uint32_t ul_status)
{
	// Because the LWIP stack can become corrupted if we work with it in parallel,
	// we may have to wait for the next Spin() call to read the next packet.
	if (LockLWIP())
	{
		ethernet_task();
		UnlockLWIP();
	}
	else
	{
		ethernet_set_rx_callback(nullptr);
		reprap.GetNetwork()->ResetCallback();
	}
}

// Callback functions for LWIP (may be called from ISR)

static void conn_err(void *arg, err_t err)
{
	// Report the error to the monitor
	reprap.GetPlatform()->MessageF(HOST_MESSAGE, "Network: Connection error, code %d\n", err);

	// Tell the higher levels about the error
	ConnectionState *cs = (ConnectionState*)arg;
	if (cs != nullptr)
	{
		cs->isTerminated = true;
		reprap.GetNetwork()->ConnectionClosed(cs, false);
	}
}

static err_t conn_poll(void *arg, tcp_pcb *pcb)
{
	ConnectionState *cs = (ConnectionState*)arg;
	if (cs == sendingConnection)
	{
		// Data could not be sent last time, check if the connection has to be timed out
		sendingRetries++;
		if (sendingRetries == TCP_MAX_SEND_RETRIES)
		{
			reprap.GetPlatform()->MessageF(HOST_MESSAGE, "Network: Could not transmit data after %.1f seconds\n", (float)TCP_WRITE_TIMEOUT / 1000.0);
			tcp_abort(pcb);
			return ERR_ABRT;
		}

		// Try to write the remaining data once again (if required)
		if (writeResult != ERR_OK)
		{
			writeResult = tcp_write(pcb, sendingWindow + (sendingWindowSize - sentDataOutstanding), sentDataOutstanding, 0);
			if (ERR_IS_FATAL(writeResult))
			{
				reprap.GetPlatform()->MessageF(HOST_MESSAGE, "Network: Failed to write data in conn_poll (code %d)\n", writeResult);
				tcp_abort(pcb);
				return ERR_ABRT;
			}

			if (writeResult != ERR_OK && reprap.Debug(moduleNetwork))
			{
				reprap.GetPlatform()->MessageF(HOST_MESSAGE, "Network: tcp_write resulted in error code %d\n", writeResult);
			}
		}

		// If that worked, try to output the remaining data (if required)
		if (outputResult != ERR_OK)
		{
			outputResult = tcp_output(pcb);
			if (ERR_IS_FATAL(outputResult))
			{
				reprap.GetPlatform()->MessageF(HOST_MESSAGE, "Network: Failed to output data in conn_poll (code %d)\n", outputResult);
				tcp_abort(pcb);
				return ERR_ABRT;
			}

			if (outputResult != ERR_OK && reprap.Debug(moduleNetwork))
			{
				reprap.GetPlatform()->MessageF(HOST_MESSAGE, "Network: tcp_output resulted in error code %d\n", outputResult);
			}
		}
	}
	else
	{
		reprap.GetPlatform()->Message(HOST_MESSAGE, "Network: Mismatched pcb in conn_poll!\n");
	}
	return ERR_OK;
}

static err_t conn_sent(void *arg, tcp_pcb *pcb, u16_t len)
{
	ConnectionState *cs = (ConnectionState*)arg;
	if (cs == sendingConnection)
	{
		if (sentDataOutstanding > len)
		{
			sentDataOutstanding -= len;
		}
		else
		{
			tcp_poll(pcb, nullptr, TCP_WRITE_TIMEOUT / TCP_SLOW_INTERVAL / TCP_MAX_SEND_RETRIES);
			sendingConnection = nullptr;
		}
	}
	else
	{
		reprap.GetPlatform()->Message(HOST_MESSAGE, "Network: Mismatched pcb in conn_sent!\n");
	}
	return ERR_OK;
}

static err_t conn_recv(void *arg, tcp_pcb *pcb, pbuf *p, err_t err)
{
	ConnectionState *cs = (ConnectionState*)arg;
	if (err == ERR_OK && cs != nullptr)
	{
		if (cs->pcb != pcb)
		{
			reprap.GetPlatform()->Message(HOST_MESSAGE, "Network: Mismatched pcb in conn_recv!\n");
			tcp_abort(pcb);
			return ERR_ABRT;
		}

		bool processingOk = true;
		if (p != nullptr)
		{
			// Tell higher levels that we are receiving data
			processingOk = reprap.GetNetwork()->ReceiveInput(p, cs);
		}
		else
		{
			// Tell higher levels that a connection has been closed
			processingOk = reprap.GetNetwork()->ConnectionClosedGracefully(cs);
		}

		if (!processingOk)
		{
			// Something went wrong, discard whatever has been received
			if (p != nullptr)
			{
				pbuf_free(p);
			}

			// Also reset the connection. This will call conn_err() too
			tcp_abort(pcb);
			return ERR_ABRT;
		}
	}

	return ERR_OK;
}

static err_t conn_accept(void *arg, tcp_pcb *pcb, err_t err)
{
	LWIP_UNUSED_ARG(arg);
	LWIP_UNUSED_ARG(err);

	/* Allocate a new ConnectionState for this connection */
	ConnectionState *cs = reprap.GetNetwork()->ConnectionAccepted(pcb);
	if (cs == nullptr)
	{
		tcp_abort(pcb);
		return ERR_ABRT;
	}

	/* Keep the listening PCBs running */
	switch (pcb->local_port)		// tell LWIP to accept further connections on the listening PCB
	{
		case FTP_PORT:		// FTP
			tcp_accepted(ftp_main_pcb);
			break;

		case TELNET_PORT:	// Telnet
			tcp_accepted(telnet_pcb);
			break;

		default:			// HTTP and FTP data
			tcp_accepted((pcb->local_port == httpPort) ? http_pcb : ftp_pasv_pcb);
			break;
	}
	tcp_arg(pcb, cs);			// tell LWIP that this is the structure we wish to be passed for our callbacks
	tcp_recv(pcb, conn_recv);	// tell LWIP that we wish to be informed of incoming data by a call to the conn_recv() function
	tcp_err(pcb, conn_err);

	return ERR_OK;
}

} // end extern "C"

/*-----------------------------------------------------------------------------------*/

void httpd_init()
{
	tcp_pcb* pcb = tcp_new();
	tcp_bind(pcb, IP_ADDR_ANY, httpPort);
	http_pcb = tcp_listen(pcb);
	tcp_accept(http_pcb, conn_accept);
}

void ftpd_init()
{
	tcp_pcb* pcb = tcp_new();
	tcp_bind(pcb, IP_ADDR_ANY, FTP_PORT);
	ftp_main_pcb = tcp_listen(pcb);
	tcp_accept(ftp_main_pcb, conn_accept);
}

void telnetd_init()
{
	tcp_pcb* pcb = tcp_new();
	tcp_bind(pcb, IP_ADDR_ANY, TELNET_PORT);
	telnet_pcb = tcp_listen(pcb);
	tcp_accept(telnet_pcb, conn_accept);
}

//***************************************************************************************************

// Network/Ethernet class

Network::Network(Platform* p) :
	platform(p), freeTransactions(nullptr), readyTransactions(nullptr), writingTransactions(nullptr),
	state(NetworkInactive), isEnabled(true), resetCallback(false),
	dataCs(nullptr), ftpCs(nullptr), telnetCs(nullptr), freeConnections(nullptr)
{
	for (size_t i = 0; i < NETWORK_TRANSACTION_COUNT; i++)
	{
		freeTransactions = new NetworkTransaction(freeTransactions);
	}

	for (size_t i = 0; i < MEMP_NUM_TCP_PCB; i++)
	{
		ConnectionState *cs = new ConnectionState;
		cs->next = freeConnections;
		freeConnections = cs;
	}

	strcpy(hostname, HOSTNAME);
}

void Network::Init()
{
	init_ethernet();

	httpd_init();
	ftpd_init();
	telnetd_init();
	netbios_init();

	longWait = platform->Time();
}

void Network::Spin()
{
	// Basically we can't do anything if we can't interact with LWIP

	if (!LockLWIP())
	{
		platform->ClassReport(longWait);
		return;
	}

	if (state == NetworkObtainingIP || state == NetworkActive)
	{
		// Is the link still up?
		if (!ethernet_link_established())
		{
			state = NetworkEstablishingLink;
			UnlockLWIP();

			platform->ClassReport(longWait);
			return;
		}

		// See if we can read any packets. They may include DHCP responses too
		ethernet_task();
		if (resetCallback)
		{
			resetCallback = false;
			ethernet_set_rx_callback(&ethernet_rx_callback);
		}

		// Have we obtained a valid IP address yet?
		if (state == NetworkObtainingIP)
		{
			const uint8_t *ip = ethernet_get_ipaddress();
			if (ip[0] != 0 && ip[1] != 0 && ip[2] != 0 && ip[3] != 0)
			{
				// Yes - we're good to go now
				state = NetworkActive;

				// Send mDNS announcement so that some routers can perform hostname mapping
				// if ths board is connected via a non-IGMP capable WiFi bridge (like the TP-Link WR701N)
				mdns_announce();
			}
		}

		// See if we can send anything
		NetworkTransaction *transaction = writingTransactions;
		if (transaction != nullptr && sendingConnection == nullptr)
		{
			if (transaction->next != nullptr)
			{
				// Data is supposed to be sent and the last packet has been acknowledged.
				// Rotate the transactions so every client is served even while multiple files are sent
				NetworkTransaction *next = transaction->next;
				writingTransactions = next;
				AppendTransaction(&writingTransactions, transaction);
				transaction = next;
			}

			if (transaction->Send())
			{
				// This transaction can be released, do this here
				writingTransactions = transaction->next;
				PrependTransaction(&freeTransactions, transaction);

				// If there is more data to write on this connection, do it sometime soon
				NetworkTransaction *nextWrite = transaction->nextWrite;
				if (nextWrite != nullptr)
				{
					PrependTransaction(&writingTransactions, nextWrite);
				}
			}
		}
	}
	else if (state == NetworkEstablishingLink && ethernet_establish_link())
	{
		if (!ethernetStarted)
		{
			start_ethernet(platform->IPAddress(), platform->NetMask(), platform->GateWay(), &ethernet_status_callback);
			ethernetStarted = true;

			// Initialise this one here, because it requires a configured IGMP network interface
			mdns_responder_init(mdns_services, ARRAY_SIZE(mdns_services), mdns_txt_records);
		}
		else
		{
			ethernet_set_configuration(platform->IPAddress(), platform->NetMask(), platform->GateWay());
		}
		state = NetworkObtainingIP;
	}

	UnlockLWIP();
	platform->ClassReport(longWait);
}

void Network::Interrupt()
{
	if (state != NetworkInactive && LockLWIP())
	{
		ethernet_timers_update();
		UnlockLWIP();
	}
}

void Network::Diagnostics()
{
	platform->Message(GENERIC_MESSAGE, "Network Diagnostics:\n");

	size_t numFreeConnections = 0;
	ConnectionState *freeConn = freeConnections;
	while (freeConn != nullptr)
	{
		numFreeConnections++;
		freeConn = freeConn->next;
	}
	platform->MessageF(GENERIC_MESSAGE, "Free connections: %d of %d\n", numFreeConnections, MEMP_NUM_TCP_PCB);

	size_t numFreeTransactions = 0;
	NetworkTransaction *freeTrans = freeTransactions;
	while (freeTrans != nullptr)
	{
		numFreeTransactions++;
		freeTrans = freeTrans->next;
	}
	platform->MessageF(GENERIC_MESSAGE, "Free transactions: %d of %d\n", numFreeTransactions, NETWORK_TRANSACTION_COUNT);

#if LWIP_STATS
	// Normally we should NOT try to display LWIP stats here, because it uses debugPrintf(), which will hang the system if no USB cable is connected.
	if (reprap.Debug(moduleNetwork))
	{
		stats_display();
	}
#endif
}

void Network::ResetCallback()
{
	resetCallback = true;
}

// Called when data has been received. Return false if we cannot process it
bool Network::ReceiveInput(pbuf *pb, ConnectionState* cs)
{
	NetworkTransaction* r = freeTransactions;
	if (r == nullptr)
	{
		platform->Message(HOST_MESSAGE, "Network::ReceiveInput() - no free transactions!\n");
		return false;
	}

	freeTransactions = r->next;
	r->Set(pb, cs, receiving);

	AppendTransaction(&readyTransactions, r);
//	debugPrintf("Network - input received\n");
	return true;
}

// This is called when a connection is being established and returns an initialised ConnectionState instance
// or NULL if no more items are available. This would reset the connection immediately
ConnectionState *Network::ConnectionAccepted(tcp_pcb *pcb)
{
	ConnectionState *cs = freeConnections;
	if (cs == nullptr)
	{
		platform->Message(HOST_MESSAGE, "Network::ConnectionAccepted() - no free ConnectionStates!\n");
		return nullptr;
	}

	NetworkTransaction* transaction = freeTransactions;
	if (transaction == nullptr)
	{
		platform->Message(HOST_MESSAGE, "Network::ConnectionAccepted() - no free transactions!\n");
		return nullptr;
	}

	// Initialise a new connection
	freeConnections = cs->next;
	cs->Init(pcb);

	// Notify the webserver about this
	transaction->Set(nullptr, cs, connected);
	freeTransactions = transaction->next;
	AppendTransaction(&readyTransactions, transaction);

	return cs;
}

// This is called when a connection is being closed or has gone down unexpectedly
void Network::ConnectionClosed(ConnectionState* cs, bool closeConnection)
{
	// Make sure these connections are not reused. Remove all references to it
	if (cs == dataCs)
	{
		// FTP data connection
		dataCs = nullptr;

		if (closingDataPort && ftp_pasv_pcb != nullptr)
		{
			tcp_accept(ftp_pasv_pcb, nullptr);
			tcp_close(ftp_pasv_pcb);
			ftp_pasv_pcb = nullptr;
		}
		closingDataPort = false;
	}
	if (cs == ftpCs)
	{
		// Main FTP connection
		ftpCs = nullptr;
	}
	if (cs == telnetCs)
	{
		telnetCs = nullptr;
	}
	if (cs == sendingConnection)
	{
		// Stop sending if the connection is going down
		sendingConnection = nullptr;
	}

	// Remove all callbacks and close the PCB if requested
	tcp_pcb *pcb = cs->pcb;
	tcp_sent(pcb, nullptr);
	tcp_recv(pcb, nullptr);
	tcp_poll(pcb, nullptr, TCP_WRITE_TIMEOUT / TCP_SLOW_INTERVAL / TCP_MAX_SEND_RETRIES);
	if (pcb != nullptr && closeConnection)
	{
		tcp_err(pcb, nullptr);
		tcp_close(pcb);
	}
	cs->pcb = nullptr;

	// Inform the Webserver that we are about to remove an existing connection
	reprap.GetWebserver()->ConnectionLost(cs);

	// Remove all transactions that point to cs from the list of ready transactions
	NetworkTransaction *previous = nullptr, *item = readyTransactions;
	while (item != nullptr)
	{
		if (item->cs == cs)
		{
			item->Discard();
			item = (previous == nullptr) ? readyTransactions : previous->next;
		}
		else
		{
			previous = item;
			item = item->next;
		}
	}

	// Do the same for the writing transaction. There is only one transaction on writingTransactions
	// per connection and cs->sendingTransaction points to it. Check if we have to free it here
	NetworkTransaction *sendingTransaction = cs->sendingTransaction;
	if (sendingTransaction != nullptr)
	{
		// Take care of other transactions that want to write data over the closed connection too
		NetworkTransaction *nextWrite = sendingTransaction->nextWrite;
		while (nextWrite != nullptr)
		{
			NetworkTransaction *temp = nextWrite;
			nextWrite = nextWrite->nextWrite;
			temp->Discard();
		}

		// Unlink the sending transaction from the writing transactions
		previous = nullptr;
		for(item = writingTransactions; item != nullptr; item = item->next)
		{
			if (item == sendingTransaction)
			{
				if (previous == nullptr)
				{
					writingTransactions = item->next;
				}
				else
				{
					previous->next = item->next;
				}
				break;
			}
			previous = item;
		}

		// Discard it. This will add it back to the list of free transactions too
		sendingTransaction->Discard();
	}

	// Free up this cs again
	cs->next = freeConnections;
	freeConnections = cs;
}

// This enqueues a new transaction to indicate a graceful reset. Do this to keep the time line of incoming transactions valid.
// Return false if we cannot process this event, which would result in an immediate connection reset
bool Network::ConnectionClosedGracefully(ConnectionState *cs)
{
	// We need a valid transaction to do something.
	// If that fails, the connection will be reset and the error handler will take care of this connection
	NetworkTransaction *transaction = freeTransactions;
	if (transaction == nullptr)
	{
		platform->Message(HOST_MESSAGE, "Network::ConnectionClosedGracefully() - no free transactions!\n");
		return false;
	}

	// Invalidate the PCB
	tcp_sent(cs->pcb, nullptr);
	tcp_recv(cs->pcb, nullptr);
	tcp_poll(cs->pcb, nullptr, TCP_WRITE_TIMEOUT / TCP_SLOW_INTERVAL / TCP_MAX_SEND_RETRIES);
	tcp_err(cs->pcb, nullptr);
	tcp_close(cs->pcb);
	cs->pcb = nullptr;

	// Close the sending transaction (if any)
	NetworkTransaction *sendingTransaction = cs->sendingTransaction;
	if (sendingTransaction != nullptr)
	{
		sendingTransaction->Close();
	}

	// Notify the webserver about this event
	freeTransactions = transaction->next;
	transaction->Set(nullptr, cs, disconnected);
	AppendTransaction(&readyTransactions, transaction);
	return true;
}

bool Network::Lock()
{
	return LockLWIP();
}

void Network::Unlock()
{
	UnlockLWIP();
}

bool Network::InLwip() const
{
	return lwipLocked;
}

const uint8_t *Network::IPAddress() const
{
	return ethernet_get_ipaddress();
}

void Network::SetIPAddress(const uint8_t ipAddress[], const uint8_t netmask[], const uint8_t gateway[])
{
	if (state == NetworkObtainingIP || state == NetworkActive)
	{
		// This performs IP changes on-the-fly
		ethernet_set_configuration(ipAddress, netmask, gateway);
	}
}

// Set the network hostname. Removes all whitespaces and converts the name to lower-case.
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

	if (i > 0)
	{
		hostname[i] = 0;
	}
	else
	{
		// Don't allow empty hostnames
		strcpy(hostname, HOSTNAME);
	}

	if (state == NetworkActive)
	{
		mdns_update_hostname();
	}
}

void Network::Enable()
{
	if (state == NetworkInactive)
	{
		if (!ethernetStarted)
		{
			// Allow the MAC address to be set only before LwIP is started...
			ethernet_configure_interface(platform->MACAddress(), hostname);
		}

		resetCallback = true;	// Reset EMAC RX callback on next Spin calls
		state = NetworkEstablishingLink;
		isEnabled = true;
	}
}

void Network::Disable()
{
	if (state != NetworkInactive)
	{
		resetCallback = false;
		ethernet_set_rx_callback(nullptr);
		state = NetworkInactive;
		isEnabled = false;
	}
}

// This is called by the web server to get a new received packet.
// If the connection parameter is nullptr, we just return the request at the head of the ready list.
// Otherwise, we are only interested in packets received from the specified connection. If we find one then
// we move it to the head of the ready list, so that a subsequent call with a null connection parameter
// will return the same one.
NetworkTransaction *Network::GetTransaction(const ConnectionState *cs)
{
	// See if there is any transaction at all
	NetworkTransaction *rs = readyTransactions;
	if (rs == nullptr)
	{
		return nullptr;
	}

	// See if the first one is the transaction we're looking for
	if (cs == nullptr || rs->cs == cs)
	{
		return rs;
	}

	// There is at least one ready transaction, but it's not on the connection we are looking for
	for (NetworkTransaction *rsNext = rs->next; rsNext != nullptr; rsNext = rs->next)
	{
		if (rsNext->cs == cs)
		{
			rs->next = rsNext->next;		// remove rsNext from the list
			rsNext->next = readyTransactions;
			readyTransactions = rsNext;
			return rsNext;
		}

		rs = rsNext;
	}

	return nullptr;
}

void Network::AppendTransaction(NetworkTransaction* volatile* list, NetworkTransaction *r)
{
	r->next = nullptr;
	while (*list != nullptr)
	{
		list = &((*list)->next);
	}
	*list = r;
}

void Network::PrependTransaction(NetworkTransaction* volatile* list, NetworkTransaction *r)
{
	r->next = *list;
	*list = r;
}

void Network::OpenDataPort(uint16_t port)
{
	closingDataPort = false;
	tcp_pcb* pcb = tcp_new();
	tcp_bind(pcb, IP_ADDR_ANY, port);
	ftp_pasv_pcb = tcp_listen(pcb);
	tcp_accept(ftp_pasv_pcb, conn_accept);
}

uint16_t Network::GetDataPort() const
{
	return (closingDataPort || (ftp_pasv_pcb == nullptr) ? 0 : ftp_pasv_pcb->local_port);
}

uint16_t Network::GetHttpPort() const
{
	return httpPort;
}

void Network::SetHttpPort(uint16_t port)
{
	if (port != httpPort)
	{
		// Close the old HTTP PCB and create a new one
		tcp_close(http_pcb);
		httpPort = port;
		httpd_init();

		// Update mDNS service
		mdns_services[MDNS_HTTP_SERVICE_INDEX].port = port;
		if (state == NetworkActive)
		{
			mdns_announce();
		}
	}
}

// Close FTP data port and purge associated PCB
void Network::CloseDataPort()
{
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
}

// These methods keep track of our connections in case we need to send to one of them
void Network::SaveDataConnection()
{
	dataCs = readyTransactions->cs;
}

void Network::SaveFTPConnection()
{
	ftpCs = readyTransactions->cs;
}

void Network::SaveTelnetConnection()
{
	telnetCs = readyTransactions->cs;
}

bool Network::AcquireFTPTransaction()
{
	return AcquireTransaction(ftpCs);
}

bool Network::AcquireDataTransaction()
{
	return AcquireTransaction(dataCs);
}

bool Network::AcquireTelnetTransaction()
{
	return AcquireTransaction(telnetCs);
}

// Retrieves the NetworkTransaction of a sending connection to which data can be appended to,
// or prepares a released NetworkTransaction, which can easily be sent via Commit().
bool Network::AcquireTransaction(ConnectionState *cs)
{
	// Make sure we have a valid connection
	if (cs == nullptr)
	{
		return false;
	}

	// If our current transaction already belongs to cs and can be used, don't look for another one
	NetworkTransaction *currentTransaction = readyTransactions;
	if (currentTransaction != nullptr && currentTransaction->GetConnection() == cs && currentTransaction->fileBeingSent == nullptr)
	{
		return true;
	}

	// See if we're already writing on this connection
	NetworkTransaction *previousTransaction = cs->sendingTransaction;
	NetworkTransaction *lastTransaction = previousTransaction;
	if (lastTransaction != nullptr)
	{
		while (lastTransaction->nextWrite != nullptr)
		{
			previousTransaction = lastTransaction;
			lastTransaction = lastTransaction->nextWrite;
		}
	}

	// Then check if this transaction is valid and safe to use
	NetworkTransaction *transactionToUse;
	if (previousTransaction != lastTransaction && lastTransaction->fileBeingSent == nullptr)
	{
		previousTransaction->nextWrite = nullptr;
		transactionToUse = lastTransaction;
	}
	// We cannot use it, so try to allocate a free one
	else
	{
		transactionToUse = freeTransactions;
		if (transactionToUse == nullptr)
		{
			platform->Message(HOST_MESSAGE, "Network: Could not acquire free transaction!\n");
			return false;
		}
		freeTransactions = transactionToUse->next;
		transactionToUse->Set(nullptr, cs, acquired);
	}

	// Replace the first entry of readyTransactions with our new transaction, so it can be used by Commit().
	PrependTransaction(&readyTransactions, transactionToUse);
	return true;
}

//***************************************************************************************************

// ConnectionState class

void ConnectionState::Init(tcp_pcb *p)
{
	pcb = p;
	localPort = p->local_port;
	remoteIPAddress = p->remote_ip.addr;
	remotePort = p->remote_port;
	next = nullptr;
	sendingTransaction = nullptr;
	persistConnection = true;
	isTerminated = false;
}

void ConnectionState::Terminate()
{
	if (pcb != nullptr)
	{
		tcp_abort(pcb);
	}
}

//***************************************************************************************************
// NetworkTransaction class

NetworkTransaction::NetworkTransaction(NetworkTransaction *n) : next(n), status(released)
{
	sendStack = new OutputStack();
}

void NetworkTransaction::Set(pbuf *p, ConnectionState *c, TransactionStatus s)
{
	cs = c;
	pb = readingPb = p;
	status = s;
	inputPointer = 0;
	sendBuffer = nullptr;
	fileBeingSent = nullptr;
	closeRequested = false;
	nextWrite = nullptr;
	dataAcknowledged = false;
}

// Read one char from the NetworkTransaction
bool NetworkTransaction::Read(char& b)
{
	if (readingPb == nullptr)
	{
		b = 0;
		return false;
	}

	b = ((const char*)readingPb->payload)[inputPointer++];
	if (inputPointer > readingPb->len)
	{
		readingPb = readingPb->next;
		inputPointer = 0;
	}
	return true;
}

// Read data from the NetworkTransaction and return true on success
bool NetworkTransaction::ReadBuffer(const char *&buffer, size_t &len)
{
	if (readingPb == nullptr)
	{
		return false;
	}

	if (inputPointer >= readingPb->len)
	{
		readingPb = readingPb->next;
		inputPointer = 0;
		if (readingPb == nullptr)
		{
			return false;
		}
	}

	buffer = (const char*)readingPb->payload + inputPointer;
	len = readingPb->len - inputPointer;
	readingPb = readingPb->next;
	inputPointer = 0;
	return true;
}

void NetworkTransaction::Write(char b)
{
	if (CanWrite())
	{
		if (sendBuffer == nullptr && !OutputBuffer::Allocate(sendBuffer))
		{
			// Should never get here
			return;
		}
		sendBuffer->cat(b);
	}
}

void NetworkTransaction::Write(const char* s)
{
	if (CanWrite())
	{
		if (sendBuffer == nullptr && !OutputBuffer::Allocate(sendBuffer))
		{
			// Should never get here
			return;
		}
		sendBuffer->cat(s);
	}
}

void NetworkTransaction::Write(StringRef ref)
{
	Write(ref.Pointer(), ref.strlen());
}

void NetworkTransaction::Write(const char* s, size_t len)
{
	if (CanWrite())
	{
		if (sendBuffer == nullptr && !OutputBuffer::Allocate(sendBuffer))
		{
			// Should never get here
			return;
		}
		sendBuffer->cat(s, len);
	}
}

void NetworkTransaction::Write(OutputBuffer *buffer)
{
	if (CanWrite())
	{
		// Note we use an individual stack here, because we don't want to link different
		// OutputBuffers for different destinations together...
		sendStack->Push(buffer);
	}
	else
	{
		// Don't keep buffers we can't send...
		OutputBuffer::ReleaseAll(buffer);
	}
}

void NetworkTransaction::Write(OutputStack *stack)
{
	if (stack != nullptr)
	{
		if (CanWrite())
		{
			sendStack->Append(stack);
		}
		else
		{
			stack->ReleaseAll();
		}
	}
}

void NetworkTransaction::Printf(const char* fmt, ...)
{
	if (CanWrite() && (sendBuffer != nullptr || OutputBuffer::Allocate(sendBuffer)))
	{
		va_list p;
		va_start(p, fmt);
		sendBuffer->vprintf(fmt, p);
		va_end(p);
	}
}

void NetworkTransaction::SetFileToWrite(FileStore *file)
{
	if (CanWrite())
	{
		fileBeingSent = file;
	}
	else if (file != nullptr)
	{
		file->Close();
	}
}

// Send exactly one TCP window of data and return true when this transaction can be released
bool NetworkTransaction::Send()
{
	// Free up this transaction if the connection is supposed to be closed
	if (closeRequested)
	{
		reprap.GetNetwork()->ConnectionClosed(cs, true);	// This will release the transaction too
		return false;
	}

	// Fill up the TCP window with some data chunks from our OutputBuffer instances
	size_t bytesBeingSent = 0, bytesLeftToSend = TCP_WND;
	while (sendBuffer != nullptr && bytesLeftToSend > 0)
	{
		size_t copyLength = min<size_t>(bytesLeftToSend, sendBuffer->BytesLeft());
		memcpy(sendingWindow + bytesBeingSent, sendBuffer->Read(copyLength), copyLength);
		bytesBeingSent += copyLength;
		bytesLeftToSend -= copyLength;

		if (sendBuffer->BytesLeft() == 0)
		{
			sendBuffer = OutputBuffer::Release(sendBuffer);
			if (sendBuffer == nullptr)
			{
				sendBuffer = sendStack->Pop();
			}
		}
	}

	// We also intend to send a file, so check if we can fill up the TCP window
	if (sendBuffer == nullptr && bytesLeftToSend != 0 && fileBeingSent != nullptr)
	{
		// For HSMCI efficiency, read from the file in multiples of 4 bytes except at the end.
		// This ensures that the second and subsequent chunks can be DMA'd directly into sendingWindow.
		size_t bytesToRead = bytesLeftToSend & (~3);
		if (bytesToRead != 0)
		{
			int bytesRead = fileBeingSent->Read(sendingWindow + bytesBeingSent, bytesToRead);
			if (bytesRead > 0)
			{
				bytesBeingSent += bytesRead;
			}

			if (bytesRead != (int)bytesToRead)
			{
				fileBeingSent->Close();
				fileBeingSent = nullptr;
			}
		}
	}

	if (bytesBeingSent == 0)
	{
		// If we have no data to send, this connection can be closed next time
		if (!cs->persistConnection && nextWrite == nullptr)
		{
			Close();
			return false;
		}

		// We want to send data from another transaction as well, so only free up this one
		cs->sendingTransaction = nextWrite;
		return true;
	}

	// The TCP window has been filled up as much as possible, so send it now. There is no need to check
	// the available space in the SNDBUF queue, because we really write only one TCP window at once.
	writeResult = tcp_write(cs->pcb, sendingWindow, bytesBeingSent, 0);
	if (ERR_IS_FATAL(writeResult))
	{
		reprap.GetPlatform()->MessageF(HOST_MESSAGE, "Network: Failed to write data in Send (code %d)\n", writeResult);
		tcp_abort(cs->pcb);
		return false;
	}

	outputResult = tcp_output(cs->pcb);
	if (ERR_IS_FATAL(outputResult))
	{
		reprap.GetPlatform()->MessageF(HOST_MESSAGE, "Network: Failed to output data in Send (code %d)\n", outputResult);
		tcp_abort(cs->pcb);
		return false;
	}

	if (outputResult != ERR_OK && reprap.Debug(moduleNetwork))
	{
		reprap.GetPlatform()->MessageF(HOST_MESSAGE, "Network: tcp_output resulted in error code %d\n", outputResult);
	}

	// Set LwIP callbacks for ACK and retransmission handling
	tcp_poll(cs->pcb, conn_poll, TCP_WRITE_TIMEOUT / TCP_SLOW_INTERVAL / TCP_MAX_SEND_RETRIES);
	tcp_sent(cs->pcb, conn_sent);

	// Set all values for the send process
	sendingConnection = cs;
	sendingRetries = 0;
	sendingWindowSize = sentDataOutstanding = bytesBeingSent;
	return false;
}

// This is called by the Webserver to send output data to a client. If keepConnectionAlive is set to false,
// the current connection will be terminated once everything has been sent.
void NetworkTransaction::Commit(bool keepConnectionAlive)
{
	// If the connection has been terminated (e.g. RST received while writing upload data), discard this transaction
	if (!IsConnected() || status == released)
	{
		Discard();
		return;
	}

	// Free buffer holding the incoming data and prepare some values for the sending process
	FreePbuf();
	cs->persistConnection = keepConnectionAlive;
	if (sendBuffer == nullptr)
	{
		sendBuffer = sendStack->Pop();
	}
	status = sending;

	// Unlink the item(s) from the list of ready transactions
	if (keepConnectionAlive)
	{
		// Our connection is still of interest, remove only this transaction from the list
		NetworkTransaction *previous = nullptr;
		for(NetworkTransaction *item = reprap.GetNetwork()->readyTransactions; item != nullptr; item = item->next)
		{
			if (item == this)
			{
				if (previous == nullptr)
				{
					reprap.GetNetwork()->readyTransactions = next;
				}
				else
				{
					previous->next = next;
				}
				break;
			}
			previous = item;
		}
	}
	else
	{
		// We will close this connection soon, stop receiving data from this PCB
		tcp_recv(cs->pcb, nullptr);

		// Also remove all ready transactions pointing to our ConnectionState
		NetworkTransaction *previous = nullptr, *item = reprap.GetNetwork()->readyTransactions;
		while (item != nullptr)
		{
			if (item->cs == cs)
			{
				if (item == this)
				{
					// Only unlink this item
					if (previous == nullptr)
					{
						reprap.GetNetwork()->readyTransactions = next;
					}
					else
					{
						previous->next = next;
					}
					item = next;
				}
				else
				{
					// Remove all others
					item->Discard();
					item = (previous == nullptr) ? reprap.GetNetwork()->readyTransactions : previous->next;
				}
			}
			else
			{
				previous = item;
				item = item->next;
			}
		}
	}

	// Enqueue this transaction, so it's sent in the right order
	NetworkTransaction *mySendingTransaction = cs->sendingTransaction;
	if (mySendingTransaction == nullptr)
	{
		cs->sendingTransaction = this;
		reprap.GetNetwork()->AppendTransaction(&reprap.GetNetwork()->writingTransactions, this);
	}
	else
	{
		while (mySendingTransaction->nextWrite != nullptr)
		{
			mySendingTransaction = mySendingTransaction->nextWrite;
		}
		mySendingTransaction->nextWrite = this;
	}
}

// Call this to perform some networking tasks while processing deferred requests.
// If keepData is true, then the pbuf won't be released so it can be reused
void NetworkTransaction::Defer(bool keepData)
{
	// See if we have to keep the data for future calls
	if (keepData)
	{
		inputPointer = 0;
		readingPb = pb;
		if (IsConnected() && pb != nullptr && !dataAcknowledged)
		{
			tcp_recved(cs->pcb, pb->tot_len);
			dataAcknowledged = true;
		}
	}
	else
	{
		FreePbuf();			// Free up the allocated pbufs
	}
	status = deferred;

	// Unlink this transaction from the list of ready transactions and append it again
	NetworkTransaction *previous = nullptr;
	for(NetworkTransaction *item = reprap.GetNetwork()->readyTransactions; item != nullptr; item = item->next)
	{
		if (item == this)
		{
			if (previous == nullptr)
			{
				reprap.GetNetwork()->readyTransactions = next;
			}
			else
			{
				previous->next = next;
			}
			break;
		}
		previous = item;
	}
	reprap.GetNetwork()->AppendTransaction(&reprap.GetNetwork()->readyTransactions, this);
}

// This method should be called if we don't want to send data to the client and if we
// don't want to interfere with the connection state. May also be called from ISR!
void NetworkTransaction::Discard()
{
	// Can we do anything?
	if (status == released)
	{
		// No - don't free up released items multiple times
		return;
	}

	// Free up some resources
	FreePbuf();

	if (fileBeingSent != nullptr)
	{
		fileBeingSent->Close();
		fileBeingSent = nullptr;
	}

	OutputBuffer::ReleaseAll(sendBuffer);
	sendStack->ReleaseAll();

	// Unlink this transactions from the list of ready transactions and free it. It is then appended to the list of
	// free transactions because we don't want to risk reusing it when the ethernet ISR processes incoming data
	NetworkTransaction *previous = nullptr;
	for(NetworkTransaction *item = reprap.GetNetwork()->readyTransactions; item != nullptr; item = item->next)
	{
		if (item == this)
		{
			if (previous == nullptr)
			{
				reprap.GetNetwork()->readyTransactions = next;
			}
			else
			{
				previous->next = next;
			}
			break;
		}
		previous = item;
	}
	reprap.GetNetwork()->AppendTransaction(&reprap.GetNetwork()->freeTransactions, this);
	bool callDisconnectHandler = (cs != nullptr && status == disconnected);
	status = released;

	// Call disconnect event if this transaction indicates a graceful disconnect and if the connection
	// still persists (may not be the case if a RST packet was received before)
	if (callDisconnectHandler)
	{
		if (reprap.Debug(moduleNetwork))
		{
			reprap.GetPlatform()->Message(HOST_MESSAGE, "Network: Discard() is handling a graceful disconnect\n");
		}
		reprap.GetNetwork()->ConnectionClosed(cs, false);
	}
}

uint32_t NetworkTransaction::GetRemoteIP() const
{
	return (cs != nullptr) ? cs->GetRemoteIP() : 0;
}

uint16_t NetworkTransaction::GetRemotePort() const
{
	return (cs != nullptr) ? cs->GetRemotePort() : 0;
}

uint16_t NetworkTransaction::GetLocalPort() const
{
	return (cs != nullptr) ? cs->GetLocalPort() : 0;
}

void NetworkTransaction::Close()
{
	tcp_pcb *pcb = cs->pcb;
	tcp_recv(pcb, nullptr);
	closeRequested = true;
}

void NetworkTransaction::FreePbuf()
{
	// See if we have to send an ACK to the client
	if (IsConnected() && pb != nullptr && !dataAcknowledged)
	{
		tcp_recved(cs->pcb, pb->tot_len);
		dataAcknowledged = true;
	}

	// Free all pbufs (pbufs are thread-safe)
	if (pb != nullptr)
	{
		pbuf_free(pb);
		pb = readingPb = nullptr;
	}
}

// End
