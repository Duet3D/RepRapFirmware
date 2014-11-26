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
#include "DueFlashStorage.h"
#include "ethernet_sam.h"

extern "C"
{
#include "lwipopts.h"
#include "lwip/src/include/lwip/debug.h"
#include "lwip/src/include/lwip/stats.h"
#include "lwip/src/include/lwip/tcp.h"

void RepRapNetworkSetMACAddress(const u8_t macAddress[]);
}

static tcp_pcb *http_pcb = NULL;
static tcp_pcb *ftp_main_pcb = NULL;
static tcp_pcb *ftp_pasv_pcb = NULL;
static tcp_pcb *telnet_pcb = NULL;

static bool closingDataPort = false;

static uint8_t volatile inLwip = 0;

static NetworkTransaction *sendingTransaction = NULL;
static char sendingWindow[TCP_WND];
static uint16_t sendingWindowSize, sentDataOutstanding;
static uint8_t sendingRetries;

// Called to put out a message via the RepRap firmware.
// Can be called from C as well as C++

extern "C" void RepRapNetworkMessage(const char* s)
{
#ifdef LWIP_DEBUG
	reprap.GetPlatform()->Message(DEBUG_MESSAGE, s);
#else
	reprap.GetPlatform()->Message(HOST_MESSAGE, s);
#endif
}

/*-----------------------------------------------------------------------------------*/

extern "C"
{

// Callback functions for the EMAC driver

static void emac_read_packet(uint32_t ul_status)
{
	// Because the LWIP stack can become corrupted if we work with it in parallel,
	// we may have to wait for the next Spin() call to read the next packet.
	// On this occasion, we can set the RX callback again.
	if (inLwip)
	{
		reprap.GetNetwork()->ReadPacket();
		ethernet_set_rx_callback(NULL);
	}
	else
	{
		++inLwip;
		do {
			// read all queued packets from the RX buffer
		} while (ethernet_read());
		--inLwip;
	}
}

// Callback functions called by LWIP

static void conn_err(void *arg, err_t err)
{
	// Report the error to the monitor
	RepRapNetworkMessage("Network connection error, code ");
	{
		char tempBuf[10];
		snprintf(tempBuf, ARRAY_SIZE(tempBuf), "%d\n", err);
		RepRapNetworkMessage(tempBuf);
	}

	ConnectionState *cs = (ConnectionState*)arg;
	if (cs != NULL)
	{
		reprap.GetNetwork()->ConnectionClosed(cs, false);	// tell the higher levels about the error
		if (sendingTransaction == cs->sendingTransaction)
		{
			sendingTransaction = NULL;
			sentDataOutstanding = 0;
		}
	}
}

/*-----------------------------------------------------------------------------------*/

static err_t conn_poll(void *arg, tcp_pcb *pcb)
{
	ConnectionState *cs = (ConnectionState*)arg;
	if (cs != NULL && sendingTransaction != NULL && cs == sendingTransaction->GetConnection())
	{
		sendingRetries++;
		if (sendingRetries == 4)
		{
			RepRapNetworkMessage("poll received error!\n");

			tcp_abort(pcb);
			return ERR_ABRT;
		}

		tcp_write(pcb, sendingWindow + (sendingWindowSize - sentDataOutstanding), sentDataOutstanding, 0);
		tcp_output(pcb);
	}

	return ERR_OK;
}

/*-----------------------------------------------------------------------------------*/

static err_t conn_sent(void *arg, tcp_pcb *pcb, u16_t len)
{
	ConnectionState *cs = (ConnectionState*)arg;
	if (cs != NULL)
	{
		reprap.GetNetwork()->SentPacketAcknowledged(cs, len);
	}
	return ERR_OK;
}

/*-----------------------------------------------------------------------------------*/

static err_t conn_recv(void *arg, tcp_pcb *pcb, pbuf *p, err_t err)
{
	ConnectionState *cs = (ConnectionState*)arg;
	if (err == ERR_OK && cs != NULL)
	{
		if (cs->pcb != pcb)
		{
			RepRapNetworkMessage("Network: mismatched pcb\n");
			tcp_abort(pcb);
			return ERR_ABRT;
		}

		if (p != NULL)
		{
			// Tell higher levels that we are receiving data
			reprap.GetNetwork()->ReceiveInput(p, cs);
#if 0	//debug
			{
				char buff[20];
				strncpy(buff, (const char*)(p->payload), 18);
				buff[18] = '\n';
				buff[19] = 0;
				RepRapNetworkMessage("Network: Accepted data: ");
				RepRapNetworkMessage(buff);
			}
#endif
		}
		else if (cs->persistConnection)
		{
			// This is called when the connection has been gracefully closed, but LWIP doesn't close these
			// connections automatically. That's why we must do it once all packets have been read.
			reprap.GetNetwork()->ConnectionClosedGracefully(cs);
		}
	}

	return ERR_OK;
}

/*-----------------------------------------------------------------------------------*/

static err_t conn_accept(void *arg, tcp_pcb *pcb, err_t err)
{
	LWIP_UNUSED_ARG(arg);
	LWIP_UNUSED_ARG(err);

	tcp_setprio(pcb, TCP_PRIO_MIN);

	//RepRapNetworkMessage("conn_accept\n");

	/* Allocate a new ConnectionState for this connection */
	ConnectionState *cs = reprap.GetNetwork()->ConnectionAccepted(pcb);
	if (cs == NULL)
	{
		tcp_abort(pcb);
		return ERR_ABRT;
	}

	/* Keep the listening PCBs running */
	switch (pcb->local_port)		// tell LWIP to accept further connections on the listening PCB
	{
	  case 80: // HTTP
		  tcp_accepted(http_pcb);
		  break;

	  case 21: // FTP
		  tcp_accepted(ftp_main_pcb);
		  break;

	  case 23: // Telnet
		  tcp_accepted(telnet_pcb);
		  break;

	  default: // FTP data
		  tcp_accepted(ftp_pasv_pcb);
		  break;
	}
	tcp_arg(pcb, cs);				// tell LWIP that this is the structure we wish to be passed for our callbacks
	tcp_recv(pcb, conn_recv);		// tell LWIP that we wish to be informed of incoming data by a call to the conn_recv() function
	tcp_err(pcb, conn_err);
	tcp_poll(pcb, conn_poll, 4);

	return ERR_OK;
}

}	// end extern "C"

/*-----------------------------------------------------------------------------------*/

// These functions (are) should be called only once at the start.

void httpd_init()
{
	static int httpInitCount = 0;

	httpInitCount++;
	if (httpInitCount > 1)
	{
		RepRapNetworkMessage("httpd_init() called more than once.\n");
	}

	tcp_pcb* pcb = tcp_new();
	tcp_bind(pcb, IP_ADDR_ANY, 80);
	http_pcb = tcp_listen(pcb);
	tcp_accept(http_pcb, conn_accept);
}

void ftpd_init()
{
	static int ftpInitCount = 0;

	ftpInitCount++;
	if (ftpInitCount > 1)
	{
		RepRapNetworkMessage("ftpd_init() called more than once.\n");
	}

	tcp_pcb* pcb = tcp_new();
	tcp_bind(pcb, IP_ADDR_ANY, 23);
	ftp_main_pcb = tcp_listen(pcb);
	tcp_accept(ftp_main_pcb, conn_accept);
}

void telnetd_init()
{
	static int telnetInitCount = 0;

	telnetInitCount++;
	if (telnetInitCount > 1)
	{
		RepRapNetworkMessage("telnetd_init() called more than once.\n");
	}

	tcp_pcb* pcb = tcp_new();
	tcp_bind(pcb, IP_ADDR_ANY, 21);
	telnet_pcb = tcp_listen(pcb);
	tcp_accept(telnet_pcb, conn_accept);
}

//***************************************************************************************************

// Network/Ethernet class

Network::Network()
	: isEnabled(true), state(NetworkInactive), readingData(false),
	  freeTransactions(NULL), readyTransactions(NULL), writingTransactions(NULL),
	  dataCs(NULL), ftpCs(NULL), telnetCs(NULL), freeSendBuffers(NULL), freeConnections(NULL)
{
	for (int8_t i = 0; i < networkTransactionCount; i++)
	{
		freeTransactions = new NetworkTransaction(freeTransactions);
	}

	for (int8_t i = 0; i < tcpOutputBufferCount; i++)
	{
		freeSendBuffers = new SendBuffer(freeSendBuffers);
	}

	for (int8_t i = 0; i < numConnections; i++)
	{
		ConnectionState *cs = new ConnectionState;
		cs->next = freeConnections;
		freeConnections = cs;
	}

	ethPinsInit();
}

void Network::AppendTransaction(NetworkTransaction* volatile* list, NetworkTransaction *r)
{
	++inLwip;
	r->next = NULL;
	while (*list != NULL)
	{
		list = &((*list)->next);
	}
	*list = r;
	--inLwip;
}

void Network::PrependTransaction(NetworkTransaction* volatile* list, NetworkTransaction *r)
{
	++inLwip;
	r->next = *list;
	*list = r;
	--inLwip;
}

void Network::Init()
{
	if (!isEnabled)
	{
		reprap.GetPlatform()->Message(HOST_MESSAGE, "Attempting to start the network when it is disabled.\n");
		return;
	}
	RepRapNetworkSetMACAddress(reprap.GetPlatform()->MACAddress());
	init_ethernet();
	state = NetworkInitializing;
}

void Network::Spin()
{
	if (state == NetworkActive)
	{
		// See if we can read any packets
		if (readingData)
		{
			readingData = false;

			++inLwip;
			do {
				// read all queued packets from the RX buffer
			} while (ethernet_read());
			--inLwip;

			ethernet_set_rx_callback(&emac_read_packet);
		}

		// See if we can send anything
		++inLwip;
		NetworkTransaction *r = writingTransactions;
		if (r != NULL && r->Send())
		{
			ConnectionState *cs = r->cs;

			writingTransactions = r->next;
			NetworkTransaction *rn = r->nextWrite;
			r->nextWrite = NULL;
			AppendTransaction(&freeTransactions, r);

			if (rn != NULL)
			{
				if (cs != NULL)
				{
					cs->sendingTransaction = (rn->cs == cs) ? rn : NULL;
				}
				PrependTransaction(&writingTransactions, rn);
			}
			else if (cs != NULL)
			{
				cs->sendingTransaction = NULL;
			}
		}
		--inLwip;
	}
	else if (state == NetworkInitializing && establish_ethernet_link())
	{
		start_ethernet(reprap.GetPlatform()->IPAddress(), reprap.GetPlatform()->NetMask(), reprap.GetPlatform()->GateWay());
		httpd_init();
		ftpd_init();
		telnetd_init();
		ethernet_set_rx_callback(&emac_read_packet);
		state = NetworkActive;
	}
}

void Network::Interrupt()
{
	if (!inLwip && state != NetworkInactive)
	{
		++inLwip;
		ethernet_timers_update();
		--inLwip;
	}
}

void Network::Diagnostics()
{
	reprap.GetPlatform()->AppendMessage(BOTH_MESSAGE, "Network diagnostics:\n");

	uint8_t numFreeConnections = 0;
	ConnectionState *freeConn = freeConnections;
	while (freeConn != NULL)
	{
		numFreeConnections++;
		freeConn = freeConn->next;
	}
	reprap.GetPlatform()->AppendMessage(BOTH_MESSAGE, "Free connections: %d of %d\n", numFreeConnections, numConnections);

	uint8_t numFreeTransactions = 0;
	NetworkTransaction *freeTrans = freeTransactions;
	while (freeTrans != NULL)
	{
		numFreeTransactions++;
		freeTrans = freeTrans->next;
	}
	reprap.GetPlatform()->AppendMessage(BOTH_MESSAGE, "Free transactions: %d of %d\n", numFreeTransactions, networkTransactionCount);

	uint16_t numFreeSendBuffs = 0;
	SendBuffer *freeSendBuff = freeSendBuffers;
	while (freeSendBuff != NULL)
	{
		numFreeSendBuffs++;
		freeSendBuff = freeSendBuff->next;
	}
	reprap.GetPlatform()->AppendMessage(BOTH_MESSAGE, "Free send buffers: %d of %d\n", numFreeSendBuffs, tcpOutputBufferCount);
}

bool Network::InLwip() const
{
	return (inLwip);
}

void Network::ReadPacket()
{
	readingData = true;
}

void Network::Enable()
{
	if (!isEnabled)
	{
		readingData = true;
		// EMAC RX callback will be reset on next Spin calls
		Init();
		isEnabled = true;
	}
}

void Network::Disable()
{
	if (isEnabled)
	{
		readingData = false;
		ethernet_set_rx_callback(NULL);
		state = NetworkInactive;
		isEnabled = false;
	}
}

bool Network::IsEnabled() const
{
	return isEnabled;
}

bool Network::AllocateSendBuffer(SendBuffer *&buffer)
{
	buffer = freeSendBuffers;
	if (buffer == NULL)
	{
		RepRapNetworkMessage("Network: Could not allocate send buffer!\n");
		return false;
	}
	freeSendBuffers = buffer->next;

	buffer->bytesToWrite = 0;
	buffer->next = NULL;

	return true;
}

SendBuffer *Network::ReleaseSendBuffer(SendBuffer *buffer)
{
	// If we used up all available send buffers, reset freeSendBuffers here
	if (freeSendBuffers == NULL)
	{
		freeSendBuffers = buffer;
		freeSendBuffers->next = NULL;
		return NULL;
	}

	// Get the last item in the chain
	SendBuffer *lastItem = freeSendBuffers;
	while (lastItem->next != NULL)
	{
		lastItem = lastItem->next;
	}

	// Append the send buffer to be released
	lastItem->next = buffer;
	lastItem = buffer->next;
	buffer->next = NULL;

	// And return the next one of the send buffer being freed
	return lastItem;
}

void Network::SentPacketAcknowledged(ConnectionState *cs, unsigned int len)
{
	if (cs != NULL && sendingTransaction != NULL && cs == sendingTransaction->GetConnection())
	{
		if (sentDataOutstanding > len)
		{
			sentDataOutstanding -= len;
		}
		else
		{
			sendingTransaction = NULL;
			sentDataOutstanding = 0;
		}
	}

//	debugPrintf("Network SentPacketAcknowledged: invalid cs=%08x\n", (unsigned int)cs);
}

// This is called when a connection is being established and returns an initialised ConnectionState instance.
ConnectionState *Network::ConnectionAccepted(tcp_pcb *pcb)
{
	++inLwip;
	ConnectionState *cs = freeConnections;
	if (cs == NULL)
	{
		--inLwip;
		reprap.GetPlatform()->Message(HOST_MESSAGE, "Network::ConnectionAccepted() - no free ConnectionStates!\n");
		return NULL;
	}

	NetworkTransaction* r = freeTransactions;
	if (r == NULL)
	{
		--inLwip;
		reprap.GetPlatform()->Message(HOST_MESSAGE, "Network::ConnectionAccepted() - no free transactions!\n");
		return NULL;
	}

	freeConnections = cs->next;
	cs->Init(pcb);

	r->Set(NULL, cs, connected);
	freeTransactions = r->next;
	--inLwip;
	AppendTransaction(&readyTransactions, r);

	return cs;
}

// This is called when a connection is being closed or has gone down.
// It must set the state of any NetworkTransaction that refers to it to connection lost.
void Network::ConnectionClosed(ConnectionState* cs, bool closeConnection)
{
	// make sure these connections are not reused
	if (cs == dataCs)
	{
		dataCs = NULL;
	}
	if (cs == ftpCs)
	{
		ftpCs = NULL;
	}
	if (cs == telnetCs)
	{
		telnetCs = NULL;
	}

	// inform the Webserver that we are about to remove an existing connection
	tcp_pcb *pcb = cs->pcb;
	if (pcb != NULL)
	{
		reprap.GetWebserver()->ConnectionLost(cs);
		if (closeConnection)
		{
			tcp_arg(pcb, NULL);
			tcp_sent(pcb, NULL);
			tcp_recv(pcb, NULL);
			tcp_poll(pcb, NULL, 4);
			tcp_close(pcb);
		}
	}

	// cs points to a connection state block that the caller is about to release, so we need to stop referring to it.
	// There may be one NetworkTransaction in the writing or closing list referring to it, and possibly more than one in the ready list.
	//RepRapNetworkMessage("Network: ConnectionError\n");

	// See if it's a ready transaction
	for (NetworkTransaction* r = readyTransactions; r != NULL; r = r->next)
	{
		if (r->cs == cs)
		{
			r->SetConnectionLost();
		}
	}

	if (cs->sendingTransaction != NULL)
	{
		cs->sendingTransaction->SetConnectionLost();
		cs->sendingTransaction = NULL;
	}

	cs->next = freeConnections;
	freeConnections = cs;
}

// May be called from ISR
void Network::ConnectionClosedGracefully(ConnectionState *cs)
{
	++inLwip;
	NetworkTransaction* r = freeTransactions;
	if (r == NULL)
	{
		--inLwip;
		reprap.GetPlatform()->Message(HOST_MESSAGE, "Network::ConnectionClosedGracefully() - no free transactions!\n");
		return;
	}
	freeTransactions = r->next;
	r->Set(NULL, cs, disconnected);
	--inLwip;

	AppendTransaction(&readyTransactions, r);
}

// May be called from ISR
void Network::ReceiveInput(pbuf *pb, ConnectionState* cs)
{
	++inLwip;
	NetworkTransaction* r = freeTransactions;
	if (r == NULL)
	{
		--inLwip;
		reprap.GetPlatform()->Message(HOST_MESSAGE, "Network::ReceiveInput() - no free transactions!\n");
		return;
	}

	freeTransactions = r->next;
	r->Set(pb, cs, dataReceiving);
	--inLwip;

	AppendTransaction(&readyTransactions, r);
//	debugPrintf("Network - input received\n");
}

// This is called by the web server to get a new received packet.
// If the connection parameter is NULL, we just return the request at the head of the ready list.
// Otherwise, we are only interested in packets received from the specified connection. If we find one than
// we move it to the head of the ready list, so that a subsequent call with a null connection parameter
// will return the same one.
NetworkTransaction *Network::GetTransaction(const ConnectionState *cs)
{
	++inLwip;
	NetworkTransaction *rs = readyTransactions;
	if (rs == NULL || cs == NULL || rs->cs == cs)
	{
		--inLwip;
		return rs;
	}

	// There is at least one ready transaction, but it's not on the connection we are looking for
	for (NetworkTransaction *rsNext = rs->next; rsNext != NULL; rsNext = rs->next)
	{
		if (rsNext->cs == cs)
		{
			rs->next = rsNext->next;		// remove rsNext from the list
			rsNext->next = readyTransactions;
			readyTransactions = rsNext;
			--inLwip;
			return rsNext;
		}

		rs = rsNext;
	}

	--inLwip;
	return NULL;
}

// Send the output data we already have, optionally with a file appended, then close the connection unless keepConnectionOpen is true.
// The file may be too large for our buffer, so we may have to send it in multiple transactions.
void Network::SendAndClose(FileStore *f, bool keepConnectionOpen)
{
	++inLwip;
	NetworkTransaction *r = readyTransactions;
	if (r == NULL)
	{
		--inLwip;
	}
	else if (r->status == dataSending)
	{
		// This transaction is already in use for sending (e.g. a Telnet request),
		// so all we have to do is to remove it from readyTransactions.
		readyTransactions = r->next;
		--inLwip;
	}
	else
	{
		readyTransactions = r->next;
		--inLwip;

		r->FreePbuf();
		if (r->LostConnection())
		{
			if (f != NULL)
			{
				f->Close();
			}
			while (r->sendBuffer != NULL)
			{
				r->sendBuffer = ReleaseSendBuffer(r->sendBuffer);
			}
			AppendTransaction(&freeTransactions, r);
//			debugPrintf("Conn lost before send\n");
		}
		else
		{
			r->cs->persistConnection = keepConnectionOpen;
			r->nextWrite = NULL;
			r->fileBeingSent = f;
			r->status = dataSending;
			if (f != NULL && r->sendBuffer == NULL)
			{
				SendBuffer *buf;
				if (AllocateSendBuffer(buf))
				{
					r->sendBuffer = buf;
					r->fileBeingSent = f;
				}
				else
				{
					r->fileBeingSent = NULL;
//					debugPrintf("Could not allocate send buffer for file transfer!\n");
				}
			}

			NetworkTransaction *sendingTransaction = r->cs->sendingTransaction;
			if (sendingTransaction == NULL)
			{
				r->cs->sendingTransaction = r;
				AppendTransaction(&writingTransactions, r);
//debug
//				r->outputBuffer[r->outputPointer] = 0;
//				debugPrintf("Transaction queued for writing to network, file=%c, data=%s\n", (f ? 'Y' : 'N'), r->outputBuffer);
			}
			else
			{
				while (sendingTransaction->nextWrite != NULL)
				{
					sendingTransaction = sendingTransaction->nextWrite;
				}
				sendingTransaction->nextWrite = r;
//				debugPrintf("Transaction appended to sending RS\n");
			}
		}
	}
}

// We have no data to read nor to write and we want to keep the current connection alive if possible.
// That way we can speed up freeing the current NetworkTransaction.
void Network::CloseTransaction()
{
	// Free the current NetworkTransaction's data (if any)
	++inLwip;
	NetworkTransaction *r = readyTransactions;
	r->FreePbuf();

	// Terminate this connection if this NetworkTransaction indicates a graceful disconnect
	TransactionStatus status = r->status;
	if (!r->LostConnection() && status == disconnected)
	{
//		debugPrintf("Network: CloseRequest is closing connection cs=%08x\n", (unsigned int)locCs);
		ConnectionClosed(r->cs, true);
	}

	// Remove the current item from readyTransactions
	readyTransactions = r->next;
	--inLwip;

	// Append it to freeTransactions again unless it's already on another list
	if (status != dataSending)
	{
		AppendTransaction(&freeTransactions, r);
	}
}


// The current NetworkTransaction must be processed again, e.g. because we're still waiting for another
// data connection.
void Network::RepeatTransaction()
{
	++inLwip;
	NetworkTransaction *r = readyTransactions;
	r->inputPointer = 0; // behave as if this request hasn't been processed yet
	if (r->next != NULL)
	{
		readyTransactions = r->next;
		--inLwip;

		AppendTransaction(&readyTransactions, r);
	}
	else
	{
		--inLwip;
	}
}

void Network::OpenDataPort(uint16_t port)
{
	++inLwip;
	tcp_pcb* pcb = tcp_new();
	tcp_bind(pcb, IP_ADDR_ANY, port);
	ftp_pasv_pcb = tcp_listen(pcb);
	tcp_accept(ftp_pasv_pcb, conn_accept);
	--inLwip;
}

// Close the data port and return true on success
bool Network::CloseDataPort()
{
	// See if it's already being closed
	if (closingDataPort)
	{
		return false;
	}

	// Close the open data connection if there is any
	if (dataCs != NULL && dataCs->pcb != NULL)
	{
		NetworkTransaction *sendingTransaction = dataCs->sendingTransaction;
		if (sendingTransaction != NULL)
		{
			// we can't close the connection as long as we're sending
			closingDataPort = true;
			sendingTransaction->Close();
			return false;
		}
		else
		{
			// close the data connection
//			debugPrintf("CloseDataPort is closing connection dataCS=%08x\n", (unsigned int)loc_cs);
			++inLwip;
			ConnectionClosed(dataCs, true);
			--inLwip;
		}
	}

	// close listening data port
	if (ftp_pasv_pcb != NULL)
	{
		++inLwip;
		tcp_accept(ftp_pasv_pcb, NULL);
		tcp_close(ftp_pasv_pcb);
		ftp_pasv_pcb = NULL;
		--inLwip;
	}

	return true;
}

// These methods keep track of our connections in case we need to send to one of them
void Network::SaveDataConnection()
{
	++inLwip;
	dataCs = readyTransactions->cs;
	--inLwip;
}

void Network::SaveFTPConnection()
{
	++inLwip;
	ftpCs = readyTransactions->cs;
	--inLwip;
}

void Network::SaveTelnetConnection()
{
	++inLwip;
	telnetCs = readyTransactions->cs;
	--inLwip;
}

// Check if there are enough resources left to allocate another NetworkTransaction for sending
bool Network::CanAcquireTransaction()
{
	++inLwip;
	if (freeTransactions == NULL)
	{
		--inLwip;
		return false;
	}
	--inLwip;

	return (freeSendBuffers != NULL);
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

// Retrieves the NetworkTransaction of a sending connection to which dataLength bytes can be appended at
// the present time or returns a released NetworkTransaction, which can easily be sent via SendAndClose.
bool Network::AcquireTransaction(ConnectionState *cs)
{
	// Make sure we have a valid connection
	if (cs == NULL)
	{
		reprap.GetPlatform()->Message(BOTH_ERROR_MESSAGE, "Attempting to allocate transaction for no connection!\n");
		return false;
	}

	// See if we're already writing on this connection
	NetworkTransaction *lastTransaction = cs->sendingTransaction;
	if (lastTransaction != NULL)
	{
		while (lastTransaction->next != NULL)
		{
			lastTransaction = lastTransaction->nextWrite;
		}
	}

	// Then check if this transaction is valid and safe to use
	NetworkTransaction *transactionToUse;
	if (lastTransaction != NULL && sendingTransaction != lastTransaction && lastTransaction->fileBeingSent == NULL)
	{
		transactionToUse = lastTransaction;
	}
	// We cannot use it, so try to allocate a free one
	else
	{
		++inLwip;
		transactionToUse = freeTransactions;
		if (transactionToUse == NULL)
		{
			--inLwip;
			reprap.GetPlatform()->Message(HOST_MESSAGE, "Could not acquire free transaction!\n");
			return false;
		}
		freeTransactions = transactionToUse->next;
		--inLwip;
		transactionToUse->Set(NULL, cs, dataReceiving); // set it to dataReceiving as we expect a response
	}

	// Replace the first entry of readyTransactions with our new transaction, so it can be used by SendAndClose().
	PrependTransaction(&readyTransactions, transactionToUse);
	return true;
}

// Initialise a ConnectionState for a new connection
void ConnectionState::Init(tcp_pcb *p)
{
	pcb = p;
	next = NULL;
	sendingTransaction = NULL;
	persistConnection = true;
}

// Get local port from a ConnectionState
uint16_t ConnectionState::GetLocalPort() const
{
	return pcb->local_port;
}

// NetRing class members
void NetworkTransaction::Set(pbuf *p, ConnectionState *c, TransactionStatus s)
{
	cs = c;
	pb = p;
	bufferLength = (p == NULL) ? 0 : pb->tot_len;
	status = s;
	inputPointer = 0;
	sendBuffer = NULL;
	fileBeingSent = NULL;
	closeRequested = false;
	nextWrite = NULL;
	lastWriteTime = NAN;
}

// Webserver calls this to read bytes that have come in from the network.
bool NetworkTransaction::Read(char& b)
{
	if (LostConnection() || pb == NULL)
	{
		return false;
	}

	if (inputPointer == pb->len)
	{
		// See if there is another pbuf in the chain
		if (inputPointer < pb->tot_len)
		{
			pb = pbuf_dechain(pb);
			if (pb == NULL)
			{
				return false;
			}
			inputPointer = 0;
		}
		else
		{
			return false;
		}
	}

	b = ((const char*)pb->payload)[inputPointer++];
	return true;
}

// Read an entire pbuf from the NetworkTransaction
bool NetworkTransaction::ReadBuffer(char *&buffer, unsigned int &len)
{
	if (LostConnection() || pb == NULL)
	{
		return false;
	}

	if (inputPointer == pb->len)
	{
		// See if there is another pbuf in the chain
		if (inputPointer < pb->tot_len)
		{
			pb = pbuf_dechain(pb);
			if (pb == NULL)
			{
				return false;
			}
			inputPointer = 0;
		}
		else
		{
			return false;
		}
	}

	len = pb->len;
	buffer = static_cast<char *>(pb->payload);
	inputPointer += len;

	return true;
}

// Webserver calls this to write bytes that need to go out to the network

void NetworkTransaction::Write(char b)
{
	if (LostConnection() || status == disconnected) return;

	if (sendBuffer == NULL)
	{
		if (reprap.GetNetwork()->AllocateSendBuffer(sendBuffer))
		{
			sendBuffer->tcpOutputBuffer[0] = b;
			sendBuffer->bytesToWrite = 1;
		}
		else
		{
			// We cannot write because there are no more send buffers available.
		}
	}
	else
	{
		// Get the last SendBuffer in the chain
		SendBuffer *lastSendBuffer = sendBuffer;
		while (lastSendBuffer->next != NULL)
		{
			lastSendBuffer = lastSendBuffer->next;
		}

		// Check if there's enough space left
		if (tcpOutputBufferSize - lastSendBuffer->bytesToWrite > 0)
		{
			lastSendBuffer->tcpOutputBuffer[lastSendBuffer->bytesToWrite] = b;
			lastSendBuffer->bytesToWrite++;
		}

		// No, try to allocate another one instead and append it to the list
		else
		{
			SendBuffer *newSendBuffer;
			if (reprap.GetNetwork()->AllocateSendBuffer(newSendBuffer))
			{
				lastSendBuffer->next = newSendBuffer;
				newSendBuffer->tcpOutputBuffer[0] = b;
				newSendBuffer->bytesToWrite = 1;
			}
			else
			{
				// We cannot write because there are no more send buffers available.
			}
		}
	}
}

// These functions attempt to store a whole string for sending.
// It may be necessary to split it up into multiple SendBuffers.
void NetworkTransaction::Write(const char* s)
{
	unsigned int len = strlen(s);
	Write(s, len);
}

void NetworkTransaction::Write(StringRef ref)
{
	Write(ref.Pointer(), ref.strlen());
}

void NetworkTransaction::Write(const char* s, size_t len)
{
	if (LostConnection() || status == disconnected) return;

	// Do we have a SendBuffer instance?

	Network *net = reprap.GetNetwork();
	if (sendBuffer == NULL)
	{
		if (!net->AllocateSendBuffer(sendBuffer))
		{
			// We cannot write because there are no more send buffers available.
			return;
		}
	}

	// Yes - retrieve the last send buffer from the chain.

	SendBuffer *lastSendBuffer = sendBuffer;
	while (lastSendBuffer->next != NULL)
	{
		lastSendBuffer = lastSendBuffer->next;
	}

	// Then try to store the whole string in multiple chunks if necessary.

	size_t bytesStored = 0, bytesToStore = min<size_t>(len, tcpOutputBufferSize - lastSendBuffer->bytesToWrite);
	do {
		// Fill up current SendBuffer

		memcpy(lastSendBuffer->tcpOutputBuffer + lastSendBuffer->bytesToWrite, s + bytesStored, bytesToStore);
		lastSendBuffer->bytesToWrite += bytesToStore;

		bytesStored += bytesToStore;
		bytesToStore = min<size_t>(tcpOutputBufferSize, len - bytesStored);

//		debugPrintf("%d of %d bytes stored, now storing %d bytes (%d bytes left in this chunk)\n", bytesStored, len, bytesToStore, tcpOutputBufferSize - currSendBuff->bytesToWrite);

		// Allocate another SendBuffer if we cannot store the whole string in the current one

		if (bytesToStore)
		{
			SendBuffer *newSendBuff;
			if (net->AllocateSendBuffer(newSendBuff))
			{
				lastSendBuffer->next = newSendBuff;
				lastSendBuffer = newSendBuff;
			}
			else
			{
				// We cannot write because there are no more send buffers available.
				break;
			}
		}
	} while (bytesToStore);
}

// Write formatted data to the output buffer
void NetworkTransaction::Printf(const char* fmt, ...)
{
	if (LostConnection() || status == disconnected) return;

	va_list p;
	va_start(p, fmt);
	char tempString[STRING_LENGTH];
	int len = vsnprintf(tempString, STRING_LENGTH, fmt, p);
	va_end(p);

	Write(tempString, len);
}

// Send exactly one TCP window of data or return true if we can free up this object
bool NetworkTransaction::Send()
{
	// We can't send any data if the connection has been lost
	if (LostConnection())
	{
		if (fileBeingSent != NULL)
		{
			fileBeingSent->Close();
			fileBeingSent = NULL;
		}

		Network *net = reprap.GetNetwork();
		while (sendBuffer != NULL)
		{
			sendBuffer = net->ReleaseSendBuffer(sendBuffer);
		}

		sendingTransaction = NULL;
		sentDataOutstanding = 0;

		return true;
	}

	// We've finished with this RS and want to close the connection, so do it here
	if (closeRequested)
	{
		// Close the file if it is still open
		if (fileBeingSent != NULL)
		{
			fileBeingSent->Close();
			fileBeingSent = NULL;
		}

		// Close the connection PCB
//		debugPrintf("NetworkTransaction is closing connection cs=%08x\n", (unsigned int)cs);
		reprap.GetNetwork()->ConnectionClosed(cs, true);

		// Close the main connection if possible
		if (closingDataPort)
		{
			if (ftp_pasv_pcb != NULL)
			{
				tcp_accept(ftp_pasv_pcb, NULL);
				tcp_close(ftp_pasv_pcb);
				ftp_pasv_pcb = NULL;
			}

			closingDataPort = false;
		}
	}

	// We're still waiting for data to be ACK'ed, so check timeouts here
	if (sentDataOutstanding)
	{
		if (!isnan(lastWriteTime))
		{
			float timeNow = reprap.GetPlatform()->Time();
			if (timeNow - lastWriteTime > writeTimeout)
			{
				reprap.GetPlatform()->Message(HOST_MESSAGE, "Network: Timing out connection cs=%08x\n", (unsigned int)cs);
				Close();
				lastWriteTime = NAN;
			}
			return false;
		}
	}
	else
	{
		sendingTransaction = NULL;
	}

	// See if we can fill up the TCP window with some data chunks from our SendBuffer instances
	uint16_t bytesBeingSent = 0, bytesLeftToSend = TCP_WND;
	while (sendBuffer != NULL && bytesLeftToSend >= sendBuffer->bytesToWrite)
	{
		memcpy(sendingWindow + bytesBeingSent, sendBuffer->tcpOutputBuffer, sendBuffer->bytesToWrite);
		bytesBeingSent += sendBuffer->bytesToWrite;

		sendBuffer = reprap.GetNetwork()->ReleaseSendBuffer(sendBuffer);
		bytesLeftToSend = TCP_WND - bytesBeingSent;
	}

	// We also intend to send a file, so check if we can fill up the TCP window
	if (sendBuffer == NULL)
	{
		/*char c;
		while (bytesBeingSent != TCP_WND)
		{
			if (!fileBeingSent->Read(c))
			{
				fileBeingSent->Close();
				fileBeingSent = NULL;
				break;
			}
			sendingWindow[bytesBeingSent] = c;
			bytesBeingSent++;
		}*/

		int bytesRead;
		size_t bytesToRead;
		while (bytesLeftToSend && fileBeingSent != NULL)
		{
			bytesToRead = min<size_t>(256, bytesLeftToSend);  // FIXME: doesn't work with higher block sizes
			bytesRead = fileBeingSent->Read(sendingWindow + bytesBeingSent, bytesToRead);

			if (bytesRead > 0)
			{
				bytesBeingSent += bytesRead;
				bytesLeftToSend = TCP_WND - bytesBeingSent;
			}

			if (bytesRead != bytesToRead)
			{
				fileBeingSent->Close();
				fileBeingSent = NULL;
			}
		}
	}

	if (!bytesBeingSent)
	{
		// If we have no data to send and fileBeingSent is NULL, we can close the connection
		if (!cs->persistConnection && nextWrite == NULL)
		{
			Close();
			return false;
		}

		// We want to send data from another transaction, so only free up this one
		return true;
	}
	else
	{
		// The TCP window has been filled up as much as possible, so send it now. There is no need to check
		// the available space in the SNDBUF queue, because we really write only one TCP window at once.
		tcp_sent(cs->pcb, conn_sent);
		if (tcp_write(cs->pcb, sendingWindow, bytesBeingSent, 0 /*TCP_WRITE_FLAG_COPY*/ ) != ERR_OK) // Final arg - 1 means make a copy
		{
			RepRapNetworkMessage("tcp_write encountered an error, this should never happen!\n");

			cs->pcb = NULL;
			tcp_abort(cs->pcb);
		}
		else
		{
			sendingTransaction = this;
			sendingRetries = 0;
			sendingWindowSize = sentDataOutstanding = bytesBeingSent;

			lastWriteTime = reprap.GetPlatform()->Time();

			tcp_output(cs->pcb);
		}
	}
	return false;
}

void NetworkTransaction::SetConnectionLost()
{
	cs = NULL;
	FreePbuf();
	for (NetworkTransaction *rs = nextWrite; rs != NULL; rs = rs->nextWrite)
	{
		rs->cs = NULL;
	}
}

uint16_t NetworkTransaction::GetLocalPort() const
{
	return (cs != NULL) ? cs->pcb->local_port : 0;
}

void NetworkTransaction::Close()
{
	++inLwip;
	tcp_pcb *pcb = cs->pcb;
	tcp_poll(pcb, NULL, 4);
	tcp_recv(pcb, NULL);
	closeRequested = true;
	--inLwip;
}

void NetworkTransaction::FreePbuf()
{
	// Tell LWIP that we have processed data
	if (cs != NULL && bufferLength > 0 && cs->pcb != NULL)
	{
		++inLwip;
		tcp_recved(cs->pcb, bufferLength);
		bufferLength = 0;
		--inLwip;
	}

	// Free pbuf (pbufs are thread-safe)
	if (pb != NULL)
	{
		pbuf_free(pb);
		pb = NULL;
	}
}

// End
