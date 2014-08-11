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

const unsigned int requestStateSize = MEMP_NUM_TCP_PCB * 2;		// number of RequestStates that can be used for network IO
const float writeTimeout = 4.0;	 								// seconds to wait for data we have written to be acknowledged

static tcp_pcb *http_pcb = NULL;
static tcp_pcb *ftp_main_pcb = NULL;
static tcp_pcb *ftp_pasv_pcb = NULL;
static tcp_pcb *telnet_pcb = NULL;

static bool closingDataPort = false;

static uint8_t volatile inLwip = 0;

// Called to put out a message via the RepRap firmware.
// Can be called from C as well as C++

extern "C" void RepRapNetworkMessage(const char* s)
{
	reprap.GetPlatform()->Message(HOST_MESSAGE, s);
}


static void SendData(tcp_pcb *pcb, ConnectionState *cs)
{
	err_t err;
	u16_t len;

	/* We cannot send more data than space available in the send buffer. */
	if (tcp_sndbuf(pcb) < cs->left)
	{
		len = tcp_sndbuf(pcb);
	}
	else
	{
		len = cs->left;
	}

	//  RepRapNetworkMessage("Sending ");
	//  sprintf(scratch, "%d", len);
	//  RepRapNetworkMessage(scratch);
	//  RepRapNetworkMessage("..");

	do {
		err = tcp_write(pcb, cs->file, len, 0 /*TCP_WRITE_FLAG_COPY*/ ); // Final arg - 1 means make a copy
		if (err == ERR_MEM) {
			len /= 2;
		}
	} while (err == ERR_MEM && len > 1);

	if (err == ERR_OK)
	{
		tcp_output(pcb);
		cs->file += len;
		cs->left -= len;
		if (cs->left == 0)
		{
			cs->file = NULL;
		}
	}
	else
	{
//		debugPrintf("send_data: error %d\n", err);
		tcp_abort(cs->pcb);
		cs->pcb = NULL;
	}
}

/*-----------------------------------------------------------------------------------*/

extern "C"
{

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
		reprap.GetNetwork()->ConnectionClosed(cs);	// tell the higher levels about the error
		mem_free(cs);								// release the state data
	}
}

/*-----------------------------------------------------------------------------------*/

static err_t conn_poll(void *arg, tcp_pcb *pcb)
{
	ConnectionState *cs = (ConnectionState*)arg;
	if (cs != NULL && cs->left > 0)
	{
		++cs->retries;
		if (cs->retries == 4)
		{
			RepRapNetworkMessage("poll received error!\n");

			tcp_abort(pcb);
			return ERR_ABRT;
		}
		SendData(pcb, cs);
	}

	return ERR_OK;
}

/*-----------------------------------------------------------------------------------*/

static err_t conn_sent(void *arg, tcp_pcb *pcb, u16_t len)
{
	ConnectionState *cs = (ConnectionState*)arg;
	if (cs != NULL)
	{
		cs->retries = 0;

		//RepRapNetworkMessage("..sent\n");

		if (cs->left > 0)
		{
			SendData(pcb, cs);
		}
		else
		{
			// See if there is more to send
			reprap.GetNetwork()->SentPacketAcknowledged(cs, len);
		}
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
			return ERR_BUF;
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
		else
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

	ConnectionState *cs = (ConnectionState*)mem_malloc(sizeof(ConnectionState));
	if (cs == NULL)
	{
	  RepRapNetworkMessage("Out of memory!\n");
	  return ERR_MEM;
	}

	/* Initialize the structure. */
	cs->pcb = pcb;
	cs->file = NULL;
	cs->left = 0;
	cs->retries = 0;
	cs->sendingRs = NULL;

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

	reprap.GetNetwork()->ConnectionAccepted(cs);

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

/*-----------------------------------------------------------------------------------*/

// RepRap calls this with data to send.
void RepRapNetworkSendOutput(const char* data, int length, ConnectionState* cs)
{
	if (cs == NULL)
	{
		RepRapNetworkMessage("Network: Attempt to write with null structure.\n");
		return;
	}

	cs->file = data;
	cs->left = length;
	cs->retries = 0;

	tcp_sent(cs->pcb, conn_sent);	// tell lwip that we wish be to informed of data that has been successfully sent by a call to the conn_sent() function
	SendData(cs->pcb, cs);
}

//***************************************************************************************************

// SendBuffer class

SendBuffer::SendBuffer(SendBuffer *n) : next(n)
{
}


// Network/Ethernet class

Network::Network()
	: state(NetworkInactive), readingData(false),
	  freeTransactions(NULL), readyTransactions(NULL), writingTransactions(NULL),
	  dataCs(NULL), ftpCs(NULL), telnetCs(NULL),
	  sendBuffer(NULL)
{
	for (int8_t i = 0; i < requestStateSize; i++)
	{
		freeTransactions = new RequestState(freeTransactions);
	}

	for (int8_t i = 0; i < tcpOutputBufferCount; i++)
	{
		sendBuffer = new SendBuffer(sendBuffer);
	}

	ethPinsInit();
}

void Network::AppendTransaction(RequestState* volatile* list, RequestState *r)
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

void Network::PrependTransaction(RequestState* volatile* list, RequestState *r)
{
	++inLwip;
	r->next = *list;
	*list = r;
	--inLwip;
}

void Network::Init()
{
	RepRapNetworkSetMACAddress(reprap.GetPlatform()->MACAddress());
	init_ethernet();
	state = NetworkInitializing;
}

void Network::Spin()
{
	if (state == NetworkActive)
	{
		// Fetch incoming data
		// ethernet_task() is called twice because the EMAC RX buffers have been increased by dc42's Arduino patch.
		++inLwip;
		ethernet_task();
		ethernet_task();
		--inLwip;

		// See if we can send anything
		++inLwip;
		RequestState* r = writingTransactions;
		if (r != NULL)
		{
			bool finished = r->Send();
			writingTransactions = r->next;

			if (finished)
			{
				RequestState *rn = r->nextWrite;
				r->nextWrite = NULL;
				ConnectionState *cs = r->cs;
				AppendTransaction(&freeTransactions, r);
				if (rn != NULL)
				{
					if (cs != NULL)
					{
						cs->sendingRs = (rn->cs == cs) ? rn : NULL;
					}
					AppendTransaction(&writingTransactions, rn);
				}
				else if (cs != NULL)
				{
					cs->sendingRs = NULL;
				}
			}
			else
			{
				AppendTransaction(&writingTransactions, r);
			}
		}
		--inLwip;
	}
	else if (state == NetworkInitializing)
	{
		if (establish_ethernet_link())
		{
			start_ethernet(reprap.GetPlatform()->IPAddress(), reprap.GetPlatform()->NetMask(), reprap.GetPlatform()->GateWay());
			httpd_init();
			ftpd_init();
			telnetd_init();
			state = NetworkActive;
		}
	}
}

bool Network::InLwip() const
{
	return (inLwip);
}

void Network::ReadPacket()
{
	readingData = true;
}

bool Network::AllocateSendBuffer(SendBuffer *&buffer)
{
	// Grab another send buffer
	buffer = sendBuffer;
	if (buffer == NULL)
	{
//		debugPrintf("Network: Could not allocate send buffer!\n");
		return false;
	}

	// Remove first item from the chain
	sendBuffer = buffer->next;
	buffer->next = NULL;

	return true;
}

void Network::FreeSendBuffer(SendBuffer *buffer)
{
	// Append the current send buffer to the free send buffers
	if (sendBuffer == NULL)
	{
		sendBuffer = buffer;
	}
	else
	{
		SendBuffer *buf = sendBuffer;
		while (buf->next != NULL)
		{
			buf = buf->next;
		}
		buf->next = buffer;
	}
}

void Network::SentPacketAcknowledged(ConnectionState *cs, unsigned int len)
{
	RequestState *r = cs->sendingRs;
	if (r != NULL)
	{
		r->SentPacketAcknowledged(len);
		return;
	}

//	debugPrintf("Network SentPacketAcknowledged: didn't find cs=%08x\n", (unsigned int)cs);
	RepRapNetworkMessage("Network::SentPacketAcknowledged: didn't find cs\n");
}

// This is called when a connection has been accepted
// It must set the state of any RequestState that refers to it to connection accepted.
void Network::ConnectionAccepted(ConnectionState *cs)
{
	RequestState* r = freeTransactions;
	if (r == NULL)
	{
		reprap.GetPlatform()->Message(HOST_MESSAGE, "Network::ConnectionAccepted() - no free transactions!\n");
		return;
	}

	freeTransactions = r->next;

	r->Set(NULL, cs, connected);
	AppendTransaction(&readyTransactions, r);
//	debugPrintf("Network - connection accepted\n");
}

// This is called when a connection is being closed or has gone down.
// It must set the state of any RequestState that refers to it to connection lost.
void Network::ConnectionClosed(ConnectionState* cs)
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
	if (cs->pcb != NULL)
	{
		reprap.GetWebserver()->ConnectionLost(cs);
	}

	// cs points to a connection state block that the caller is about to release, so we need to stop referring to it.
	// There may be one RequestState in the writing or closing list referring to it, and possibly more than one in the ready list.
	//RepRapNetworkMessage("Network: ConnectionError\n");

	// See if it's a ready transaction
	for (RequestState* r = readyTransactions; r != NULL; r = r->next)
	{
		if (r->cs == cs)
		{
			r->SetConnectionLost();
		}
	}

	if (cs->sendingRs != NULL)
	{
		cs->sendingRs->SetConnectionLost();
		cs->sendingRs = NULL;
	}
}

void Network::ConnectionClosedGracefully(ConnectionState *cs)
{
	RequestState* r = freeTransactions;
	if (r == NULL)
	{
		reprap.GetPlatform()->Message(HOST_MESSAGE, "Network::ConnectionClosedGracefully() - no free transactions!\n");
		return;
	}
	freeTransactions = r->next;
	r->Set(NULL, cs, disconnected);

	AppendTransaction(&readyTransactions, r);
}

void Network::ReceiveInput(pbuf *pb, ConnectionState* cs)
{
	RequestState* r = freeTransactions;
	if (r == NULL)
	{
		reprap.GetPlatform()->Message(HOST_MESSAGE, "Network::ReceiveInput() - no free transactions!\n");
		return;
	}

	freeTransactions = r->next;

	r->Set(pb, cs, dataReceiving);
	AppendTransaction(&readyTransactions, r);
//	debugPrintf("Network - input received\n");
}

// This is called by the web server to get a new received packet.
// If the connection parameter is NULL, we just return the request at the head of the ready list.
// Otherwise, we are only interested in packets received from the specified connection. If we find one than
// we move it to the head of the ready list, so that a subsequent call with a null connection parameter
// will return the same one.
RequestState *Network::GetRequest(const ConnectionState *cs)
{
	++inLwip;
	RequestState *rs = readyTransactions;
	if (rs == NULL || cs == NULL || rs->cs == cs)
	{
		--inLwip;
		return rs;
	}

	// There is at least one ready transaction, but it's not on the connection we are looking for
	for (RequestState *rsNext = rs->next; rsNext != NULL; rsNext = rs->next)
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
	RequestState *r = readyTransactions;
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
			SendBuffer *buf = r->sendBuffer;
			if (buf != NULL)
			{
				FreeSendBuffer(buf);
				buf = NULL;
			}
			AppendTransaction(&freeTransactions, r);
//			debugPrintf("Conn lost before send\n");
		}
		else
		{
			r->persistConnection = keepConnectionOpen;
			r->nextWrite = NULL;
			r->fileBeingSent = f;
			r->status = dataSending;
			if (f != NULL && r->sendBuffer == NULL)
			{
				SendBuffer *buf;
				if (AllocateSendBuffer(buf))
				{
					r->sendBuffer = buf;
					r->outputBuffer = r->sendBuffer->tcpOutputBuffer;
					r->fileBeingSent = f;
				}
				else
				{
					r->fileBeingSent = NULL;
//					debugPrintf("Could not allocate send buffer for file transfer!\n");
				}
			}

			RequestState *sendingRs = r->cs->sendingRs;
			if (sendingRs == NULL)
			{
				r->cs->sendingRs = r;
				AppendTransaction(&writingTransactions, r);
//debug
//				r->outputBuffer[r->outputPointer] = 0;
//				debugPrintf("Transaction queued for writing to network, file=%c, data=%s\n", (f ? 'Y' : 'N'), r->outputBuffer);
			}
			else
			{
				while (sendingRs->nextWrite != NULL)
				{
					sendingRs = sendingRs->nextWrite;
				}
				sendingRs->nextWrite = r;
//				debugPrintf("Transaction appended to sending RS\n");
			}
		}
	}
}

// We have no data to read nor to write and we want to keep the current connection alive if possible.
// That way we can speed up freeing the current RequestState.
void Network::CloseRequest()
{
	// Free the current RequestState's data (if any)
	++inLwip;
	RequestState *r = readyTransactions;
	r->FreePbuf();

	// Terminate this connection if this RequestState indicates a graceful disconnect
	RequestStatus status = r->status;
	if (!r->LostConnection() && status == disconnected)
	{
		ConnectionState *locCs = r->cs;		// take a copy because our cs field is about to get cleared
//		debugPrintf("Network: CloseRequest is closing connection cs=%08x\n", (unsigned int)locCs);
		ConnectionClosed(locCs);
		tcp_pcb *pcb = locCs->pcb;
		tcp_arg(pcb, NULL);
		tcp_sent(pcb, NULL);
		tcp_recv(pcb, NULL);
		tcp_poll(pcb, NULL, 4);
		mem_free(locCs);
		tcp_close(pcb);
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

// The current RequestState must be processed again, e.g. because we're still waiting for another
// data connection.
void Network::RepeatRequest()
{
	++inLwip;
	RequestState *r = readyTransactions;
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
		RequestState *sendingRs = dataCs->sendingRs;
		if (sendingRs != NULL)
		{
			// we can't close the connection as long as we're sending
			closingDataPort = true;
			sendingRs->Close();
			return false;
		}
		else
		{
			// close the data connection
			ConnectionState *loc_cs = dataCs;
//			debugPrintf("CloseDataPort is closing connection dataCS=%08x\n", (unsigned int)loc_cs);
			ConnectionClosed(loc_cs);
			++inLwip;
			tcp_pcb *pcb = loc_cs->pcb;
			tcp_arg(pcb, NULL);
			tcp_sent(pcb, NULL);
			tcp_recv(pcb, NULL);
			tcp_poll(pcb, NULL, 4);
			mem_free(loc_cs);
			tcp_close(pcb);
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

// Check if there are enough resources left to allocate another RequestState for sending
bool Network::CanMakeRequest()
{
	++inLwip;
	if (freeTransactions == NULL)
	{
		--inLwip;
		return false;
	}
	--inLwip;

	return (sendBuffer != NULL);
}

// Stored connections can be be restored by calling one of the following functions
bool Network::MakeDataRequest()
{
	return MakeRequest(0, dataCs);
}

bool Network::MakeFTPRequest()
{
	return MakeRequest(0, ftpCs);
}

bool Network::MakeTelnetRequest(unsigned int dataLength)
{
	return MakeRequest(dataLength, telnetCs);
}

// Retrieves the sending connection to which dataLength bytes can be appended or
// returns a free transaction if dataLength is 0.
bool Network::MakeRequest(unsigned int dataLength, ConnectionState *cs)
{
	// Make sure we have a connection and that we have another SendBuffer left
	if (cs == NULL || sendBuffer == NULL)
	{
		return false;
	}

	// See if we are already writing data on this connection
	RequestState *r = (dataLength > 0) ? writingTransactions : NULL;
	++inLwip;
	while (r != NULL)
	{
		if (r->cs == cs)
		{
			while (r->nextWrite != NULL)
			{
				r = r->nextWrite;
			}
			break;
		}
		r = r->next;
	}

	// If we are sending and there isn't enough space to append a message, we must get a free transaction
	if (r == NULL || r->fileBeingSent != NULL || r->outputPointer + dataLength > tcpOutputBufferSize)
	{
		r = freeTransactions;
		if (r == NULL)
		{
			--inLwip;
			reprap.GetPlatform()->Message(HOST_MESSAGE, "Network::MakeRequest() - no free transactions!\n");
			return false;
		}
		freeTransactions = r->next;
		r->Set(NULL, cs, dataReceiving);
	}
	--inLwip;

	// Replace the first entry of readyTransactions with our new transaction,
	// so it can be removed again by SendAndClose().
	PrependTransaction(&readyTransactions, r);

	return true;
}

// Get local port from a ConnectionState
uint16_t ConnectionState::GetLocalPort() const
{
	return pcb->local_port;
}

// NetRing class members
RequestState::RequestState(RequestState* n) : next(n)
{
}

void RequestState::Set(pbuf *p, ConnectionState *c, RequestStatus s)
{
	cs = c;
	pb = p;
	bufferLength = (p == NULL) ? 0 : pb->tot_len;
	status = s;
	inputPointer = 0;
	sendBuffer = NULL;
	outputBuffer = NULL;
	outputPointer = 0;
	unsentPointer = 0;
	sentDataOutstanding = 0;
	fileBeingSent = NULL;
	persistConnection = false;
	closeRequested = false;
	nextWrite = NULL;
	lastWriteTime = NAN;
}

// Webserver calls this to read bytes that have come in from the network.
bool RequestState::Read(char& b)
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

// Read an entire pbuf from the RequestState
bool RequestState::ReadBuffer(char *&buffer, unsigned int &len)
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

void RequestState::Write(char b)
{
	if (LostConnection() || status == disconnected) return;

	if (sendBuffer == NULL)
	{
		Network *net = reprap.GetNetwork();
		if (net->AllocateSendBuffer(sendBuffer))
		{
			outputBuffer = sendBuffer->tcpOutputBuffer;
		}
		else
		{
			// We cannot write because there are no more send buffers available.
			return;
		}
	}

	if (outputPointer >= tcpOutputBufferSize)
	{
		reprap.GetPlatform()->Message(HOST_MESSAGE, "Network::Write(char b) - Output buffer overflow! \n");
		return;
	}

	// Add the byte to the buffer
	outputBuffer[outputPointer] = b;
	outputPointer++;
}

// This functions attempts to append a string to the send buffer. If the string is too big,
// this function will return false, so another RequestState must be used for sending.
bool RequestState::Write(const char* s)
{
	unsigned int len = strlen(s);
	if (outputPointer + len >= tcpOutputBufferSize)
	{
		return false;
	}

	if (sendBuffer == NULL)
	{
		Network *net = reprap.GetNetwork();
		if (net->AllocateSendBuffer(sendBuffer))
		{
			outputBuffer = sendBuffer->tcpOutputBuffer;
		}
		else
		{
			// We cannot write because there are no more send buffers available.
			return false;
		}
	}

	memcpy(outputBuffer + outputPointer, s, len);
	outputPointer += len;
	return true;
}

// Write formatted data to the output buffer
void RequestState::Printf(const char* fmt, ...)
{
	if (LostConnection() || status == disconnected) return;

	if (sendBuffer == NULL)
	{
		Network *net = reprap.GetNetwork();
		if (net->AllocateSendBuffer(sendBuffer))
		{
			outputBuffer = sendBuffer->tcpOutputBuffer;
		}
		else
		{
			// We cannot write because there are no more send buffers available.
			return;
		}
	}

	va_list p;
	va_start(p, fmt);
	int spaceLeft = tcpOutputBufferSize - outputPointer;
	if (spaceLeft > 0)
	{
		int len = vsnprintf(&outputBuffer[outputPointer], spaceLeft, fmt, p);
		if (len > 0)
		{
			outputPointer += min(len, spaceLeft);
		}
	}
	va_end(p);
}

// Send some data or free this RequestState if we can, returning true if we can free up this object
bool RequestState::Send()
{
	if (LostConnection())
	{
		if (fileBeingSent != NULL)
		{
			fileBeingSent->Close();
			fileBeingSent = NULL;
		}
		if (sendBuffer != NULL)
		{
			Network *net = reprap.GetNetwork();
			net->FreeSendBuffer(sendBuffer);
		}

		return true;
	}
	// We've finished with this RS, so close the connection
	else if (closeRequested)
	{
		// Close the file if it is still open
		if (fileBeingSent != NULL)
		{
			fileBeingSent->Close();
			fileBeingSent = NULL;
		}

		// Close the connection PCB
		ConnectionState *locCs = cs;		// take a copy because our cs field is about to get cleared
//		debugPrintf("RequestState is closing connection cs=%08x\n", (unsigned int)cs);
		reprap.GetNetwork()->ConnectionClosed(locCs);
		tcp_pcb *pcb = locCs->pcb;
		tcp_arg(pcb, NULL);
		tcp_sent(pcb, NULL);
		tcp_close(pcb);
		mem_free(locCs);

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

	if (cs->SendInProgress())
	{
		return false;
	}

	if (sentDataOutstanding != 0 && !isnan(lastWriteTime))
	{
		float timeNow = reprap.GetPlatform()->Time();
		if (timeNow - lastWriteTime > writeTimeout)
		{
//			debugPrintf("Network: Timing out connection cs=%08x\n", (unsigned int)cs);
			Close();
			return false;	// this RS will be freed next time round
		}
	}

	if (sentDataOutstanding >= tcpOutputBufferSize / 2)
	{
		return false;		// don't send until at least half the output buffer is free
	}

	if (fileBeingSent != NULL)
	{
		unsigned int outputLimit = (sentDataOutstanding == 0)
				? tcpOutputBufferSize
						: min<unsigned int>(unsentPointer + tcpOutputBufferSize/2, tcpOutputBufferSize);

		while (outputPointer < outputLimit)
		{
			bool ok = fileBeingSent->Read(outputBuffer[outputPointer]);
			if (!ok)
			{
				fileBeingSent->Close();
				fileBeingSent = NULL;
				break;
			}
			++outputPointer;
		}
	}

	// We've finished sending data
	if (outputPointer == unsentPointer)
	{
		// If we have no data to send and fileBeingSent is NULL, we can close the connection
		if (!persistConnection && nextWrite == NULL)
		{
			// Finish this RequestState next time Spin() is called
			Close();
			return false;
		}

		// Send is complete as soon as all data has been sent
		if (sentDataOutstanding == 0)
		{
			if (sendBuffer != NULL)
			{
				Network *net = reprap.GetNetwork();
				net->FreeSendBuffer(sendBuffer);
			}

			return true;
		}
	}
	// Send remaining data
	else
	{
		sentDataOutstanding += (outputPointer - unsentPointer);
		RepRapNetworkSendOutput(outputBuffer + unsentPointer, outputPointer - unsentPointer, cs);
		unsentPointer = (outputPointer == tcpOutputBufferSize) ? 0 : outputPointer;
		outputPointer = unsentPointer;
		lastWriteTime = reprap.GetPlatform()->Time();
	}
	return false;
}

void RequestState::SentPacketAcknowledged(unsigned int len)
{
	if (sentDataOutstanding >= len)
	{
		sentDataOutstanding -= len;
	}
	else
	{
		sentDataOutstanding = 0;
	}
}

void RequestState::SetConnectionLost()
{
	cs = NULL;
	FreePbuf();
	for (RequestState *rs = nextWrite; rs != NULL; rs = rs->nextWrite)
	{
		rs->cs = NULL;
	}
}

uint16_t RequestState::GetLocalPort() const
{
	return (cs != NULL) ? cs->pcb->local_port : 0;
}

void RequestState::Close()
{
	++inLwip;
	tcp_pcb *pcb = cs->pcb;
	tcp_poll(pcb, NULL, 4);
	tcp_recv(pcb, NULL);
	closeRequested = true;
	--inLwip;
}

void RequestState::FreePbuf()
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
