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

}

const int connectionStateSize = MEMP_NUM_TCP_PCB + 2;	// the +2 are reserved for network errors and for connection establishments
const float writeTimeout = 2.0;	 						// seconds to wait for data we have written to be acknowledged

static tcp_pcb *http_pcb = NULL;
static tcp_pcb *ftp_main_pcb = NULL;
static tcp_pcb *ftp_pasv_pcb = NULL;
static tcp_pcb *telnet_pcb = NULL;

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
		RepRapNetworkMessage("send_data: error\n");
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
		reprap.GetNetwork()->ConnectionClosing(cs);	// tell the higher levels about the error
		mem_free(cs);								// release the state data
	}
}

/*-----------------------------------------------------------------------------------*/

static err_t conn_poll(void *arg, tcp_pcb *pcb)
{
	ConnectionState *cs = (ConnectionState*)arg;
	if (cs != NULL)
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
	if (cs != NULL)
	{
		if (cs->pcb != pcb)
		{
			RepRapNetworkMessage("Network: mismatched pcb\n");
			return ERR_BUF;
		}

		if (err == ERR_OK && p != NULL)
		{
			// FIXME: this concept won't work for FTP upload connections, because we will eventually
			// run out of free transactions. We need another approach to call tcp_recved depending
			// on whether free transactions are available or not, but I (zpl) don't know how and
			// when conn_recv will be called if we don't call tcp_recved here...
			tcp_recved(pcb, p->tot_len);		// Inform TCP that we have taken the data

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
		else if (err == ERR_OK)
		{
			// This is called when the connection has been gracefully closed, but LWIP doesn't close
			// these connections automatically. That's why we must do it here.
			reprap.GetNetwork()->ConnectionClosing(cs);

			tcp_arg(pcb, NULL);
			tcp_sent(pcb, NULL);
			tcp_recv(pcb, NULL);
			tcp_poll(pcb, NULL, 4);
			mem_free(cs);
			tcp_close(pcb);
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
	cs->justConnected = true;
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

// Network/Ethernet class

Network::Network()
{
	active = false;
	inLwip = 0;

	freeTransactions = NULL;
	readyTransactions = NULL;
	writingTransactions = NULL;
	for (int8_t i = 0; i < connectionStateSize; i++)
	{
		freeTransactions = new RequestState(freeTransactions);
	}

	dataCs = NULL;
	mainCs = NULL;

	ethPinsInit();
}

void Network::AppendTransaction(RequestState* volatile* list, RequestState *r)
{
	irqflags_t flags = cpu_irq_save();
	r->next = NULL;
	while (*list != NULL)
	{
		list = &((*list)->next);
	}
	*list = r;
	cpu_irq_restore(flags);
}

void Network::PrependTransaction(RequestState* volatile* list, RequestState *r)
{
	irqflags_t flags = cpu_irq_save();
	r->next = *list;
	*list = r;
	cpu_irq_restore(flags);
}

void Network::Init()
{
	init_ethernet();
}

void Network::Spin()
{
	if (active)
	{
//		++inLwip;
		ethernet_task();			// keep the Ethernet running
//		--inLwip;

		// See if we can send anything
		RequestState* r = writingTransactions;
		if (r != NULL)
		{
			bool finished = r->Send();

			// Disable interrupts while we mess around with the lists in case we get a callback from a network ISR
			irqflags_t flags = cpu_irq_save();
			writingTransactions = r->next;

			if (finished)
			{
				r->nextWrite = NULL;
				ConnectionState *cs = r->cs;
				AppendTransaction(&freeTransactions, r);

				RequestState *rn = r->nextWrite;
				if (rn != NULL)
				{
					cs->sendingRs = (cs != NULL && rn->cs == cs) ? rn : NULL;
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
			cpu_irq_restore(flags);
		}
	}
	else if (establish_ethernet_link())
	{
		start_ethernet(reprap.GetPlatform()->IPAddress(), reprap.GetPlatform()->NetMask(), reprap.GetPlatform()->GateWay());
		httpd_init();
		ftpd_init();
		telnetd_init();
		active = true;
	}
}

void Network::SetInterpreters(void *http, void *ftp, void *telnet)
{
	httpInterpreter = http;
	ftpInterpreter = ftp;
	telnetInterpreter = telnet;
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
	RepRapNetworkMessage("Network SentPacketAcknowledged: didn't find cs\n");
}

// This is called when a connection has been accepted
// It must set the state of any RequestState that refers to it to connection accepted.
void Network::ConnectionAccepted(ConnectionState *cs)
{
	irqflags_t flags = cpu_irq_save();
	RequestState* r = freeTransactions;
	if (r == NULL)
	{
		cpu_irq_restore(flags);
		reprap.GetPlatform()->Message(HOST_MESSAGE, "Network::ConnectionAccepted() - no free transactions!\n");
		return;
	}

	freeTransactions = r->next;
	cpu_irq_restore(flags);

	r->Set(NULL, cs);
	AppendTransaction(&readyTransactions, r);
//	debugPrintf("Network - connection accepted\n");
}

// This is called when a connection is being closed or has gone down.
// It must set the state of any RequestState that refers to it to connection lost.
void Network::ConnectionClosing(ConnectionState* cs)
{
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

	if (cs == dataCs)
	{
		dataCs = NULL;
	}
	if (cs == mainCs)
	{
		mainCs = NULL;
	}

	if (cs->pcb != NULL)
	{
		reprap.GetWebserver()->ConnectionLost(cs->pcb->local_port);
	}
}

void Network::ReceiveInput(pbuf *pb, ConnectionState* cs)
{
	irqflags_t flags = cpu_irq_save();
	RequestState* r = freeTransactions;
	if (r == NULL)
	{
		cpu_irq_restore(flags);
		reprap.GetPlatform()->Message(HOST_MESSAGE, "Network::ReceiveInput() - no free transactions!\n");
		return;
	}

	freeTransactions = r->next;
	cpu_irq_restore(flags);

	r->Set(pb, cs);
	AppendTransaction(&readyTransactions, r);
//	debugPrintf("Network - input received\n");
}

// Send the output data we already have, optionally with a file appended, then close the connection unless keepConnectionOpen is true.
// The file may be too large for our buffer, so we may have to send it in multiple transactions.
void Network::SendAndClose(FileStore *f, bool keepConnectionOpen)
{
	irqflags_t flags = cpu_irq_save();
	RequestState *r = readyTransactions;
	if (r == NULL)
	{
		cpu_irq_restore(flags);
	}
	else
	{
		readyTransactions = r->next;
		cpu_irq_restore(flags);
		r->FreePbuf();
		if (r->LostConnection())
		{
			if (f != NULL)
			{
				f->Close();
			}
			AppendTransaction(&freeTransactions, r);
//			debugPrintf("Conn lost before send\n");
		}
		else
		{
			r->cs->justConnected = false;
			r->fileBeingSent = f;
			r->persistConnection = keepConnectionOpen;
			AppendTransaction(&writingTransactions, r);

			RequestState *sendingRs = r->cs->sendingRs;
			if (sendingRs == NULL)
			{
				r->cs->sendingRs = r;
//debug
// r->outputBuffer[r->outputPointer] = 0;
// debugPrintf("Transaction queued for writing to network, file=%c, data=%s\n", (f ? 'Y' : 'N'), r->outputBuffer);
			}
			else
			{
				while (sendingRs->nextWrite != NULL)
				{
					sendingRs = sendingRs->nextWrite;
				}
				sendingRs->nextWrite = r;
			}
		}
	}
}

void Network::IgnoreRequest()
{
	irqflags_t flags = cpu_irq_save();
	RequestState *r = readyTransactions;
	readyTransactions = r->next;
	cpu_irq_restore(flags);

	if (r->cs != NULL)
	{
		r->cs->justConnected = false;
	}
	AppendTransaction(&freeTransactions, r);
}

void Network::RepeatRequest()
{
	irqflags_t flags = cpu_irq_save();
	RequestState *r = readyTransactions;
	r->inputPointer = 0; // behave as if this request hasn't been processed yet
	if (r->next != NULL)
	{
		readyTransactions = r->next;
		cpu_irq_restore(flags);

		AppendTransaction(&readyTransactions, r);
	}
	else
	{
		cpu_irq_restore(flags);
	}
}

void Network::OpenDataPort(uint16_t port)
{
	tcp_pcb* pcb = tcp_new();
	tcp_bind(pcb, IP_ADDR_ANY, port);
	ftp_pasv_pcb = tcp_listen(pcb);
	tcp_accept(ftp_pasv_pcb, conn_accept);
}

void Network::SaveDataConnection()
{
	// store our data connection so we can identify it again later
	dataCs = readyTransactions->cs;
}

void Network::SaveMainConnection()
{
	mainCs = readyTransactions->cs;
}

bool Network::RestoreDataConnection()
{
	if (dataCs == NULL)
	{
		return false;
	}

	// Our data connection should be in freeTransactions, because we don't expect it
	// to read or write data when this method is called.
	irqflags_t flags = cpu_irq_save();
	RequestState *r = freeTransactions;
	RequestState *r_prev = NULL;
	while (r != NULL)
	{
		if (r->cs == dataCs)
		{
			if (r_prev != NULL)
			{
				r_prev->next = r->next;
			}
			cpu_irq_restore(flags);

			PrependTransaction(&readyTransactions, r);
			return true;
		}

		r_prev = r;
		r = r->next;
	}

	cpu_irq_restore(flags);
	return false;
}

bool Network::MakeMainRequest()
{
	if (mainCs == NULL)
	{
		return false;
	}

	// it's possible that the connection's RequestState is in use for sending, so we should use a free one
	irqflags_t flags = cpu_irq_save();
	RequestState *r = freeTransactions;
	if (r == NULL)
	{
		cpu_irq_restore(flags);
		reprap.GetPlatform()->Message(HOST_MESSAGE, "Network::MakeMainRequest() - no free transactions!\n");
		return false;
	}
	freeTransactions = r->next;

	r->Set(NULL, mainCs);
	cpu_irq_restore(flags);
	PrependTransaction(&readyTransactions, r);

	return true;
}

void Network::CloseDataPort()
{
	// close open connection if there is any
	if (dataCs != NULL && dataCs->pcb != NULL)
	{
		ConnectionState *loc_cs = dataCs;
		ConnectionClosing(loc_cs);
//		debugPrintf("Closing connection dataCS=%08x\n", (unsigned int)loc_cs);
		tcp_pcb *pcb = loc_cs->pcb;
		tcp_arg(pcb, NULL);
		tcp_sent(pcb, NULL);
		tcp_recv(pcb, NULL);
		tcp_poll(pcb, NULL, 4);
		mem_free(loc_cs);
		loc_cs = NULL;
		tcp_close(pcb);
	}

	// close listening data port
	if (ftp_pasv_pcb != NULL)
	{
		tcp_accept(ftp_pasv_pcb, NULL);
		tcp_close(ftp_pasv_pcb);
		ftp_pasv_pcb = NULL;
	}
}


//queries the PHY for link status, true = link is up, false, link is down or there is some other error
bool Network::LinkIsUp()
{
	return status_link_up();
}

bool Network::Active() const
{
	return active;
}


// NetRing class members
RequestState::RequestState(RequestState* n) : next(n)
{
}

void RequestState::Set(pbuf *p, ConnectionState *c)
{
	cs = c;
	pb = p;
	inputPointer = 0;
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
			pbuf* next = pb->next;
			pb->next = NULL;
			pbuf_free(pb);
			pb = next;
			if (next == NULL)
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

// Webserver calls this to write bytes that need to go out to the network

void RequestState::Write(char b)
{
	if (LostConnection()) return;

	if (outputPointer >= ARRAY_SIZE(outputBuffer))
	{
		reprap.GetPlatform()->Message(HOST_MESSAGE, "Network::Write(char b) - Output buffer overflow! \n");
		return;
	}

	// Add the byte to the buffer

	outputBuffer[outputPointer] = b;
	outputPointer++;
}

// This is not called for data, only for internally-
// generated short strings at the start of a transmission,
// so it should never overflow the buffer (which is checked
// anyway).

void RequestState::Write(const char* s)
{
	while (*s)
	{
		Write(*s++);
	}
}

// Write formatted data to the output buffer
void RequestState::Printf(const char* fmt, ...)
{
	va_list p;
	va_start(p, fmt);
	int spaceLeft = ARRAY_SIZE(outputBuffer) - outputPointer;
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

// Send some data or close this connection if we can, returning true if we can free up this object
bool RequestState::Send()
{
	if (LostConnection())
	{
		if (fileBeingSent != NULL)
		{
			fileBeingSent->Close();
			fileBeingSent = NULL;
		}
		return true;
	}

	if (cs->SendInProgress())
	{
		return false;
	}

	if (sentDataOutstanding != 0 && lastWriteTime != NAN)
	{
		float timeNow = reprap.GetPlatform()->Time();
		if (timeNow - lastWriteTime > writeTimeout)
		{
			debugPrintf("Network: Timing out connection cs=%08x\n", (unsigned int)cs);

			ConnectionState *locCs = cs;		// take a copy because our cs field is about to get cleared
			reprap.GetNetwork()->ConnectionClosing(locCs);
			tcp_pcb *pcb = locCs->pcb;
			tcp_arg(pcb, NULL);
			tcp_sent(pcb, NULL);
			tcp_recv(pcb, NULL);
			tcp_poll(pcb, NULL, 4);
			tcp_abort(pcb);
			mem_free(locCs);
			return false;				// this RS will be freed next time round
		}
	}

	if (sentDataOutstanding >= ARRAY_SIZE(outputBuffer)/2)
	{
		return false;	// don't send until at least half the output buffer is free
	}

	if (fileBeingSent != NULL)
	{
		unsigned int outputLimit = (sentDataOutstanding == 0)
				? ARRAY_SIZE(outputBuffer)
						: min<unsigned int>(unsentPointer + ARRAY_SIZE(outputBuffer)/2, ARRAY_SIZE(outputBuffer));

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

	if (outputPointer == unsentPointer)
	{
		// We have no data to send. fileBeingSent must already be NULL here.
		if (!closeRequested && nextWrite == NULL)
		{
			if (persistConnection)
			{
				// this RS may be reused, so make sure the connection does not time out
				lastWriteTime = NAN;
			}
			else
			{
				tcp_close(cs->pcb);
				closeRequested = true;
				// Don't release this RS yet, the write buffer may still be needed to do send retries
				return false;
			}
		}

		if (sentDataOutstanding != 0)
		{
			return false;
		}

		// We've finished with this RS
		if (closeRequested)
		{
			// debugPrintf("Closing connection cs=%08x\n", (unsigned int)cs);

			ConnectionState *locCs = cs;		// take a copy because our cs field is about to get cleared
			reprap.GetNetwork()->ConnectionClosing(locCs);
			tcp_pcb *pcb = locCs->pcb;
			tcp_arg(pcb, NULL);
			tcp_sent(pcb, NULL);
			tcp_recv(pcb, NULL);
			tcp_poll(pcb, NULL, 4);
			mem_free(locCs);
		}
		return true;
	}
	else
	{
		sentDataOutstanding += (outputPointer - unsentPointer);
		RepRapNetworkSendOutput(outputBuffer + unsentPointer, outputPointer - unsentPointer, cs);
		unsentPointer = (outputPointer == ARRAY_SIZE(outputBuffer)) ? 0 : outputPointer;
		outputPointer = unsentPointer;
		lastWriteTime = reprap.GetPlatform()->Time();
		return false;
	}
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
	return cs->pcb->local_port;
}

void RequestState::FreePbuf()
{
	if (pb != NULL)
	{
		pbuf_free(pb);
		pb = NULL;
	}
}

// End
