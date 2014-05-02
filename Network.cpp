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

const uint8_t windowedSendPackets = 2;
const int httpStateSize = MEMP_NUM_TCP_PCB + 1;		// the +1 is in case of recovering from network errors

// Called to put out a message via the RepRap firmware.
// Can be called from C as well as C++

extern "C" void RepRapNetworkMessage(const char* s)
{
	reprap.GetPlatform()->Message(HOST_MESSAGE, s);
}


static void SendData(struct tcp_pcb *pcb, HttpState *hs)
{
  err_t err;
  u16_t len;

  /* We cannot send more data than space available in the send buffer. */
  if (tcp_sndbuf(pcb) < hs->left)
  {
    len = tcp_sndbuf(pcb);
  }
  else
  {
    len = hs->left;
  }

//  RepRapNetworkMessage("Sending ");
//  sprintf(scratch, "%d", len);
//  RepRapNetworkMessage(scratch);
//  RepRapNetworkMessage("..");

  do {
    err = tcp_write(pcb, hs->file, len, 0); // Final arg - 1 means make a copy
    if (err == ERR_MEM) {
      len /= 2;
    }
  } while (err == ERR_MEM && len > 1);

  if (err == ERR_OK)
  {
	  tcp_output(pcb);
	  hs->file += len;
	  hs->left -= len;
  } else
  {
	  RepRapNetworkMessage("send_data: error\n");
	  //%s len %d %d\n", lwip_strerr(err), len, tcp_sndbuf(pcb));
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
	snprintf(tempBuf, sizeof(tempBuf)/sizeof(char), "%d\n", err);
	RepRapNetworkMessage(tempBuf);
  }

  HttpState *hs = (HttpState*)arg;
  reprap.GetNetwork()->ConnectionError(hs);	// tell the higher levels about the error
  mem_free(hs);							// release the state data
}

/*-----------------------------------------------------------------------------------*/

static err_t http_poll(void *arg, struct tcp_pcb *pcb)
{
  HttpState *hs = (HttpState*)arg;

  if (hs == NULL)
  {
	RepRapNetworkMessage("Null, abort\n");
    tcp_abort(pcb);
    return ERR_ABRT;
  }
  else
  {
    ++hs->retries;
    if (hs->retries == 4)
    {
      tcp_abort(pcb);
      return ERR_ABRT;
    }
    SendData(pcb, hs);
  }

  return ERR_OK;
}

/*-----------------------------------------------------------------------------------*/

static err_t http_sent(void *arg, struct tcp_pcb *pcb, u16_t len)
{
  HttpState *hs = (HttpState*)arg;

  LWIP_UNUSED_ARG(len);

  hs->retries = 0;

  //RepRapNetworkMessage("..sent\n");

  if (hs->left > 0)
  {
	  SendData(pcb, hs);
  }
  else
  {
	  // See if there is more to send
	  reprap.GetNetwork()->SentPacketAcknowledged(hs);
  }

  return ERR_OK;
}

/*-----------------------------------------------------------------------------------*/

static err_t http_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
	HttpState *hs = (HttpState*)arg;

	if (err == ERR_OK && p != NULL)
	{
		/* Inform TCP that we have taken the data. */
		tcp_recved(pcb, p->tot_len);

		if (hs->file == NULL)
		{
			hs->pb = p;
			reprap.GetNetwork()->ReceiveInput((const char*)(p->payload), p->len, pcb, hs);
		}
		else
		{
			// We are already sending data on this connection, so not expecting any messages on it
			pbuf_free(p);
		}
	}
	return ERR_OK;
}

/*-----------------------------------------------------------------------------------*/

static err_t http_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{
  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(err);

  tcp_setprio(pcb, TCP_PRIO_MIN);

  //RepRapNetworkMessage("http_accept\n");

  HttpState *hs = (HttpState*)mem_malloc(sizeof(HttpState));

  if (hs == NULL)
  {
	  RepRapNetworkMessage("Out of memory!\n");
	  return ERR_MEM;
  }

  /* Initialize the structure. */
  hs->pb = NULL;
  hs->file = NULL;
  hs->left = 0;
  hs->retries = 0;

  /* Tell TCP that this is the structure we wish to be passed for our callbacks. */
  tcp_arg(pcb, hs);

  /* Tell TCP that we wish to be informed of incoming data by a call to the http_recv() function. */
  tcp_recv(pcb, http_recv);

  tcp_err(pcb, conn_err);

  tcp_poll(pcb, http_poll, 4);
  return ERR_OK;
}

}	// end extern "C"

/*-----------------------------------------------------------------------------------*/

// This function (is) should be called only once at the start.

void httpd_init()
{
  static int initCount = 0;

  initCount++;
  if (initCount > 1)
  {
	  RepRapNetworkMessage("httpd_init() called more than once.\n");
  }

  struct tcp_pcb* pcb = tcp_new();
  tcp_bind(pcb, IP_ADDR_ANY, 80);
  pcb = tcp_listen(pcb);
  tcp_accept(pcb, http_accept);
}

/*-----------------------------------------------------------------------------------*/

static void close_conn(struct tcp_pcb *pcb, HttpState *hs)
{
//  RepRapNetworkMessage("close_conn called.\n");
  tcp_arg(pcb, NULL);
  tcp_sent(pcb, NULL);
  tcp_recv(pcb, NULL);
  mem_free(hs);
  tcp_close(pcb);
}

/*-----------------------------------------------------------------------------------*/
// RepRap calls this with data to send.
// A null transmission implies the end of the data to be sent.

void RepRapNetworkSendOutput(char* data, int length, void* pc, void* h)
{
	struct tcp_pcb* pcb = (tcp_pcb*)pc;
	HttpState* hs = (HttpState*)h;

	if (hs == 0)
	{
		RepRapNetworkMessage("Attempt to write with null structure.\n");
		return;
	}

	if (hs->pb != NULL)
	{
		pbuf_free(hs->pb);
		hs->pb = NULL;
	}

	if (length <= 0)
	{
		close_conn(pcb, hs);
		return;
	}

	hs->file = data;
	hs->left = length;
	hs->retries = 0;

	SendData(pcb, hs);

	/* Tell TCP that we wish be to informed of data that has been successfully sent by a call to the http_sent() function. */

	tcp_sent(pcb, http_sent);
}


//***************************************************************************************************

// Network/Ethernet class

Network::Network()
{
	active = false;
	ethPinsInit();

	freeTransactions = NULL;
	readyTransactions = NULL;
	writingTransactions = NULL;
	closingTransactions = NULL;
	for (int8_t i = 0; i < httpStateSize; i++)
	{
		freeTransactions = new RequestState(freeTransactions);
	}
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

RequestState *Network::FindHs(RequestState* const volatile * list, HttpState *hs)
{
	irqflags_t flags = cpu_irq_save();
	RequestState *r = *list;
	while (r != NULL && r->hs != hs)
	{
		r = r->next;
	}
	cpu_irq_restore(flags);
	return r;
}

void Network::Init()
{
	init_ethernet();
}

void Network::Spin()
{
	if (active)
	{
		ethernet_task();			// keep the Ethernet running

		// See if we can send anything
		RequestState* r = writingTransactions;
		if (r != NULL)
		{
			bool doClose = r->TrySendData();
			irqflags_t flags = cpu_irq_save();
			writingTransactions = r->next;
			cpu_irq_restore(flags);
			if (doClose)
			{
				r->closeRequestedTime = reprap.GetPlatform()->Time();
				AppendTransaction(&closingTransactions, r);
			}
			else
			{
				AppendTransaction(&writingTransactions, r);
			}
		}

		// See if we can close any connections
		r = closingTransactions;
		if (r != NULL)
		{
			if (reprap.GetPlatform()->Time() - r->closeRequestedTime >= clientCloseDelay)
			{
				r->Close();
				irqflags_t flags = cpu_irq_save();
				closingTransactions = r->next;
				cpu_irq_restore(flags);
				AppendTransaction(&freeTransactions, r);
			}
		}
	}
	else if (establish_ethernet_link())
	{
		start_ethernet(reprap.GetPlatform()->IPAddress(), reprap.GetPlatform()->NetMask(), reprap.GetPlatform()->GateWay());
		httpd_init();
		active = true;
	}
}

bool Network::HaveData() const
{
	return active && readyTransactions != NULL;
}

bool Network::Read(char& b)
{
	if (readyTransactions != NULL)
	{
		return readyTransactions->Read(b);
	}
	return false;
}

void Network::Write(char b)
{
	if (readyTransactions != NULL)
	{
		readyTransactions->Write(b);
	}
}

void Network::Write(const char* s)
{
	if (readyTransactions != NULL)
	{
		readyTransactions->Write(s);
	}
}

// Write formatted data to the output buffer
void Network::Printf(const char* fmt, ...)
{
	if (readyTransactions != NULL)
	{
		va_list p;
		va_start(p, fmt);
		readyTransactions->Vprintf(fmt, p);
		va_end(p);
	}
}

void Network::SentPacketAcknowledged(HttpState *hs)
{
	RequestState *r = FindHs(&writingTransactions, hs);
	if (r != NULL)
	{
		r->SentPacketAcknowledged();
		return;
	}

	r = FindHs(&closingTransactions, hs);
	if (r != NULL)
	{
		r->SentPacketAcknowledged();
		return;
	}

//	debugPrintf("Network SentPacketAcknowledged: didn't find hs=%08x\n", (unsigned int)hs);
	RepRapNetworkMessage("Network SentPacketAcknowledged: didn't find hs\n");
}

void Network::ConnectionError(HttpState* hs)
{
	// h points to an http state block that the caller is about to release, so we need to stop referring to it.
	//RepRapNetworkMessage("Network: ConnectionError\n");

	// See if it's a ready transaction
	RequestState* r = FindHs(&readyTransactions, hs);
	if (r != NULL)
	{
		r->SetConnectionLost();
		return;
	}

	// See if we were sending it
	r = FindHs(&writingTransactions, hs);
	if (r != NULL)
	{
		r->SetConnectionLost();
		return;
	}

	// See if we were closing it
	r = FindHs(&closingTransactions, hs);
	if (r != NULL)
	{
		r->SetConnectionLost();
		return;
	}

	// Else we didn't identify the transaction - maybe we already asked to close it
	RepRapNetworkMessage("Network ConnectionError: didn't find hs\n");
//	debugPrintf("Network ConnectionError: didn't find hs=%08x\n", (unsigned int)hs);
}

void Network::ReceiveInput(const char* data, int length, void* pcb, HttpState* hs)
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

	r->Set(data, length, pcb, hs);
	AppendTransaction(&readyTransactions, r);
//	debugPrintf("Network - input received\n");
}

// Send the output data we already have, optionally with a file appended, then close the connection.
// The file may be too large for our buffer, so we may have to send it in multiple transactions.
void Network::SendAndClose(FileStore *f)
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
			r->fileBeingSent = f;
			AppendTransaction(&writingTransactions, r);
//debug
//			r->outputBuffer[r->outputPointer] = 0;
//			debugPrintf("Transaction queued for writing to network, file=%c, data=%s\n", (f ? 'Y' : 'N'), r->outputBuffer);
		}
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

void RequestState::Set(const char* d, int l, void* pc, HttpState* h)
{
	pcb = pc;
	hs = h;
	inputData = d;
	inputLength = l;
	inputPointer = 0;
	outputPointer = 0;
	sentPacketsOutstanding = 0;
	fileBeingSent = NULL;
}

// Webserver calls this to read bytes that have come in from the network

bool RequestState::Read(char& b)
{
	if (LostConnection() || inputPointer >= inputLength)
	{
		return false;
	}

	b = inputData[inputPointer];
	inputPointer++;
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
void RequestState::Vprintf(const char* fmt, va_list v)
{
	int spaceLeft = ARRAY_SIZE(outputBuffer) - outputPointer;
	if (spaceLeft > 0)
	{
		int len = vsnprintf(&outputBuffer[outputPointer], spaceLeft, fmt, v);
		if (len > 0)
		{
			outputPointer += min(len, spaceLeft);
		}
	}
}

// Send some data if we can, returning true if all data has been sent
bool RequestState::TrySendData()
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

	if (hs->SendInProgress())
	{
		return false;
	}

	if (sentPacketsOutstanding >= windowedSendPackets)
	{
		return false;		// still waiting for earlier packets to be acknowledged
	}

	if (fileBeingSent != NULL)
	{
		while (outputPointer < ARRAY_SIZE(outputBuffer))
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

	if (outputPointer == 0)
	{
		// fileBeingSent must already be NULL here
		return true;
	}
	else
	{
		++sentPacketsOutstanding;
		RepRapNetworkSendOutput(outputBuffer, outputPointer, pcb, hs);
		outputPointer = 0;
		return false;
	}
}

void RequestState::SentPacketAcknowledged()
{
	if (sentPacketsOutstanding != 0)
	{
		--sentPacketsOutstanding;
	}
}

// Close this connection. Return true if it really is closed, false if it needs to go in the deferred close list.
void RequestState::Close()
{
	if (!LostConnection())
	{
//		debugPrintf("Closing connection hs=%08x\n", (unsigned int)hs);
		RepRapNetworkSendOutput((char*) NULL, 0, pcb, hs);
	}
}

void RequestState::SetConnectionLost()
{
	hs = NULL;
}

bool RequestState::LostConnection() const
{
	return hs == NULL;
}


// End
