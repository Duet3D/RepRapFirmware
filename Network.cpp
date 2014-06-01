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

const int httpStateSize = MEMP_NUM_TCP_PCB + 1;		// the +1 is in case of recovering from network errors
const float writeTimeout = 2.0;	 					// seconds to wait for data we have written to be acknowledged

static tcp_pcb *listening_pcb = NULL;

// Called to put out a message via the RepRap firmware.
// Can be called from C as well as C++

extern "C" void RepRapNetworkMessage(const char* s)
{
	reprap.GetPlatform()->Message(HOST_MESSAGE, s);
}


static void SendData(tcp_pcb *pcb, HttpState *hs)
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
    err = tcp_write(pcb, hs->file, len, 0 /*TCP_WRITE_FLAG_COPY*/ ); // Final arg - 1 means make a copy
    if (err == ERR_MEM) {
      len /= 2;
    }
  } while (err == ERR_MEM && len > 1);

  if (err == ERR_OK)
  {
	  tcp_output(pcb);
	  hs->file += len;
	  hs->left -= len;
	  if (hs->left == 0)
	  {
		  hs->file = NULL;
	  }
  } else
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

	HttpState *hs = (HttpState*)arg;
	if (hs != NULL)
	{
		reprap.GetNetwork()->ConnectionClosing(hs);	// tell the higher levels about the error
		mem_free(hs);								// release the state data
	}
}

/*-----------------------------------------------------------------------------------*/

static err_t http_poll(void *arg, tcp_pcb *pcb)
{
  HttpState *hs = (HttpState*)arg;
  if (hs != NULL)
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

static err_t http_sent(void *arg, tcp_pcb *pcb, u16_t len)
{
  HttpState *hs = (HttpState*)arg;
  if (hs != NULL)
  {
	  hs->retries = 0;

	  //RepRapNetworkMessage("..sent\n");

	  if (hs->left > 0)
	  {
		  SendData(pcb, hs);
	  }
	  else
	  {
		  // See if there is more to send
		  reprap.GetNetwork()->SentPacketAcknowledged(hs, len);
	  }
  }
  return ERR_OK;
}

/*-----------------------------------------------------------------------------------*/

static err_t http_recv(void *arg, tcp_pcb *pcb, pbuf *p, err_t err)
{
	HttpState *hs = (HttpState*)arg;
	if (hs != NULL)
	{
		if (hs->pcb != pcb)
		{
			RepRapNetworkMessage("Network: mismatched pcb\n");
			return ERR_BUF;
		}

		if (err == ERR_OK && p != NULL)
		{
			tcp_recved(pcb, p->tot_len);		// Inform TCP that we have taken the data
			reprap.GetNetwork()->ReceiveInput(p, hs);
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
	}
	return ERR_OK;
}

/*-----------------------------------------------------------------------------------*/

static err_t http_accept(void *arg, tcp_pcb *pcb, err_t err)
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
  hs->pcb = pcb;
  hs->file = NULL;
  hs->left = 0;
  hs->retries = 0;
  hs->sendingRs = NULL;

  tcp_accepted(listening_pcb);	// tell TCP to accept further connections
  tcp_arg(pcb, hs);				// tell TCP that this is the structure we wish to be passed for our callbacks
  tcp_recv(pcb, http_recv);		// tell TCP that we wish to be informed of incoming data by a call to the http_recv() function
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

  tcp_pcb* pcb = tcp_new();
  tcp_bind(pcb, IP_ADDR_ANY, 80);
  listening_pcb = tcp_listen(pcb);
  tcp_accept(listening_pcb, http_accept);
}

/*-----------------------------------------------------------------------------------*/

// RepRap calls this with data to send.

void RepRapNetworkSendOutput(const char* data, int length, HttpState* hs)
{
	if (hs == 0)
	{
		RepRapNetworkMessage("Network: Attempt to write with null structure.\n");
		return;
	}

	hs->file = data;
	hs->left = length;
	hs->retries = 0;

	tcp_sent(hs->pcb, http_sent);	// tell lwip that we wish be to informed of data that has been successfully sent by a call to the http_sent() function
	SendData(hs->pcb, hs);
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
	for (int8_t i = 0; i < httpStateSize; i++)
	{
		freeTransactions = new RequestState(freeTransactions);
	}

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
				RequestState *rn = r->nextWrite;
				r->nextWrite = NULL;
				HttpState *hs = r->hs;
				AppendTransaction(&freeTransactions, r);
				if (rn != NULL)
				{
					if (hs != NULL)
					{
						hs->sendingRs = (rn->hs == hs) ? rn : NULL;
					}
					AppendTransaction(&writingTransactions, rn);
				}
				else if (hs != NULL)
				{
					hs->sendingRs = NULL;
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
		active = true;
	}
}

void Network::SentPacketAcknowledged(HttpState *hs, unsigned int len)
{
	RequestState *r = hs->sendingRs;
	if (r != NULL)
	{
		r->SentPacketAcknowledged(len);
		return;
	}

//	debugPrintf("Network SentPacketAcknowledged: didn't find hs=%08x\n", (unsigned int)hs);
	RepRapNetworkMessage("Network SentPacketAcknowledged: didn't find hs\n");
}

// This is called when a connection is being closed or has gone down.
// It must set the state of any RequestState that refers to it to connection lost.
void Network::ConnectionClosing(HttpState* hs)
{
	// h points to an http state block that the caller is about to release, so we need to stop referring to it.
	// There may be one RequestState in the writing or closing list referring to it, and possibly more than one in the ready list.
	//RepRapNetworkMessage("Network: ConnectionError\n");

	// See if it's a ready transaction
	for (RequestState* r = readyTransactions; r != NULL; r = r->next)
	{
		if (r->hs == hs)
		{
			r->SetConnectionLost();
		}
	}

	if (hs->sendingRs != NULL)
	{
		hs->sendingRs->SetConnectionLost();
		hs->sendingRs = NULL;
	}

	reprap.GetWebserver()->ResetState(hs);	// if the webserver is waiting on this connection, it needs to give up
}

void Network::ReceiveInput(pbuf *pb, HttpState* hs)
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

	r->Set(pb, hs);
	AppendTransaction(&readyTransactions, r);
//	debugPrintf("Network - input received\n");
}

// This is called by the web server to get a new received packet.
// If the connection parameter is NULL, we just return the request at the head of the ready list.
// Otherwise, we are only interested in packets received from the specified connection. If we find one than
// we move it to the head of the ready list, so that a subsequent call with a null connection parameter
// will return the same one.
RequestState *Network::GetRequest(const HttpState *connection)
{
	RequestState *rs = readyTransactions;
	if (rs == NULL || connection == NULL || rs->hs == connection)
	{
		return rs;
	}

	// There is at least one ready transaction, but it's not on the connection we are looking for
	for (;;)
	{
		RequestState *rsNext = rs->next;
		if (rsNext == NULL)
		{
			return rsNext;
		}
		if (rsNext->hs == connection)
		{
			irqflags_t flags = cpu_irq_save();
			rs->next = rsNext->next;		// remove rsNext from the list
			rsNext->next = readyTransactions;
			readyTransactions = rsNext;
			cpu_irq_restore(flags);
			return rsNext;
		}
		rs = rsNext;
	}
	return NULL;		// to keep Eclipse happy
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
			r->nextWrite = NULL;
			r->fileBeingSent = f;
			r->persistConnection = keepConnectionOpen;
			RequestState *sendingRs = r->hs->sendingRs;
			if (sendingRs == NULL)
			{
				r->hs->sendingRs = r;
				AppendTransaction(&writingTransactions, r);
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

void RequestState::Set(pbuf *p, HttpState* h)
{
	hs = h;
	pb = p;
	inputPointer = 0;
	outputPointer = 0;
	unsentPointer = 0;
	sentDataOutstanding = 0;
	fileBeingSent = NULL;
	persistConnection = false;
	closeRequested = false;
	nextWrite = NULL;
}

// Webserver calls this to read bytes that have come in from the network.

bool RequestState::Read(char& b)
{
	if (LostConnection() || pb == NULL)
	{
		return false;
	}

	while (inputPointer == pb->len)
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

	if (hs->SendInProgress())
	{
		return false;
	}

	if (sentDataOutstanding != 0)
	{
		float timeNow = reprap.GetPlatform()->Time();
		if (timeNow - lastWriteTime > writeTimeout)
		{
			debugPrintf("Timing out connection hs=%08x\n", (unsigned int)hs);

			HttpState *locHs = hs;		// take a copy because our hs field is about to get cleared
			reprap.GetNetwork()->ConnectionClosing(locHs);
			tcp_pcb *pcb = locHs->pcb;
			tcp_arg(pcb, NULL);
			tcp_sent(pcb, NULL);
			tcp_recv(pcb, NULL);
			tcp_poll(pcb, NULL, 4);
			tcp_abort(pcb);
			mem_free(locHs);
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
		if (!persistConnection && !closeRequested && nextWrite == NULL)
		{
			tcp_close(hs->pcb);
			closeRequested = true;
			// Don't release this RS yet, the write buffer may still be needed to do send retries
			return false;
		}

		if (sentDataOutstanding != 0)
		{
			return false;
		}

		// We've finished with this RS
		if (closeRequested)
		{
			// debugPrintf("Closing connection hs=%08x\n", (unsigned int)hs);

			HttpState *locHs = hs;		// take a copy because our hs field is about to get cleared
			reprap.GetNetwork()->ConnectionClosing(locHs);
			tcp_pcb *pcb = locHs->pcb;
			tcp_arg(pcb, NULL);
			tcp_sent(pcb, NULL);
			tcp_recv(pcb, NULL);
			tcp_poll(pcb, NULL, 4);
			mem_free(locHs);
		}
		return true;
	}
	else
	{
		sentDataOutstanding += (outputPointer - unsentPointer);
		RepRapNetworkSendOutput(outputBuffer + unsentPointer, outputPointer - unsentPointer, hs);
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
	hs = NULL;
	FreePbuf();
	for (RequestState *rs = nextWrite; rs != NULL; rs = rs->nextWrite)
	{
		rs->hs = NULL;
	}
}

bool RequestState::LostConnection() const
{
	return hs == NULL;
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
