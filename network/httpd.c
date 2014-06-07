/*
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
 */


/*
 * Heavily modified by Adrian
 *
 * RepRapPro Ltd
 * http://reprappro.com
 *
 * 2 October 2013
 *
 */

//#include "lwipopts.h"
//#if defined(HTTP_RAW_USED)
//
//#include <string.h>
//#include "lwip/debug.h"
//#include "lwip/stats.h"
//#include "httpd.h"
//#include "lwip/tcp.h"
//#include "fs.h"

#include "lwipopts.h"
#if defined(HTTP_RAW_USED)

#include <string.h>
#include "lwip/src/include/lwip/debug.h"
#include "lwip/src/include/lwip/stats.h"
#include "httpd.h"
#include "lwip/src/include/lwip/tcp.h"
#include "fs.h"

struct http_state {
  char *file;
  u16_t left;
  u8_t retries;
};

// Prototypes for the RepRap functions in Platform.cpp that we
// need to call.

void RepRapNetworkReceiveInput(char* ip, int length, void* pbuf, void* pcb, void* hs);
void RepRapNetworkInputBufferReleased(void* pbuf);
void RepRapNetworkConnectionError(void* h);
void RepRapNetworkMessage(char* s);
void RepRapNetworkSentPacketAcknowledged();
bool RepRapNetworkHasALiveClient();

// Sanity check on initialisations.

static int initCount = 0;

/*-----------------------------------------------------------------------------------*/

static void
conn_err(void *arg, err_t err)
{
  // Report the error to the monitor
  RepRapNetworkMessage("Network connection error, code ");
  {
	char tempBuf[10];
	snprintf(tempBuf, sizeof(tempBuf)/sizeof(char), "%d\n", err);
	RepRapNetworkMessage(tempBuf);
  }

  struct http_state *hs = arg;
  RepRapNetworkConnectionError(hs);		// tell the higher levels about the error
  mem_free(hs);							// release the state data
}

/*-----------------------------------------------------------------------------------*/

static void
close_conn(struct tcp_pcb *pcb, struct http_state *hs)
{
  //RepRapNetworkMessage("close_conn called.\n");
  tcp_arg(pcb, NULL);
  tcp_sent(pcb, NULL);
  tcp_recv(pcb, NULL);
  mem_free(hs);
  tcp_close(pcb);
}

//char scratch[40];

/*-----------------------------------------------------------------------------------*/

static void
send_data(struct tcp_pcb *pcb, struct http_state *hs)
{
  err_t err;
  u16_t len;

  /* We cannot send more data than space available in the send
     buffer. */
  if (tcp_sndbuf(pcb) < hs->left) {
    len = tcp_sndbuf(pcb);
  } else {
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
	  //if(hs->left <= 0)
	  //		RepRapNetworkAllowWriting();
  } else
  {
	  RepRapNetworkMessage("send_data: error\n");
	  //%s len %d %d\n", lwip_strerr(err), len, tcp_sndbuf(pcb));
  }
}
/*-----------------------------------------------------------------------------------*/

static err_t
http_poll(void *arg, struct tcp_pcb *pcb)
{
  struct http_state *hs;

  hs = arg;

  /*  printf("Polll\n");*/
  if (hs == NULL) {
    /*    printf("Null, close\n");*/
    tcp_abort(pcb);
    return ERR_ABRT;
  } else {
    ++hs->retries;
    if (hs->retries == 4) {
      tcp_abort(pcb);
      return ERR_ABRT;
    }
    send_data(pcb, hs);
  }

  return ERR_OK;
}

/*-----------------------------------------------------------------------------------*/

static err_t
http_sent(void *arg, struct tcp_pcb *pcb, u16_t len)
{
  struct http_state *hs;

  LWIP_UNUSED_ARG(len);

  hs = arg;

  hs->retries = 0;

  //RepRapNetworkMessage("..sent\n");

  if (hs->left > 0)
  {
    send_data(pcb, hs);
  } else
  {
	  // See if there is more to send
	  RepRapNetworkSentPacketAcknowledged();
  }

  return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/


// ReoRap calls this with data to send.
// A null transmission implies the end of the data to be sent.

void RepRapNetworkSendOutput(char* data, int length, void* pb, void* pc, void* h)
{
	struct pbuf* p = pb;
	struct tcp_pcb* pcb = pc;
	struct http_state* hs = h;

	if(length <= 0)
	{
		//tcp_output(pcb);
		//pbuf_free(p);
		close_conn(pcb, hs);
		return;
	}

	if(hs == 0)
	{
		RepRapNetworkMessage("Attempt to write with null structure.\n");
		return;
	}

	hs->file = data;
	hs->left = length;
	hs->retries = 0;

	if(pb != 0)
	{
		RepRapNetworkInputBufferReleased(p);
		pbuf_free(p);
	}
	send_data(pcb, hs);

	//if(hs->left <= 0)
		//RepRapNetworkAllowWriting();

	/* Tell TCP that we wish be to informed of data that has been
	           successfully sent by a call to the http_sent() function. */

	tcp_sent(pcb, http_sent);
}


static err_t
http_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
	int i;
	struct http_state *hs;

	hs = arg;

	if (err == ERR_OK && p != NULL)
	{
		/* Inform TCP that we have taken the data. */
		tcp_recved(pcb, p->tot_len);

		if (hs->file == NULL)
		{
			RepRapNetworkReceiveInput(p->payload, p->len, p, pcb, hs);
		} else
		{
			pbuf_free(p);
		}
	}

	if (err == ERR_OK && p == NULL) {
		close_conn(pcb, hs);
	}
	return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/

static err_t
http_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{
  struct http_state *hs;

  // This is a bit nasty.  Fake an out of memory error to prevent new page
  // requests coming in while we are still sending the old ones.

  if(RepRapNetworkHasALiveClient())
	  return ERR_MEM;

  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(err);

  tcp_setprio(pcb, TCP_PRIO_MIN);

  //RepRapNetworkMessage("http_accept\n");

  hs = (struct http_state *)mem_malloc(sizeof(struct http_state));

  if (hs == NULL) {
	  RepRapNetworkMessage("Out of memory!\n");
    return ERR_MEM;
  }

  /* Initialize the structure. */
  hs->file = NULL;
  hs->left = 0;
  hs->retries = 0;

  /* Tell TCP that this is the structure we wish to be passed for our
     callbacks. */
  tcp_arg(pcb, hs);

  /* Tell TCP that we wish to be informed of incoming data by a call
     to the http_recv() function. */
  tcp_recv(pcb, http_recv);

  tcp_err(pcb, conn_err);

  tcp_poll(pcb, http_poll, 4);
  return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/

// This function (is)x should be called only once at the start.

void
httpd_init(void)
{
  struct tcp_pcb* pcb;

  initCount++;
  if(initCount > 1)
	  RepRapNetworkMessage("httpd_init() called more than once.\n");

  pcb = tcp_new();
  tcp_bind(pcb, IP_ADDR_ANY, 80);
  pcb = tcp_listen(pcb);
  tcp_accept(pcb, http_accept);
}
/*-----------------------------------------------------------------------------------*/

#endif


