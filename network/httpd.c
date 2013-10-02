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

void RepRapNetworkReceiveInput(char* ip, int length);
void RepRapNetworkMessage(char* s);
void RepRapNetworkAllowWriting();
bool NoMoreData();

// Static storage for pointers that need to be saved when we go
// out to the RepRap firmware for when it calls back in again.
// Note that this means that the code is not reentrant, but in
// our context it doesn't need to be.

static struct tcp_pcb* activePcb;
static struct tcp_pcb* pcbToClose = 0;
static struct http_state* activeHttpState;
static struct pbuf* pbufToFree = 0;
static struct tcp_pcb* sendingPcb = 0;
static int initCount = 0;

/*-----------------------------------------------------------------------------------*/
static void
conn_err(void *arg, err_t err)
{
  //struct http_state *hs;

  LWIP_UNUSED_ARG(err);

  //hs = arg;
  //mem_free(hs);
}
/*-----------------------------------------------------------------------------------*/

// Added to allow RepRap to close the connection.

void CloseConnection()
{
	if(pcbToClose == 0)
		return;
	tcp_arg(pcbToClose, NULL);
	tcp_sent(pcbToClose, NULL);
	tcp_recv(pcbToClose, NULL);
	//mem_free(hs);
	tcp_close(pcbToClose);
	pcbToClose = 0;
	RepRapNetworkMessage("CloseConnection() called.\n");
}

// httpd.c's close function, slightly mashed...

static void
close_conn(struct tcp_pcb *pcb, struct http_state *hs)
{
  RepRapNetworkMessage("Internal close_conn called.\n");
  tcp_arg(pcb, NULL);
  tcp_sent(pcb, NULL);
  tcp_recv(pcb, NULL);
  //mem_free(hs);
  tcp_close(pcb);
}

char scratch[40];

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

  RepRapNetworkMessage("Sending ");
  sprintf(scratch, "%d", len);
  RepRapNetworkMessage(scratch);
  RepRapNetworkMessage("..");

  do {
    err = tcp_write(pcb, hs->file, len, 0);
    if (err == ERR_MEM) {
      len /= 2;
    }
  } while (err == ERR_MEM && len > 1);

  tcp_output(pcb);

  if (err == ERR_OK) {
    hs->file += len;
    hs->left -= len;
    /*  } else {
    printf("send_data: error %s len %d %d\n", lwip_strerr(err), len, tcp_sndbuf(pcb));*/
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

  RepRapNetworkMessage("..sent\n");

  if (hs->left > 0)
  {
    send_data(pcb, hs);
  } else
  {
	  // See if there is more to send
	  // TODO - possible memory leak?
	  RepRapNetworkAllowWriting();
  }

  pcbToClose = pcb;

  return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/

// ReoRap calls this with data to send.
// It has the side effect of freeing the input buffer
// that prompted the transmission, as that must now have been fully read.
// If RepRap ignores input, is this another potential memory leak?

void SetNetworkDataToSend(char* data, int length)
{
	//RepRapNetworkMessage("Some data arrived.\n");
	activeHttpState->file = data;
	activeHttpState->left = length;
	/* printf("data %p len %ld\n", hs->file, hs->left);*/

	if(pbufToFree != 0)
	{
		pbuf_free(pbufToFree);
		pbufToFree = 0;
	}

	send_data(sendingPcb, activeHttpState);

	/* Tell TCP that we wish be to informed of data that has been
	           successfully sent by a call to the http_sent() function. */


	tcp_sent(sendingPcb, http_sent);
}

static err_t
http_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
	int i;
	char *data;

	if (err == ERR_OK && p != NULL)
	{
		/* Inform TCP that we have taken the data. */
		tcp_recved(pcb, p->tot_len);

		if (activeHttpState->file == NULL)
		{
			data = p->payload;
			RepRapNetworkReceiveInput(data, p->len);
			pbufToFree = p;
			sendingPcb = pcb;
		} else
		{
			pbuf_free(p);
		}
	}

	if (err == ERR_OK && p == NULL) {
		close_conn(pcb, activeHttpState);
	}
	return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/
static err_t
http_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{
  struct http_state *hs;

  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(err);

  tcp_setprio(pcb, TCP_PRIO_MIN);

  // Ignore arg; we know it must be our static activeHttpState

  hs = activeHttpState;

  if (hs == NULL) {
    //printf("http_accept: Out of memory\n");
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
  initCount++;
  if(initCount > 1)
	  RepRapNetworkMessage("httpd_init() called more than once.\n");

  activeHttpState = (struct http_state *)mem_malloc(sizeof(struct http_state));
  activePcb = tcp_new();
  tcp_bind(activePcb, IP_ADDR_ANY, 80);
  activePcb = tcp_listen(activePcb);
  tcp_accept(activePcb, http_accept);
}
/*-----------------------------------------------------------------------------------*/

#endif


