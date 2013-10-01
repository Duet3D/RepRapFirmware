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
#if 0
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
  uint16_t left;
  uint8_t retries;
};

/*-----------------------------------------------------------------------------------*/

//static void
//send_data(struct tcp_pcb *pcb, struct http_state *hs)
//{
//  err_t err;
//  uint16_t len;
//  int i;
//
////  if(hs->left <= 0)
////  {
////	  hs->left = GetRepRapNetworkDataToSendLength();
////	  hs->file = GetRepRapNetworkDataToSend();
////  }
//
//  /* We cannot send more data than space available in the send
//     buffer. */
//  if (tcp_sndbuf(pcb) < hs->left) {
//    len = tcp_sndbuf(pcb);
//  } else {
//    len = hs->left;
//  }
//
//  do {
//    err = tcp_write(pcb, hs->file, len, 0);
//    if (err == ERR_MEM) {
//      len /= 2;
//    }
//  } while (err == ERR_MEM && len > 1);
//
//  if (err == ERR_OK) {
//    hs->file += len;
//    hs->left -= len;
//    /*  } else {
//    printf("send_data: error %s len %d %d\n", lwip_strerr(err), len, tcp_sndbuf(pcb));*/
//  }
//}

/*-----------------------------------------------------------------------------------*/
static void
conn_err(void *arg, err_t err)
{
  struct http_state *hs;

  LWIP_UNUSED_ARG(err);

  hs = arg;
  mem_free(hs);
}
/*-----------------------------------------------------------------------------------*/
static void
close_conn(struct tcp_pcb *pcb, struct http_state *hs)
{
  tcp_arg(pcb, NULL);
  tcp_sent(pcb, NULL);
  tcp_recv(pcb, NULL);
  mem_free(hs);
  tcp_close(pcb);
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
//static err_t
//http_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
//{
//	int i;
//	char *data;
//	struct fs_file file;
//	struct http_state *hs;
//
//	hs = arg;
//
//	if (err == ERR_OK && p != NULL)
//	{
//
//		/* Inform TCP that we have taken the data. */
//		tcp_recved(pcb, p->tot_len);
//
//		if (hs->file == NULL)
//		{
//			data = p->payload;
//
//			if (strncmp(data, "GET ", 4) == 0)
//			{
//				for(i = 0; i < 40; i++) {
//					if (((char *)data + 4)[i] == ' ' ||
//							((char *)data + 4)[i] == '\r' ||
//							((char *)data + 4)[i] == '\n')
//					{
//						((char *)data + 4)[i] = 0;
//					}
//				}
//
//				if (*(char *)(data + 4) == '/' && *(char *)(data + 5) == 0)
//				{
//					fs_open("/index.html", &file);
//				} else if (!fs_open((char *)data + 4, &file))
//				{
//					fs_open("/404.html", &file);
//				}
//
//				hs->file = file.data;
//				hs->left = file.len;
//				/* printf("data %p len %ld\n", hs->file, hs->left);*/
//
//				pbuf_free(p);
//				send_data(pcb, hs);
//
//				/* Tell TCP that we wish be to informed of data that has been
//           successfully sent by a call to the http_sent() function. */
//				tcp_sent(pcb, http_sent);
//			} else
//			{
//				pbuf_free(p);
//				close_conn(pcb, hs);
//			}
//		} else
//		{
//			pbuf_free(p);
//		}
//	}
//
//	if (err == ERR_OK && p == NULL)
//	{
//		close_conn(pcb, hs);
//	}
//	return ERR_OK;
//}

static struct tcp_pcb* activePcb = 0;
static struct pbuf* activePbuf = 0;
static struct http_state* activeHttpState = 0;
static char* dataPayload = 0;
//static char outputBuffer[1200];
static int lastDataSentLength = 0;

int GetRepRapNetworkDataToSendLength();
char* GetRepRapNetworkDataToSend();
void RepRapNetworkMessage(char* s);
void RepRapNetworkReceiveInput(char* ip, int length);
void RepRapNetworkDataTransmitted(int length);

int SendBufferSize()
{
	if(activePcb == 0)
		return -1;
	return tcp_sndbuf(activePcb);
}

static err_t DataHasGone(void *arg, struct tcp_pcb *pcb, uint16_t len)
{
	RepRapNetworkDataTransmitted(len);
//  struct http_state *hs;
//
//  LWIP_UNUSED_ARG(len);
//
//  hs = arg;
//
//  activePcb = pcb;
//  activeHttpState = hs;
//
//  hs->retries = 0;
//
//  if (hs->left > 0)
//  {
//    send_data(pcb, hs);
//  } else
//	  SetDataToSend();

//  if (hs->left <= 0)
//  {
//	 CloseConnection();
//   // close_conn(pcb, hs);
//  }

  return ERR_OK;
}

int SendData(char* data, int length)
{
	if(activePcb == 0)
		return -1;
	int err = tcp_write(activePcb, data, length, 0);
	lastDataSentLength = length;
	tcp_sent(activePcb, DataHasGone);
	return err;
}

//int SetDataToSend()
//{
//	int i;
//
//	if(activeHttpState == 0)
//		return -1;
//
//	if(activeHttpState->left > 0)
//		return 0;
//
//	activeHttpState->left = GetRepRapNetworkDataToSendLength();
//	activeHttpState->file = outputBuffer;
//
//	if(activeHttpState->left > 0)
//	{
//		i = 0;
//		while(i < activeHttpState->left)
//		{
//			outputBuffer[i] = GetRepRapNetworkDataToSend()[i];
//			i++;
//		}
//		outputBuffer[i] = 0;
//	} else
//		outputBuffer[0] = 0;
//
//	return 1;
//}

static void FreeBuffer()
{
	if(activePbuf != 0)
	{
		pbuf_free(activePbuf);
		activePbuf = 0;
		dataPayload = 0;
	}
}

static void CloseConnection()
{
	if(activePcb != 0 && activeHttpState != 0)
	{
		close_conn(activePcb, activeHttpState);
		//activePcb = 0;
		//activeHttpState = 0;
	}
}

static void FreeBufferAndCloseConnection()
{
	FreeBuffer();
	CloseConnection();
}

/*-----------------------------------------------------------------------------------*/

static err_t
http_sent(void *arg, struct tcp_pcb *pcb, uint16_t len)
{
  struct http_state *hs;

  LWIP_UNUSED_ARG(len);

  hs = arg;

  activePcb = pcb;
  activeHttpState = hs;

  hs->retries = 0;

  if (hs->left > 0)
  {
    send_data(pcb, hs);
  } else
	  SetDataToSend();

  if (hs->left <= 0)
  {
	 CloseConnection();
   // close_conn(pcb, hs);
  }

  return ERR_OK;
}

//int HttpSend()
//{
//	if(activePcb != 0 && activeHttpState != 0)
//	{
//		send_data(activePcb, activeHttpState);
//
//		/* Tell TCP that we wish be to informed of data that has been
//	   successfully sent by a call to the http_sent() function. */
//		tcp_sent(activePcb, http_sent);
//	}
//}



//static int InterpretAndSend()
//{
//	if(dataPayload == 0)
//		return false;
//
//	int i;
//
//	struct fs_file file;
//
//	if (strncmp(dataPayload, "GET ", 4) == 0)
//	//if(GetRepRapNetworkDataToSendLength() > 0)
//	{
//		for(i = 0; i < 40; i++) {
//			if (((char *)dataPayload + 4)[i] == ' ' ||
//					((char *)dataPayload + 4)[i] == '\r' ||
//					((char *)dataPayload + 4)[i] == '\n')
//			{
//				((char *)dataPayload + 4)[i] = 0;
//			}
//		}
//
//		if (*(char *)(dataPayload + 4) == '/' && *(char *)(dataPayload + 5) == 0)
//		{
//			fs_open("/index.html", &file);
//		} else if (!fs_open((char *)dataPayload + 4, &file))
//		{
//			fs_open("/404.html", &file);
//		}
//
//		activeHttpState->file = file.data;
//		activeHttpState->left = file.len;
//
//		//activeHttpState->file = GetRepRapNetworkDataToSend();
//		//activeHttpState->left = GetRepRapNetworkDataToSendLength();
//
//		/* printf("data %p len %ld\n", hs->file, hs->left);*/
//
//		//pbuf_free(p);
//		FreeBuffer();
////		send_data(activePcb, activeHttpState);
////
////		/* Tell TCP that we wish be to informed of data that has been
////   successfully sent by a call to the http_sent() function. */
////		tcp_sent(activePcb, http_sent);
//
//		//HttpSend();
//	} else
//	{
//		FreeBufferAndCloseConnection();
//		//pbuf_free(p);
//		//close_conn(pcb, hs);
//	}
//	return true;
//}

static err_t
http_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
	int i;
	char *data;

	struct http_state *hs;

	hs = arg;

	activePcb = pcb;
	activePbuf = p;
	activeHttpState = hs;

	if (err == ERR_OK && p != NULL)
	{

		/* Inform TCP that we have taken the data. */
		tcp_recved(pcb, p->tot_len);

		//if (hs->file == NULL)
		//if(GetRepRapNetworkDataToSendLength() <= 0)
		//{
			data = p->payload;
			dataPayload = data;

			// Deal with data received

			RepRapNetworkReceiveInput(data, p->len);

			//InterpretAndSend();

		//} /*else
		//{
			FreeBuffer();
			//pbuf_free(p);
		//}*/
		//FreeBuffer();
	}


//	if (err == ERR_OK && p == NULL)
//	{
//		CloseConnection();
//		//close_conn(pcb, hs);
//	}
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

  /* Allocate memory for the structure that holds the state of the
     connection. */
  hs = (struct http_state *)mem_malloc(sizeof(struct http_state));

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
void
httpd_init(void)
{
  //struct tcp_pcb *pcb;

  activePcb = tcp_new();
  tcp_bind(activePcb, IP_ADDR_ANY, 80);
  activePcb = tcp_listen(activePcb);
  tcp_accept(activePcb, http_accept);
}
/*-----------------------------------------------------------------------------------*/

#endif

#endif
