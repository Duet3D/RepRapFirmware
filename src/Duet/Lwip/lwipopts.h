/*****************************************************************************
 *
 * \file
 *
 * \brief lwIP configuration.
 *
 * Copyright (c) 2010-2012 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 *****************************************************************************/



#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

/* Include user defined options first */
#include "WMath.h"

/* Uncomment the following line to enable LWIP debugging */
//#define LWIP_DEBUG


/* ---------- System options ---------- */
/* Specify NO_SYS because we are not using an RTOS */
#define NO_SYS                		1
#define LWIP_RAW                  	0

/* Network interface options */
#define LWIP_NETIF_HOSTNAME			1
#define LWIP_NETIF_STATUS_CALLBACK	1

/* These are not available when using "NO_SYS" */
#define LWIP_NETCONN            	0
#define LWIP_SOCKET             	0


/* ---------- Memory options ---------- */

/* These two control is reclaimer functions should be compiled
   in. Should always be turned on (1). */
#define MEM_RECLAIM             	1
#define MEMP_RECLAIM            	1

/* MEM_ALIGNMENT: should be set to the alignment of the CPU for which
   lwIP is compiled. 4 byte alignment -> define MEM_ALIGNMENT to 4, 2
   byte alignment -> define MEM_ALIGNMENT to 2. */
#define MEM_ALIGNMENT           	4

/* MEM_SIZE: the size of the heap memory. If the application will send
a lot of data that needs to be copied, this should be set high. */
#define MEM_SIZE					1024

/* MEMP_NUM_PBUF: the number of memp struct pbufs. If the application
   sends a lot of data out of ROM (or other static memory), this
   should be set high. */
#define MEMP_NUM_PBUF				4

/* Number of raw connection PCBs */
#define MEMP_NUM_RAW_PCB			0


/* --------- IGMP options ---------- */
#define LWIP_IGMP					1	/* looks like this is needed for mdns support */

#ifdef __SAM3X8E__
# define LWIP_RAND					trueRandom
#else
# define LWIP_RAND					random
#endif

#define SO_REUSE					1


/* ---------- DNS options ---------- */
#define LWIP_DNS						1
#define DNS_MAX_NAME_LENGTH				64
#define DNS_LOCAL_HOSTLIST_IS_DYNAMIC	1


/* ---------- UDP options ---------- */
#define LWIP_UDP					1
#define UDP_TTL						255

/* MEMP_NUM_UDP_PCB: the number of UDP protocol control blocks. One
 per active UDP "connection". */
#define MEMP_NUM_UDP_PCB			5

/* MEMP_NUM_TCP_PCB: the number of simultaneously active TCP connections. */
#define MEMP_NUM_TCP_PCB			16

/* MEMP_NUM_TCP_PCB_LISTEN: the number of listening TCP connections. */
#define MEMP_NUM_TCP_PCB_LISTEN		4

/* MEMP_NUM_TCP_SEG: the number of simultaneously queued TCP segments. */
#define MEMP_NUM_TCP_SEG			16

/* MEMP_NUM_REASSDATA: the number of IP packets simultaneously queued for
 * reassembly (whole packets, not fragments!) */
#define MEMP_NUM_REASSDATA			2

/* MEMP_NUM_FRAG_PBUF: the number of IP fragments simultaneously sent
 * (fragments, not whole packets!). */
#define MEMP_NUM_FRAG_PBUF			8

/* MEMP_NUM_SYS_TIMEOUT: the number of simultaneously active timeouts. */
#define MEMP_NUM_SYS_TIMEOUT		8

/* The following four are used only with the sequential API and can be
   set to 0 if the application only will use the raw API. */
/* MEMP_NUM_NETBUF: the number of struct netbufs. */
#define MEMP_NUM_NETBUF				0

/* MEMP_NUM_NETCONN: the number of struct netconns. */
#define MEMP_NUM_NETCONN			0


/* ---------- Pbuf options ---------- */
/* PBUF_POOL_SIZE: the number of buffers in the pbuf pool. */
#define PBUF_POOL_SIZE				5

/* PBUF_POOL_BUFSIZE: the size of each pbuf in the pbuf pool.
 * DO NOT DEFINE IT HERE, the correct value needs to be calculated from TCP_MSS. */

/**
 * IP_REASS_MAX_PBUFS: Total maximum amount of pbufs waiting to be reassembled.
 * Since the received pbufs are enqueued, be sure to configure
 * PBUF_POOL_SIZE > IP_REASS_MAX_PBUFS so that the stack is still able to receive
 * packets even if the maximum amount of fragments is enqueued for reassembly!
 */
#define IP_REASS_MAX_PBUFS			3

/**
 * IP_SOF_BROADCAST=1: Use the SOF_BROADCAST field to enable broadcast
 * filter per pcb on udp and raw send operations. To enable broadcast filter
 * on recv operations, you also have to set IP_SOF_BROADCAST_RECV=1.
 */
#define IP_SOF_BROADCAST			1

/** ETH_PAD_SIZE: number of bytes added before the ethernet header to ensure
 * alignment of payload after that header. Since the header is 14 bytes long,
 * without this padding e.g. addresses in the IP header will not be aligned
 * on a 32-bit boundary, so setting this to 2 can speed up 32-bit-platforms.
 */
#define ETH_PAD_SIZE				2

/* PBUF_LINK_HLEN: the number of bytes that should be allocated for a
   link level header. */
#define PBUF_LINK_HLEN				(16 + ETH_PAD_SIZE)


/* ---------- TCP options ---------- */
#define LWIP_TCP					1
#define TCP_TTL						255

/* Controls if TCP should queue segments that arrive out of
   order. Define to 0 if your device is low on memory. */
#define TCP_QUEUE_OOSEQ				1

/* TCP Maximum segment size. */
#define TCP_MSS						1460

/* TCP receive window. */
#define TCP_WND						(2 * TCP_MSS)

/* TCP sender buffer space (bytes). */
#define TCP_SND_BUF					(2 * TCP_MSS)

/* TCP sender buffer space (pbufs). This must be at least = 2 * TCP_SND_BUF/TCP_MSS for things to work. */
#define TCP_SND_QUEUELEN			(3 * TCP_SND_BUF / TCP_MSS)

/* Maximum number of retransmissions of data segments. */
#define TCP_MAXRTX					8

/* Maximum number of retransmissions of SYN segments. */
#define TCP_SYNMAXRTX				4

/* How frequently are the timer tasks called (in ms)? */
#define TCP_TMR_INTERVAL			125


/* ---------- ARP options ---------- */
#define ARP_TABLE_SIZE				16
#define ARP_QUEUEING				0


/* ---------- IP options ---------- */
/* Define IP_FORWARD to 1 if you wish to have the ability to forward
   IP packets across network interfaces. If you are going to run lwIP
   on a device with only one network interface, define this to 0. */
#define IP_FORWARD					0

/* If defined to 1, IP options are allowed (but not parsed). If
   defined to 0, all packets with IP options are dropped. */
#define IP_OPTIONS					1


/* ---------- ICMP options ---------- */
#define ICMP_TTL					255


/* ---------- DHCP options ---------- */
/* Define LWIP_DHCP to 1 if you want DHCP configuration of
   interfaces. DHCP is not implemented in lwIP 0.5.1, however, so
   turning this on does currently not work. */
#define LWIP_DHCP					1

/* 1 if you want to do an ARP check on the offered address
   (recommended). */
#define DHCP_DOES_ARP_CHECK			1


/* ---------- Statistics options ---------- */
#define LWIP_STATS					1
#define LWIP_STATS_DISPLAY			1


/* ---------- Lwip Debug options ---------- */
/* Define default values for unconfigured parameters. */
// 1 == To suppress some errors for now (no debug output)
#define LWIP_NOASSERT				0

#ifdef LWIP_DEBUG
#define DHCP_DEBUG					LWIP_DBG_ON
#define ETHARP_DEBUG				LWIP_DBG_ON
#define NETIF_DEBUG				 	LWIP_DBG_ON
#define PBUF_DEBUG					LWIP_DBG_ON
#define API_LIB_DEBUG				LWIP_DBG_ON
#define API_MSG_DEBUG				LWIP_DBG_ON
#define SOCKETS_DEBUG				LWIP_DBG_ON
#define ICMP_DEBUG					LWIP_DBG_ON
#define INET_DEBUG					LWIP_DBG_ON
#define IP_DEBUG					LWIP_DBG_ON
#define IP_REASS_DEBUG				LWIP_DBG_ON
#define RAW_DEBUG					LWIP_DBG_ON
#define MEM_DEBUG					LWIP_DBG_ON
#define MEMP_DEBUG					LWIP_DBG_ON
#define SYS_DEBUG					LWIP_DBG_ON
#define TCP_DEBUG					LWIP_DBG_ON
#define TCP_INPUT_DEBUG		 		LWIP_DBG_ON
#define TCP_FR_DEBUG				LWIP_DBG_ON
#define TCP_RTO_DEBUG				LWIP_DBG_ON
#define TCP_CWND_DEBUG				LWIP_DBG_ON
#define TCP_WND_DEBUG				LWIP_DBG_ON
#define TCP_OUTPUT_DEBUG			LWIP_DBG_ON
#define TCP_RST_DEBUG				LWIP_DBG_ON
#define TCP_QLEN_DEBUG				LWIP_DBG_ON
#define UDP_DEBUG					LWIP_DBG_ON
#define TCPIP_DEBUG				 	LWIP_DBG_ON
#define LWIP_DBG_MIN_LEVEL			LWIP_DBG_LEVEL_ALL
#else
#define DHCP_DEBUG					LWIP_DBG_OFF
#define ETHARP_DEBUG				LWIP_DBG_ON
#define NETIF_DEBUG				 	LWIP_DBG_ON
#define PBUF_DEBUG					LWIP_DBG_ON
#define API_LIB_DEBUG				LWIP_DBG_ON
#define API_MSG_DEBUG				LWIP_DBG_ON
#define SOCKETS_DEBUG				LWIP_DBG_ON
#define ICMP_DEBUG					LWIP_DBG_ON
#define INET_DEBUG					LWIP_DBG_ON
#define IP_DEBUG					LWIP_DBG_ON
#define IP_REASS_DEBUG				LWIP_DBG_OFF
#define RAW_DEBUG					LWIP_DBG_OFF
#define MEM_DEBUG					LWIP_DBG_OFF
#define MEMP_DEBUG					LWIP_DBG_OFF
#define SYS_DEBUG					LWIP_DBG_ON
#define TCP_DEBUG					LWIP_DBG_OFF
#define TCP_INPUT_DEBUG			 	LWIP_DBG_OFF
#define TCP_FR_DEBUG				LWIP_DBG_OFF
#define TCP_RTO_DEBUG				LWIP_DBG_OFF
#define TCP_CWND_DEBUG				LWIP_DBG_OFF
#define TCP_WND_DEBUG				LWIP_DBG_OFF
#define TCP_OUTPUT_DEBUG			LWIP_DBG_OFF
#define TCP_RST_DEBUG				LWIP_DBG_OFF
#define TCP_QLEN_DEBUG				LWIP_DBG_OFF
#define UDP_DEBUG					LWIP_DBG_OFF
#define TCPIP_DEBUG				 	LWIP_DBG_OFF
#define LWIP_DBG_MIN_LEVEL			LWIP_DBG_LEVEL_SERIOUS
#endif


// \note For a list of all possible lwIP configurations, check http://lwip.wikia.com/wiki/Lwipopts.h

#endif /* __LWIPOPTS_H__ */

// vim: ts=4:sw=4
