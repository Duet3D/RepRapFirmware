/**
 * \file
 *
 * \brief GMAC (Gigabit MAC) driver for lwIP.
 *
 * Copyright (c) 2015-2016 Atmel Corporation. All rights reserved.
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
 */

#include "ethernet_phy.h"
#include "same70_gmac.h"
#include "pmc.h"
#include "sysclk.h"
#include "conf_eth.h"
#include <string.h>

#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/opt.h"
#include "lwip/pbuf.h"
#include "lwip/snmp.h"
#include "lwip/stats.h"
#include "lwip/sys.h"
#include "netif/etharp.h"


/** Network interface identifier. */
#define IFNAME0               'e'
#define IFNAME1               'n'

/** Maximum transfer unit. */
#define NET_MTU               1500

/** Network link speed. */
#define NET_LINK_SPEED        100000000

#if (NO_SYS == 0)
/* Interrupt priorities. (lowest value = highest priority) */
/* ISRs using FreeRTOS *FromISR APIs must have priorities below or equal to */
/* configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY. */
#define INT_PRIORITY_GMAC     12

/** The GMAC interrupts to enable */
#define GMAC_INT_GROUP (GMAC_ISR_RCOMP | GMAC_ISR_ROVR | GMAC_ISR_HRESP | GMAC_ISR_TCOMP | GMAC_ISR_TUR | GMAC_ISR_TFC)

/** The GMAC TX errors to handle */
#define GMAC_TX_ERRORS (GMAC_TSR_TFC | GMAC_TSR_HRESP)

/** The GMAC RX errors to handle */
#define GMAC_RX_ERRORS (GMAC_RSR_RXOVR | GMAC_RSR_HNO)
#elif 1		// chrishamm
#define GMAC_INT_GROUP       (GMAC_ISR_RCOMP | GMAC_ISR_ROVR)		// call ISR only when a new frame has arrived
#define GMAC_TX_ERRORS       0
#define GMAC_RX_ERRORS       0
#else
#define INT_PRIORITY_GMAC    0
#define GMAC_INT_GROUP       0
#define GMAC_TX_ERRORS       0
#define GMAC_RX_ERRORS       0
#endif

/** TX descriptor lists */
COMPILER_ALIGNED(8)
static gmac_tx_descriptor_t gs_tx_desc_null;
/** RX descriptors lists */
COMPILER_ALIGNED(8)
static gmac_rx_descriptor_t gs_rx_desc_null;
/**
 * GMAC driver structure.
 */
struct gmac_device {
	/**
	 * Pointer to allocated TX buffer.
	 * Section 3.6 of AMBA 2.0 spec states that burst should not cross
	 * 1K Boundaries.
	 * Receive buffer manager writes are burst of 2 words => 3 lsb bits
	 * of the address shall be set to 0.
	 */
	/** Pointer to Rx descriptor list (must be 8-byte aligned). */
	gmac_rx_descriptor_t rx_desc[GMAC_RX_BUFFERS];
	/** Pointer to Tx descriptor list (must be 8-byte aligned). */
	gmac_tx_descriptor_t tx_desc[GMAC_TX_BUFFERS];
	/** RX pbuf pointer list. */
	struct pbuf *rx_pbuf[GMAC_RX_BUFFERS];
	/** TX buffers. */
	uint8_t tx_buf[GMAC_TX_BUFFERS][GMAC_TX_UNITSIZE];

	/** RX index for current processing TD. */
	uint32_t us_rx_idx;
	/** Circular buffer head pointer by upper layer (buffer to be sent). */
	uint32_t us_tx_idx;

	/** Reference to lwIP netif structure. */
	struct netif *netif;

#if NO_SYS == 0
	/** RX task notification semaphore. */
	sys_sem_t rx_sem;
#endif
};

/**
 * GMAC driver instance.
 */
COMPILER_ALIGNED(8)
static struct gmac_device gs_gmac_dev;

/**
 * MAC address to use.
 */
static uint8_t gs_uc_mac_address[] =
{
	ETHERNET_CONF_ETHADDR0,
	ETHERNET_CONF_ETHADDR1,
	ETHERNET_CONF_ETHADDR2,
	ETHERNET_CONF_ETHADDR3,
	ETHERNET_CONF_ETHADDR4,
	ETHERNET_CONF_ETHADDR5
};

#if LWIP_STATS
/** Used to compute lwIP bandwidth. */
uint32_t lwip_tx_count = 0;
uint32_t lwip_rx_count = 0;
uint32_t lwip_tx_rate = 0;
uint32_t lwip_rx_rate = 0;
#endif

static gmac_dev_tx_cb_t gmac_rx_cb = NULL;

/**
 * \brief GMAC interrupt handler.
 */
void GMAC_Handler(void)
{
#if 1
	volatile uint32_t ul_isr;

	/* Get interrupt status. */
	ul_isr = gmac_get_interrupt_status(GMAC);

	/* RX interrupts. */
	if ((ul_isr & GMAC_INT_GROUP) != 0 && gmac_rx_cb != NULL)
	{
		gmac_rx_cb(ul_isr);
	}
#elif NO_SYS == 1
    NVIC_DisableIRQ(GMAC_IRQn);
#else
	volatile uint32_t ul_isr;
	portBASE_TYPE xGMACTaskWoken = pdFALSE;

	/* Get interrupt status. */
	ul_isr = gmac_get_interrupt_status(GMAC);

	/* RX interrupts. */
	if (ul_isr & GMAC_INT_GROUP) {
		xSemaphoreGiveFromISR(gs_gmac_dev.rx_sem, &xGMACTaskWoken);
	}

	portEND_SWITCHING_ISR(xGMACTaskWoken);
#endif
}

/**
 * \brief Populate the RX descriptor ring buffers with pbufs.
 *
 * \note Make sure that the p->payload pointer is 32 bits aligned.
 * (since the lsb are used as status bits by GMAC).
 *
 * \param p_gmac_dev Pointer to driver data structure.
 */
static void gmac_rx_populate_queue(struct gmac_device *p_gmac_dev)
{
	uint32_t ul_index = 0;
	struct pbuf *p = 0;

	/* Set up the RX descriptors. */
	for (ul_index = 0; ul_index < GMAC_RX_BUFFERS; ul_index++) {
		if (p_gmac_dev->rx_pbuf[ul_index] == 0) {

			/* Allocate a new pbuf with the maximum size. */
			p = pbuf_alloc(PBUF_RAW, (u16_t) GMAC_FRAME_LENTGH_MAX, PBUF_POOL);
			if (p == NULL) {
				LWIP_DEBUGF(NETIF_DEBUG, ("gmac_rx_populate_queue: pbuf allocation failure\n"));
				break;
			}

			/* Make sure lwIP is well configured so one pbuf can contain the maximum packet size. */
			LWIP_ASSERT("gmac_rx_populate_queue: pbuf size too small!", pbuf_clen(p) <= 1);

			/* Make sure that the payload buffer is properly aligned. */
			LWIP_ASSERT("gmac_rx_populate_queue: unaligned p->payload buffer address",
					(((uint32_t)p->payload & 0xFFFFFFFC) == (uint32_t)p->payload));

			if (ul_index == GMAC_RX_BUFFERS - 1)
				p_gmac_dev->rx_desc[ul_index].addr.val = (u32_t) p->payload | GMAC_RXD_WRAP;
			else
				p_gmac_dev->rx_desc[ul_index].addr.val = (u32_t) p->payload;

			/* Reset status value. */
			p_gmac_dev->rx_desc[ul_index].status.val = 0;

			/* Save pbuf pointer to be sent to lwIP upper layer. */
			p_gmac_dev->rx_pbuf[ul_index] = p;

			LWIP_DEBUGF(NETIF_DEBUG,
					("gmac_rx_populate_queue: new pbuf allocated: %p [idx=%u]\n",
					p, ul_index));
		}
	}
}

/**
 * \brief Set up the RX descriptor ring buffers.
 *
 * This function sets up the descriptor list used for receive packets.
 *
 * \param ps_gmac_dev Pointer to driver data structure.
 */
static void gmac_rx_init(struct gmac_device *ps_gmac_dev)
{
	uint32_t ul_index = 0;

	/* Init RX index. */
	ps_gmac_dev->us_rx_idx = 0;

	/* Set up the RX descriptors. */
	for (ul_index = 0; ul_index < GMAC_RX_BUFFERS; ul_index++) {
		ps_gmac_dev->rx_pbuf[ul_index] = 0;
		ps_gmac_dev->rx_desc[ul_index].addr.val = 0;
		ps_gmac_dev->rx_desc[ul_index].status.val = 0;
	}
	ps_gmac_dev->rx_desc[ul_index - 1].addr.val |= GMAC_RXD_WRAP;

	/* Build RX buffer and descriptors. */
	gmac_rx_populate_queue(ps_gmac_dev);

	/* Set receive buffer queue base address pointer. */
	gmac_set_rx_queue(GMAC, (uint32_t) &ps_gmac_dev->rx_desc[0]);
}

/**
 * \brief Set up the TX descriptor ring buffers.
 *
 * This function sets up the descriptor list used for receive packets.
 *
 * \param ps_gmac_dev Pointer to driver data structure.
 */
static void gmac_tx_init(struct gmac_device *ps_gmac_dev)
{
	uint32_t ul_index;

	/* Init TX index pointer. */
	ps_gmac_dev->us_tx_idx = 0;

	/* Set up the TX descriptors. */
	for (ul_index = 0; ul_index < GMAC_TX_BUFFERS; ul_index++) {
		ps_gmac_dev->tx_desc[ul_index].addr = (uint32_t)&ps_gmac_dev->tx_buf[ul_index][0];
		ps_gmac_dev->tx_desc[ul_index].status.val = GMAC_TXD_USED | GMAC_TXD_LAST;
	}
	ps_gmac_dev->tx_desc[ul_index - 1].status.val |= GMAC_TXD_WRAP;

	/* Set receive buffer queue base address pointer. */
	gmac_set_tx_queue(GMAC, (uint32_t) &ps_gmac_dev->tx_desc[0]);
}

/**
 * \brief Initialize GMAC and PHY.
 *
 * \note Called from ethernetif_init().
 *
 * \param netif the lwIP network interface structure for this ethernetif.
 */
static void gmac_low_level_init(struct netif *netif)
{
#if 0			// chrishamm
	volatile uint32_t ul_delay;
#endif

	/* Set MAC hardware address length. */
	netif->hwaddr_len = sizeof(gs_uc_mac_address);
	/* Set MAC hardware address. */
	netif->hwaddr[0] = gs_uc_mac_address[0];
	netif->hwaddr[1] = gs_uc_mac_address[1];
	netif->hwaddr[2] = gs_uc_mac_address[2];
	netif->hwaddr[3] = gs_uc_mac_address[3];
	netif->hwaddr[4] = gs_uc_mac_address[4];
	netif->hwaddr[5] = gs_uc_mac_address[5];

	/* Set maximum transfer unit. */
	netif->mtu = NET_MTU;

	/* Device capabilities. */
	netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP
#if LWIP_IGMP	// chrishamm
			| NETIF_FLAG_IGMP
#endif
	;

#if 0			// chrishamm: Just like with the EMAC on the Duet, we initialise the GMAC step-by-step to avoid blocking
	/* Wait for PHY to be ready (CAT811: Max400ms). */
	ul_delay = sysclk_get_cpu_hz() / 1000 / 3 * 400;
	while (ul_delay--) {
	}
#endif

	/* Init MAC PHY driver. */
	if (ethernet_phy_init(GMAC, BOARD_GMAC_PHY_ADDR, sysclk_get_cpu_hz()) != GMAC_OK) {
		LWIP_DEBUGF(NETIF_DEBUG, ("gmac_low_level_init: PHY init ERROR!\n"));
		return;
	}

#if 0			// chrishamm: See ethernetif_establish_link()
	/* Auto Negotiate, work in RMII mode. */
	if (ethernet_phy_auto_negotiate(GMAC, BOARD_GMAC_PHY_ADDR) != GMAC_OK) {
		LWIP_DEBUGF(NETIF_DEBUG, ("gmac_low_level_init: auto negotiate ERROR!\n"));
		return;
	}

	/* Establish ethernet link. */
	while (ethernet_phy_set_link(GMAC, BOARD_GMAC_PHY_ADDR, 1) != GMAC_OK) {
		LWIP_DEBUGF(NETIF_DEBUG, ("gmac_low_level_init: set link ERROR!\n"));
		return;
	}

	/* Set link up*/
	netif->flags |= NETIF_FLAG_LINK_UP;
#endif
}

/**
 * \brief This function should do the actual transmission of the packet. The
 * packet is contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * \param netif the lwIP network interface structure for this ethernetif.
 * \param p the MAC packet to send (e.g. IP packet including MAC addresses and type).
 *
 * \return ERR_OK if the packet could be sent.
 * an err_t value if the packet couldn't be sent.
 */
static err_t gmac_low_level_output(struct netif *netif, struct pbuf *p)
{
	struct gmac_device *ps_gmac_dev = netif->state;
	struct pbuf *q = NULL;
	uint8_t *buffer = 0;

	/* Handle GMAC underrun or AHB errors. */
	if (gmac_get_tx_status(GMAC) & GMAC_TX_ERRORS) {
		LWIP_DEBUGF(NETIF_DEBUG, ("gmac_low_level_output: GMAC ERROR, reinit TX...\n"));

		gmac_enable_transmit(GMAC, false);

		LINK_STATS_INC(link.err);
		LINK_STATS_INC(link.drop);

		/* Reinit TX descriptors. */
		gmac_tx_init(ps_gmac_dev);

		/* Clear error status. */
		gmac_clear_tx_status(GMAC, GMAC_TX_ERRORS);

		gmac_enable_transmit(GMAC, true);
	}

	buffer = (uint8_t*)ps_gmac_dev->tx_desc[ps_gmac_dev->us_tx_idx].addr;

	/* Copy pbuf chain into TX buffer. */
	for (q = p; q != NULL; q = q->next) {
		memcpy(buffer, q->payload, q->len);
		buffer += q->len;
	}

	/* Set len and mark the buffer to be sent by GMAC. */
	ps_gmac_dev->tx_desc[ps_gmac_dev->us_tx_idx].status.bm.b_len = p->tot_len;
	ps_gmac_dev->tx_desc[ps_gmac_dev->us_tx_idx].status.bm.b_used = 0;

	LWIP_DEBUGF(NETIF_DEBUG,
			("gmac_low_level_output: DMA buffer sent, size=%d [idx=%u]\n",
			p->tot_len, ps_gmac_dev->us_tx_idx));

	ps_gmac_dev->us_tx_idx = (ps_gmac_dev->us_tx_idx + 1) % GMAC_TX_BUFFERS;

	/* Now start to transmission. */
	gmac_start_transmission(GMAC);

#if LWIP_STATS
	lwip_tx_count += p->tot_len;
#endif
	LINK_STATS_INC(link.xmit);

	return ERR_OK;
}

/**
 * \brief Use pre-allocated pbuf as DMA source and return the incoming packet.
 *
 * \param netif the lwIP network interface structure for this ethernetif.
 *
 * \return a pbuf filled with the received packet (including MAC header).
 * 0 on memory error.
 */
static struct pbuf *gmac_low_level_input(struct netif *netif)
{
	struct gmac_device *ps_gmac_dev = netif->state;
	struct pbuf *p = 0;
	uint32_t length = 0;
	uint32_t ul_index = 0;
	gmac_rx_descriptor_t *p_rx = &ps_gmac_dev->rx_desc[ps_gmac_dev->us_rx_idx];

	/* Handle GMAC overrun or AHB errors. */
	if (gmac_get_rx_status(GMAC) & GMAC_RX_ERRORS) {

		gmac_enable_receive(GMAC, false);

		LINK_STATS_INC(link.err);
		LINK_STATS_INC(link.drop);

		/* Free all RX pbufs. */
		for (ul_index = 0; ul_index < GMAC_RX_BUFFERS; ul_index++) {
			if (ps_gmac_dev->rx_pbuf[ul_index] != 0) {
				pbuf_free(ps_gmac_dev->rx_pbuf[ul_index]);
				ps_gmac_dev->rx_pbuf[ul_index] = 0;
			}
		}

		/* Reinit TX descriptors. */
		gmac_rx_init(ps_gmac_dev);

		/* Clear error status. */
		gmac_clear_rx_status(GMAC, GMAC_RX_ERRORS);

		gmac_enable_receive(GMAC, true);
	}

	/* Check that a packet has been received and processed by GMAC. */
	if ((p_rx->addr.val & GMAC_RXD_OWNERSHIP) == GMAC_RXD_OWNERSHIP) {
		/* Packet is a SOF since packet size is set to maximum. */
		length = p_rx->status.val & GMAC_RXD_LEN_MASK;

		/* Fetch pre-allocated pbuf. */
		p = ps_gmac_dev->rx_pbuf[ps_gmac_dev->us_rx_idx];
		p->len = length;

		/* Remove this pbuf from its desriptor. */
		ps_gmac_dev->rx_pbuf[ps_gmac_dev->us_rx_idx] = 0;

		LWIP_DEBUGF(NETIF_DEBUG,
				("gmac_low_level_input: DMA buffer %p received, size=%u [idx=%u]\n",
				p, length, ps_gmac_dev->us_rx_idx));

		/* Set pbuf total packet size. */
		p->tot_len = length;
		LINK_STATS_INC(link.recv);

		/* Fill empty descriptors with new pbufs. */
		gmac_rx_populate_queue(ps_gmac_dev);

		/* Mark the descriptor ready for transfer. */
		p_rx->addr.val &= ~(GMAC_RXD_OWNERSHIP);

		ps_gmac_dev->us_rx_idx = (ps_gmac_dev->us_rx_idx + 1) % GMAC_RX_BUFFERS;

#if LWIP_STATS
	lwip_rx_count += length;
#endif
	}

	return p;
}

#if NO_SYS == 0
/**
 * \brief GMAC task function. This function waits for the notification
 * semaphore from the interrupt, processes the incoming packet and then
 * passes it to the lwIP stack.
 *
 * \param pvParameters A pointer to the gmac_device instance.
 */
static void gmac_task(void *pvParameters)
{
	struct gmac_device *ps_gmac_dev = pvParameters;

	while (1) {
		/* Wait for the RX notification semaphore. */
		sys_arch_sem_wait(&ps_gmac_dev->rx_sem, 0);

		/* Process the incoming packet. */
		ethernetif_input(ps_gmac_dev->netif);
	}
}
#endif

/**
 * \brief This function should be called when a packet is ready to be
 * read from the interface. It uses the function gmac_low_level_input()
 * that handles the actual reception of bytes from the network interface.
 * Then the type of the received packet is determined and the appropriate
 * input function is called.
 *
 * \param netif the lwIP network interface structure for this ethernetif.
 */
bool ethernetif_input(struct netif *netif)
{
	struct eth_hdr *ethhdr;
	struct pbuf *p;

	/* Move received packet into a new pbuf. */
	p = gmac_low_level_input(netif);
	if (p == NULL)
	{
		return false;
	}

	/* Points to packet payload, which starts with an Ethernet header. */
	ethhdr = p->payload;

	switch (htons(ethhdr->type)) {
		case ETHTYPE_IP:
		case ETHTYPE_ARP:
#if PPPOE_SUPPORT
		case ETHTYPE_PPPOEDISC:
		case ETHTYPE_PPPOE:
#endif /* PPPOE_SUPPORT */
			/* Send packet to lwIP for processing. */
			if (netif->input(p, netif) != ERR_OK) {
				LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
				/* Free buffer. */
				pbuf_free(p);
			}
			break;

		default:
			/* Free buffer. */
			pbuf_free(p);
			break;
	}
	return true;
}

/**
 * \brief Should be called at the beginning of the program to set up the
 * network interface. It calls the function gmac_low_level_init() to do the
 * actual setup of the hardware.
 *
 * \param netif the lwIP network interface structure for this ethernetif.
 *
 * \return ERR_OK if the loopif is initialized.
 * ERR_MEM if private data couldn't be allocated.
 * any other err_t on error.
 */
err_t ethernetif_init(struct netif *netif)
{
	LWIP_ASSERT("netif != NULL", (netif != NULL));

	gs_gmac_dev.netif = netif;

#if LWIP_NETIF_HOSTNAME && 0	// chrishamm: RRF sets the hostname explicitly
	/* Initialize interface hostname. */
	netif->hostname = "gmacdev";
#endif /* LWIP_NETIF_HOSTNAME */

	/*
	 * Initialize the snmp variables and counters inside the struct netif.
	 * The last argument should be replaced with your link speed, in units
	 * of bits per second.
	 */
#if LWIP_SNMP
	NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, NET_LINK_SPEED);
#endif /* LWIP_SNMP */

	netif->state = &gs_gmac_dev;
	netif->name[0] = IFNAME0;
	netif->name[1] = IFNAME1;

	/* We directly use etharp_output() here to save a function call.
	 * You can instead declare your own function an call etharp_output()
	 * from it if you have to do some checks before sending (e.g. if link
	 * is available...) */
	netif->output = etharp_output;
	netif->linkoutput = gmac_low_level_output;

	/* Initialize the hardware */
	gmac_low_level_init(netif);

#if NO_SYS == 0
	err_t err;
	sys_thread_t id;

	/* Incoming packet notification semaphore. */
	err = sys_sem_new(&gs_gmac_dev.rx_sem, 0);
	LWIP_ASSERT("ethernetif_init: GMAC RX semaphore allocation ERROR!\n",
			(err == ERR_OK));
	if (err == ERR_MEM)
		return ERR_MEM;

	id = sys_thread_new("GMAC", gmac_task, &gs_gmac_dev,
			netifINTERFACE_TASK_STACK_SIZE, netifINTERFACE_TASK_PRIORITY);
	LWIP_ASSERT("ethernetif_init: GMAC Task allocation ERROR!\n",
			(id != 0));
	if (id == 0)
		return ERR_MEM;
#endif

	return ERR_OK;
}

#if 1	// chrishamm

void ethernetif_hardware_init(void)
{
	/* Enable GMAC clock. */
	pmc_enable_periph_clk(ID_GMAC);

	/* Disable TX & RX and more. */
	gmac_network_control(GMAC, 0);
	gmac_disable_interrupt(GMAC, ~0u);

	gmac_clear_statistics(GMAC);

	/* Clear all status bits in the receive status register. */
	gmac_clear_rx_status(GMAC, GMAC_RSR_BNA | GMAC_RSR_REC | GMAC_RSR_RXOVR
			| GMAC_RSR_HNO);

	/* Clear all status bits in the transmit status register. */
	gmac_clear_tx_status(GMAC, GMAC_TSR_UBR | GMAC_TSR_COL | GMAC_TSR_RLE
			| GMAC_TSR_TXGO | GMAC_TSR_TFC | GMAC_TSR_TXCOMP
			| GMAC_TSR_HRESP);

	/* Clear interrupts. */
	gmac_get_interrupt_status(GMAC);

	/* Enable the copy of data into the buffers
	   ignore broadcasts, and not copy FCS. */
	gmac_enable_copy_all(GMAC, false);
	gmac_disable_broadcast(GMAC, false);

	/* Set RX buffer size to 1536. */
	gmac_set_rx_bufsize(GMAC, 0x18);

	/* Clear interrupts */
	gmac_get_priority_interrupt_status(GMAC, GMAC_QUE_2);
	gmac_get_priority_interrupt_status(GMAC, GMAC_QUE_1);
#if (SAMV71B || SAME70B)
	gmac_get_priority_interrupt_status(GMAC, GMAC_QUE_3);
	gmac_get_priority_interrupt_status(GMAC, GMAC_QUE_4);
	gmac_get_priority_interrupt_status(GMAC, GMAC_QUE_5);
#endif

	/* Set Tx Priority */
	gs_tx_desc_null.addr = (uint32_t)0xFFFFFFFF;
	gs_tx_desc_null.status.val = GMAC_TXD_WRAP | GMAC_TXD_USED;
	gmac_set_tx_priority_queue(GMAC, (uint32_t)&gs_tx_desc_null, GMAC_QUE_2);
	gmac_set_tx_priority_queue(GMAC, (uint32_t)&gs_tx_desc_null, GMAC_QUE_1);
#if (SAMV71B || SAME70B)
	gmac_set_tx_priority_queue(GMAC, (uint32_t)&gs_tx_desc_null, GMAC_QUE_3);
	gmac_set_tx_priority_queue(GMAC, (uint32_t)&gs_tx_desc_null, GMAC_QUE_4);
	gmac_set_tx_priority_queue(GMAC, (uint32_t)&gs_tx_desc_null, GMAC_QUE_5);
#endif

	/* Set Rx Priority */
	gs_rx_desc_null.addr.val = (uint32_t)0xFFFFFFFF & GMAC_RXD_ADDR_MASK;
	gs_rx_desc_null.addr.val |= GMAC_RXD_WRAP;
	gs_rx_desc_null.status.val = 0;
	gmac_set_rx_priority_queue(GMAC, (uint32_t)&gs_rx_desc_null, GMAC_QUE_2);
	gmac_set_rx_priority_queue(GMAC, (uint32_t)&gs_rx_desc_null, GMAC_QUE_1);
#if (SAMV71B || SAME70B)
	gmac_set_rx_priority_queue(GMAC, (uint32_t)&gs_rx_desc_null, GMAC_QUE_3);
	gmac_set_rx_priority_queue(GMAC, (uint32_t)&gs_rx_desc_null, GMAC_QUE_4);
	gmac_set_rx_priority_queue(GMAC, (uint32_t)&gs_rx_desc_null, GMAC_QUE_5);
#endif

	gmac_rx_init(&gs_gmac_dev);
	gmac_tx_init(&gs_gmac_dev);

	/* Enable Rx, Tx and the statistics register. */
	gmac_enable_transmit(GMAC, true);
	gmac_enable_receive(GMAC, true);
	gmac_enable_statistics_write(GMAC, true);

	/* Set up the interrupts for transmission and errors. */
	gmac_enable_interrupt(GMAC, GMAC_INT_GROUP);

	/* Set GMAC address. */
	gmac_set_address(GMAC, 0, gs_uc_mac_address);

	/* Enable NVIC GMAC interrupt. */
#if 0		// chrishamm: NVIC priorities are assigned by RepRapFirmware
	NVIC_SetPriority(GMAC_IRQn, INT_PRIORITY_GMAC);
#endif
	NVIC_EnableIRQ(GMAC_IRQn);
}

bool ethernetif_establish_link(void)
{
	/* Auto Negotiate, work in RMII mode. */
	uint8_t result = ethernet_phy_auto_negotiate(GMAC, BOARD_GMAC_PHY_ADDR);
	if (result != GMAC_OK) {
		if (result != GMAC_TIMEOUT)
		{
			// chrishamm: It is expected that the function above will return ERR_TIMEOUT a few times
			LWIP_DEBUGF(NETIF_DEBUG, ("gmac_low_level_init: auto negotiate ERROR!\n"));
		}
		return false;
	}

	/* Establish ethernet link. */
	if (ethernet_phy_set_link(GMAC, BOARD_GMAC_PHY_ADDR, 1) != GMAC_OK) {
		LWIP_DEBUGF(NETIF_DEBUG, ("gmac_low_level_init: set link ERROR!\n"));
		return false;
	}

	return true;
}

// Ask the PHY if the link is still up
bool ethernetif_link_established(void)
{
	gmac_enable_management(GMAC, true);

	uint32_t ul_stat1;
	if (gmac_phy_read(GMAC, BOARD_GMAC_PHY_ADDR, GMII_BMSR, &ul_stat1) != GMAC_OK) {
		gmac_enable_management(GMAC, false);
		return false;
	}

	if ((ul_stat1 & GMII_LINK_STATUS) == 0) {
		gmac_enable_management(GMAC, false);
		return false;
	}

	gmac_enable_management(GMAC, false);
	return true;
}

void ethernetif_set_rx_callback(gmac_dev_tx_cb_t callback)
{
	gmac_rx_cb = callback;
}

void ethernetif_set_mac_address(const uint8_t macAddress[])
{
	// This function must be called once before low_level_init(), because that is where the
	// MAC address of the netif is assigned
	for (size_t i = 0; i < 6; ++i)
	{
		gs_uc_mac_address[i] = macAddress[i];
	}
}

u32_t sys_now(void)
{
	extern u32_t millis();
	return millis();
}

#endif
