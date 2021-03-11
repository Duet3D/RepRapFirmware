/**
 * \file
 *
 * \brief GMAC (Gigabit MAC) driver for lwIP.
 *
 * Copyright (c) 2015-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

#include "GmacInterface.h"
#include <pmc/pmc.h>

#include <conf_eth.h>
#include <cstring>

extern "C" {
#include "ksz8081rna/ethernet_phy.h"
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/stats.h"
#include "lwip/snmp.h"
#include "netif/etharp.h"
}

#define __nocache	__attribute__((section(".ram_nocache")))

#include <RepRapFirmware.h>
#include <RTOSIface/RTOSIface.h>
#include <Platform/TaskPriorities.h>

extern Mutex lwipMutex;

constexpr size_t EthernetTaskStackWords = 250;
static Task<EthernetTaskStackWords> ethernetTask;

// Error counters
unsigned int rxErrorCount;
unsigned int rxBuffersNotFullyPopulatedCount;
unsigned int txErrorCount;
unsigned int txBufferNotFreeCount;
unsigned int txBufferTooShortCount;

/** Network interface identifier. */
#define IFNAME0               'e'
#define IFNAME1               'n'

/** Maximum transfer unit. */
#define NET_MTU               1500

/** Network link speed. */
#define NET_LINK_SPEED        100000000

/* Interrupt priorities. (lowest value = highest priority) */
/* ISRs using FreeRTOS *FromISR APIs must have priorities below or equal to */
/* configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY. */

/** The GMAC interrupts to enable */
#define GMAC_INT_GROUP (GMAC_ISR_RCOMP | GMAC_ISR_ROVR | GMAC_ISR_HRESP | GMAC_ISR_TCOMP | GMAC_ISR_TUR | GMAC_ISR_TFC)

/** The GMAC TX errors to handle */
#define GMAC_TX_ERRORS (GMAC_TSR_TFC | GMAC_TSR_HRESP)

/** The GMAC RX errors to handle */
#define GMAC_RX_ERRORS (GMAC_RSR_RXOVR | GMAC_RSR_HNO)

/** TX descriptor lists */
alignas(8) static gmac_tx_descriptor_t gs_tx_desc_null;
/** RX descriptors lists */
alignas(8) static gmac_rx_descriptor_t gs_rx_desc_null;
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
	volatile gmac_rx_descriptor_t rx_desc[GMAC_RX_BUFFERS];
	/** Pointer to Tx descriptor list (must be 8-byte aligned). */
	volatile gmac_tx_descriptor_t tx_desc[GMAC_TX_BUFFERS];
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
};

/**
 * GMAC driver instance.
 */
__nocache __aligned(8) static struct gmac_device gs_gmac_dev;

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

static bool rxPbufsFullyPopulated = false;

#if LWIP_STATS
/** Used to compute lwIP bandwidth. */
uint32_t lwip_tx_count = 0;
uint32_t lwip_rx_count = 0;
uint32_t lwip_tx_rate = 0;
uint32_t lwip_rx_rate = 0;
#endif

/**
 * \brief GMAC interrupt handler.
 */
extern "C" void GMAC_Handler() noexcept
{
	/* Get interrupt status. */
	const uint32_t ul_isr = gmac_get_interrupt_status(GMAC);

	/* RX interrupts. */
	if (ul_isr & GMAC_INT_GROUP)
	{
		ethernetTask.GiveFromISR();
	}
}

/**
 * \brief Populate the RX descriptor ring buffers with pbufs.
 *
 * \note Make sure that the p->payload pointer is 32 bits aligned.
 * (since the lsb are used as status bits by GMAC).
 *
 * \param p_gmac_dev Pointer to driver data structure.
 * \param startAt The index to start populating from
 */
static void gmac_rx_populate_queue(struct gmac_device *p_gmac_dev, uint32_t startAt) noexcept
{
	uint32_t ul_index = startAt;

	/* Set up the RX descriptors. */
	do
	{
		if (p_gmac_dev->rx_pbuf[ul_index] == nullptr)
		{
			/* Allocate a new pbuf with the maximum size. */
			pbuf * const p = pbuf_alloc(PBUF_RAW, (u16_t) GMAC_FRAME_LENTGH_MAX, PBUF_POOL);
			if (p == nullptr)
			{
				LWIP_DEBUGF(NETIF_DEBUG, ("gmac_rx_populate_queue: pbuf allocation failure\n"));
				rxPbufsFullyPopulated = false;
				++rxBuffersNotFullyPopulatedCount;
				return;
			}

			/* Make sure lwIP is well configured so one pbuf can contain the maximum packet size. */
			LWIP_ASSERT("gmac_rx_populate_queue: pbuf size too small!", pbuf_clen(p) <= 1);

			/* Make sure that the payload buffer is properly aligned. */
			LWIP_ASSERT("gmac_rx_populate_queue: unaligned p->payload buffer address",
					(((uint32_t)p->payload & 0xFFFFFFFC) == (uint32_t)p->payload));

			// dc42 do this first to avoid a race condition with DMA, because writing addr.val transfers ownership back to the GMAC, so it should be the last thing we do
			/* Reset status value. */
			p_gmac_dev->rx_desc[ul_index].status.val = 0;

			/* Save pbuf pointer to be sent to lwIP upper layer. */
			p_gmac_dev->rx_pbuf[ul_index] = p;

			if (ul_index == GMAC_RX_BUFFERS - 1)
			{
				p_gmac_dev->rx_desc[ul_index].addr.val = (u32_t) p->payload | GMAC_RXD_WRAP;
			}
			else
			{
				p_gmac_dev->rx_desc[ul_index].addr.val = (u32_t) p->payload;
			}

			LWIP_DEBUGF(NETIF_DEBUG,
					("gmac_rx_populate_queue: new pbuf allocated: %p [idx=%u]\n",
					p, (unsigned int)ul_index));
		}

		++ul_index;
		if (ul_index == GMAC_RX_BUFFERS)
		{
			ul_index = 0;
		}
	} while (ul_index != startAt);

	rxPbufsFullyPopulated = true;
}

/**
 * \brief Set up the RX descriptor ring buffers.
 *
 * This function sets up the descriptor list used for receive packets.
 *
 * \param ps_gmac_dev Pointer to driver data structure.
 */
static void gmac_rx_init(struct gmac_device *ps_gmac_dev) noexcept
{
	uint32_t ul_index = 0;

	/* Init RX index. */
	ps_gmac_dev->us_rx_idx = 0;

	/* Set up the RX descriptors. */
	for (ul_index = 0; ul_index < GMAC_RX_BUFFERS; ul_index++)
	{
		ps_gmac_dev->rx_pbuf[ul_index] = nullptr;
		ps_gmac_dev->rx_desc[ul_index].addr.val = GMAC_RXD_OWNERSHIP;		// mark it as not free for hardware to use, until we have allocated the pbuf
		ps_gmac_dev->rx_desc[ul_index].status.val = 0;
	}
	ps_gmac_dev->rx_desc[ul_index - 1].addr.val |= GMAC_RXD_WRAP;

	/* Build RX buffer and descriptors. */
	gmac_rx_populate_queue(ps_gmac_dev, 0);

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
static void gmac_tx_init(struct gmac_device *ps_gmac_dev) noexcept
{
	uint32_t ul_index;

	/* Init TX index pointer. */
	ps_gmac_dev->us_tx_idx = 0;

	/* Set up the TX descriptors. */
	for (ul_index = 0; ul_index < GMAC_TX_BUFFERS; ul_index++)
	{
		ps_gmac_dev->tx_desc[ul_index].addr = reinterpret_cast<uint32_t>(&ps_gmac_dev->tx_buf[ul_index][0]);
		ps_gmac_dev->tx_desc[ul_index].status.val = GMAC_TXD_USED | GMAC_TXD_LAST;
	}
	ps_gmac_dev->tx_desc[ul_index - 1].status.val |= GMAC_TXD_WRAP;

	/* Set transmit buffer queue base address pointer. */
	gmac_set_tx_queue(GMAC, (uint32_t) &ps_gmac_dev->tx_desc[0]);
}

/**
 * \brief Initialize GMAC and PHY.
 *
 * \note Called from ethernetif_init().
 *
 * \param netif the lwIP network interface structure for this ethernetif.
 */
static void gmac_low_level_init(struct netif *netif) noexcept
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
	if (ethernet_phy_init(GMAC, BOARD_GMAC_PHY_ADDR, SystemCoreClock) != GMAC_OK) {
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
static err_t gmac_low_level_output(netif *p_netif, struct pbuf *p) noexcept
{
	gmac_device *const ps_gmac_dev = static_cast<gmac_device *>(p_netif->state);

	/* Handle GMAC underrun or AHB errors. */
	if (gmac_get_tx_status(GMAC) & GMAC_TX_ERRORS)
	{
		++txErrorCount;
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

	while ((ps_gmac_dev->tx_desc[ps_gmac_dev->us_tx_idx].status.val & GMAC_TXD_USED) == 0)
	{
		++txBufferNotFreeCount;
		delay(1);
	}

	// Copy pbuf chain into TX buffer
	{
		uint8_t *buffer = reinterpret_cast<uint8_t*>(ps_gmac_dev->tx_desc[ps_gmac_dev->us_tx_idx].addr);
		size_t totalLength = 0;
		for (const pbuf *q = p; q != nullptr; q = q->next)
		{
			totalLength += q->len;
			if (totalLength > GMAC_TX_UNITSIZE)
			{
				++txBufferTooShortCount;
				return ERR_BUF;
			}
			memcpy(buffer, q->payload, q->len);
			buffer += q->len;
		}
	}

	// Set length and mark the buffer to be sent by GMAC
	uint32_t txStat = p->tot_len | GMAC_TXD_LAST;
	if (ps_gmac_dev->us_tx_idx == GMAC_TX_BUFFERS - 1)
	{
		txStat |= GMAC_TXD_WRAP;
	}
	ps_gmac_dev->tx_desc[ps_gmac_dev->us_tx_idx].status.val = txStat;

	LWIP_DEBUGF(NETIF_DEBUG,
			("gmac_low_level_output: DMA buffer sent, size=%d [idx=%u]\n",
			p->tot_len, (unsigned int)ps_gmac_dev->us_tx_idx));

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
static pbuf *gmac_low_level_input(struct netif *netif) noexcept
{
	gmac_device *ps_gmac_dev = static_cast<gmac_device *>(netif->state);

	if (gmac_get_rx_status(GMAC) & GMAC_RX_ERRORS)
	{
		// Handle GMAC overrun or AHB errors
		++rxErrorCount;
		gmac_enable_receive(GMAC, false);

		LINK_STATS_INC(link.err);
		LINK_STATS_INC(link.drop);

		/* Free all RX pbufs. */
		for (uint32_t ul_index = 0; ul_index < GMAC_RX_BUFFERS; ul_index++)
		{
			if (ps_gmac_dev->rx_pbuf[ul_index] != 0)
			{
				pbuf_free(ps_gmac_dev->rx_pbuf[ul_index]);
				ps_gmac_dev->rx_pbuf[ul_index] = nullptr;
			}
		}

		/* Reinit RX descriptors. */
		gmac_rx_init(ps_gmac_dev);

		/* Clear error status. */
		gmac_clear_rx_status(GMAC, GMAC_RX_ERRORS);

		gmac_enable_receive(GMAC, true);
		return nullptr;
	}

	volatile gmac_rx_descriptor_t * const p_rx = &ps_gmac_dev->rx_desc[ps_gmac_dev->us_rx_idx];
	pbuf * const p = ((p_rx->addr.val & GMAC_RXD_OWNERSHIP) == GMAC_RXD_OWNERSHIP)
						? ps_gmac_dev->rx_pbuf[ps_gmac_dev->us_rx_idx]
							: nullptr;

	/* Check if a packet has been received and processed by GMAC. */
	if (p != nullptr)
	{
		/* Packet is a SOF since packet size is set to maximum. */
		const uint32_t length = p_rx->status.val & GMAC_RXD_LEN_MASK;

		/* Fetch pre-allocated pbuf. */
		p->len = length;

		/* Remove this pbuf from its descriptor. */
		ps_gmac_dev->rx_pbuf[ps_gmac_dev->us_rx_idx] = nullptr;
		rxPbufsFullyPopulated = false;

		LWIP_DEBUGF(NETIF_DEBUG,
				("gmac_low_level_input: DMA buffer %p received, size=%u [idx=%u]\n",
				p, (unsigned int)length, (unsigned int)ps_gmac_dev->us_rx_idx));

		/* Set pbuf total packet size. */
		p->tot_len = length;
		LINK_STATS_INC(link.recv);

		ps_gmac_dev->us_rx_idx = (ps_gmac_dev->us_rx_idx + 1) % GMAC_RX_BUFFERS;

#if LWIP_STATS
		lwip_rx_count += length;
#endif
	}

	/* Fill empty descriptors with new pbufs. */
	if (!rxPbufsFullyPopulated)
	{
		gmac_rx_populate_queue(ps_gmac_dev, ps_gmac_dev->us_rx_idx);
	}

	return p;
}

/**
 * \brief GMAC task function. This function waits for the notification
 * semaphore from the interrupt, processes the incoming packet and then
 * passes it to the lwIP stack.
 *
 * \param pvParameters A pointer to the gmac_device instance.
 */
extern "C" [[noreturn]] void gmac_task(void *pvParameters) noexcept
{
	gmac_device * const ps_gmac_dev = static_cast<gmac_device*>(pvParameters);
	netif * const p_netif = ps_gmac_dev->netif;

	while (1)
	{
		// Process the incoming packets
		{
			MutexLocker lock(lwipMutex);
			while (ethernetif_input(p_netif)) { }
		}

		// Wait for the RX notification from the ISR
		TaskBase::Take((rxPbufsFullyPopulated) ? 1000 : 20);
	}
}

/**
 * \brief This function should be called when a packet is ready to be
 * read from the interface. It uses the function gmac_low_level_input()
 * that handles the actual reception of bytes from the network interface.
 * Then the type of the received packet is determined and the appropriate
 * input function is called.
 *
 * \param netif the lwIP network interface structure for this ethernetif.
 */
bool ethernetif_input(struct netif *netif) noexcept
{
	struct eth_hdr *ethhdr;
	struct pbuf *p;

	/* Move received packet into a new pbuf. */
	p = gmac_low_level_input(netif);
	if (p == nullptr)
	{
		return false;
	}

	/* Points to packet payload, which starts with an Ethernet header. */
	ethhdr = static_cast<struct eth_hdr*>(p->payload);

	switch (htons(ethhdr->type)) {
		case ETHTYPE_IP:
		case ETHTYPE_ARP:
#if defined(PPPOE_SUPPORT) && PPPOE_SUPPORT
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
err_t ethernetif_init(struct netif *netif) noexcept
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
#if defined(LWIP_SNMP) && LWIP_SNMP
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

	ethernetTask.Create(gmac_task, "ETHERNET", &gs_gmac_dev, TaskPriority::EthernetPriority);
	return ERR_OK;
}

void ethernetif_hardware_init() noexcept
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

bool ethernetif_establish_link() noexcept
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
bool ethernetif_link_established() noexcept
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

void ethernetif_set_mac_address(const uint8_t macAddress[]) noexcept
{
	// This function must be called once before low_level_init(), because that is where the
	// MAC address of the netif is assigned
	for (size_t i = 0; i < 6; ++i)
	{
		gs_uc_mac_address[i] = macAddress[i];
	}
}

// This is called when we shut down
void ethernetif_terminate() noexcept
{
	NVIC_DisableIRQ(GMAC_IRQn);
	ethernetTask.TerminateAndUnlink();
}

extern "C" u32_t sys_now() noexcept
{
	return millis();
}

// End
