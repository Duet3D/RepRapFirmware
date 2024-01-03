/*
 * GmacInterface.cpp
 *
 *  Created on: 19 Jun 2020
 *      Author: David
 */

#include <Core.h>
#include <Cache.h>
#include "GmacInterface.h"
#include "gmac.h"
#include <hri_mclk_e54.h>
#include <hri_gmac_e54.h>

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

#include <RepRapFirmware.h>
#include <RTOSIface/RTOSIface.h>
#include <Platform/TaskPriorities.h>
#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <AppNotifyIndices.h>

extern Mutex lwipMutex;

#if defined(LWIP_DEBUG)
constexpr size_t EthernetTaskStackWords = 700;
#else
constexpr size_t EthernetTaskStackWords = 300;
#endif

static Task<EthernetTaskStackWords> ethernetTask;

// Error counters
static unsigned int rxErrorCount = 0;
static unsigned int rxBuffersNotFullyPopulatedCount = 0;
static unsigned int rxBufferNotAvailableCount = 0;
static unsigned int txErrorCount = 0;
static unsigned int txBufferNotFreeCount = 0;
static unsigned int txBufferTooShortCount = 0;

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
#define GMAC_INT_GROUP (GMAC_ISR_RCOMP | GMAC_ISR_ROVR)

/** The GMAC TX errors to handle */
#define GMAC_TX_ERRORS (GMAC_TSR_TFC | GMAC_TSR_HRESP | GMAC_TSR_UND)

/** The GMAC RX errors to handle */
#define GMAC_RX_ERRORS (GMAC_RSR_RXOVR | GMAC_RSR_HNO)

/**
 * GMAC driver structure.
 */
struct alignas(8) gmac_device {
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
	alignas(8) uint8_t tx_buf[GMAC_TX_BUFFERS][(GMAC_TX_UNITSIZE + 3u) & (~3u)];

	/** RX index for current processing TD. */
	uint32_t us_rx_idx;
	/** Circular buffer head pointer by upper layer (buffer to be sent). */
	uint32_t us_tx_idx;

	/** Reference to lwIP netif structure. */
	struct netif *netif;

	bool rxPbufsFullyPopulated = false;
};

/**
 * GMAC driver instance.
 */
__nocache alignas(8) static struct gmac_device gs_gmac_dev;

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

// GMAC interrupt handler
// At present, we only use receive interrupts
extern "C" void GMAC_Handler() noexcept
{
	/* Get interrupt status. */
	const uint32_t ul_isr = gmac_get_interrupt_status(GMAC);

	/* RX interrupts. */
	if (ul_isr & GMAC_INT_GROUP)
	{
		ethernetTask.GiveFromISR(NotifyIndices::EthernetHardware);
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
			pbuf * const p = pbuf_alloc(PBUF_RAW, (u16_t) GMAC_FRAME_LENGTH_MAX, PBUF_POOL);
			if (p == nullptr)
			{
				LWIP_DEBUGF(NETIF_DEBUG, ("gmac_rx_populate_queue: pbuf allocation failure\n"));
				p_gmac_dev->rxPbufsFullyPopulated = false;
				++rxBuffersNotFullyPopulatedCount;
				return;
			}

			/* Make sure lwIP is well configured so one pbuf can contain the maximum packet size. */
			LWIP_ASSERT("gmac_rx_populate_queue: pbuf size too small!", pbuf_clen(p) <= 1);

			/* Make sure that the payload buffer is properly aligned. */
			LWIP_ASSERT("gmac_rx_populate_queue: unaligned p->payload buffer address", (((uint32_t)p->payload & 0xFFFFFFFC) == (uint32_t)p->payload));

			/* Save pbuf pointer to be sent to lwIP upper layer. */
			p_gmac_dev->rx_pbuf[ul_index] = p;

			// dc42 do this first to avoid a race condition with DMA, because writing addr.val transfers ownership back to the GMAC, so it should be the last thing we do
			p_gmac_dev->rx_desc[ul_index].status.val = 0;				// reset status value

			__DSB();
			p_gmac_dev->rx_desc[ul_index].addr.val =  (ul_index == GMAC_RX_BUFFERS - 1) ? (u32_t) p->payload | GMAC_RXD_WRAP : (u32_t) p->payload;
			Cache::FlushBeforeDMASend(&p_gmac_dev->rx_desc[ul_index], sizeof(p_gmac_dev->rx_desc[ul_index]));
			LWIP_DEBUGF(NETIF_DEBUG, ("gmac_rx_populate_queue: new pbuf allocated: %p [idx=%u]\n", p, (unsigned int)ul_index));
		}

		++ul_index;
		if (ul_index == GMAC_RX_BUFFERS)
		{
			ul_index = 0;
		}
	} while (ul_index != startAt);

	p_gmac_dev->rxPbufsFullyPopulated = true;
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
	Cache::FlushBeforeDMASend(ps_gmac_dev->tx_desc, sizeof(ps_gmac_dev->tx_desc));

	/* Set transmit buffer queue base address pointer. */
	gmac_set_tx_queue(GMAC, (uint32_t) &ps_gmac_dev->tx_desc[0]);

	/* Clear error status. */
	gmac_clear_tx_status(GMAC, GMAC_TX_ERRORS);
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
	netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_IGMP;

	/* Init MAC PHY driver. */
	if (ethernet_phy_init(GMAC, BOARD_GMAC_PHY_ADDR, SystemCoreClockFreq) != GMAC_OK)
	{
		LWIP_DEBUGF(NETIF_DEBUG, ("gmac_low_level_init: PHY init ERROR!\n"));
		return;
	}
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

	while (true)
	{
		// Handle GMAC underrun or AHB errors
		if (gmac_get_tx_status(GMAC) & GMAC_TX_ERRORS)
		{
			++txErrorCount;
			LWIP_DEBUGF(NETIF_DEBUG, ("gmac_low_level_output: GMAC ERROR, reinit TX...\n"));

			gmac_enable_transmit(GMAC, false);

			LINK_STATS_INC(link.err);
			LINK_STATS_INC(link.drop);

			/* Reinit TX descriptors. */
			gmac_tx_init(ps_gmac_dev);			// this also clears the Tx errors

			gmac_enable_transmit(GMAC, true);
		}

		volatile gmac_tx_descriptor_t& txDescriptor = ps_gmac_dev->tx_desc[ps_gmac_dev->us_tx_idx];
		Cache::InvalidateAfterDMAReceive(&txDescriptor, sizeof(gmac_tx_descriptor_t));
		if ((txDescriptor.status.val & GMAC_TXD_USED) != 0)
		{
			// Copy pbuf chain into TX buffer
			uint8_t *buffer = reinterpret_cast<uint8_t*>(txDescriptor.addr);
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
			Cache::FlushBeforeDMASend(reinterpret_cast<const uint8_t*>(txDescriptor.addr), totalLength);

			// Set length and mark the buffer to be sent by GMAC
			uint32_t txStat = totalLength | GMAC_TXD_LAST;
			if (ps_gmac_dev->us_tx_idx == GMAC_TX_BUFFERS - 1)
			{
				txStat |= GMAC_TXD_WRAP;
			}
			txDescriptor.status.val = txStat;
			Cache::FlushBeforeDMASend(&txDescriptor, sizeof(gmac_tx_descriptor_t));
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

		++txBufferNotFreeCount;
		delay(2);	//TODO use an interrupt instead
	}
}

/**
 * \brief Use pre-allocated pbuf as DMA source and return the incoming packet.
 *
 * \param netif the lwIP network interface structure for this ethernetif.
 *
 * \return a pbuf filled with the received packet (including MAC header).
 * nullptr if no received packet available.
 */
static pbuf *gmac_low_level_input(struct netif *netif) noexcept
{
	gmac_device *const ps_gmac_dev = static_cast<gmac_device *>(netif->state);

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

	if (gmac_get_rx_status(GMAC) & GMAC_RSR_BNA)
	{
		++rxBufferNotAvailableCount;
		gmac_clear_rx_status(GMAC, GMAC_RSR_BNA);
	}

	/* Check if a packet has been received and processed by GMAC. */
	uint32_t rxIdx = ps_gmac_dev->us_rx_idx;
	volatile gmac_rx_descriptor_t * const p_rx = &ps_gmac_dev->rx_desc[rxIdx];
	Cache::InvalidateAfterDMAReceive(p_rx, sizeof(gmac_rx_descriptor_t));

	pbuf * p = ((p_rx->addr.val & GMAC_RXD_OWNERSHIP) != 0)
					? ps_gmac_dev->rx_pbuf[rxIdx]
							: nullptr;

	/* Check if a packet has been received and processed by GMAC. */
	if (p != nullptr)
	{
		const uint32_t status = p_rx->status.val;
		const uint32_t length = status & GMAC_RXD_LEN_MASK;

		/* Fetch pre-allocated pbuf. */
		Cache::InvalidateAfterDMAReceive(p->payload, length);

		/* Remove this pbuf from its descriptor. */
		ps_gmac_dev->rx_pbuf[rxIdx] = nullptr;
		ps_gmac_dev->rxPbufsFullyPopulated = false;
		p->tot_len = p->len = length;
		LINK_STATS_INC(link.recv);

		ps_gmac_dev->us_rx_idx = rxIdx = (rxIdx + 1) % GMAC_RX_BUFFERS;

		if ((status & (GMAC_RXD_SOF | GMAC_RXD_EOF)) == (GMAC_RXD_SOF | GMAC_RXD_EOF))
		{
#if LWIP_STATS
			lwip_rx_count += length;
#endif
		}
		else
		{
			pbuf_free(p);
			p = nullptr;
		}
	}

	/* Fill empty descriptors with new pbufs. */
	if (!ps_gmac_dev->rxPbufsFullyPopulated)
	{
		gmac_rx_populate_queue(ps_gmac_dev, rxIdx);
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
		TaskBase::TakeIndexed(NotifyIndices::EthernetHardware, (ps_gmac_dev->rxPbufsFullyPopulated) ? 1000 : 20);
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
	/* Move received packet into a new pbuf. */
	pbuf *const p = gmac_low_level_input(netif);
	if (p == nullptr)
	{
		return false;
	}

	/* Points to packet payload, which starts with an Ethernet header. */
	const eth_hdr *ethhdr = static_cast<struct eth_hdr*>(p->payload);

	switch (lwip_htons(ethhdr->type))
	{
		case ETHTYPE_IP:
		case ETHTYPE_ARP:
#if defined(PPPOE_SUPPORT) && PPPOE_SUPPORT
		case ETHTYPE_PPPOEDISC:
		case ETHTYPE_PPPOE:
#endif /* PPPOE_SUPPORT */
			/* Send packet to lwIP for processing. */
			if (netif->input(p, netif) != ERR_OK)
			{
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

	/* Set up the interrupts for transmission and errors. */
	gmac_enable_interrupt(GMAC, GMAC_INT_GROUP);

	/* Enable NVIC GMAC interrupt. */
	NVIC_ClearPendingIRQ(GMAC_IRQn);
	NVIC_EnableIRQ(GMAC_IRQn);

	return ERR_OK;
}

// GMAC configuration

// <o> MDC Clock Division
// <i> Set according to MCK speed. These three bits determine the number MCK
// <i> will be divided by to generate Management Data Clock (MDC). For
// <i> conformance with the 802.3 specification, MDC must not exceed 2.5 MHz
// <i> (MDC is only active during MDIO read and write operations).
// <0=> 8
// <1=> 16
// <2=> 32
// <3=> 48
// <4=> 64
// <5=> 96
// <id> gmac_arch_ncfgr_clk
#define CONF_GMAC_NCFGR_CLK 4

/**
 * For conformance with the 802.3 specification, MDC must not exceed 2.5 MHz
 **/
#define CONF_GMAC_FREQUENCY		120000000
#ifndef CONF_GMAC_MCK_FREQUENCY
#if CONF_GMAC_NCFGR_CLK == 0
#define CONF_GMAC_MCK_FREQUENCY (CONF_GMAC_FREQUENCY / 8)
#elif CONF_GMAC_NCFGR_CLK == 1
#define CONF_GMAC_MCK_FREQUENCY (CONF_GMAC_FREQUENCY / 16)
#elif CONF_GMAC_NCFGR_CLK == 2
#define CONF_GMAC_MCK_FREQUENCY (CONF_GMAC_FREQUENCY / 32)
#elif CONF_GMAC_NCFGR_CLK == 3
#define CONF_GMAC_MCK_FREQUENCY (CONF_GMAC_FREQUENCY / 48)
#elif CONF_GMAC_NCFGR_CLK == 4
#define CONF_GMAC_MCK_FREQUENCY (CONF_GMAC_FREQUENCY / 64)
#elif CONF_GMAC_NCFGR_CLK == 5
#define CONF_GMAC_MCK_FREQUENCY (CONF_GMAC_FREQUENCY / 96)
#endif
#endif

#if CONF_GMAC_MCK_FREQUENCY > 2500000
#warning For conformance with the 802.3 specification, MDC must not exceed 2.5 MHz
#endif

// <o> Fixed Burst Length for DMA Data Operations
// <i> Selects the burst length to attempt to use on the AHB when transferring
// <i> frame data. Not used for DMA management operations and only used where
// <i> space and data size allow. Otherwise SINGLE type AHB transfers are used.
// <1=> Always use SINGLE AHB bursts
// <4=> Always use INCR4 AHB bursts
// <8=> Always use INCR8 AHB bursts
// <16=> Always use INCR16 AHB bursts
// <id> gmac_arch_dcfgr_fbldo
#define CONF_GMAC_DCFGR_FBLDO 4

// <o> Receiver Packet Buffer Memory Size Select
// <i> Select the receive packet buffer size
// <0=> 0.5 Kbytes
// <1=> 1 Kbytes
// <2=> 2 Kbytes
// <3=> 4 Kbytes
// <id> gmac_arch_dcfgr_rxbms
#define CONF_GMAC_DCFGR_RXBMS 3

// <o> DMA Receive Buffer Size <1-255>
// <i> DMA receive buffer size in AHB system memory. The value defined by these
// <i> bits determines the size of buffer to use in main AHB system memory when
// <i> writing received data. The value is defined in multiples of 64 bytes,
// <i> thus a value of 0x01 corresponds to buffers of 64 bytes, 0x02
// <i> corresponds to 128 bytes etc.
// <id> gmac_arch_dcfgr_drbs
#define CONF_GMAC_DCFGR_DRBS 0x18

// <o> IPG Stretch Multiple <0-15>
// <i> This value will multiplied with the previously transmitted frame length
// <i> (including preamble)
// <id> gmac_arch_ipgs_fl_mul
#define CONF_GMAC_IPGS_FL_MUL 1

// <o> IPG Stretch Divide <1-16>
// <i> Divide the frame length. If the resulting number is greater than 96 and
// <i> IP Stretch Enabled then the resulting number is used for the transmit
// <i> inter-packet-gap
// <id> gmac_arch_ipgs_fl_div
#define CONF_GMAC_IPGS_FL_DIV 1

// <<< end of configuration section >>>

// Initialise the GMAC and Phy
void ethernetif_hardware_init() noexcept
{
	// Set up PHY clock
	// GCLK2: XOSC1 direct, 25MHz output for Ethernet PHY
	ConfigureGclk(GclkNumEthernetPhy, GclkSource::xosc1, 1, true);
	SetPinFunction(EthernetClockOutPin, EthernetClockOutPinFunction);

	// Set up GMAC clock
	hri_mclk_set_AHBMASK_GMAC_bit(MCLK);
	hri_mclk_set_APBCMASK_GMAC_bit(MCLK);

	// Setup Ethernet pins
	for (Pin p : EthernetMacPins)
	{
		SetPinFunction(p, EthernetMacPinsPinFunction);
	}

	hri_gmac_write_NCR_reg(GMAC, GMAC_NCR_MPE);
	hri_gmac_write_NCFGR_reg(GMAC, GMAC_NCFGR_SPD | GMAC_NCFGR_FD | GMAC_NCFGR_MAXFS | GMAC_NCFGR_CLK(CONF_GMAC_NCFGR_CLK));
	hri_gmac_write_UR_reg(GMAC, 0);
	hri_gmac_write_DCFGR_reg(GMAC, GMAC_DCFGR_FBLDO(CONF_GMAC_DCFGR_FBLDO) | GMAC_DCFGR_RXBMS(CONF_GMAC_DCFGR_RXBMS) | GMAC_DCFGR_TXPBMS | GMAC_DCFGR_DRBS(CONF_GMAC_DCFGR_DRBS));
	hri_gmac_write_WOL_reg(GMAC, 0);
	hri_gmac_write_IPGS_reg(GMAC, GMAC_IPGS_FL((CONF_GMAC_IPGS_FL_MUL << 8) | CONF_GMAC_IPGS_FL_DIV));

	/* Disable TX & RX and more. */
	gmac_disable_interrupt(GMAC, ~0u);

	/* Enable the copy of data into the buffers ignore broadcasts, and not copy FCS. */
	gmac_enable_copy_all(GMAC, false);
	gmac_disable_broadcast(GMAC, false);
	GMAC->NCR.reg |= GMAC_NCFGR_RXCOEN;			// check IP, UDP and TCP checksums so that we don't need to do it in lwip

#if SUPPORT_MULTICAST_DISCOVERY
	// Without this code, we don't receive any multicast packets
	GMAC->NCR.reg |= GMAC_NCFGR_MTIHEN;			// enable multicast hash reception
	GMAC->HRB.reg = 0xFFFFFFFF;					// enable reception of all multicast frames
	GMAC->HRT.reg = 0xFFFFFFFF;
#endif

	/* Set RX buffer size to 1536. */
	gmac_set_rx_bufsize(GMAC, 0x18);

	/* Clear all status bits in the receive status register. */
	gmac_clear_rx_status(GMAC, GMAC_RSR_BNA | GMAC_RSR_REC | GMAC_RSR_RXOVR | GMAC_RSR_HNO);

	/* Clear all status bits in the transmit status register. */
	gmac_clear_tx_status(GMAC, GMAC_TSR_UBR | GMAC_TSR_COL | GMAC_TSR_RLE | GMAC_TSR_TXGO | GMAC_TSR_TFC | GMAC_TSR_TXCOMP | GMAC_TSR_HRESP);

	/* Clear interrupts. */
	gmac_get_interrupt_status(GMAC);

	gmac_rx_init(&gs_gmac_dev);
	gmac_tx_init(&gs_gmac_dev);

	/* Enable Rx, Tx and the statistics register. */
	gmac_enable_transmit(GMAC, true);
	gmac_enable_receive(GMAC, true);
	gmac_enable_statistics_write(GMAC, true);

	/* Set GMAC address. */
	gmac_set_address(GMAC, 0, gs_uc_mac_address);
}

bool ethernetif_establish_link() noexcept
{
	/* Auto Negotiate, work in RMII mode. */
	const uint8_t result = ethernet_phy_auto_negotiate(GMAC, BOARD_GMAC_PHY_ADDR);
	if (result != GMAC_OK)
	{
		if (result != GMAC_TIMEOUT)
		{
			// chrishamm: It is expected that the function above will return ERR_TIMEOUT a few times
			LWIP_DEBUGF(NETIF_DEBUG, ("gmac_low_level_init: auto negotiate ERROR!\n"));
		}
		return false;
	}

	return true;
}

// Ask the PHY if the link is still up
bool ethernetif_link_established() noexcept
{
	gmac_enable_management(GMAC, true);

	uint32_t ul_stat1;
	if (gmac_phy_read(GMAC, BOARD_GMAC_PHY_ADDR, GMII_BMSR, &ul_stat1) != GMAC_OK)
	{
		gmac_enable_management(GMAC, false);
		return false;
	}

	if ((ul_stat1 & GMII_LINK_STATUS) == 0)
	{
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
	gmac_enable_transmit(GMAC, false);
	gmac_enable_receive(GMAC, false);
	NVIC_DisableIRQ(GMAC_IRQn);
	ethernetTask.TerminateAndUnlink();
}

extern "C" uint32_t sys_now() noexcept
{
	return millis();
}

void ethernetif_diagnostics(MessageType mtype) noexcept
{
	reprap.GetPlatform().MessageF(mtype, "Error counts: %u %u %u %u %u %u\nSocket states:",
								rxErrorCount, rxBuffersNotFullyPopulatedCount, rxBufferNotAvailableCount, txErrorCount, txBufferNotFreeCount, txBufferTooShortCount);
	rxErrorCount = rxBuffersNotFullyPopulatedCount = rxBufferNotAvailableCount = txErrorCount = txBufferNotFreeCount = txBufferTooShortCount = 0;
}

// End
