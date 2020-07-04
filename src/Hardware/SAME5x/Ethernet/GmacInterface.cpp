/*
 * GmacInterface.cpp
 *
 *  Created on: 19 Jun 2020
 *      Author: David
 */

#include <Core.h>
#include <Hardware/Cache.h>
#include "GmacInterface.h"
#include "gmac.h"

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
#include <TaskPriorities.h>

extern Mutex lwipMutex;

#if defined(LWIP_DEBUG)
constexpr size_t EthernetTaskStackWords = 700;
#else
constexpr size_t EthernetTaskStackWords = 250;
#endif

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

	/** RX index for current processing TD. */
	uint32_t us_rx_idx;
	/** Circular buffer head pointer by upper layer (buffer to be sent). */
	uint32_t us_tx_idx;

	bool rxPbufsFullyPopulated = false;

	/** Reference to lwIP netif structure. */
	struct netif *netif;

	/** RX pbuf pointer list. */
	struct pbuf *rx_pbuf[GMAC_RX_BUFFERS];

	/** TX buffers. */
	alignas(8) uint8_t tx_buf[GMAC_TX_BUFFERS][(GMAC_TX_UNITSIZE + 3u) & (~3u)];
};

/**
 * GMAC driver instance.
 */
__nocache static struct gmac_device gs_gmac_dev;

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

/**
 * \brief GMAC interrupt handler.
 */
extern "C" void GMAC_Handler() noexcept
{
#if 1
	uint32_t tsr = hri_gmac_read_TSR_reg(GMAC);
	uint32_t rsr = hri_gmac_read_RSR_reg(GMAC);

	/* Must be Clear ISR (Clear on read) */
	hri_gmac_read_ISR_reg(GMAC);

	/* Frame transmited */
	if (tsr & GMAC_TSR_TXCOMP)
	{
		hri_gmac_write_TSR_reg(GMAC, tsr);
//		if ((_txbuf_descrs[_txbuf_index].status.bm.used) && (_gmac_dev->cb.transmited != NULL)) {
//			_gmac_dev->cb.transmited(_gmac_dev);
//		}
	}

	/* Frame received */
	if (rsr & GMAC_RSR_REC)
	{
		ethernetTask.GiveFromISR();
	}
	hri_gmac_write_RSR_reg(GMAC, rsr);
#else
	/* Get interrupt status. */
	const uint32_t ul_isr = gmac_get_interrupt_status(GMAC);

	/* RX interrupts. */
	if (ul_isr & GMAC_INT_GROUP)
	{
		ethernetTask.GiveFromISR();
	}
#endif
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
				p_gmac_dev->rxPbufsFullyPopulated = false;
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
	if (ethernet_phy_init(GMAC, BOARD_GMAC_PHY_ADDR, SystemCoreClock) != GMAC_OK)
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

#include <General/Portability.h>
static err_t gmac_low_level_output(netif *p_netif, struct pbuf *p) noexcept
{
#if 0
	debugPrintf("%u %u %" PRIu32 "\n",
				LoadBE16((const uint8_t*)p->payload + 0x24),			// destination port
				LoadBE16((const uint8_t*)p->payload + 0x10),			// length
				LoadBE32((const uint8_t*)p->payload + 0x26)				// sequence number
		);
#endif
	gmac_device *const ps_gmac_dev = static_cast<gmac_device *>(p_netif->state);

	while (true)
	{
		{	// Start locked region
			MutexLocker lock(lwipMutex);

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
		}				// end locked region

		++txBufferNotFreeCount;
		delay(2);
	}
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
	Cache::InvalidateAfterDMAReceive(p_rx, sizeof(gmac_rx_descriptor_t));

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
		ps_gmac_dev->rxPbufsFullyPopulated = false;
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
	if (!ps_gmac_dev->rxPbufsFullyPopulated)
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
		TaskBase::Take((ps_gmac_dev->rxPbufsFullyPopulated) ? 1000 : 20);
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

	/* Set up the interrupts for transmission and errors. */
	gmac_enable_interrupt(GMAC, GMAC_INT_GROUP);

	/* Enable NVIC GMAC interrupt. */
	NVIC_ClearPendingIRQ(GMAC_IRQn);
	NVIC_EnableIRQ(GMAC_IRQn);

	return ERR_OK;
}

/* Auto-generated config file hpl_gmac_config.h */

// <<< Use Configuration Wizard in Context Menu >>>

#include <peripheral_clk_config.h>

// <h> Network Control configuration

// <q> Enable LoopBack Local
// <i> Connects GTX to GRX, GTXEN to GRXDV and forces full duplex mode.
// <id> gmac_arch_ncr_lbl
#ifndef CONF_GMAC_NCR_LBL
#define CONF_GMAC_NCR_LBL 0
#endif

// <q> Management Port Enable
// <i> Enable the Management port
// <id> gmac_arch_ncr_mpe
#ifndef CONF_GMAC_NCR_MPE
#define CONF_GMAC_NCR_MPE 1
#endif

// <q> Enable write for Static Register
// <i> Make the statistics registers writable for functional test proposes.
// <id> gmac_arch_ncr_westat
#ifndef CONF_GMAC_NCR_WESTAT
#define CONF_GMAC_NCR_WESTAT 0
#endif

// <q> Enable Back pressure
// <i> If set in 10M or 100M half duplex mode, forces collisions on all received frames.
// <id> gmac_arch_ncr_bp
#ifndef CONF_GMAC_NCR_BP
#define CONF_GMAC_NCR_BP 0
#endif

// <q> Enable PFC Priority-based Pause Reception
// <i> Enables PFC negotiation and recognition of priority-based pause frames.
// <id> gmac_arch_ncr_enpbpr
#ifndef CONF_GMAC_NCR_ENPBPR
#define CONF_GMAC_NCR_ENPBPR 0
#endif

// <q> Enable PFC Priority-based Pause Frame
// <i> Takes the values stored in the Transmit PFC Pause Register.
// <id> gmac_arch_ncr_txpbpf
#ifndef CONF_GMAC_NCR_TXPBPF
#define CONF_GMAC_NCR_TXPBPF 0
#endif

// </h>

// <h> Network Configuration

// <q> 100Mbps Speed
// <i> Set to one to indicate 100 Mbps operation, zero for 10 Mbps.
// <id> gmac_arch_ncfgr_spd
#ifndef CONF_GMAC_NCFGR_SPD
#define CONF_GMAC_NCFGR_SPD 1
#endif

// <q> Enable Full Duplex
// <i> Enable Full duplex
// <id> gmac_arch_ncfgr_df
#ifndef CONF_GMAC_NCFGR_FD
#define CONF_GMAC_NCFGR_FD 1
#endif

// <q> Discard Non-VLAN Frames
// <i> Discard Non-VLAN Frames
// <id> gmac_arch_ncfgr_dnvlan
#ifndef CONF_GMAC_NCFGR_DNVLAN
#define CONF_GMAC_NCFGR_DNVLAN 0
#endif

// <q> Enable Jumbo Frame
// <i> Enable jumbo frames up to 10240 bytes to be accepted.
// <id> gmac_arch_ncfgr_jframe
#ifndef CONF_GMAC_NCFGR_JFRAME
#define CONF_GMAC_NCFGR_JFRAME 0
#endif

// <q> Copy All Frames
// <i> All valid frames will be accepted
// <id> gmac_arch_ncfgr_caf
#ifndef CONF_GMAC_NCFGR_CAF
#define CONF_GMAC_NCFGR_CAF 0
#endif

// <q> No broadcast
// <i> Frames addressed to the broadcast address of all ones will not be accepted.
// <id> gmac_arch_ncfgr_nbc
#ifndef CONF_GMAC_NCFGR_NBC
#define CONF_GMAC_NCFGR_NBC 0
#endif

// <q> Multicast Hash Enable
// <i> Multicast frames will be accepted when the 6-bit hash function of the destination address points to a bit that is set in the Hash Register.
// <id> gmac_arch_ncfgr_mtihen
#ifndef CONF_GMAC_NCFGR_MTIHEN
#define CONF_GMAC_NCFGR_MTIHEN 0
#endif

// <q> Unicast Hash Enable
// <i> Unicast frames will be accepted when the 6-bit hash function of the destination address points to a bit that is set in the Hash Register.
// <id> gmac_arch_ncfgr_unihen
#ifndef CONF_GMAC_NCFGR_UNIHEN
#define CONF_GMAC_NCFGR_UNIHEN 0
#endif

// <q> 1536 Maximum Frame Size
// <i> Accept frames up to 1536 bytes in length.
// <id> gmac_arch_ncfgr_maxfs
#ifndef CONF_GMAC_NCFGR_MAXFS
#define CONF_GMAC_NCFGR_MAXFS 1
#endif

// <q> Retry Test
// <i> Must be set to zero for normal operation. If set to one the backoff
// <i> between collisions will always be one slot time. Setting this bit to
// <i> one helps test the too many retries condition. Also used in the pause
// <i> frame tests to reduce the pause counter's decrement time from 512 bit
// <i> times, to every GRXCK cycle.
// <id> gmac_arch_ncfgr_rty
#ifndef CONF_GMAC_NCFGR_RTY
#define CONF_GMAC_NCFGR_RTY 0
#endif

// <q> Pause Enable
// <i> When set, transmission will pause if a non-zero 802.3 classic pause
// <i> frame is received and PFC has not been negotiated
// <id> gmac_arch_ncfgr_pen
#ifndef CONF_GMAC_NCFGR_PEN
#define CONF_GMAC_NCFGR_PEN 0
#endif

// <o> Receive Buffer Offset <0-3>
// <i> Indicates the number of bytes by which the received data is offset from
// <i> the start of the receive buffer.
// <id> gmac_arch_ncfgr_rxbufo
#ifndef CONF_GMAC_NCFGR_RXBUFO
#define CONF_GMAC_NCFGR_RXBUFO 0
#endif

// <q> Length Field Error Frame Discard
// <i> Setting this bit causes frames with a measured length shorter than the
// <i> extracted length field (as indicated by bytes 13 and 14 in a non-VLAN
// <i> tagged frame) to be discarded. This only applies to frames with a length
// <i> field less than 0x0600.
// <id> gmac_arch_ncfgr_lferd
#ifndef CONF_GMAC_NCFGR_LFERD
#define CONF_GMAC_NCFGR_LFERD 0
#endif

// <q> Remove FCS
// <i> Setting this bit will cause received frames to be written to memory
// <i> without their frame check sequence (last 4 bytes). The frame length
// <i> indicated will be reduced by four bytes in this mode.
// <id> gmac_arch_ncfgr_rfcs
#ifndef CONF_GMAC_NCFGR_RFCS
#define CONF_GMAC_NCFGR_RFCS 0
#endif

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
#ifndef CONF_GMAC_NCFGR_CLK
#define CONF_GMAC_NCFGR_CLK 4
#endif

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
// <q> Disable Copy of Pause Frames
// <i> Set to one to prevent valid pause frames being copied to memory. When
// <i> set, pause frames are not copied to memory regardless of the state of
// <i> the Copy All Frames bit, whether a hash match is found or whether a
// <i> type ID match is identified. If a destination address match is found,
// <i> the pause frame will be copied to memory. Note that valid pause frames
// <i> received will still increment pause statistics and pause the
// <i> transmission of frames as required.
// <id> gmac_arch_ncfgr_dcpf
#ifndef CONF_GMAC_NCFGR_DCPF
#define CONF_GMAC_NCFGR_DCPF 0
#endif

// <q> Receive Checksum Offload Enable
// <i> When set, the receive checksum engine is enabled. Frames with bad IP,
// <i> TCP or UDP checksums are discarded.
// <id> gmac_arch_ncfgr_rxcoen
#ifndef CONF_GMAC_NCFGR_RXCOEN
#define CONF_GMAC_NCFGR_RXCOEN 0
#endif

// <q> Enable Frames Received in Half Duplex
// <i> Enable frames to be received in half-duplex mode while transmittinga.
// <id> gmac_arch_ncfgr_efrhd
#ifndef CONF_GMAC_NCFGR_EFRHD
#define CONF_GMAC_NCFGR_EFRHD 0
#endif

// <q> Ignore RX FCS
// <i> When set, frames with FCS/CRC errors will not be rejected. FCS error
// <i> statistics will still be collected for frames with bad FCS and FCS
// <i> status will be recorded in frame's DMA descriptor. For normal operation
// <i> this bit must be set to zero.
// <id> gmac_arch_ncfgr_irxfcs
#ifndef CONF_GMAC_NCFGR_IRXFCS
#define CONF_GMAC_NCFGR_IRXFCS 0
#endif

// <q> IP Stretch Enable
// <i> When set, the transmit IPG can be increased above 96 bit times depending
// <i> on the previous frame length using the IPG Stretch Register.
// <id> gmac_arch_ncfgr_ipgsen
#ifndef CONF_GMAC_NCFGR_IPGSEN
#define CONF_GMAC_NCFGR_IPGSEN 0
#endif

// <q> Receive Bad Preamble
// <i> When set, frames with non-standard preamble are not rejected.
// <id> gmac_arch_ncfgr_rxbp
#ifndef CONF_GMAC_NCFGR_RXBP
#define CONF_GMAC_NCFGR_RXBP 0
#endif

// <q> Ignore IPG GRXER
// <i> When set, GRXER has no effect on the GMAC's operation when GRXDV is low.
// <id> gmac_arch_ncfgr_irxer
#ifndef CONF_GMAC_NCFGR_IRXER
#define CONF_GMAC_NCFGR_IRXER 0
#endif

// </h>

// <e> MII Configuration
// <id> gmac_arch_mii_cfg

// <o> MII Mode
// <i> Select MII or RMII mode
// <0=> RMII
// <1=> MII
// <id> gmac_arch_ur_mii
#ifndef CONF_GMAC_ur_mii
#define CONF_GMAC_UR_MII 0
#endif

// <o> PHY Clause Operation
// <i> Chose which Clause operation will be used
// <0=>Clause 45 Operation
// <1=>Clause 22 Operation
// <id> gmac_arch_cltto
#ifndef CONF_GMAC_CLTTO
#define CONF_GMAC_CLTTO 1
#endif

// </e>

// <e> Stacked VLAN Processing
// <i> When enabled, the first VLAN tag in a received frame will only be
// <i> accepted if the VLAN type field is equal to the User defined VLAN Type,
// <i> OR equal to the standard VLAN type (0x8100). Note that the second VLAN
// <i> tag of a Stacked VLAN packet will only be matched correctly if its
// <i> VLAN_TYPE field equals 0x8100.
// <id> gmac_arch_svlan_enable
#ifndef CONF_GMAC_SVLAN_ENABLE
#define CONF_GMAC_SVLAN_ENABLE 0
#endif

// <o> User Defined VLAN Type <0x0-0xFFFF>
// <i> User defined VLAN TYPE
// <id> gmac_arch_svlan_type
#ifndef CONF_GMAC_SVLAN_TYPE
#define CONF_GMAC_SVLAN_TYPE 0x8100
#endif
// </e>

// <e> DMA Configuration
// <i> The GMAC DMA controller is connected to the MAC FIFO interface and
// <i> provides a scatter-gather type capability for packet data storage.
// <i> The DMA implements packet buffering where dual-port memories are used
// <i> to buffer multiple frames.
// <id> gmac_arch_dma_cfg
#ifndef CONF_GMAC_DMA_CFG
#define CONF_GMAC_DMACFG 1
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
#ifndef CONF_GMAC_DCFGR_FBLDO
#define CONF_GMAC_DCFGR_FBLDO 4
#endif

// <q> Endian Swap Mode Enable for Management Descriptor Accesses
// <i> When set, selects swapped endianism for AHB transfers. When clear,
// <i> selects little endian mode.
// <id> gmac_arch_dcfgr_esma
#ifndef CONF_GMAC_DCFGR_ESMA
#define CONF_GMAC_DCFGR_ESMA 0
#endif

// <q> Endian Swap Mode Enable for Packet Data Accesses
// <i> When set, selects swapped endianism for AHB transfers. When clear,
// <i> selects little endian mode.
// <id> gmac_arch_dcfgr_espa
#ifndef CONF_GMAC_DCFGR_ESPA
#define CONF_GMAC_DCFGR_ESPA 0
#endif

// <o> Receiver Packet Buffer Memory Size Select
// <i> Select the receive packet buffer size
// <0=> 0.5 Kbytes
// <1=> 1 Kbytes
// <2=> 2 Kbytes
// <3=> 4 Kbytes
// <id> gmac_arch_dcfgr_rxbms
#ifndef CONF_GMAC_DCFGR_RXBMS
#define CONF_GMAC_DCFGR_RXBMS 3
#endif

// <o> Transmitter Packet Buffer Memory Size Select
// <i> Select the Transmitter packet buffer size
// <0=> 2 Kbytes
// <1=> 4 Kbytes
// <id> gmac_arch_dcfgr_txpbms
#ifndef CONF_GMAC_DCFGR_TXPBMS
#define CONF_GMAC_DCFGR_TXPBMS 1
#endif

// <q> Transmitter Checksum Generation Offload Enable
// <i> Transmitter IP, TCP and UDP checksum generation offload enable. When
// <i> set, the transmitter checksum generation engine is enabled to calculate
// <i> and substitute checksums for transmit frames. When clear, frame data is
// <i> unaffected
// <id> gmac_arch_dcfgr_txcoen
#ifndef CONF_GMAC_DCFGR_TXCOEN
#define CONF_GMAC_DCFGR_TXCOEN 0
#endif

// <o> DMA Receive Buffer Size <1-255>
// <i> DMA receive buffer size in AHB system memory. The value defined by these
// <i> bits determines the size of buffer to use in main AHB system memory when
// <i> writing received data. The value is defined in multiples of 64 bytes,
// <i> thus a value of 0x01 corresponds to buffers of 64 bytes, 0x02
// <i> corresponds to 128 bytes etc.
// <id> gmac_arch_dcfgr_drbs
#ifndef CONF_GMAC_DCFGR_DRBS
#define CONF_GMAC_DCFGR_DRBS 0x18	//2
#endif

// <q> DMA Discard Received Packets
// <i> When set, the GMAC DMA will automatically discard receive packets from
// <i> the receiver packet buffer memory when no AHB resource is available.
// <i> When low, the received packets will remain to be stored in the SRAM
// <i> based packet buffer until AHB buffer resource next becomes available.
// <i> Note: packet buffer full store and forward mode should be enabled.
// <id> gmac_arch_dcfgr_ddrp
#ifndef CONF_GMAC_DCFGR_DDRP
#define CONF_GMAC_DCFGR_DDRP 0
#endif
// </e>

// <e> Advanced configuration
// <id> gmac_arch_adv_cfg
#ifndef CONF_GMAC_ADV_CFG
#define CONF_GMAC_ADV_CFG 1
#endif

// <o> Number of Transmit Buffer Descriptor <1-255>
// <i> Number of Transmit Buffer Descriptor
// <id> gmac_arch_txdescr_num
#ifndef CONF_GMAC_TXDESCR_NUM
#define CONF_GMAC_TXDESCR_NUM 2
#endif

// <o> Number of Receive Buffer Descriptor <1-255>
// <i> Number of Receive Buffer Descriptor
// <id> gmac_arch_rxdescr_num
#ifndef CONF_GMAC_RXDESCR_NUM
#define CONF_GMAC_RXDESCR_NUM 16
#endif

// <o> Byte size of Transmit Buffer <64-10240>
// <i> Byte size of buffer for each transmit buffer descriptor.
// <id> gmac_arch_txbuf_size
#ifndef CONF_GMAC_TXBUF_SIZE
#define CONF_GMAC_TXBUF_SIZE 1500
#endif

#ifndef CONF_GMAC_RXBUF_SIZE
#define CONF_GMAC_RXBUF_SIZE (CONF_GMAC_DCFGR_DRBS * 64)
#endif

// <e> Enable Transmit Partial Store and Forward
// <i> This allows for a reduced latency but there are performance implications.
// <id> gmac_arch_tpsf_en
#ifndef CONF_GMAC_TPSF_EN
#define CONF_GMAC_TPSF_EN 0
#endif

// <o> Watermark <20-4095>
// <i> Byte size of buffer for each transmit buffer descriptor.
// <id> gmac_arch_tpsf_wm
#ifndef CONF_GMAC_TPSF_WM
#define CONF_GMAC_TPSF_WM 100
#endif
// </e>

// <e> Enable Receive Partial Store and Forward
// <i> This allows for a reduced latency but there are performance implications.
// <id> gmac_arch_rpsf_en
#ifndef CONF_GMAC_RPSF_EN
#define CONF_GMAC_RPSF_EN 0
#endif

// <o> Watermark <20-4095>
// <i> Byte size of buffer for each transmite buffer descriptor.
// <id> gmac_arch_rpsf_wm
#ifndef CONF_GMAC_RPSF_WM
#define CONF_GMAC_RPSF_WM 100
#endif

// <o> IPG Stretch Multiple <0-15>
// <i> This value will multiplied with the previously transmitted frame length
// <i> (including preamble)
// <id> gmac_arch_ipgs_fl_mul
#ifndef CONF_GMAC_IPGS_FL_MUL
#define CONF_GMAC_IPGS_FL_MUL 1
#endif

// <o> IPG Stretch Divide <1-16>
// <i> Divide the frame length. If the resulting number is greater than 96 and
// <i> IP Stretch Enabled then the resulting number is used for the transmit
// <i> inter-packet-gap
// <id> gmac_arch_ipgs_fl_div
#ifndef CONF_GMAC_IPGS_FL_DIV
#define CONF_GMAC_IPGS_FL_DIV 1
#endif

// </e>

// </e>

// <<< end of configuration section >>>

// Initialise the GMAC and Phy
void ethernetif_hardware_init() noexcept
{
	// Set up Ethernet clock
	hri_mclk_set_AHBMASK_GMAC_bit(MCLK);
	hri_mclk_set_APBCMASK_GMAC_bit(MCLK);

	// Setup Ethernet pins
	SetPinFunction(EthernetClockOutPin, EthernetClockOutPinFunction);
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
//	_mac_init_bufdescr(dev);



	/* Disable TX & RX and more. */
//	gmac_network_control(GMAC, 0);
	gmac_disable_interrupt(GMAC, ~0u);

//	gmac_clear_statistics(GMAC);

	/* Clear all status bits in the receive status register. */
	gmac_clear_rx_status(GMAC, GMAC_RSR_BNA | GMAC_RSR_REC | GMAC_RSR_RXOVR | GMAC_RSR_HNO);

	/* Clear all status bits in the transmit status register. */
	gmac_clear_tx_status(GMAC, GMAC_TSR_UBR | GMAC_TSR_COL | GMAC_TSR_RLE | GMAC_TSR_TXGO | GMAC_TSR_TFC | GMAC_TSR_TXCOMP | GMAC_TSR_HRESP);

	/* Clear interrupts. */
	gmac_get_interrupt_status(GMAC);

	/* Enable the copy of data into the buffers, ignore broadcasts, and not copy FCS. */
//	gmac_enable_copy_all(GMAC, false);
//	gmac_disable_broadcast(GMAC, false);

	/* Set RX buffer size to 1536. */
//	gmac_set_rx_bufsize(GMAC, 0x18);

	gmac_rx_init(&gs_gmac_dev);
	gmac_tx_init(&gs_gmac_dev);

	/* Enable Rx, Tx and the statistics register. */
	gmac_enable_transmit(GMAC, true);
	gmac_enable_receive(GMAC, true);
//	gmac_enable_statistics_write(GMAC, true);

	/* Set GMAC address. */
	gmac_set_address(GMAC, 0, gs_uc_mac_address);
}

bool ethernetif_establish_link() noexcept
{
	/* Auto Negotiate, work in RMII mode. */
	uint8_t result = ethernet_phy_auto_negotiate(GMAC, BOARD_GMAC_PHY_ADDR);
	if (result != GMAC_OK)
	{
		if (result != GMAC_TIMEOUT)
		{
			// chrishamm: It is expected that the function above will return ERR_TIMEOUT a few times
			LWIP_DEBUGF(NETIF_DEBUG, ("gmac_low_level_init: auto negotiate ERROR!\n"));
		}
		return false;
	}

	/* Establish ethernet link. */
	if (ethernet_phy_set_link(GMAC, BOARD_GMAC_PHY_ADDR, 1) != GMAC_OK)
	{
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
	NVIC_DisableIRQ(GMAC_IRQn);
	ethernetTask.TerminateAndUnlink();
}

extern "C" uint32_t sys_now() noexcept
{
	return millis();
}

// End
