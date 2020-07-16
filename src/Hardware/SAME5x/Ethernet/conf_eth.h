// Ethernet configuration for SAME54

#ifndef CONF_ETH_H_INCLUDED
#define CONF_ETH_H_INCLUDED

#include <same54.h>
#include <core_cm4.h>
#include <component/gmac.h>

/**
 * LWIP_NETIF_TX_SINGLE_PBUF: if this is set to 1, lwIP tries to put all data
 * to be sent into one single pbuf. This is for compatibility with DMA-enabled
 * MACs that do not support scatter-gather.
 */
#define LWIP_NETIF_TX_SINGLE_PBUF                     1

/** Number of buffer for RX */
#define GMAC_RX_BUFFERS  8

/** Number of buffer for TX */
#define GMAC_TX_BUFFERS  4

/** The MAC can support frame lengths up to 1536 bytes */
#define GMAC_FRAME_LENTGH_MAX   1536

/** MAC PHY operation max retry count */
#define MAC_PHY_RETRY_MAX 1000000

/** Ethernet MII/RMII mode */
#define ETH_PHY_MODE                                  GMAC_PHY_RMII

/** GMAC HW configurations */
#define BOARD_GMAC_PHY_ADDR   0

/** MAC address definition.  The MAC address must be unique on the network. */
#define ETHERNET_CONF_ETHADDR0                        0xBE
#define ETHERNET_CONF_ETHADDR1                        0xEF
#define ETHERNET_CONF_ETHADDR2                        0xDE
#define ETHERNET_CONF_ETHADDR3                        0xAD
#define ETHERNET_CONF_ETHADDR4                        0xFE
#define ETHERNET_CONF_ETHADDR5                        0xED

#endif /* CONF_ETH_H_INCLUDED */
