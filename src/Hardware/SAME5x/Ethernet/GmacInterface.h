/*
 * SAME5x GmacInterface.h
 *
 *  Created on: 19 Jun 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_SAME5X_ETHERNET_GMACINTERFACE_H_
#define SRC_HARDWARE_SAME5X_ETHERNET_GMACINTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "netif/etharp.h"

#ifdef __cplusplus
}

err_t ethernetif_init(struct netif *netif) noexcept;		// called by LwIP to initialise the interface

void ethernetif_terminate() noexcept;						// called when we shut down

bool ethernetif_input(struct netif *netif) noexcept;		// checks for a new packet and returns true if one was processed

void ethernetif_hardware_init() noexcept;					// initialises the low-level hardware interface

bool ethernetif_establish_link() noexcept;					// attempts to establish link and returns true on success

bool ethernetif_link_established() noexcept;				// asks the PHY if the link is still up

void ethernetif_set_mac_address(const uint8_t macAddress[]) noexcept;

#endif

// Error counters
extern unsigned int rxErrorCount;
extern unsigned int rxBuffersNotFullyPopulatedCount;
extern unsigned int txErrorCount;
extern unsigned int txBufferNotFreeCount;
extern unsigned int txBufferTooShortCount;

#endif /* SRC_HARDWARE_SAME5X_ETHERNET_GMACINTERFACE_H_ */
