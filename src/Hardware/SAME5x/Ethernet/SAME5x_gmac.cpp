/*
 * SAME5x_gmac.cpp
 *
 *  Created on: 19 Jun 2020
 *      Author: David
 */

//TODO

#include <Core.h>

extern "C" {
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/stats.h"
#include "lwip/snmp.h"
#include "netif/etharp.h"
}

// Error counters
unsigned int rxErrorCount;
unsigned int rxBuffersNotFullyPopulatedCount;
unsigned int txErrorCount;
unsigned int txBufferNotFreeCount;
unsigned int txBufferTooShortCount;

void ethernetif_hardware_init() noexcept
{
	//TODO
}

bool ethernetif_establish_link() noexcept
{
	//TODO
	return false;
}

// Ask the PHY if the link is still up
bool ethernetif_link_established() noexcept
{
	//TODO
	return false;
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
	//TODO
	return ERR_MEM;
}

void ethernetif_set_mac_address(const uint8_t macAddress[]) noexcept
{
	// This function must be called once before low_level_init(), because that is where the MAC address of the netif is assigned

	//TODO
}

// This is called when we shut down
void ethernetif_terminate() noexcept
{
	NVIC_DisableIRQ(GMAC_IRQn);
	//TODO
//	ethernetTask.TerminateAndUnlink();
}

extern "C" uint32_t sys_now() noexcept
{
	return millis();
}

// End
