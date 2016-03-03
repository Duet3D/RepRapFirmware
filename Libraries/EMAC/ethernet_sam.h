/**
 * \file
 *
 * \brief Ethernet management definitions for the Standalone lwIP example.
 *
 * Copyright (c) 2012 Atmel Corporation. All rights reserved.
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

#ifndef ETHERNET_SAM_H_INCLUDED
#define ETHERNET_SAM_H_INCLUDED

#include <lwip/src/include/lwip/netif.h>

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

/**
 * \brief Initialize the ethernet interface.
 *
 */
void init_ethernet(const uint8_t macAddress[], const char *hostname);

bool establish_ethernet_link(void);
void start_ethernet(const uint8_t ipAddress[], const uint8_t netMask[], const uint8_t gateWay[]);
bool ethernet_is_ready();

struct netif* ethernet_get_configuration();
void ethernet_set_configuration(const uint8_t ipAddress[], const uint8_t netMask[], const uint8_t gateWay[]);

void ethernet_timers_update(void);

/**
 * \brief Status callback used to print address given by DHCP.
 *
 * \param netif Instance to network interface.
 *
 */
void ethernet_status_callback(struct netif *netif);

/**
 * \brief Manage the ethernet packets, if any received process them.
 *
 * \return Returns true if data has been processed.
 */
bool ethernet_read(void);

/*
 * \brief Sets the EMAC RX callback. It will be called when a new packet
 * can be processed and should be called with a NULL parameter inside
 * the actual callback.
 *
 * \param callback The callback to be called when a new packet is ready
 */
void ethernet_set_rx_callback(emac_dev_tx_cb_t callback);

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond

#endif /* ETHERNET_SAM_H_INCLUDED */
