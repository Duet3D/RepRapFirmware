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

#include "lwip/src/include/lwip/netif.h"
#include "emac/emac.h"							// for emac_dev_tx_cb_t

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond


// Perform low-level initialisation of the network interface
void init_ethernet();

// Configure the ethernet interface
void ethernet_configure_interface(const u8_t macAddress[], const char *hostname);

// Perform ethernet auto-negotiation and establish link. Returns true when ready
bool ethernet_establish_link(void);

// Is the link still up? Also updates the interface status if the link has gone down
bool ethernet_link_established(void);

// Initialise network interface and set network interface status callback
void start_ethernet(const uint8_t ipAddress[], const uint8_t netMask[], const uint8_t gateWay[], netif_status_callback_fn status_cb);

// Update IPv4 configuration on demand
void ethernet_set_configuration(const unsigned char ipAddress[], const unsigned char netMask[], const unsigned char gateWay[]);

// Must be called periodically to keep the LwIP timers running
void ethernet_timers_update(void);

// Reads all stored network packets and processes them
void ethernet_task(void);

// Set the RX callback for incoming network packets
void ethernet_set_rx_callback(emac_dev_tx_cb_t callback);

// Returns the network interface's current IPv4 address
const uint8_t *ethernet_get_ipaddress();


/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond

#endif /* ETHERNET_SAM_H_INCLUDED */
