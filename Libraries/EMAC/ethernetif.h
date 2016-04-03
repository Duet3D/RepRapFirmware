/**
 *
 * \file
 *
 * \brief Ethernet Interface Skeleton.
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

#ifndef ETHERNETIF_H_INCLUDED
#define ETHERNETIF_H_INCLUDED

#include "lwip/src/include/lwip/netif.h"
#include "lwip/src/include/ipv4/lwip/ip_addr.h"
#include "lwip/src/include/lwip/err.h"
#include "lwip/src/include/netif/etharp.h"

bool ethernetif_phy_link_status(void);	//*****************************AB

err_t ethernetif_init(struct netif *netif);
void ethernetif_set_params(const u8_t macAddress[], const char *hostname);

bool ethernetif_input(void *pv_parameters);

void ethernetif_hardware_init(void);
bool ethernetif_establish_link(void);
bool ethernetif_link_established(void);

void ethernetif_set_rx_callback(emac_dev_tx_cb_t callback);
void ethernetif_set_mac_address(const u8_t macAddress[]);

#endif /* ETHERNETIF_H_INCLUDED */
