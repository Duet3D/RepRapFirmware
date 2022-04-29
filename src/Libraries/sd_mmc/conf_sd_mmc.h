/**
 * \file
 *
 * \brief SD/MMC stack configuration file.
 *
 * Copyright (c) 2014-2015 Atmel Corporation. All rights reserved.
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
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef SD_MMC_CONF_SD_MMC_H_INCLUDED
#define SD_MMC_CONF_SD_MMC_H_INCLUDED

// Include the version of this file for ASF to ensure we get consistent definitions, and to get the definition of CONF_HSMCI_XDMAC_CHANNEL
#if SAM4E || SAM4S || SAME70
# include <asf/conf_sd_mmc.h>
#endif

// SD card configuration for Duet and Duet WiFi
#define SD_MMC_ENABLE

#define SD_MMC_HSMCI_MEM_CNT		1			// Number of HSMCI card slots supported
#define SD_MMC_HSMCI_SLOT_0_SIZE	4			// HSMCI bus width

#ifdef PCCB
# define SD_MMC_SPI_MEM_CNT			0			// Number of SPI card slots supported
#else
# define SD_MMC_SPI_MEM_CNT			1			// Number of SPI card slots supported
#endif

#define SD_MMC_SPI_MAX_CLOCK		(4000000)	// Max 4MHz clock for SPI cards, to allow a reasonable cable length

#define SUPPORT_WRITE_PROTECT		0

#if SUPPORT_WRITE_PROTECT
# define SD_MMC_WP_DETECT_VALUE		false
#endif

#define SD_MMC_MEM_CNT				(SD_MMC_HSMCI_MEM_CNT + SD_MMC_SPI_MEM_CNT)

#endif /* SD_MMC_CONF_SD_MMC_H_INCLUDED */

