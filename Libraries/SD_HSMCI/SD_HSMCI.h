/*
 * SD_HSMCI ver 0.1
 *
 * HSMCI SD implementation for SAM3X8E from the ATMEL Studio Framework example project
 * using FatFs
 *
 * Currently does not provide a wrapper - it uses the FatFS API directly:
 * http://elm-chan.org/fsw/ff/00index_e.html
 *
 * A future aspiration is to provide a wrapper to enable it to be a drop in replacement
 * for the Arduino SD libarary
 *
 * The ATMEL files are Copyright (c) 2012 - 2013 Atmel Corporation. All rights reserved. - see the license text in each ATMEL source file
 * FatFs is Copyright (C) 2011, ChaN, all right reserved. - see the License text in the FatFs files
 *
 * tony@Think3dPrint3d
 *
 */

#ifndef SD_HSMCI_H_
#define SD_HSMCI_H_

#include <Arduino.h>

// From module: Common SAM compiler driver
#include "utility/compiler.h"
#include "utility/status_codes.h"
#include "utility/preprocessor.h"
// From module: Memory Control Access Interface
#include "utility/conf_access.h"
#include "utility/ctrl_access.h"


// From module: SD/MMC stack on Multimedia Card interface
#include "utility/sd_mmc_mem.h"
#include "utility/sd_mmc.h"
#include "utility/sd_mmc_protocol.h"

// From module: High Speed Multimedia Card Interface
#include "utility/hsmci.h"

// From module: DMAC - DMAC Controller
#include "utility/dmac.h"

// From module: FatFS file system
#include "utility/diskio.h"
#include "utility/ff.h"


// From module: Part identification macros
#include "sam.h"

#ifdef __SAM3X8E__
#define SAM3XE	(1)
#endif

// From module: RTC - Real Time Clock
// Does not work right now
#include "utility/rtc.h"

/** Enable SD MMC interface pins through HSMCI */
#define CONF_BOARD_SD_MMC_HSMCI

#define REPRAPFIRMWARE	(1)		// enable changes specific to this firmware

/* Define it to enable the SPI mode instead of Multimedia Card interface mode */
//#define SD_MMC_SPI_MODE

/* Define it to enable the SDIO support */
//#define SDIO_SUPPORT_ENABLE

// Debugging
extern void debugPrintf(const char *_fmt, ...);

//#define HSMCI_DEBUG
//#define SD_MMC_DEBUG

/* ------------------------------------------------------------------------ */
/* HSMCI                                                                      */
/* ------------------------------------------------------------------------ */
/*! Number of slot connected on HSMCI interface */
#define SD_MMC_HSMCI_MEM_CNT        1
#define SD_MMC_HSMCI_SLOT_0_SIZE    4

#define SD_MMC_0_CD_GPIO            13		// Arduino digital pin 13
#define SD_MMC_0_CD_DETECT_VALUE    0

extern void sd_mmc_init(void);

inline void delay_ms(uint32_t ms)
{
	delay(ms);
}

inline uint32_t sysclk_get_cpu_hz()
{
	return SystemCoreClock;
}

#endif /* SD_HSMCI_H_ */
