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

// From module: RTC - Real Time Clock
// Does not work right now
#include "utility/rtc.h"

/** Enable SD MMC interface pins through HSMCI */
#define CONF_BOARD_SD_MMC_HSMCI

/* Define it to enable the SPI mode instead of Multimedia Card interface mode */
//#define SD_MMC_SPI_MODE

/* Define it to enable the SDIO support */
//#define SDIO_SUPPORT_ENABLE


/* ------------------------------------------------------------------------ */
/* HSMCI                                                                      */
/* ------------------------------------------------------------------------ */
/*! Number of slot connected on HSMCI interface */
#define SD_MMC_HSMCI_MEM_CNT        1
#define SD_MMC_HSMCI_SLOT_0_SIZE    4
#define PINS_HSMCI\
	{ PIO_PA20A_MCCDA | PIO_PA19A_MCCK | PIO_PA21A_MCDA0 | PIO_PA22A_MCDA1\
	| PIO_PA23A_MCDA2 | PIO_PA24A_MCDA3,\
	PIOA, ID_PIOA, PIO_PERIPH_A, PIO_PULLUP }

#define PIN_HSMCI_CD {PIO_PB27, PIOB, ID_PIOB, PIO_INPUT, PIO_PULLUP}
#define SD_MMC_0_CD_GPIO            13//(PIO_PB27_IDX) //Arduino digital pin 13
#define SD_MMC_0_CD_PIO_ID          ID_PIOB
#define SD_MMC_0_CD_FLAGS           (PIO_INPUT | PIO_PULLUP)
#define SD_MMC_0_CD_DETECT_VALUE    0


extern void sd_mmc_init(void);
//C:\arduino-1.5.2/sketch_may19a.ino:105: warning: undefined reference to `sd_mmc_check'
//C:\arduino-1.5.2/sketch_may19a.ino:108: warning: undefined reference to `sd_mmc_get_capacity'
//C:\arduino-1.5.2/sketch_may19a.ino:109: warning: undefined reference to `sd_mmc_get_bus_clock'
//C:\arduino-1.5.2/sketch_may19a.ino:110: warning: undefined reference to `sd_mmc_get_bus_width'


#endif /* SD_HSMCI_H_ */
