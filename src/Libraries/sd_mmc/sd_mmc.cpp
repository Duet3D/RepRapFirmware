/**
 * \file
 *
 * \brief Common SD/MMC stack
 *
 * Copyright (c) 2012-2015 Atmel Corporation. All rights reserved.
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

// 2021-02-03: MC and DC converted this to be re-entrant, provided that only one task uses each interface.
// Currently all RRF configurations for Duets support at most one HSMCI and one SPI card, and there is a mutex for each volume, so this is the case.

#include <RepRapFirmware.h>
#include <cstring>

#define SUPPORT_SDHC	1

#include "sd_mmc_protocol.h"
#include "sd_mmc.h"
#include "conf_sd_mmc.h"

#if SAME70	//DC
# define __nocache		__attribute__((section(".ram_nocache")))
#else
# define __nocache		// nothing
#endif
/**
 * \ingroup sd_mmc_stack
 * \defgroup sd_mmc_stack_internal Implementation of SD/MMC/SDIO Stack
 * @{
 */

// Enable debug information for SD/MMC module
#ifdef SD_MMC_DEBUG
extern void debugPrintf(const char* fmt, ...);
#  define sd_mmc_debug(...)      debugPrintf(__VA_ARGS__)
#else
#  define sd_mmc_debug(...)
#endif

#ifndef SD_MMC_HSMCI_MEM_CNT
#  error SD_MMC_HSMCI_MEM_CNT not defined
#endif
#ifndef SD_MMC_SPI_MEM_CNT
#  error SD_MMC_SPI_MEM_CNT not defined
#endif

typedef void (*driverIdleFunc_t)(uint32_t, uint32_t) noexcept;

struct DriverInterface
{
	bool (*select_device)(uint8_t slot, uint32_t clock, uint8_t bus_width, bool high_speed) noexcept;
	void (*deselect_device)(uint8_t slot) noexcept;
	uint8_t (*get_bus_width)(uint8_t slot) noexcept;
	bool (*is_high_speed_capable)(void) noexcept;
	void (*send_clock)(void) noexcept;
	bool (*send_cmd)(sdmmc_cmd_def_t cmd, uint32_t arg) noexcept;
	uint32_t (*get_response)(void) noexcept;
	void (*get_response_128)(uint8_t* response) noexcept;
	bool (*adtc_start)(sdmmc_cmd_def_t cmd, uint32_t arg, uint16_t block_size, uint16_t nb_block, const void* dmaAddr) noexcept;
	bool (*adtc_stop)(sdmmc_cmd_def_t cmd, uint32_t arg) noexcept;
	bool (*read_word)(uint32_t* value) noexcept;
	bool (*write_word)(uint32_t value) noexcept;
	bool (*start_read_blocks)(void *dest, uint16_t nb_block) noexcept;
	bool (*wait_end_of_read_blocks)(void) noexcept;
	bool (*start_write_blocks)(const void *src, uint16_t nb_block) noexcept;
	bool (*wait_end_of_write_blocks)(void) noexcept;
	uint32_t (*getInterfaceSpeed)(void) noexcept;
	driverIdleFunc_t (*set_idle_func)(driverIdleFunc_t);
	bool is_spi;			// true if the interface is SPI, false if it is HSMCI
};

#if (SD_MMC_HSMCI_MEM_CNT != 0)

# if SAME5x

#  include <Sdhc.h>

driverIdleFunc_t hsmci_set_idle_func(driverIdleFunc_t func) noexcept
{
	// This function is not used on the SAME5x because the low-level driver is FreeRTOS-aware
	return NULL;
}

# else

#  include <hsmci/hsmci.h>

static bool hsmci_adtc_start_glue(sdmmc_cmd_def_t cmd, uint32_t arg, uint16_t block_size, uint16_t nb_block, const void *dmaAddr) noexcept
{
	return hsmci_adtc_start(cmd, arg, block_size, nb_block, dmaAddr != NULL);
}

static bool hsmci_select_device_glue(uint8_t slot, uint32_t clock, uint8_t bus_width, bool high_speed) noexcept
{
	hsmci_select_device(slot, clock, bus_width, high_speed);
	return true;
}

# endif

static const struct DriverInterface hsmciInterface = {
# if SAME5x
	.select_device = hsmci_select_device,
# else
	.select_device = hsmci_select_device_glue,
# endif
	.deselect_device = hsmci_deselect_device,
	.get_bus_width = hsmci_get_bus_width,
	.is_high_speed_capable = hsmci_is_high_speed_capable,
	.send_clock = hsmci_send_clock,
	.send_cmd = hsmci_send_cmd,
	.get_response = hsmci_get_response,
	.get_response_128 = hsmci_get_response_128,
# if SAME5x
	.adtc_start = hsmci_adtc_start,
# else
	.adtc_start = hsmci_adtc_start_glue,
# endif
	.adtc_stop = hsmci_send_cmd,			// adtc_stop aliased to send_cmd as in the ASF original
	.read_word = hsmci_read_word,
	.write_word = hsmci_write_word,
	.start_read_blocks = hsmci_start_read_blocks,
	.wait_end_of_read_blocks = hsmci_wait_end_of_read_blocks,
	.start_write_blocks = hsmci_start_write_blocks,
	.wait_end_of_write_blocks = hsmci_wait_end_of_write_blocks,
	.getInterfaceSpeed = hsmci_get_speed,
	.set_idle_func = hsmci_set_idle_func,
	.is_spi = false
};

#endif

#if (SD_MMC_SPI_MEM_CNT != 0)
# include "sd_mmc_spi.h"

static const struct DriverInterface spiInterface = {
	.select_device = sd_mmc_spi_select_device,
	.deselect_device = sd_mmc_spi_deselect_device,
	.get_bus_width = sd_mmc_spi_get_bus_width,
	.is_high_speed_capable = sd_mmc_spi_is_high_speed_capable,
	.send_clock = sd_mmc_spi_send_clock,
	.send_cmd = sd_mmc_spi_send_cmd,
	.get_response = sd_mmc_spi_get_response,
	.get_response_128 = sd_mmc_spi_get_response_128,
	.adtc_start = sd_mmc_spi_adtc_start,
	.adtc_stop = sd_mmc_spi_send_cmd,			// adtc_stop aliased to send_cmd as in the ASF original
	.read_word = sd_mmc_spi_read_word,
	.write_word = sd_mmc_spi_write_word,
	.start_read_blocks = sd_mmc_spi_start_read_blocks,
	.wait_end_of_read_blocks = sd_mmc_spi_wait_end_of_read_blocks,
	.start_write_blocks = sd_mmc_spi_start_write_blocks,
	.wait_end_of_write_blocks = sd_mmc_spi_wait_end_of_write_blocks,
	.getInterfaceSpeed = spi_mmc_get_speed,
	.set_idle_func = sd_mmc_spi_set_idle_func,
	.is_spi = true
};
#endif

#ifdef SDIO_SUPPORT_ENABLE
# define IS_SDIO()  (sd_mmc_card->type & CARD_TYPE_SDIO)
#else
# define IS_SDIO()  false
#endif

//! This SD MMC stack supports only the high voltage
#define SD_MMC_VOLTAGE_SUPPORT \
		(OCR_VDD_27_28 | OCR_VDD_28_29 | \
		OCR_VDD_29_30 | OCR_VDD_30_31 | \
		OCR_VDD_31_32 | OCR_VDD_32_33)

//! SD/MMC card states
enum card_state {
	SD_MMC_CARD_STATE_READY    = 0, //!< Ready to use
	SD_MMC_CARD_STATE_DEBOUNCE = 1, //!< Debounce ongoing
	SD_MMC_CARD_STATE_INIT     = 2, //!< Initialization ongoing
	SD_MMC_CARD_STATE_UNUSABLE = 3, //!< Unusable card
	SD_MMC_CARD_STATE_NO_CARD  = 4, //!< No SD/MMC card inserted
};

//! SD/MMC card information structure
struct sd_mmc_card {
	const struct DriverInterface *iface;	// Pointer to driver interface functions
	uint32_t clock;				//!< Card access clock
	uint32_t capacity;			//!< Card capacity in KBytes
#if SUPPORT_WRITE_PROTECT
	Pin wp_gpio;				//!< Card write protection pin number, or -1 if none present
#endif
	uint16_t rca;				//!< Relative card address
	enum card_state state;		//!< Card state
	card_type_t type;			//!< Card type
	card_version_t version;		//!< Card version
	uint8_t slot;				// Slot number within the driver
	uint8_t bus_width;			//!< Number of DATA lines on bus (MCI only)
	uint8_t csd[CSD_REG_BSIZE];	//!< CSD register
	uint8_t high_speed;			//!< High speed card (1)
};

//! SD/MMC card list
//DC added __nocache for SAME70 because 'csd' is read by DMA
static __nocache struct sd_mmc_card sd_mmc_cards[SD_MMC_MEM_CNT];

//! Number of block to read or write on the current transfer
static uint16_t sd_mmc_nb_block_to_tranfer[SD_MMC_MEM_CNT] = { 0 };
//! Number of block remaining to read or write on the current transfer
static uint16_t sd_mmc_nb_block_remaining[SD_MMC_MEM_CNT] = { 0 };

//! SD/MMC transfer rate unit codes (10K) list
const uint32_t sd_mmc_trans_units[7] = {
	10, 100, 1000, 10000, 0, 0, 0
};
//! SD transfer multiplier factor codes (1/10) list
const uint32_t sd_trans_multipliers[16] = {
	0, 10, 12, 13, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80
};
//! MMC transfer multiplier factor codes (1/10) list
const uint32_t mmc_trans_multipliers[16] = {
	0, 10, 12, 13, 15, 20, 26, 30, 35, 40, 45, 52, 55, 60, 70, 80
};

//! \name MMC, SD and SDIO commands process
//! @{
static bool mmc_spi_op_cond(uint8_t slot);
static bool mmc_mci_op_cond(uint8_t slot);
static bool sd_spi_op_cond(uint8_t v2, uint8_t slot);
static bool sd_mci_op_cond(uint8_t v2, uint8_t slot);
static bool sdio_op_cond(void);
static bool sdio_get_max_speed(void);
static bool sdio_cmd52_set_bus_width(void);
static bool sdio_cmd52_set_high_speed(void);
static bool sd_cm6_set_high_speed(uint8_t slot);
static bool mmc_cmd6_set_bus_width(uint8_t bus_width, uint8_t slot);
static bool mmc_cmd6_set_high_speed(uint8_t slot);
static bool sd_cmd8(uint8_t * v2, uint8_t slot);
static bool mmc_cmd8(uint8_t *b_authorize_high_speed, uint8_t slot);
static bool sd_mmc_cmd9_spi(uint8_t slot);
static bool sd_mmc_cmd9_mci(uint8_t slot);
static void mmc_decode_csd(uint8_t slot);
static void sd_decode_csd(uint8_t slot);
static bool sd_mmc_cmd13(uint8_t slot);
#ifdef SDIO_SUPPORT_ENABLE
static bool sdio_cmd52(uint8_t rw_flag, uint8_t func_nb,
		uint32_t reg_addr, uint8_t rd_after_wr, uint8_t *io_data);
static bool sdio_cmd53(uint8_t rw_flag, uint8_t func_nb, uint32_t reg_addr,
		uint8_t inc_addr, uint32_t size, bool access_block);
#endif // SDIO_SUPPORT_ENABLE
static bool sd_acmd6(uint8_t slot);
static bool sd_acmd51(uint8_t slot);
//! @}

//! \name Internal function to process the initialization and install
//! @{
static sd_mmc_err_t sd_mmc_select_slot(uint8_t slot);
static bool sd_mmc_configure_slot(uint8_t slot);
static void sd_mmc_deselect_slot(uint8_t slot);
static bool sd_mmc_spi_card_init(uint8_t slot);
static bool sd_mmc_mci_card_init(uint8_t slot);
static bool sd_mmc_spi_install_mmc(uint8_t slot);
static bool sd_mmc_mci_install_mmc(uint8_t slot);
//! @}


//! \name Internal functions to manage a large timeout after a card insertion
//! @{
#define SD_MMC_DEBOUNCE_TIMEOUT   1000 // Unit ms


/**
 * \brief Sends operation condition command and read OCR (SPI only)
 * - CMD1 sends operation condition command
 * - CMD58 reads OCR
 *
 * \return true if success, otherwise false
 */
static bool mmc_spi_op_cond(uint8_t slot)
{
	uint32_t retry, resp;
	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];

	/*
	 * Timeout 1s = 400KHz / ((6+1)*8) cylces = 7150 retry
	 * 6 = cmd byte size
	 * 1 = response byte size
	 */
	retry = 7150;
	do {
		if (!sd_mmc_card->iface->send_cmd(MMC_SPI_CMD1_SEND_OP_COND, 0)) {
			sd_mmc_debug("%s: CMD1 SPI Fail - Busy retry %d\n\r",
					__func__, (int)(7150 - retry));
			return false;
		}
		// Check busy flag
		resp = sd_mmc_card->iface->get_response();
		if (!(resp & R1_SPI_IDLE)) {
			break;
		}
		if (retry-- == 0) {
			sd_mmc_debug("%s: CMD1 Timeout on busy\n\r", __func__);
			return false;
		}
	} while (1);

	// Read OCR for SPI mode
	if (!sd_mmc_card->iface->send_cmd(SDMMC_SPI_CMD58_READ_OCR, 0)) {
		sd_mmc_debug("%s: CMD58 Fail\n\r", __func__);
		return false;
	}
	// Check OCR value
	if ((sd_mmc_card->iface->get_response() & OCR_ACCESS_MODE_MASK)
			== OCR_ACCESS_MODE_SECTOR) {
		sd_mmc_card->type |= CARD_TYPE_HC;
	}
	return true;
}

/**
 * \brief Sends operation condition command and read OCR (MCI only)
 * - CMD1 sends operation condition command
 * - CMD1 reads OCR
 *
 * \return true if success, otherwise false
 */
static bool mmc_mci_op_cond(uint8_t slot)
{
	uint32_t retry, resp;
	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];

	/*
	 * Timeout 1s = 400KHz / ((6+6)*8) cylces = 4200 retry
	 * 6 = cmd byte size
	 * 6 = response byte size
	 */
	retry = 4200;
	do {
		if (!sd_mmc_card->iface->send_cmd(MMC_MCI_CMD1_SEND_OP_COND,
				SD_MMC_VOLTAGE_SUPPORT | OCR_ACCESS_MODE_SECTOR)) {
			sd_mmc_debug("%s: CMD1 MCI Fail - Busy retry %d\n\r",
					__func__, (int)(4200 - retry));
			return false;
		}
		// Check busy flag
		resp = sd_mmc_card->iface->get_response();
		if (resp & OCR_POWER_UP_BUSY) {
			// Check OCR value
			if ((resp & OCR_ACCESS_MODE_MASK)
					== OCR_ACCESS_MODE_SECTOR) {
				sd_mmc_card->type |= CARD_TYPE_HC;
			}
			break;
		}
		if (retry-- == 0) {
			sd_mmc_debug("%s: CMD1 Timeout on busy\n\r", __func__);
			return false;
		}
	} while (1);
	return true;
}

/**
 * \brief Ask to all cards to send their operations conditions (SPI only).
 * - ACMD41 sends operation condition command.
 * - CMD58 reads OCR
 *
 * \param v2   Shall be 1 if it is a SD card V2
 *
 * \return true if success, otherwise false
 */
static bool sd_spi_op_cond(uint8_t v2, uint8_t slot)
{
	uint32_t arg, retry, resp;
	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];

	/*
	 * Timeout 1s = 400KHz / ((6+1)*8) cylces = 7150 retry
	 * 6 = cmd byte size
	 * 1 = response byte size
	 */
	retry = 7150;
	do {
		// CMD55 - Indicate to the card that the next command is an
		// application specific command rather than a standard command.
		if (!sd_mmc_card->iface->send_cmd(SDMMC_CMD55_APP_CMD, 0)) {
			sd_mmc_debug("%s: CMD55 Fail\n\r", __func__);
			return false;
		}

		// (ACMD41) Sends host OCR register
		arg = 0;
		if (v2) {
			arg |= SD_ACMD41_HCS;
		}
		// Check response
		if (!sd_mmc_card->iface->send_cmd(SD_SPI_ACMD41_SD_SEND_OP_COND, arg)) {
			sd_mmc_debug("%s: ACMD41 Fail\n\r", __func__);
			return false;
		}
		resp = sd_mmc_card->iface->get_response();
		if (!(resp & R1_SPI_IDLE)) {
			// Card is ready
			break;
		}
		if (retry-- == 0) {
			sd_mmc_debug("%s: ACMD41 Timeout on busy, resp32 0x%08x \n\r",
					__func__, resp);
			return false;
		}
	} while (1);

	// Read OCR for SPI mode
	if (!sd_mmc_card->iface->send_cmd(SDMMC_SPI_CMD58_READ_OCR, 0)) {
		sd_mmc_debug("%s: CMD58 Fail\n\r", __func__);
		return false;
	}
	if ((sd_mmc_card->iface->get_response() & OCR_CCS) != 0) {
		sd_mmc_card->type |= CARD_TYPE_HC;
	}
	return true;
}

/**
 * \brief Ask to all cards to send their operations conditions (MCI only).
 * - ACMD41 sends operation condition command.
 * - ACMD41 reads OCR
 *
 * \param v2   Shall be 1 if it is a SD card V2
 *
 * \return true if success, otherwise false
 */
static bool sd_mci_op_cond(uint8_t v2, uint8_t slot)
{
	uint32_t arg, retry, resp;
	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];

	/*
	 * Timeout 1s = 400KHz / ((6+6+6+6)*8) cylces = 2100 retry
	 * 6 = cmd byte size
	 * 6 = response byte size
	 * 6 = cmd byte size
	 * 6 = response byte size
	 */
	retry = 2100;
	do {
		// CMD55 - Indicate to the card that the next command is an
		// application specific command rather than a standard command.
		if (!sd_mmc_card->iface->send_cmd(SDMMC_CMD55_APP_CMD, 0)) {
			sd_mmc_debug("%s: CMD55 Fail\n\r", __func__);
			return false;
		}

		// (ACMD41) Sends host OCR register
		arg = SD_MMC_VOLTAGE_SUPPORT;
		if (v2) {
			arg |= SD_ACMD41_HCS;
		}
		// Check response
		if (!sd_mmc_card->iface->send_cmd(SD_MCI_ACMD41_SD_SEND_OP_COND, arg)) {
			sd_mmc_debug("%s: ACMD41 Fail\n\r", __func__);
			return false;
		}
		resp = sd_mmc_card->iface->get_response();
		if (resp & OCR_POWER_UP_BUSY) {
			// Card is ready
			if ((resp & OCR_CCS) != 0) {
				sd_mmc_card->type |= CARD_TYPE_HC;
			}
			break;
		}
		if (retry-- == 0) {
			sd_mmc_debug("%s: ACMD41 Timeout on busy, resp32 0x%08x \n\r",
					__func__, resp);
			return false;
		}
	} while (1);
	return true;
}

#ifdef SDIO_SUPPORT_ENABLE
/**
 * \brief Try to get the SDIO card's operating condition
 * - CMD5 to read OCR NF field
 * - CMD5 to wait OCR power up busy
 * - CMD5 to read OCR MP field
 *   sd_mmc_card->type is updated
 *
 * \return true if success, otherwise false
 */
static bool sdio_op_cond(void)
{
	uint32_t resp;

	// CMD5 - SDIO send operation condition (OCR) command.
	if (!sd_mmc_card->iface->send_cmd(SDIO_CMD5_SEND_OP_COND, 0)) {
		sd_mmc_debug("%s: CMD5 Fail\n\r", __func__);
		return true; // No error but card type not updated
	}
	resp = sd_mmc_card->iface->get_response();
	if ((resp & OCR_SDIO_NF) == 0) {
		return true; // No error but card type not updated
	}

	/*
	 * Wait card ready
	 * Timeout 1s = 400KHz / ((6+4)*8) cylces = 5000 retry
	 * 6 = cmd byte size
	 * 4(SPI) 6(MCI) = response byte size
	 */
	uint32_t cmd5_retry = 5000;
	while (1) {
		// CMD5 - SDIO send operation condition (OCR) command.
		if (!sd_mmc_card->iface->send_cmd(SDIO_CMD5_SEND_OP_COND,
				resp & SD_MMC_VOLTAGE_SUPPORT)) {
			sd_mmc_debug("%s: CMD5 Fail\n\r", __func__);
			return false;
		}
		resp = sd_mmc_card->iface->get_response();
		if ((resp & OCR_POWER_UP_BUSY) == OCR_POWER_UP_BUSY) {
			break;
		}
		if (cmd5_retry-- == 0) {
			sd_mmc_debug("%s: CMD5 Timeout on busy\n\r", __func__);
			return false;
		}
	}
	// Update card type at the end of busy
	if ((resp & OCR_SDIO_MP) > 0) {
		sd_mmc_card->type = CARD_TYPE_SD_COMBO;
	} else {
		sd_mmc_card->type = CARD_TYPE_SDIO;
	}
	return true; // No error and card type updated with SDIO type
}

/**
 * \brief Get SDIO max transfer speed in Hz.
 * - CMD53 reads CIS area address in CCCR area.
 * - Nx CMD53 search Fun0 tuple in CIS area
 * - CMD53 reads TPLFE_MAX_TRAN_SPEED in Fun0 tuple
 * - Compute maximum speed of SDIO
 *   and update sd_mmc_card->clock
 *
 * \return true if success, otherwise false
 */
static bool sdio_get_max_speed(void)
{
	uint32_t addr_new, addr_old;
	uint8_t buf[6];
 	uint32_t unit;
	uint32_t mul;
	uint8_t tplfe_max_tran_speed, i;
	uint8_t addr_cis[4];

	/* Read CIS area address in CCCR area */
	addr_old = SDIO_CCCR_CIS_PTR;
	for(i = 0; i < 4; i++) {
		sdio_cmd52(SDIO_CMD52_READ_FLAG, SDIO_CIA, addr_old, 0, &addr_cis[i]);
		addr_old++;
	}
	addr_old = addr_cis[0] + (addr_cis[1] << 8) + \
				(addr_cis[2] << 16) + (addr_cis[3] << 24);
	addr_new = addr_old;

	while (1) {
		// Read a sample of CIA area
		for(i=0; i<3; i++) {
			sdio_cmd52(SDIO_CMD52_READ_FLAG, SDIO_CIA, addr_new, 0, &buf[i]);
			addr_new++;
		}
		if (buf[0] == SDIO_CISTPL_END) {
			sd_mmc_debug("%s: CMD52 Tuple error\n\r", __func__);
			return false; // Tuple error
		}
		if (buf[0] == SDIO_CISTPL_FUNCE && buf[2] == 0x00) {
			break; // Fun0 tuple found
		}
		if (buf[1] == 0) {
			sd_mmc_debug("%s: CMD52 Tuple error\n\r", __func__);
			return false; // Tuple error
		}

		// Next address
		addr += (buf[1] + 2);
		if (addr > (addr_cis + 256)) {
			sd_mmc_debug("%s: CMD52 Outoff CIS area\n\r", __func__);
			return false; // Outoff CIS area
		}
	}

	// Read all Fun0 tuple fields: fn0_blk_siz & max_tran_speed
	addr_new -= 3;
	for(i = 0; i < 6; i++) {
		sdio_cmd52(SDIO_CMD52_READ_FLAG, SDIO_CIA, addr_new, 0, &buf[i]);
		addr_new++;
	}

	tplfe_max_tran_speed = buf[5];
	if (tplfe_max_tran_speed > 0x32) {
		/* Error on SDIO register, the high speed is not activated
		 * and the clock can not be more than 25MHz.
		 * This error is present on specific SDIO card
		 * (H&D wireless card - HDG104 WiFi SIP).
		 */
		tplfe_max_tran_speed = 0x32; // 25Mhz
	}

	// Decode transfer speed in Hz.
	unit = sd_mmc_trans_units[tplfe_max_tran_speed & 0x7];
	mul = sd_trans_multipliers[(tplfe_max_tran_speed >> 3) & 0xF];
	sd_mmc_card->clock = unit * mul * 1000;
	/**
	 * Note: A combo card shall be a Full-Speed SDIO card
	 * which supports upto 25MHz.
	 * A SDIO card alone can be:
	 * - a Low-Speed SDIO card which supports 400Khz minimum
	 * - a Full-Speed SDIO card which supports upto 25MHz
	 */
	return true;
}

/**
 * \brief CMD52 for SDIO - Switches the bus width mode to 4
 *
 * \note sd_mmc_card->bus_width is updated.
 *
 * \return true if success, otherwise false
 */
static bool sdio_cmd52_set_bus_width(void)
{
	/**
	 * A SD memory card always supports bus 4bit
	 * A SD COMBO card always supports bus 4bit
	 * A SDIO Full-Speed alone always supports 4bit
	 * A SDIO Low-Speed alone can supports 4bit (Optional)
	 */
	uint8_t u8_value;

	// Check 4bit support in 4BLS of "Card Capability" register
	if (!sdio_cmd52(SDIO_CMD52_READ_FLAG, SDIO_CIA, SDIO_CCCR_CAP,
			0, &u8_value)) {
		return false;
	}
	if ((u8_value & SDIO_CAP_4BLS) != SDIO_CAP_4BLS) {
		// No supported, it is not a protocol error
		return true;
	}
	// HS mode possible, then enable
	u8_value = SDIO_BUSWIDTH_4B;
	if (!sdio_cmd52(SDIO_CMD52_WRITE_FLAG, SDIO_CIA, SDIO_CCCR_BUS_CTRL,
			1, &u8_value)) {
		return false;
	}
	sd_mmc_card->bus_width = 4;
	sd_mmc_debug("%d-bit bus width enabled.\n\r", (int)sd_mmc_card->bus_width);
	return true;
}

/**
 * \brief CMD52 for SDIO - Enable the high speed mode
 *
 * \note sd_mmc_card->high_speed is updated.
 * \note sd_mmc_card->clock is updated.
 *
 * \return true if success, otherwise false
 */
static bool sdio_cmd52_set_high_speed(void)
{
	uint8_t u8_value;

	// Check CIA.HS
	if (!sdio_cmd52(SDIO_CMD52_READ_FLAG, SDIO_CIA, SDIO_CCCR_HS, 0, &u8_value)) {
		return false;
	}
	if ((u8_value & SDIO_SHS) != SDIO_SHS) {
		// No supported, it is not a protocol error
		return true;
	}
	// HS mode possible, then enable
	u8_value = SDIO_EHS;
	if (!sdio_cmd52(SDIO_CMD52_WRITE_FLAG, SDIO_CIA, SDIO_CCCR_HS,
			1, &u8_value)) {
		return false;
	}
	sd_mmc_card->high_speed = 1;
	sd_mmc_card->clock *= 2;
	return true;
}

#else
static bool sdio_op_cond(void)
{
	return true; // No error but card type not updated
}
static bool sdio_get_max_speed(void)
{
	return false;
}
static bool sdio_cmd52_set_bus_width(void)
{
	return false;
}
static bool sdio_cmd52_set_high_speed(void)
{
	return false;
}
#endif // SDIO_SUPPORT_ENABLE

/**
 * \brief CMD6 for SD - Switch card in high speed mode
 *
 * \note CMD6 for SD is valid under the "trans" state.
 * \note sd_mmc_card->high_speed is updated.
 * \note sd_mmc_card->clock is updated.
 *
 * \return true if success, otherwise false
 */
static bool sd_cm6_set_high_speed(uint8_t slot)
{

#if SAME70
	static __nocache uint8_t switch_status_array[SD_MMC_MEM_CNT][SD_SW_STATUS_BSIZE];
	uint8_t * const switch_status = switch_status_array[slot];
#else
	uint8_t switch_status[SD_SW_STATUS_BSIZE];
#endif

	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];
	if (!sd_mmc_card->iface->adtc_start(SD_CMD6_SWITCH_FUNC,
			SD_CMD6_MODE_SWITCH
			| SD_CMD6_GRP6_NO_INFLUENCE
			| SD_CMD6_GRP5_NO_INFLUENCE
			| SD_CMD6_GRP4_NO_INFLUENCE
			| SD_CMD6_GRP3_NO_INFLUENCE
			| SD_CMD6_GRP2_DEFAULT
			| SD_CMD6_GRP1_HIGH_SPEED,
			SD_SW_STATUS_BSIZE, 1, switch_status)) {
		return false;
	}
	if (!sd_mmc_card->iface->start_read_blocks(switch_status, 1)) {
		return false;
	}
	if (!sd_mmc_card->iface->wait_end_of_read_blocks()) {
		return false;
	}

	if (sd_mmc_card->iface->get_response() & CARD_STATUS_SWITCH_ERROR) {
		sd_mmc_debug("%s: CMD6 CARD_STATUS_SWITCH_ERROR\n\r", __func__);
		return false;
	}
	if (SD_SW_STATUS_FUN_GRP1_RC(switch_status)
			== SD_SW_STATUS_FUN_GRP_RC_ERROR) {
		// No supported, it is not a protocol error
		return true;
	}
	if (SD_SW_STATUS_FUN_GRP1_BUSY(switch_status)) {
		sd_mmc_debug("%s: CMD6 SD_SW_STATUS_FUN_GRP1_BUSY\n\r", __func__);
		return false;
	}
	// CMD6 function switching period is within 8 clocks
	// after the end bit of status data.
	sd_mmc_card->iface->send_clock();
	sd_mmc_card->high_speed = 1;
	sd_mmc_card->clock *= 2;
	return true;
}

/**
 * \brief CMD6 for MMC - Switches the bus width mode
 *
 * \note CMD6 is valid under the "trans" state.
 * \note sd_mmc_card->bus_width is updated.
 *
 * \param bus_width   Bus width to set
 *
 * \return true if success, otherwise false
 */
static bool mmc_cmd6_set_bus_width(uint8_t bus_width, uint8_t slot)
{
	uint32_t arg;

	switch (bus_width) {
	case 8:
		arg = MMC_CMD6_ACCESS_SET_BITS
				| MMC_CMD6_INDEX_BUS_WIDTH
				| MMC_CMD6_VALUE_BUS_WIDTH_8BIT;
		break;
	case 4:
		arg = MMC_CMD6_ACCESS_SET_BITS
				| MMC_CMD6_INDEX_BUS_WIDTH
				| MMC_CMD6_VALUE_BUS_WIDTH_4BIT;
		break;
	default:
		arg = MMC_CMD6_ACCESS_SET_BITS
				| MMC_CMD6_INDEX_BUS_WIDTH
				| MMC_CMD6_VALUE_BUS_WIDTH_1BIT;
		break;
	}
	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];
	if (!sd_mmc_card->iface->send_cmd(MMC_CMD6_SWITCH, arg)) {
		return false;
	}
	if (sd_mmc_card->iface->get_response() & CARD_STATUS_SWITCH_ERROR) {
		// No supported, it is not a protocol error
		sd_mmc_debug("%s: CMD6 CARD_STATUS_SWITCH_ERROR\n\r", __func__);
		return false;
	}
	sd_mmc_card->bus_width = bus_width;
	sd_mmc_debug("%d-bit bus width enabled.\n\r", (int)sd_mmc_card->bus_width);
	return true;
}

/**
 * \brief CMD6 for MMC - Switches in high speed mode
 *
 * \note CMD6 is valid under the "trans" state.
 * \note sd_mmc_card->high_speed is updated.
 * \note sd_mmc_card->clock is updated.
 *
 * \return true if success, otherwise false
 */
static bool mmc_cmd6_set_high_speed(uint8_t slot)
{
	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];
	if (!sd_mmc_card->iface->send_cmd(MMC_CMD6_SWITCH,
			MMC_CMD6_ACCESS_WRITE_BYTE
			| MMC_CMD6_INDEX_HS_TIMING
			| MMC_CMD6_VALUE_HS_TIMING_ENABLE)) {
		return false;
	}
	if (sd_mmc_card->iface->get_response() & CARD_STATUS_SWITCH_ERROR) {
		// No supported, it is not a protocol error
		sd_mmc_debug("%s: CMD6 CARD_STATUS_SWITCH_ERROR\n\r", __func__);
		return false;
	}
	sd_mmc_card->high_speed = 1;
	sd_mmc_card->clock = 52000000lu;
	return true;
}

/**
 * \brief CMD8 for SD card - Send Interface Condition Command.
 *
 * \note
 * Send SD Memory Card interface condition, which includes host supply
 * voltage information and asks the card whether card supports voltage.
 * Should be performed at initialization time to detect the card type.
 *
 * \param v2 Pointer to v2 flag to update
 *
 * \return true if success, otherwise false
 *         with a update of \ref sd_mmc_err.
 */
static bool sd_cmd8(uint8_t * v2, uint8_t slot)
{
	uint32_t resp;
	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];

	*v2 = 0;
	// Test for SD version 2
	if (!sd_mmc_card->iface->send_cmd(SD_CMD8_SEND_IF_COND,
			SD_CMD8_PATTERN | SD_CMD8_HIGH_VOLTAGE)) {
		return true; // It is not a V2
	}
	// Check R7 response
	resp = sd_mmc_card->iface->get_response();
	if (resp == 0xFFFFFFFF) {
		// No compliance R7 value
		return true; // It is not a V2
	}
	if ((resp & (SD_CMD8_MASK_PATTERN | SD_CMD8_MASK_VOLTAGE))
				!= (SD_CMD8_PATTERN | SD_CMD8_HIGH_VOLTAGE)) {
		sd_mmc_debug("%s: CMD8 resp32 0x%08x UNUSABLE CARD\n\r",
				__func__, resp);
		return false;
	}
	sd_mmc_debug("SD card V2\n\r");
	*v2 = 1;
	return true;
}

/**
 * \brief CMD8 - The card sends its EXT_CSD register as a block of data.
 *
 * \param b_authorize_high_speed Pointer to update with the high speed
 * support information
 *
 * \return true if success, otherwise false
 */
static bool mmc_cmd8(uint8_t *b_authorize_high_speed, uint8_t slot)
{
	uint16_t i;
	uint32_t ext_csd;
	uint32_t sec_count;
	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];

	if (!sd_mmc_card->iface->adtc_start(MMC_CMD8_SEND_EXT_CSD, 0, EXT_CSD_BSIZE, 1, nullptr)) {
		return false;
	}
	//** Read and decode Extended Extended CSD
	// Note: The read access is done in byte to avoid a buffer
	// of EXT_CSD_BSIZE Byte in stack.

	// Read card type
	for (i = 0; i < (EXT_CSD_CARD_TYPE_INDEX + 4) / 4; i++) {
		if (!sd_mmc_card->iface->read_word(&ext_csd)) {
			return false;
		}
	}
	*b_authorize_high_speed = (ext_csd >> ((EXT_CSD_CARD_TYPE_INDEX % 4) * 8))
			& MMC_CTYPE_52MHZ;

	if (MMC_CSD_C_SIZE(sd_mmc_card->csd) == 0xFFF) {
		// For high capacity SD/MMC card,
		// memory capacity = SEC_COUNT * 512 byte
		for (; i <(EXT_CSD_SEC_COUNT_INDEX + 4) / 4; i++) {
			if (!sd_mmc_card->iface->read_word(&sec_count)) {
				return false;
			}
		}
		sd_mmc_card->capacity = sec_count / 2;
	}
	for (; i < EXT_CSD_BSIZE / 4; i++) {
		if (!sd_mmc_card->iface->read_word(&sec_count)) {
			return false;
		}
	}
	return true;
}

/**
 * \brief CMD9: Addressed card sends its card-specific
 * data (CSD) on the CMD line spi.
 *
 * \return true if success, otherwise false
 */
static bool sd_mmc_cmd9_spi(uint8_t slot)
{
	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];
	if (!sd_mmc_card->iface->adtc_start(SDMMC_SPI_CMD9_SEND_CSD, (uint32_t)sd_mmc_card->rca << 16, CSD_REG_BSIZE, 1, sd_mmc_card->csd)) {
		return false;
	}
	if (!sd_mmc_card->iface->start_read_blocks(sd_mmc_card->csd, 1)) {
		return false;
	}
	return sd_mmc_card->iface->wait_end_of_read_blocks();
}

/**
 * \brief CMD9: Addressed card sends its card-specific
 * data (CSD) on the CMD line mci.
 *
 * \return true if success, otherwise false
 */
static bool sd_mmc_cmd9_mci(uint8_t slot)
{
	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];
	if (!sd_mmc_card->iface->send_cmd(SDMMC_MCI_CMD9_SEND_CSD, (uint32_t)sd_mmc_card->rca << 16)) {
		return false;
	}
	sd_mmc_card->iface->get_response_128(sd_mmc_card->csd);
	return true;
}

/**
 * \brief Decodes MMC CSD register
 */
static void mmc_decode_csd(uint8_t slot)
{
 	uint32_t unit;
	uint32_t mul;
	uint32_t tran_speed;
	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];

	// Get MMC System Specification version supported by the card
	switch (MMC_CSD_SPEC_VERS(sd_mmc_card->csd)) {
	default:
	case 0:
		sd_mmc_card->version = CARD_VER_MMC_1_2;
		break;

	case 1:
		sd_mmc_card->version = CARD_VER_MMC_1_4;
		break;

	case 2:
		sd_mmc_card->version = CARD_VER_MMC_2_2;
		break;

	case 3:
		sd_mmc_card->version = CARD_VER_MMC_3;
		break;

	case 4:
		sd_mmc_card->version = CARD_VER_MMC_4;
		break;
	}

	// Get MMC memory max transfer speed in Hz.
	tran_speed = CSD_TRAN_SPEED(sd_mmc_card->csd);
	unit = sd_mmc_trans_units[tran_speed & 0x7];
	mul = mmc_trans_multipliers[(tran_speed >> 3) & 0xF];
	sd_mmc_card->clock = unit * mul * 1000;

	/*
	 * Get card capacity.
	 * ----------------------------------------------------
	 * For normal SD/MMC card:
	 * memory capacity = BLOCKNR * BLOCK_LEN
	 * Where
	 * BLOCKNR = (C_SIZE+1) * MULT
	 * MULT = 2 ^ (C_SIZE_MULT+2)       (C_SIZE_MULT < 8)
	 * BLOCK_LEN = 2 ^ READ_BL_LEN      (READ_BL_LEN < 12)
	 * ----------------------------------------------------
	 * For high capacity SD/MMC card:
	 * memory capacity = SEC_COUNT * 512 byte
	 */
	if (MMC_CSD_C_SIZE(sd_mmc_card->csd) != 0xFFF) {
		uint32_t blocknr = ((MMC_CSD_C_SIZE(sd_mmc_card->csd) + 1) *
			(1 << (MMC_CSD_C_SIZE_MULT(sd_mmc_card->csd) + 2)));
		sd_mmc_card->capacity = blocknr *
			(1 << MMC_CSD_READ_BL_LEN(sd_mmc_card->csd)) / 1024;
	}
}

/**
 * \brief Decodes SD CSD register
 */
static void sd_decode_csd(uint8_t slot)
{
 	uint32_t unit;
	uint32_t mul;
	uint32_t tran_speed;
	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];

	// Get SD memory maximum transfer speed in Hz.
	tran_speed = CSD_TRAN_SPEED(sd_mmc_card->csd);
	unit = sd_mmc_trans_units[tran_speed & 0x7];
	mul = sd_trans_multipliers[(tran_speed >> 3) & 0xF];
	sd_mmc_card->clock = unit * mul * 1000;

	/*
	 * Get card capacity.
	 * ----------------------------------------------------
	 * For normal SD/MMC card:
	 * memory capacity = BLOCKNR * BLOCK_LEN
	 * Where
	 * BLOCKNR = (C_SIZE+1) * MULT
	 * MULT = 2 ^ (C_SIZE_MULT+2)       (C_SIZE_MULT < 8)
	 * BLOCK_LEN = 2 ^ READ_BL_LEN      (READ_BL_LEN < 12)
	 * ----------------------------------------------------
	 * For high capacity SD card:
	 * memory capacity = (C_SIZE+1) * 512K byte
	 */
	if (CSD_STRUCTURE_VERSION(sd_mmc_card->csd) >= SD_CSD_VER_2_0) {
		sd_mmc_card->capacity =
				(SD_CSD_2_0_C_SIZE(sd_mmc_card->csd) + 1)
				* 512;
	} else {
		uint32_t blocknr = ((SD_CSD_1_0_C_SIZE(sd_mmc_card->csd) + 1) *
				(1 << (SD_CSD_1_0_C_SIZE_MULT(sd_mmc_card->csd) + 2)));
		sd_mmc_card->capacity = blocknr *
				(1 << SD_CSD_1_0_READ_BL_LEN(sd_mmc_card->csd))
				/ 1024;
	}
}

/**
 * \brief CMD13 - Addressed card sends its status register.
 * This function waits the clear of the busy flag
 *
 * \return true if success, otherwise false
 */
static bool sd_mmc_cmd13(uint8_t slot)
{
	uint32_t nec_timeout;
	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];

	/* Wait for data ready status.
	 * Nec timing: 0 to unlimited
	 * However a timeout is used.
	 * 200 000 * 8 cycles
	 */
	nec_timeout = 200000;
	do {
		if (sd_mmc_card->iface->is_spi) {
			if (!sd_mmc_card->iface->send_cmd(SDMMC_SPI_CMD13_SEND_STATUS, 0)) {
				return false;
			}
			// Check busy flag
			if (!(sd_mmc_card->iface->get_response() & 0xFF)) {
				break;
			}
		} else {
			if (!sd_mmc_card->iface->send_cmd(SDMMC_MCI_CMD13_SEND_STATUS, (uint32_t)sd_mmc_card->rca << 16)) {
				return false;
			}
			// Check busy flag
			if (sd_mmc_card->iface->get_response() & CARD_STATUS_READY_FOR_DATA) {
				break;
			}
		}
		if (nec_timeout-- == 0) {
			sd_mmc_debug("%s: CMD13 Busy timeout\n\r", __func__);
			return false;
		}
	} while (1);

	return true;
}

#ifdef SDIO_SUPPORT_ENABLE
/**
 * \brief CMD52 - SDIO IO_RW_DIRECT command
 *
 * \param rw_flag   Direction, 1:write, 0:read.
 * \param func_nb   Number of the function.
 * \param rd_after_wr Read after Write flag.
 * \param reg_addr  register address.
 * \param io_data   Pointer to input argument and response buffer.
 *
 * \return true if success, otherwise false
 */
static bool sdio_cmd52(uint8_t rw_flag, uint8_t func_nb,
		uint32_t reg_addr, uint8_t rd_after_wr, uint8_t *io_data)
{
	Assert(io_data != NULL);
	if (!sd_mmc_card->iface->send_cmd(SDIO_CMD52_IO_RW_DIRECT,
		((uint32_t)*io_data << SDIO_CMD52_WR_DATA)
		| ((uint32_t)rw_flag << SDIO_CMD52_RW_FLAG)
		| ((uint32_t)func_nb << SDIO_CMD52_FUNCTION_NUM)
		| ((uint32_t)rd_after_wr << SDIO_CMD52_RAW_FLAG)
		| ((uint32_t)reg_addr << SDIO_CMD52_REG_ADRR))) {
		return false;
	}
	*io_data = sd_mmc_card->iface->get_response() & 0xFF;
	return true;
}

/**
 * \brief CMD53 - SDIO IO_RW_EXTENDED command
 * This implementation support only the SDIO multi-byte transfer mode which is
 * similar to the single block transfer on memory.
 * Note: The SDIO block transfer mode is optional for SDIO card.
 *
 * \param rw_flag   Direction, 1:write, 0:read.
 * \param func_nb   Number of the function.
 * \param reg_addr  Register address.
 * \param inc_addr  1:Incrementing address, 0: fixed.
 * \param size      Transfer data size.
 * \param access_block  true, if the block access (DMA) is used
 *
 * \return true if success, otherwise false
 */
static bool sdio_cmd53(uint8_t rw_flag, uint8_t func_nb, uint32_t reg_addr,
		uint8_t inc_addr, uint32_t size, bool access_block)
{
	Assert(size != 0);
	Assert(size <= 512);

	return sd_mmc_card->iface->adtc_start((rw_flag == SDIO_CMD53_READ_FLAG)?
			SDIO_CMD53_IO_R_BYTE_EXTENDED :
			SDIO_CMD53_IO_W_BYTE_EXTENDED,
			((size % 512) << SDIO_CMD53_COUNT)
			| ((uint32_t)reg_addr << SDIO_CMD53_REG_ADDR)
			| ((uint32_t)inc_addr << SDIO_CMD53_OP_CODE)
			| ((uint32_t)0 << SDIO_CMD53_BLOCK_MODE)
			| ((uint32_t)func_nb << SDIO_CMD53_FUNCTION_NUM)
			| ((uint32_t)rw_flag << SDIO_CMD53_RW_FLAG),
			size, 1, access_block);
}
#endif // SDIO_SUPPORT_ENABLE

/**
 * \brief ACMD6 - Define the data bus width to 4 bits bus
 *
 * \return true if success, otherwise false
 */
static bool sd_acmd6(uint8_t slot)
{
	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];
	// CMD55 - Indicate to the card that the next command is an
	// application specific command rather than a standard command.
	if (!sd_mmc_card->iface->send_cmd(SDMMC_CMD55_APP_CMD, (uint32_t)sd_mmc_card->rca << 16)) {
		return false;
	}
	// 10b = 4 bits bus
	if (!sd_mmc_card->iface->send_cmd(SD_ACMD6_SET_BUS_WIDTH, 0x2)) {
		return false;
	}
	sd_mmc_card->bus_width = 4;
	sd_mmc_debug("%d-bit bus width enabled.\n\r", (int)sd_mmc_card->bus_width);
	return true;
}

/**
 * \brief ACMD51 - Read the SD Configuration Register.
 *
 * \note
 * SD Card Configuration Register (SCR) provides information on the SD Memory
 * Card's special features that were configured into the given card. The size
 * of SCR register is 64 bits.
 *
 *
 * \return true if success, otherwise false
 */
static bool sd_acmd51(uint8_t slot)
{
#if SAME70
	static __nocache uint8_t scr_array[SD_MMC_MEM_CNT][SD_SCR_REG_BSIZE];
	uint8_t * const scr = scr_array[slot];
#else
	uint8_t scr[SD_SCR_REG_BSIZE];
#endif
	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];

	// CMD55 - Indicate to the card that the next command is an
	// application specific command rather than a standard command.
	if (!sd_mmc_card->iface->send_cmd(SDMMC_CMD55_APP_CMD, (uint32_t)sd_mmc_card->rca << 16)) {
		return false;
	}
	if (!sd_mmc_card->iface->adtc_start(SD_ACMD51_SEND_SCR, 0, SD_SCR_REG_BSIZE, 1, scr)) {
		return false;
	}
	if (!sd_mmc_card->iface->start_read_blocks(scr, 1)) {
		return false;
	}
	if (!sd_mmc_card->iface->wait_end_of_read_blocks()) {
		return false;
	}

	// Get SD Memory Card - Spec. Version
	switch (SD_SCR_SD_SPEC(scr)) {
	case SD_SCR_SD_SPEC_1_0_01:
		sd_mmc_card->version = CARD_VER_SD_1_0;
		break;

	case SD_SCR_SD_SPEC_1_10:
		sd_mmc_card->version = CARD_VER_SD_1_10;
		break;

	case SD_SCR_SD_SPEC_2_00:
		if (SD_SCR_SD_SPEC3(scr) == SD_SCR_SD_SPEC_3_00) {
			sd_mmc_card->version = CARD_VER_SD_3_0;
		} else {
			sd_mmc_card->version = CARD_VER_SD_2_0;
		}
		break;

	default:
		sd_mmc_card->version = CARD_VER_SD_1_0;
		break;
	}
	return true;
}

/**
 * \brief Select a card slot and initialize the associated driver
 *
 * \param slot  Card slot number
 *
 * \retval SD_MMC_ERR_SLOT     Wrong slot number
 * \retval SD_MMC_ERR_NO_CARD  No card present on slot
 * \retval SD_MMC_ERR_UNUSABLE Unusable card
 * \retval SD_MMC_INIT_ONGOING Card initialization requested
 * \retval SD_MMC_OK           Card present
 * \retval SD_MMC_CD_DEBOUNCING Giving the card time to be ready
 */
static sd_mmc_err_t sd_mmc_select_slot(uint8_t slot)
{
	if (slot >= SD_MMC_MEM_CNT) {
		return SD_MMC_ERR_SLOT;
	}
	Assert(sd_mmc_nb_block_remaining == 0);

#if 1	// dc42
	// RepRapFirmware now handles the card detect pin and debouncing, so ignore the card detect pin here
#else
	if (sd_mmc_cards[slot].cd_gpio != NoPin) {
		//! Card Detect pins
		if (digitalRead(sd_mmc_cards[slot].cd_gpio) != SD_MMC_CD_DETECT_VALUE) {
			if (sd_mmc_cards[slot].state == SD_MMC_CARD_STATE_DEBOUNCE) {
				SD_MMC_STOP_TIMEOUT();
			}
			sd_mmc_cards[slot].state = SD_MMC_CARD_STATE_NO_CARD;
			return SD_MMC_ERR_NO_CARD;
		}
		if (sd_mmc_cards[slot].state == SD_MMC_CARD_STATE_NO_CARD) {
			// A card plug on going, but this is not initialized
			sd_mmc_cards[slot].state = SD_MMC_CARD_STATE_DEBOUNCE;
			// Debounce + Power On Setup
			SD_MMC_START_TIMEOUT();
			return SD_MMC_CD_DEBOUNCING;
		}
		if (sd_mmc_cards[slot].state == SD_MMC_CARD_STATE_DEBOUNCE) {
			if (!SD_MMC_IS_TIMEOUT()) {
				// Debounce on going
				return SD_MMC_CD_DEBOUNCING;
			}
			// Card is not initialized
			sd_mmc_cards[slot].state = SD_MMC_CARD_STATE_INIT;
			// Set 1-bit bus width and low clock for initialization
			sd_mmc_cards[slot].clock = SDMMC_CLOCK_INIT;
			sd_mmc_cards[slot].bus_width = 1;
			sd_mmc_cards[slot].high_speed = 0;
		}
		if (sd_mmc_cards[slot].state == SD_MMC_CARD_STATE_UNUSABLE) {
			return SD_MMC_ERR_UNUSABLE;
		}
	}
	else
#endif
	{
		// No pin card detection, then always try to install it
		if ((sd_mmc_cards[slot].state == SD_MMC_CARD_STATE_NO_CARD)
				|| (sd_mmc_cards[slot].state == SD_MMC_CARD_STATE_UNUSABLE)) {
			// Card is not initialized
			sd_mmc_cards[slot].state = SD_MMC_CARD_STATE_INIT;
			// Set 1-bit bus width and low clock for initialization
			sd_mmc_cards[slot].clock = SDMMC_CLOCK_INIT;
			sd_mmc_cards[slot].bus_width = 1;
			sd_mmc_cards[slot].high_speed = 0;
		}
	}

	// Initialize interface
//	sd_mmc_slot_sel = slot;
//	sd_mmc_card = &sd_mmc_cards[slot];
	sd_mmc_configure_slot(slot);
	return (sd_mmc_cards[slot].state == SD_MMC_CARD_STATE_INIT) ?
			SD_MMC_INIT_ONGOING : SD_MMC_OK;
}

/**
 * \brief Configures the driver with the selected card configuration
 */
static __attribute__((warn_unused_result)) bool sd_mmc_configure_slot(uint8_t slot)
{
	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];
	return sd_mmc_card->iface->select_device(sd_mmc_card->slot, sd_mmc_card->clock, sd_mmc_card->bus_width, sd_mmc_card->high_speed);
}

/**
 * \brief Deselect the current card slot
 */
static void sd_mmc_deselect_slot(uint8_t slot)
{
	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];
	if (slot < SD_MMC_MEM_CNT) {
		sd_mmc_card->iface->deselect_device(sd_mmc_card->slot);
//		sd_mmc_slot_sel = 0xFF;					// No slot selected FIXME
	}
}

/**
 * \brief Initialize the SD card in SPI mode.
 *
 * \note
 * This function runs the initialization procedure and the identification
 * process, then it sets the SD/MMC card in transfer state.
 * At last, it will automatically enable maximum bus width and transfer speed.
 *
 * \return true if success, otherwise false
 */
static bool sd_mmc_spi_card_init(uint8_t slot)
{
	uint8_t v2 = 0;
	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];

	// In first, try to install SD/SDIO card
	sd_mmc_card->type = CARD_TYPE_SD;
	sd_mmc_card->version = CARD_VER_UNKNOWN;
	sd_mmc_card->rca = 0;
	sd_mmc_debug("Start SD card install\n\r");

	// Card need of 74 cycles clock minimum to start
	sd_mmc_card->iface->send_clock();

	// CMD0 - Reset all cards to idle state.
	if (!sd_mmc_card->iface->send_cmd(SDMMC_SPI_CMD0_GO_IDLE_STATE, 0)) {
		return false;
	}
	if (!sd_cmd8(&v2, slot)) {
		return false;
	}
	// Try to get the SDIO card's operating condition
	if (!sdio_op_cond()) {
		return false;
	}

	if (sd_mmc_card->type & CARD_TYPE_SD) {
		// Try to get the SD card's operating condition
		if (!sd_spi_op_cond(v2, slot)) {
			// It is not a SD card
			sd_mmc_debug("Start MMC Install\n\r");
			sd_mmc_card->type = CARD_TYPE_MMC;
			return sd_mmc_spi_install_mmc(slot);
		}

		/* The CRC on card is disabled by default.
		 * However, to be sure, the CRC OFF command is send.
		 * Unfortunately, specific SDIO card does not support it
		 * (H&D wireless card - HDG104 WiFi SIP)
		 * and the command is send only on SD card.
		 */
		if (!sd_mmc_card->iface->send_cmd(SDMMC_SPI_CMD59_CRC_ON_OFF, 0)) {
			return false;
		}
	}
	// SD MEMORY
	if (sd_mmc_card->type & CARD_TYPE_SD) {
		// Get the Card-Specific Data
		if (!sd_mmc_cmd9_spi(slot)) {
			return false;
		}
		sd_decode_csd(slot);
		// Read the SCR to get card version
		if (!sd_acmd51(slot)) {
			return false;
		}
	}
	if (IS_SDIO()) {
		if (!sdio_get_max_speed()) {
			return false;
		}
	}
	// SD MEMORY not HC, Set default block size
	if ((sd_mmc_card->type & CARD_TYPE_SD) &&
			(0 == (sd_mmc_card->type & CARD_TYPE_HC))) {
		if (!sd_mmc_card->iface->send_cmd(SDMMC_CMD16_SET_BLOCKLEN, SD_MMC_BLOCK_SIZE)) {
			return false;
		}
	}
	// Check communication
	if (sd_mmc_card->type & CARD_TYPE_SD) {
		if (!sd_mmc_cmd13(slot)) {
			return false;
		}
	}
	// Re-initialize the slot with the new speed
	return sd_mmc_configure_slot(slot);
}

/**
 * \brief Initialize the SD card in MCI mode.
 *
 * \note
 * This function runs the initialization procedure and the identification
 * process, then it sets the SD/MMC card in transfer state.
 * At last, it will automatically enable maximum bus width and transfer speed.
 *
 * \return true if success, otherwise false
 */
static bool sd_mmc_mci_card_init(uint8_t slot)
{
	uint8_t v2 = 0;
	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];

	// In first, try to install SD/SDIO card
	sd_mmc_card->type = CARD_TYPE_SD;
	sd_mmc_card->version = CARD_VER_UNKNOWN;
	sd_mmc_card->rca = 0;
	sd_mmc_debug("Start SD card install\n\r");

	// Card need of 74 cycles clock minimum to start
	sd_mmc_card->iface->send_clock();

	// CMD0 - Reset all cards to idle state.
	if (!sd_mmc_card->iface->send_cmd(SDMMC_MCI_CMD0_GO_IDLE_STATE, 0))
	{
		return false;
	}
	if (!sd_cmd8(&v2, slot))
	{
		return false;
	}
	// Try to get the SDIO card's operating condition
	if (!sdio_op_cond())
	{
		return false;
	}

	if (sd_mmc_card->type & CARD_TYPE_SD)
	{
		// Try to get the SD card's operating condition
		if (!sd_mci_op_cond(v2, slot))
		{
			// It is not a SD card
			sd_mmc_debug("Start MMC Install\n\r");
			sd_mmc_card->type = CARD_TYPE_MMC;
			return sd_mmc_mci_install_mmc(slot);
		}
	}

	if (sd_mmc_card->type & CARD_TYPE_SD)
	{
		// SD MEMORY, Put the Card in Identify Mode
		// Note: The CID is not used in this stack
		if (!sd_mmc_card->iface->send_cmd(SDMMC_CMD2_ALL_SEND_CID, 0))
		{
			return false;
		}
	}
	// Ask the card to publish a new relative address (RCA).
	if (!sd_mmc_card->iface->send_cmd(SD_CMD3_SEND_RELATIVE_ADDR, 0))
	{
		return false;
	}
	sd_mmc_card->rca = (sd_mmc_card->iface->get_response() >> 16) & 0xFFFF;

	// SD MEMORY, Get the Card-Specific Data
	if (sd_mmc_card->type & CARD_TYPE_SD)
	{
		if (!sd_mmc_cmd9_mci(slot))
		{
			return false;
		}
		sd_decode_csd(slot);
	}
	// Select the and put it into Transfer Mode
	if (!sd_mmc_card->iface->send_cmd(SDMMC_CMD7_SELECT_CARD_CMD, (uint32_t)sd_mmc_card->rca << 16))
	{
		return false;
	}
	// SD MEMORY, Read the SCR to get card version
	if (sd_mmc_card->type & CARD_TYPE_SD)
	{
		if (!sd_acmd51(slot))
		{
			return false;
		}
	}
	if (IS_SDIO())
	{
		if (!sdio_get_max_speed())
		{
			return false;
		}
	}
	if ((4 <= sd_mmc_card->iface->get_bus_width(sd_mmc_card->slot)))
	{
		// TRY to enable 4-bit mode
		if (IS_SDIO())
		{
			if (!sdio_cmd52_set_bus_width())
			{
				return false;
			}
		}
		if (sd_mmc_card->type & CARD_TYPE_SD)
		{
			if (!sd_acmd6(slot))
			{
				return false;
			}
		}
		// Switch to selected bus mode
		if (!sd_mmc_configure_slot(slot))
		{
			return false;
		}
	}
	if (sd_mmc_card->iface->is_high_speed_capable())
	{
		// TRY to enable High-Speed Mode
		if (IS_SDIO())
		{
			if (!sdio_cmd52_set_high_speed())
			{
				return false;
			}
		}
		if (sd_mmc_card->type & CARD_TYPE_SD)
		{
			if (sd_mmc_card->version > CARD_VER_SD_1_0)
			{
				if (!sd_cm6_set_high_speed(slot))
				{
					return false;
				}
			}
		}
		// Valid new configuration
		if (!sd_mmc_configure_slot(slot))
		{
			return false;
		}
	}
	// SD MEMORY, Set default block size
	if (sd_mmc_card->type & CARD_TYPE_SD)
	{
		if (!sd_mmc_card->iface->send_cmd(SDMMC_CMD16_SET_BLOCKLEN, SD_MMC_BLOCK_SIZE))
		{
			return false;
		}
	}
	return true;
}

/**
 * \brief Initialize the MMC card in SPI mode.
 *
 * \note
 * This function runs the initialization procedure and the identification
 * process, then it sets the SD/MMC card in transfer state.
 * At last, it will automatically enable maximum bus width and transfer speed.
 *
 * \return true if success, otherwise false
 */
static bool sd_mmc_spi_install_mmc(uint8_t slot)
{
	uint8_t b_authorize_high_speed;
	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];

	// CMD0 - Reset all cards to idle state.
	if (!sd_mmc_card->iface->send_cmd(SDMMC_SPI_CMD0_GO_IDLE_STATE, 0)) {
		return false;
	}

	if (!mmc_spi_op_cond(slot)) {
		return false;
	}

	// Disable CRC check for SPI mode
	if (!sd_mmc_card->iface->send_cmd(SDMMC_SPI_CMD59_CRC_ON_OFF, 0)) {
		return false;
	}
	// Get the Card-Specific Data
	if (!sd_mmc_cmd9_spi(slot)) {
		return false;
	}
	mmc_decode_csd(slot);
	// For MMC 4.0 Higher version
	if (sd_mmc_card->version >= CARD_VER_MMC_4) {
		// Get EXT_CSD
		if (!mmc_cmd8(&b_authorize_high_speed, slot)) {
			return false;
		}
	}
	// Set default block size
	if (!sd_mmc_card->iface->send_cmd(SDMMC_CMD16_SET_BLOCKLEN, SD_MMC_BLOCK_SIZE)) {
		return false;
	}
	// Check communication
	if (!sd_mmc_cmd13(slot)) {
		return false;
	}
	// Re-initialize the slot with the new speed
	return sd_mmc_configure_slot(slot);
}


/**
 * \brief Initialize the MMC card in MCI mode.
 *
 * \note
 * This function runs the initialization procedure and the identification
 * process, then it sets the SD/MMC card in transfer state.
 * At last, it will automatically enable maximum bus width and transfer speed.
 *
 * \return true if success, otherwise false
 */
static bool sd_mmc_mci_install_mmc(uint8_t slot)
{
	uint8_t b_authorize_high_speed;
	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];

	// CMD0 - Reset all cards to idle state.
	if (!sd_mmc_card->iface->send_cmd(SDMMC_MCI_CMD0_GO_IDLE_STATE, 0))
	{
		return false;
	}

	if (!mmc_mci_op_cond(slot))
	{
		return false;
	}

	// Put the Card in Identify Mode
	// Note: The CID is not used in this stack
	if (!sd_mmc_card->iface->send_cmd(SDMMC_CMD2_ALL_SEND_CID, 0))
	{
		return false;
	}
	// Assign relative address to the card.
	sd_mmc_card->rca = 1;
	if (!sd_mmc_card->iface->send_cmd(MMC_CMD3_SET_RELATIVE_ADDR, (uint32_t)sd_mmc_card->rca << 16))
	{
		return false;
	}
	// Get the Card-Specific Data
	if (!sd_mmc_cmd9_mci(slot))
	{
		return false;
	}
	mmc_decode_csd(slot);
	// Select the and put it into Transfer Mode
	if (!sd_mmc_card->iface->send_cmd(SDMMC_CMD7_SELECT_CARD_CMD, (uint32_t)sd_mmc_card->rca << 16))
	{
		return false;
	}
	if (sd_mmc_card->version >= CARD_VER_MMC_4)
	{
		// For MMC 4.0 Higher version
		// Get EXT_CSD
		if (!mmc_cmd8(&b_authorize_high_speed, slot))
		{
			return false;
		}
		if (4 <= sd_mmc_card->iface->get_bus_width(sd_mmc_card->slot))
		{
			// Enable more bus width
			if (!mmc_cmd6_set_bus_width(sd_mmc_card->iface->get_bus_width(sd_mmc_card->slot), slot))
			{
				return false;
			}
			// Re-initialize the slot with the bus width
			if (!sd_mmc_configure_slot(slot))
			{
				return false;
			}
		}
		if (sd_mmc_card->iface->is_high_speed_capable() && b_authorize_high_speed)
		{
			// Enable HS
			if (!mmc_cmd6_set_high_speed(slot))
			{
				return false;
			}
			// Re-initialize the slot with the new speed
			if (!sd_mmc_configure_slot(slot))
			{
				return false;
			}
		}
	}
	else
	{
		// Re-initialize the slot with the new speed
		if (!sd_mmc_configure_slot(slot))
		{
			return false;
		}
	}

	uint8_t retry = 10;
	while (retry--)
	{
		// Retry is a WORKAROUND for no compliance card (Atmel Internal ref. MMC19):
		// These cards seem not ready immediately
		// after the end of busy of mmc_cmd6_set_high_speed()

		// Set default block size
		if (sd_mmc_card->iface->send_cmd(SDMMC_CMD16_SET_BLOCKLEN, SD_MMC_BLOCK_SIZE))
		{
			return true;
		}
	}
	return false;
}

//-------------------------------------------------------------------
//--------------------- PUBLIC FUNCTIONS ----------------------------

void sd_mmc_init(const Pin wpPins[], const Pin spiCsPins[]) noexcept
{
	for (size_t slot = 0; slot < SD_MMC_MEM_CNT; slot++)
	{
		struct sd_mmc_card *card = &sd_mmc_cards[slot];
		card->state = SD_MMC_CARD_STATE_NO_CARD;
#if SUPPORT_WRITE_PROTECT
		card->wp_gpio = wpPins[slot];
		if (card->wp_gpio != NoPin)
		{
			pinMode(card->wp_gpio, INPUT_PULLUP);
		}
#endif
#if SD_MMC_HSMCI_MEM_CNT != 0
		if (slot < SD_MMC_HSMCI_MEM_CNT)
		{
			card->iface = &hsmciInterface;
			card->slot = slot;
		}
		else
#endif
		{
#if (SD_MMC_SPI_MEM_CNT != 0)
			card->iface = &spiInterface;
			card->slot = slot - SD_MMC_HSMCI_MEM_CNT;
#endif
		}
	}
//	sd_mmc_slot_sel = 0xFF;					// No slot selected

#if SD_MMC_HSMCI_MEM_CNT != 0
# if SAME5x
	hsmci_init(SdhcDevice, SdhcIRQn);
# else
	hsmci_init();
# endif
#endif

#if SD_MMC_SPI_MEM_CNT != 0
	sd_mmc_spi_init(spiCsPins);
#endif
}

uint8_t sd_mmc_nb_slot(void) noexcept
{
	return SD_MMC_MEM_CNT;
}

// Check that the card is ready and initialise it if necessary
// The card is not selected on entry or at exit
sd_mmc_err_t sd_mmc_check(uint8_t slot) noexcept
{
	sd_mmc_err_t sd_mmc_err = sd_mmc_select_slot(slot);
	if (sd_mmc_err != SD_MMC_INIT_ONGOING)
	{
		sd_mmc_deselect_slot(slot);
		return sd_mmc_err;
	}

	// Initialization of the card requested
	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];
	if (sd_mmc_card->iface->is_spi ? sd_mmc_spi_card_init(slot) : sd_mmc_mci_card_init(slot)) {
		sd_mmc_debug("SD/MMC card ready\n\r");
		sd_mmc_card->state = SD_MMC_CARD_STATE_READY;
		sd_mmc_deselect_slot(slot);
		// To notify that the card has been just initialized
		// It is necessary for USB Device MSC
		return SD_MMC_INIT_ONGOING;
	}
	sd_mmc_debug("SD/MMC card initialization failed\n\r");
	sd_mmc_card->state = SD_MMC_CARD_STATE_UNUSABLE;
	sd_mmc_deselect_slot(slot);
	return SD_MMC_ERR_UNUSABLE;
}

card_type_t sd_mmc_get_type(uint8_t slot) noexcept
{
	if (SD_MMC_OK != sd_mmc_select_slot(slot)) {
		return CARD_TYPE_UNKNOWN;
	}
	sd_mmc_deselect_slot(slot);
	return sd_mmc_cards[slot].type;
}

card_version_t sd_mmc_get_version(uint8_t slot) noexcept
{
	if (SD_MMC_OK != sd_mmc_select_slot(slot)) {
		return CARD_VER_UNKNOWN;
	}
	sd_mmc_deselect_slot(slot);
	return sd_mmc_cards[slot].version;
}

uint32_t sd_mmc_get_capacity(uint8_t slot) noexcept
{
#if 1 // This will only check for already present data. The old code below is unsafe if another task is accessing data already.
	if (slot < SD_MMC_MEM_CNT && sd_mmc_cards[slot].state == SD_MMC_CARD_STATE_READY)
	{
		return sd_mmc_cards[slot].capacity;
	}
	else
	{
		return 0;
	}
#else
	if (SD_MMC_OK != sd_mmc_select_slot(slot)) {
		return 0;
	}
	sd_mmc_deselect_slot(slot);
	return sd_mmc_cards[slot].capacity;
#endif
}

#if SUPPORT_WRITE_PROTECT
bool sd_mmc_is_write_protected(uint8_t slot)
{
	return sd_mmc_cards[slot].wp_gpio != NoPin && digitalRead(sd_mmc_cards[slot].wp_gpio) == SD_MMC_WP_DETECT_VALUE;
}
#endif

#if 1	// dc42

// Unmount the card. Must call this to force it to be re-initialised when changing card.
void sd_mmc_unmount(uint8_t slot) noexcept
{
	sd_mmc_cards[slot].state = SD_MMC_CARD_STATE_NO_CARD;
}

// Get the interface speed in bytes/sec
uint32_t sd_mmc_get_interface_speed(uint8_t slot) noexcept
{
	return sd_mmc_cards[slot].iface->getInterfaceSpeed();
}

#endif

#if SD_MMC_SPI_MEM_CNT != 0

// Change the CS pin used by an SPI-connected card slot. Only used by the Duet 3 MB6HC. Linker garbage collection will eliminate this function in other builds.
void sd_mmc_change_cs_pin(uint8_t slot, Pin csPin) noexcept
{
	if (slot >= SD_MMC_HSMCI_MEM_CNT)
	{
		sd_mmc_spi_change_cs_pin(slot - SD_MMC_HSMCI_MEM_CNT, csPin);
	}
}

#endif

// Initialise for reading blocks
// On entry the card is not selected
// If SD_MMC_OK is returned then the card is selected, otherwise it is not selected
sd_mmc_err_t sd_mmc_init_read_blocks(uint8_t slot, uint32_t start, uint16_t nb_block, void *dmaAddr) noexcept
{
	sd_mmc_err_t sd_mmc_err;
	uint32_t cmd, arg, resp;

	sd_mmc_err = sd_mmc_select_slot(slot);
	if (sd_mmc_err != SD_MMC_OK) {
		return sd_mmc_err;
	}

	// Wait for data ready status
	if (!sd_mmc_cmd13(slot)) {
		sd_mmc_deselect_slot(slot);
		return SD_MMC_ERR_COMM;
	}

	if (nb_block > 1) {
		cmd = SDMMC_CMD18_READ_MULTIPLE_BLOCK;
	} else {
		cmd = SDMMC_CMD17_READ_SINGLE_BLOCK;
	}

	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];

	/*
	 * SDSC Card (CCS=0) uses byte unit address,
	 * SDHC and SDXC Cards (CCS=1) use block unit address (512 Bytes unit).
	 */
	if (sd_mmc_card->type & CARD_TYPE_HC) {
		arg = start;
	} else {
		arg = (start * SD_MMC_BLOCK_SIZE);
	}

	if (!sd_mmc_card->iface->adtc_start(cmd, arg, SD_MMC_BLOCK_SIZE, nb_block, dmaAddr)) {
		sd_mmc_deselect_slot(slot);
		return SD_MMC_ERR_COMM;
	}
	// Check response
	if (!sd_mmc_card->iface->is_spi) {
		resp = sd_mmc_card->iface->get_response();
		if (resp & CARD_STATUS_ERR_RD_WR) {
			sd_mmc_debug("%s: Read blocks %02d resp32 0x%08x CARD_STATUS_ERR_RD_WR\n\r",
					__func__, (int)SDMMC_CMD_GET_INDEX(cmd), resp);
			sd_mmc_deselect_slot(slot);
			return SD_MMC_ERR_COMM;
		}
	}
	sd_mmc_nb_block_remaining[slot] = nb_block;
	sd_mmc_nb_block_to_tranfer[slot] = nb_block;
	return SD_MMC_OK;
}

// Start reading blocks
// On entry the card is selected
// If SD_MMC_OK is returned then the card is selected, otherwise it is not selected
sd_mmc_err_t sd_mmc_start_read_blocks(void *dest, uint16_t nb_block, uint8_t slot) noexcept
{
	Assert(sd_mmc_nb_block_remaining[slot] >= nb_block);

	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];
	if (!sd_mmc_card->iface->start_read_blocks(dest, nb_block)) {
		sd_mmc_nb_block_remaining[slot] = 0;
		sd_mmc_deselect_slot(slot);
		return SD_MMC_ERR_COMM;
	}
	sd_mmc_nb_block_remaining[slot] -= nb_block;
	return SD_MMC_OK;
}

// Wait until all blocks have been read
// On entry the device is selected
// On return it is not selected
sd_mmc_err_t sd_mmc_wait_end_of_read_blocks(bool abort, uint8_t slot) noexcept
{
	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];
	if (!sd_mmc_card->iface->wait_end_of_read_blocks()) {
		sd_mmc_deselect_slot(slot);
		return SD_MMC_ERR_COMM;
	}
	if (abort) {
		sd_mmc_nb_block_remaining[slot] = 0;
	} else if (sd_mmc_nb_block_remaining[slot]) {
		sd_mmc_deselect_slot(slot);
		return SD_MMC_OK;
	}

	// All blocks are transfered then stop read operation
	if (sd_mmc_nb_block_to_tranfer[slot] == 1) {
		// Single block transfer, then nothing to do
		sd_mmc_deselect_slot(slot);
		return SD_MMC_OK;
	}
	// WORKAROUND for no compliance card (Atmel Internal ref. !MMC7 !SD19):
	// The errors on this command must be ignored
	// and one retry can be necessary in SPI mode for no compliance card.
	if (!sd_mmc_card->iface->adtc_stop(SDMMC_CMD12_STOP_TRANSMISSION, 0)) {
		sd_mmc_card->iface->adtc_stop(SDMMC_CMD12_STOP_TRANSMISSION, 0);
	}
	sd_mmc_deselect_slot(slot);
	return SD_MMC_OK;
}

// Initialise for writing blocks
// On entry the card is not selected
// If SD_MMC_OK is returned then the card is selected, otherwise it is not selected
sd_mmc_err_t sd_mmc_init_write_blocks(uint8_t slot, uint32_t start, uint16_t nb_block, const void *dmaAddr) noexcept
{
	sd_mmc_err_t sd_mmc_err;
	uint32_t cmd, arg, resp;

	sd_mmc_err = sd_mmc_select_slot(slot);
	if (sd_mmc_err != SD_MMC_OK) {
		return sd_mmc_err;
	}

#if SUPPORT_WRITE_PROTECT
	if (sd_mmc_is_write_protected(slot)) {
		sd_mmc_deselect_slot(slot);
		return SD_MMC_ERR_WP;
	}
#endif

	if (nb_block > 1) {
		cmd = SDMMC_CMD25_WRITE_MULTIPLE_BLOCK;
	} else {
		cmd = SDMMC_CMD24_WRITE_BLOCK;
	}

	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];

	/*
	 * SDSC Card (CCS=0) uses byte unit address,
	 * SDHC and SDXC Cards (CCS=1) use block unit address (512 Bytes unit).
	 */
	if (sd_mmc_card->type & CARD_TYPE_HC) {
		arg = start;
	} else {
		arg = (start * SD_MMC_BLOCK_SIZE);
	}
	if (!sd_mmc_card->iface->adtc_start(cmd, arg, SD_MMC_BLOCK_SIZE, nb_block, dmaAddr)) {
		sd_mmc_deselect_slot(slot);
		return SD_MMC_ERR_COMM;
	}
	// Check response
	if (!sd_mmc_card->iface->is_spi) {
		resp = sd_mmc_card->iface->get_response();
		if (resp & CARD_STATUS_ERR_RD_WR) {
			sd_mmc_debug("%s: Write blocks %02d r1 0x%08x CARD_STATUS_ERR_RD_WR\n\r",
					__func__, (int)SDMMC_CMD_GET_INDEX(cmd), resp);
			sd_mmc_deselect_slot(slot);
			return SD_MMC_ERR_COMM;
		}
	}
	sd_mmc_nb_block_remaining[slot] = nb_block;
	sd_mmc_nb_block_to_tranfer[slot] = nb_block;
	return SD_MMC_OK;
}

// Start writing blocks
// On entry the card is selected
// If SD_MMC_OK is returned then the card is selected, otherwise it is not selected
sd_mmc_err_t sd_mmc_start_write_blocks(const void *src, uint16_t nb_block, uint8_t slot) noexcept
{
	Assert(sd_mmc_nb_block_remaining[slot] >= nb_block);
	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];
	if (!sd_mmc_card->iface->start_write_blocks(src, nb_block)) {
		sd_mmc_nb_block_remaining[slot] = 0;
		sd_mmc_deselect_slot(slot);
		return SD_MMC_ERR_COMM;
	}
	sd_mmc_nb_block_remaining[slot] -= nb_block;
	return SD_MMC_OK;
}

// Wait until all blocks have been written
// On entry the device is selected
// On return it is not selected
sd_mmc_err_t sd_mmc_wait_end_of_write_blocks(bool abort, uint8_t slot) noexcept
{
	struct sd_mmc_card * const sd_mmc_card = &sd_mmc_cards[slot];
	if (!sd_mmc_card->iface->wait_end_of_write_blocks()) {
		sd_mmc_deselect_slot(slot);
		return SD_MMC_ERR_COMM;
	}
	if (abort) {
		sd_mmc_nb_block_remaining[slot] = 0;
	} else if (sd_mmc_nb_block_remaining[slot]) {
		sd_mmc_deselect_slot(slot);
		return SD_MMC_OK;
	}

	// All blocks are transfered then stop write operation
	if (sd_mmc_nb_block_to_tranfer[slot] == 1) {
		// Single block transfer, then nothing to do
		sd_mmc_deselect_slot(slot);
		return SD_MMC_OK;
	}

	if (!sd_mmc_card->iface->is_spi) {
		// Note: SPI multi block writes terminate using a special
		// token, not a STOP_TRANSMISSION request.
		if (!sd_mmc_card->iface->adtc_stop(SDMMC_CMD12_STOP_TRANSMISSION, 0)) {
			sd_mmc_deselect_slot(slot);
			return SD_MMC_ERR_COMM;
		}
	}
	sd_mmc_deselect_slot(slot);
	return SD_MMC_OK;
}

#ifdef SDIO_SUPPORT_ENABLE
sd_mmc_err_t sdio_read_direct(uint8_t slot, uint8_t func_num, uint32_t addr, uint8_t *dest) noexcept
{
	sd_mmc_err_t sd_mmc_err;

	if (dest == NULL) {
		return SD_MMC_ERR_PARAM;
	}

	sd_mmc_err = sd_mmc_select_slot(slot);
	if (sd_mmc_err != SD_MMC_OK) {
		return sd_mmc_err;
	}

	if (!sdio_cmd52(SDIO_CMD52_READ_FLAG, func_num, addr, 0, dest)) {
		sd_mmc_deselect_slot();
		return SD_MMC_ERR_COMM;
	}
	sd_mmc_deselect_slot();
	return SD_MMC_OK;
}

sd_mmc_err_t sdio_write_direct(uint8_t slot, uint8_t func_num, uint32_t addr, uint8_t data) noexcept
{
	sd_mmc_err_t sd_mmc_err;

	sd_mmc_err = sd_mmc_select_slot(slot);
	if (sd_mmc_err != SD_MMC_OK) {
		return sd_mmc_err;
	}

	if (!sdio_cmd52(SDIO_CMD52_WRITE_FLAG, func_num, addr, 0, &data)) {
		sd_mmc_deselect_slot();
		return SD_MMC_ERR_COMM;
	}

	sd_mmc_deselect_slot();
	return SD_MMC_OK;
}

sd_mmc_err_t sdio_read_extended(uint8_t slot, uint8_t func_num, uint32_t addr, uint8_t inc_addr, uint8_t *dest, uint16_t size) noexcept
{
	sd_mmc_err_t sd_mmc_err;

	if ((size == 0) || (size > 512)) {
		return SD_MMC_ERR_PARAM;
	}

	sd_mmc_err = sd_mmc_select_slot(slot);
	if (sd_mmc_err != SD_MMC_OK) {
		return sd_mmc_err;
	}

	if (!sdio_cmd53(SDIO_CMD53_READ_FLAG, func_num, addr, inc_addr,
			size, true)) {
		sd_mmc_deselect_slot();
		return SD_MMC_ERR_COMM;
	}
	if (!sd_mmc_card->iface->start_read_blocks(dest, 1)) {
		sd_mmc_deselect_slot();
		return SD_MMC_ERR_COMM;
	}
	if (!sd_mmc_card->iface->wait_end_of_read_blocks()) {
		sd_mmc_deselect_slot();
		return SD_MMC_ERR_COMM;
	}

	sd_mmc_deselect_slot();
	return SD_MMC_OK;
}

sd_mmc_err_t sdio_write_extended(uint8_t slot, uint8_t func_num, uint32_t addr, uint8_t inc_addr, uint8_t *src, uint16_t size) noexcept
{
	sd_mmc_err_t sd_mmc_err;

	if ((size == 0) || (size > 512)) {
		return SD_MMC_ERR_PARAM;
	}

	sd_mmc_err = sd_mmc_select_slot(slot);
	if (sd_mmc_err != SD_MMC_OK) {
		return sd_mmc_err;
	}

	if (!sdio_cmd53(SDIO_CMD53_WRITE_FLAG, func_num, addr, inc_addr,
			size, true)) {
		sd_mmc_deselect_slot();
		return SD_MMC_ERR_COMM;
	}
	if (!sd_mmc_card->iface->start_write_blocks(src, 1)) {
		sd_mmc_deselect_slot();
		return SD_MMC_ERR_COMM;
	}
	if (!sd_mmc_card->iface->wait_end_of_write_blocks()) {
		sd_mmc_deselect_slot();
		return SD_MMC_ERR_COMM;
	}

	sd_mmc_deselect_slot();
	return SD_MMC_OK;
}
#endif // SDIO_SUPPORT_ENABLE

//! @}
