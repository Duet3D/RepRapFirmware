/**
 * \file
 *
 * \brief Common SPI interface for SD/MMC stack
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
#include <Core.h>
#include "conf_sd_mmc.h"

#if SD_MMC_SPI_MEM_CNT != 0

#include "sd_mmc_protocol.h"
#include "sd_mmc_spi.h"
#include "sd_mmc.h"
#include <cstring>

#include <Hardware/Spi/SharedSpiClient.h>
#include <Hardware/Spi/SharedSpiDevice.h>
#include <General/Portability.h>

// Enable debug information for SD/MMC SPI module
//#define SD_MMC_SPI_DEBUG
#ifdef SD_MMC_SPI_DEBUG
extern void debugPrintf(const char* fmt, ...) noexcept;
#define sd_mmc_spi_debug(...)      debugPrintf(__VA_ARGS__)
#else
#define sd_mmc_spi_debug(...)
#endif

//! Internal global error status
static sd_mmc_spi_errno_t sd_mmc_spi_err;

//! Slot array of SPI structures
static SharedSpiClient *sd_mmc_spi_devices[SD_MMC_SPI_MEM_CNT] = { 0 };
static SharedSpiClient *currentSpiClient = nullptr;

//! 32 bits response of the last command
static uint32_t sd_mmc_spi_response_32;
//! Current position (byte) of the transfer started by mci_adtc_start()
static uint32_t sd_mmc_spi_transfert_pos;
//! Size block requested by last mci_adtc_start()
static uint16_t sd_mmc_spi_block_size;
//! Total number of block requested by last mci_adtc_start()
static uint16_t sd_mmc_spi_nb_block;

static uint8_t sd_mmc_spi_crc7(uint8_t * buf, uint8_t size) noexcept;
static bool sd_mmc_spi_wait_busy() noexcept;
static bool sd_mmc_spi_start_read_block() noexcept;
static void sd_mmc_spi_stop_read_block() noexcept;
static void sd_mmc_spi_start_write_block() noexcept;
static bool sd_mmc_spi_stop_write_block() noexcept;
static bool sd_mmc_spi_stop_multiwrite_block() noexcept;

/**
 * \brief Calculates the CRC7
 *
 * \param buf     Buffer data to compute
 * \param size    Size of buffer data
 *
 * \return CRC7 computed
 */
static uint8_t sd_mmc_spi_crc7(uint8_t * buf, uint8_t size) noexcept
{
	uint8_t crc, value, i;

	crc = 0;
	while (size--)
	{
		value = *buf++;
		for (i = 0; i < 8; i++) {
			crc <<= 1;
#if 1	// DC
			if ((value ^ crc) & 0x80)
#else
			if ((value & 0x80) ^ (crc & 0x80))
#endif
			{
				crc ^= 0x09;
			}
			value <<= 1;
		}
	}
	crc = (crc << 1) | 1;
	return crc;
}

/**
 * \brief Wait the end of busy on DAT0 line
 *
 * \return true if success, otherwise false
 */
static bool sd_mmc_spi_wait_busy() noexcept
{
	uint8_t line = 0xFF;

	/* Delay before check busy
	 * Nbr timing minimum = 8 cylces
	 */
	currentSpiClient->ReadPacket(&line, 1);

	/* Wait end of busy signal
	 * Nec timing: 0 to unlimited
	 * However a timeout is used.
	 * 200 000 * 8 cycles
	 */
	uint32_t nec_timeout = 200000;
	currentSpiClient->ReadPacket(&line, 1);
	do {
		currentSpiClient->ReadPacket(&line, 1);
		if (!(nec_timeout--))
		{
			return false;
		}
	} while (line != 0xFF);
	return true;
}

/**
 * \brief Sends the correct TOKEN on the line to start a read block transfer
 *
 * \return true if success, otherwise false
 *         with a update of \ref sd_mmc_spi_err.
 */
static bool sd_mmc_spi_start_read_block() noexcept
{
	uint32_t i;
	uint8_t token;

	Assert(!(sd_mmc_spi_transfert_pos % sd_mmc_spi_block_size));

	/* Wait for start data token:
	 * The read timeout is the Nac timing.
	 * Nac must be computed trough CSD values,
	 * or it is 100ms for SDHC / SDXC
	 * Compute the maximum timeout:
	 * Frequency maximum = 25MHz
	 * 1 byte = 8 cycles
	 * 100ms = 312500 x sd_mmc_spi_drv_read_packet() maximum
	 */
	token = 0;
	i = 500000;
	do {
		if (i-- == 0)
		{
			sd_mmc_spi_err = SD_MMC_SPI_ERR_READ_TIMEOUT;
			sd_mmc_spi_debug("%s: Read blocks timeout\n\r", __func__);
			return false;
		}
		currentSpiClient->ReadPacket(&token, 1);
		if (SPI_TOKEN_DATA_ERROR_VALID(token))
		{
			Assert(SPI_TOKEN_DATA_ERROR_ERRORS & token);
			if (token & (SPI_TOKEN_DATA_ERROR_ERROR
					| SPI_TOKEN_DATA_ERROR_ECC_ERROR
					| SPI_TOKEN_DATA_ERROR_CC_ERROR)) {
				sd_mmc_spi_debug("%s: CRC data error token\n\r", __func__);
				sd_mmc_spi_err = SD_MMC_SPI_ERR_READ_CRC;
			}
			else
			{
				sd_mmc_spi_debug("%s: Out of range data error token\n\r", __func__);
				sd_mmc_spi_err = SD_MMC_SPI_ERR_OUT_OF_RANGE;
			}
			return false;
		}
	} while (token != SPI_TOKEN_SINGLE_MULTI_READ);

	return true;
}

/**
 * \brief Executed the end of a read block transfer
 */
static void sd_mmc_spi_stop_read_block() noexcept
{
	uint8_t crc[2];
	// Read 16-bit CRC (not checked)
	currentSpiClient->ReadPacket(crc, 2);
}

/**
 * \brief Sends the correct TOKEN on the line to start a write block transfer
 */
static void sd_mmc_spi_start_write_block() noexcept
{
	uint8_t dummy = 0xFF;
	Assert(!(sd_mmc_spi_transfert_pos % sd_mmc_spi_block_size));
	// Delay before start write block:
	// Nwr timing minimum = 8 cycles
	currentSpiClient->WritePacket(&dummy, 1);
	// Send start token
	uint8_t token;
	if (1 == sd_mmc_spi_nb_block)
	{
		token = SPI_TOKEN_SINGLE_WRITE;
	}
	else
	{
		token = SPI_TOKEN_MULTI_WRITE;
	}
	currentSpiClient->WritePacket(&token, 1);
}

/**
 * \brief Waits the TOKEN which notify the end of write block transfer
 *
 * \return true if success, otherwise false
 *         with a update of \ref sd_mmc_spi_err.
 */
static bool sd_mmc_spi_stop_write_block() noexcept
{
	uint8_t resp;
	uint16_t crc;

	// Send CRC
	crc = 0xFFFF; /// CRC is disabled in SPI mode
	currentSpiClient->WritePacket((uint8_t *)&crc, 2);
	// Receive data response token
	currentSpiClient->ReadPacket(&resp, 1);
	if (!SPI_TOKEN_DATA_RESP_VALID(resp))
	{
		sd_mmc_spi_err = SD_MMC_SPI_ERR;
		sd_mmc_spi_debug("%s: Invalid Data Response Token 0x%x\n\r", __func__, resp);
		return false;
	}
	// Check data response
	switch (SPI_TOKEN_DATA_RESP_CODE(resp))
	{
	case SPI_TOKEN_DATA_RESP_ACCEPTED:
		break;
	case SPI_TOKEN_DATA_RESP_CRC_ERR:
		sd_mmc_spi_err = SD_MMC_SPI_ERR_WRITE_CRC;
		sd_mmc_spi_debug("%s: Write blocks, SD_MMC_SPI_ERR_CRC, resp 0x%x\n\r", __func__, resp);
		return false;
	case SPI_TOKEN_DATA_RESP_WRITE_ERR:
	default:
		sd_mmc_spi_err = SD_MMC_SPI_ERR_WRITE;
		sd_mmc_spi_debug("%s: Write blocks SD_MMC_SPI_ERR_WR, resp 0x%x\n\r", __func__, resp);
		return false;
	}
	return true;
}

/**
 * \brief Executed the end of a multi blocks write transfer
 *
 * \return true if success, otherwise false
 *         with a update of \ref sd_mmc_spi_err.
 */
static bool sd_mmc_spi_stop_multiwrite_block() noexcept
{
	uint8_t value;

	if (1 == sd_mmc_spi_nb_block)
	{
		return true; // Single block write
	}
	if (sd_mmc_spi_nb_block > (sd_mmc_spi_transfert_pos / sd_mmc_spi_block_size))
	{
		return true; // It is not the End of multi write
	}

	// Delay before start write block:
	// Nwr timing minimum = 8 cylces
	value = 0xFF;
	currentSpiClient->WritePacket(&value, 1);
	// Send stop token
	value = SPI_TOKEN_STOP_TRAN;
	currentSpiClient->WritePacket(&value, 1);
	// Wait busy
	if (!sd_mmc_spi_wait_busy())
	{
		sd_mmc_spi_err = SD_MMC_SPI_ERR_WRITE_TIMEOUT;
		sd_mmc_spi_debug("%s: Stop write blocks timeout\n\r", __func__);
		return false;
	}
	return true;
}


//-------------------------------------------------------------------
//--------------------- PUBLIC FUNCTIONS ----------------------------

// Get the speed of the SPI SD card interface for reporting purposes, in bytes/sec
uint32_t spi_mmc_get_speed() noexcept
{
	return SD_MMC_SPI_MAX_CLOCK/8;
}

static spiIdleFunc_t spiIdleFunc = NULL;

// Set the idle function and return the old one
spiIdleFunc_t sd_mmc_spi_set_idle_func(spiIdleFunc_t p) noexcept
{
	spiIdleFunc_t ret = spiIdleFunc;
	spiIdleFunc = p;
	return ret;
}

sd_mmc_spi_errno_t sd_mmc_spi_get_errno() noexcept
{
	return sd_mmc_spi_err;
}

void sd_mmc_spi_init(const Pin csPins[SD_MMC_SPI_MEM_CNT]) noexcept
{
	sd_mmc_spi_err = SD_MMC_SPI_NO_ERR;

	// Initialize SPI interface and enable it
	for (size_t i = 0; i < SD_MMC_SPI_MEM_CNT; ++i)
	{
		sd_mmc_spi_devices[i] = new SharedSpiClient(SharedSpiDevice::GetMainSharedSpiDevice(), SD_MMC_SPI_MAX_CLOCK, SpiMode::mode0, csPins[i], false);
	}
}

#if 1

// This function is used by the Duet 3 MB6HC to enable support for a second SD card
void sd_mmc_spi_change_cs_pin(uint8_t spiSlot, Pin csPin) noexcept
{
	if (spiSlot < SD_MMC_SPI_MEM_CNT)
	{
		sd_mmc_spi_devices[spiSlot]->SetCsPin(csPin);
	}
}

#endif

// Note, every call to sd_mmc_spi_select_device must be matched to a call to sd_mmc_spi_deselect_device so that the SPI mutex gets released!
// Unfortunately, sd_mmc.c calls this more than one without deselecting it in between. So check whether it is already selected.
bool sd_mmc_spi_select_device(uint8_t slot, uint32_t clock, uint8_t bus_width, bool high_speed) noexcept
{
	UNUSED(bus_width);
	UNUSED(high_speed);
	sd_mmc_spi_err = SD_MMC_SPI_NO_ERR;

	if (currentSpiClient == nullptr)
	{
		if (clock > SD_MMC_SPI_MAX_CLOCK)
		{
			clock = SD_MMC_SPI_MAX_CLOCK;
		}

		currentSpiClient = sd_mmc_spi_devices[slot];
		currentSpiClient->SetClockFrequency(clock);
		currentSpiClient->Select();
		delayMicroseconds(1);
	}
	return true;
}

void sd_mmc_spi_deselect_device(uint8_t slot) noexcept
{
	sd_mmc_spi_err = SD_MMC_SPI_NO_ERR;
	if (currentSpiClient != nullptr)
	{
		currentSpiClient->Deselect();
		currentSpiClient = nullptr;
		delayMicroseconds(1);
	}
}

void sd_mmc_spi_send_clock() noexcept
{
	uint8_t i;
	uint8_t dummy = 0xFF;

	sd_mmc_spi_err = SD_MMC_SPI_NO_ERR;
	//! Send 80 cycles
	for (i = 0; i < 10; i++)
	{
		currentSpiClient->WritePacket(&dummy, 1); // 8 cycles
	}
}

bool sd_mmc_spi_send_cmd(sdmmc_cmd_def_t cmd, uint32_t arg) noexcept
{
	return sd_mmc_spi_adtc_start(cmd, arg, 0, 0, nullptr);
}

bool sd_mmc_spi_adtc_start(sdmmc_cmd_def_t cmd, uint32_t arg, uint16_t block_size, uint16_t nb_block, const void *dmaAddr) noexcept
{
	uint8_t dummy = 0xFF;
	uint8_t cmd_token[6];
	uint8_t ncr_timeout;
	uint8_t r1; //! R1 response

	UNUSED(dmaAddr);
	Assert(cmd & SDMMC_RESP_PRESENT); // Always a response in SPI mode
	sd_mmc_spi_err = SD_MMC_SPI_NO_ERR;

	// Encode SPI command
	cmd_token[0] = SPI_CMD_ENCODE(SDMMC_CMD_GET_INDEX(cmd));
	cmd_token[1] = arg >> 24;
	cmd_token[2] = arg >> 16;
	cmd_token[3] = arg >> 8;
	cmd_token[4] = arg;
	cmd_token[5] = sd_mmc_spi_crc7(cmd_token, 5);

	// 8 cycles to respect Ncs timing
	// Note: This byte does not include start bit "0",
	// thus it is ignored by card.
	currentSpiClient->WritePacket(&dummy, 1);
	// Send command
	currentSpiClient->WritePacket(cmd_token, sizeof(cmd_token));

	// Wait for response
	// Two retry will be done to manage the Ncr timing between command and reponse
	// Ncr: Min. 1x8 clock  cycle, Max. 8x8 clock cycles
	// WORKAROUND for no compliance card (Atmel Internal ref. SD13):
	r1 = 0xFF;
	// Ignore first byte because Ncr min. = 8 clock cylces
	currentSpiClient->ReadPacket(&r1, 1);
	ncr_timeout = 7;
	while (1)
	{
		currentSpiClient->ReadPacket(&r1, 1); // 8 cycles
		if ((r1 & R1_SPI_ERROR) == 0)
		{
			// Valid R1 response
			break;
		}
		if (--ncr_timeout == 0)
		{
			// Here Valid R1 response received
			sd_mmc_spi_debug("%s: cmd %02d, arg 0x%08lX, R1 timeout\n\r",
					__func__, (int)SDMMC_CMD_GET_INDEX(cmd), arg);
			sd_mmc_spi_err = SD_MMC_SPI_ERR_RESP_TIMEOUT;
			return false;
		}
	}

	// Save R1 (Specific to SPI interface) in 32 bit response
	// The R1_SPI_IDLE bit can be checked by high level
	sd_mmc_spi_response_32 = r1;

	// Manage error in R1
	if (r1 & R1_SPI_COM_CRC)
	{
		sd_mmc_spi_debug("%s: cmd %02d, arg 0x%08lx, r1 0x%02x, R1_SPI_COM_CRC\n\r",
				__func__, (int)SDMMC_CMD_GET_INDEX(cmd), arg, r1);
		sd_mmc_spi_err = SD_MMC_SPI_ERR_RESP_CRC;
		return false;
	}
	if (r1 & R1_SPI_ILLEGAL_COMMAND)
	{
		sd_mmc_spi_debug("%s: cmd %02d, arg 0x%08lx, r1 0x%x, R1 ILLEGAL_COMMAND\n\r",
				__func__, (int)SDMMC_CMD_GET_INDEX(cmd), arg, r1);
		sd_mmc_spi_err = SD_MMC_SPI_ERR_ILLEGAL_COMMAND;
		return false;
	}
	if (r1 & ~R1_SPI_IDLE)
	{
		// Other error
		sd_mmc_spi_debug("%s: cmd %02d, arg 0x%08lx, r1 0x%x, R1 error\n\r",
				__func__, (int)SDMMC_CMD_GET_INDEX(cmd), arg, r1);
		sd_mmc_spi_err = SD_MMC_SPI_ERR;
		return false;
	}

	// Manage other responses
	if (cmd & SDMMC_RESP_BUSY)
	{
		if (!sd_mmc_spi_wait_busy())
		{
			sd_mmc_spi_err = SD_MMC_SPI_ERR_RESP_BUSY_TIMEOUT;
			sd_mmc_spi_debug("%s: cmd %02d, arg 0x%08lx, Busy signal always high\n\r",
					__func__, (int)SDMMC_CMD_GET_INDEX(cmd), arg);
			return false;
		}
	}
	if (cmd & SDMMC_RESP_8)
	{
		sd_mmc_spi_response_32 = 0;
		currentSpiClient->ReadPacket((uint8_t*) & sd_mmc_spi_response_32, 1);
		sd_mmc_spi_response_32 = LoadLE32(&sd_mmc_spi_response_32);
	}
	if (cmd & SDMMC_RESP_32)
	{
		currentSpiClient->ReadPacket((uint8_t*) & sd_mmc_spi_response_32, 4);
		sd_mmc_spi_response_32 = LoadBE32(&sd_mmc_spi_response_32);
	}

	sd_mmc_spi_block_size = block_size;
	sd_mmc_spi_nb_block = nb_block;
	sd_mmc_spi_transfert_pos = 0;
	return true; // Command complete
}

uint32_t sd_mmc_spi_get_response() noexcept
{
	return sd_mmc_spi_response_32;
}

bool sd_mmc_spi_read_word(uint32_t* value) noexcept
{
	sd_mmc_spi_err = SD_MMC_SPI_NO_ERR;
	Assert(sd_mmc_spi_nb_block > (sd_mmc_spi_transfert_pos / sd_mmc_spi_block_size));

	if (!(sd_mmc_spi_transfert_pos % sd_mmc_spi_block_size))
	{
		// New block
		if (!sd_mmc_spi_start_read_block())
		{
			return false;
		}
	}
	// Read data
	currentSpiClient->ReadPacket((uint8_t*)value, 4);
	*value = LoadLE32(value);
	sd_mmc_spi_transfert_pos += 4;

	if (!(sd_mmc_spi_transfert_pos % sd_mmc_spi_block_size))
	{
		// End of block
		sd_mmc_spi_stop_read_block();
	}
	return true;
}

bool sd_mmc_spi_write_word(uint32_t value) noexcept
{
	sd_mmc_spi_err = SD_MMC_SPI_NO_ERR;
	Assert(sd_mmc_spi_nb_block > (sd_mmc_spi_transfert_pos / sd_mmc_spi_block_size));

	if (!(sd_mmc_spi_transfert_pos % sd_mmc_spi_block_size))
	{
		// New block
		sd_mmc_spi_start_write_block();
	}

	// Write data
	value = LoadBE32(&value);
	currentSpiClient->WritePacket((uint8_t*)&value, 4);
	sd_mmc_spi_transfert_pos += 4;

	if (!(sd_mmc_spi_transfert_pos % sd_mmc_spi_block_size))
	{
		// End of block
		if (!sd_mmc_spi_stop_write_block())
		{
			return false;
		}
		// Wait busy due to data programmation
		if (!sd_mmc_spi_wait_busy())
		{
			sd_mmc_spi_err = SD_MMC_SPI_ERR_WRITE_TIMEOUT;
			sd_mmc_spi_debug("%s: Write blocks timeout\n\r", __func__);
			return false;
		}
	}
	return sd_mmc_spi_stop_multiwrite_block();
}

bool sd_mmc_spi_start_read_blocks(void *dest, uint16_t nb_block) noexcept
{
	uint32_t pos;

	sd_mmc_spi_err = SD_MMC_SPI_NO_ERR;
	pos = 0;
	while (nb_block--)
	{
		Assert(sd_mmc_spi_nb_block >
				(sd_mmc_spi_transfert_pos / sd_mmc_spi_block_size));
		if (!sd_mmc_spi_start_read_block())
		{
			return false;
		}

		// Read block
		currentSpiClient->ReadPacket(&((uint8_t*)dest)[pos], sd_mmc_spi_block_size);
		pos += sd_mmc_spi_block_size;
		sd_mmc_spi_transfert_pos += sd_mmc_spi_block_size;

		sd_mmc_spi_stop_read_block();
	}
	return true;
}

bool sd_mmc_spi_wait_end_of_read_blocks() noexcept
{
	return true;
}

bool sd_mmc_spi_start_write_blocks(const void *src, uint16_t nb_block) noexcept
{
	uint32_t pos;

	sd_mmc_spi_err = SD_MMC_SPI_NO_ERR;
	pos = 0;
	while (nb_block--)
	{
		Assert(sd_mmc_spi_nb_block >
				(sd_mmc_spi_transfert_pos / sd_mmc_spi_block_size));
		sd_mmc_spi_start_write_block();

		// Write block
		currentSpiClient->WritePacket(&((uint8_t*)src)[pos], sd_mmc_spi_block_size);
		pos += sd_mmc_spi_block_size;
		sd_mmc_spi_transfert_pos += sd_mmc_spi_block_size;

		if (!sd_mmc_spi_stop_write_block())
		{
			return false;
		}
		// Do not check busy of last block
		// but delay it to mci_wait_end_of_write_blocks()
		if (nb_block) {
			// Wait busy due to data programmation
			if (!sd_mmc_spi_wait_busy())
			{
				sd_mmc_spi_err = SD_MMC_SPI_ERR_WRITE_TIMEOUT;
				sd_mmc_spi_debug("%s: Write blocks timeout\n\r", __func__);
				return false;
			}
		}
	}
	return true;
}

bool sd_mmc_spi_wait_end_of_write_blocks() noexcept
{
	// Wait busy due to data programmation of last block writed
	if (!sd_mmc_spi_wait_busy())
	{
		sd_mmc_spi_err = SD_MMC_SPI_ERR_WRITE_TIMEOUT;
		sd_mmc_spi_debug("%s: Write blocks timeout\n\r", __func__);
		return false;
	}
	return sd_mmc_spi_stop_multiwrite_block();
}

//! @}

#endif

// End
