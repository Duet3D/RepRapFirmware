// ASF 3.27.0

/**
 * \file
 *
 * \brief SPI master common service for SAM.
 *
 * Copyright (c) 2011-2015 Atmel Corporation. All rights reserved.
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
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include "Arduino.h"
#include "compiler.h"
#include "spi_master.h"
#include "variant.h"

#ifndef F_CPU
#define F_CPU 84000000UL
#endif

#ifndef MOSI_PIN
#define MOSI_PIN MOSI
#endif

#ifndef MISO_PIN
#define MISO_PIN MISO
#endif

#ifndef SCK_PIN
#define SCK_PIN SCK
#endif

#define	WRITE(pin, v) do{if(v) {g_APinDescription[pin].pPort->PIO_SODR = g_APinDescription[pin].ulPin;} else {g_APinDescription[pin].pPort->PIO_CODR = g_APinDescription[pin].ulPin; }}while(0)

/**
 * \brief Max number when the chip selects are connected to a 4- to 16-bit decoder.
 */
#define MAX_NUM_WITH_DECODER 0x10

/**
 * \brief Max number when the chip selects are directly connected to peripheral device.
 */
#define MAX_NUM_WITHOUT_DECODER 0x04

/**
 * \brief Max number when the chip selects are directly connected to peripheral device.
 */
#define NONE_CHIP_SELECT_ID 0x0f

/**
 * \brief The default chip select id.
 */
#define DEFAULT_CHIP_ID 1

/**
 * \brief Issue a LASTXFER command.
 *  The next transfer is the last transfer and after that CS is de-asserted.
 *
 * \param p_spi Pointer to an SPI instance.
 */
static inline void spi_set_lastxfer(Spi *p_spi)
{
        p_spi->SPI_CR = SPI_CR_LASTXFER;
}

/**
 * \brief Configure CS behavior for SPI transfer (\ref spi_cs_behavior_t).
 *
 * \param p_spi Pointer to an SPI instance.
 * \param ul_pcs_ch Peripheral Chip Select channel (0~3).
 * \param ul_cs_behavior Behavior of the Chip Select after transfer.
 */
void spi_configure_cs_behavior(Spi *p_spi, uint32_t ul_pcs_ch,
                uint32_t ul_cs_behavior)
{
        if (ul_cs_behavior == SPI_CS_RISE_FORCED) {
                p_spi->SPI_CSR[ul_pcs_ch] &= (~SPI_CSR_CSAAT);
                p_spi->SPI_CSR[ul_pcs_ch] |= SPI_CSR_CSNAAT;
        } else if (ul_cs_behavior == SPI_CS_RISE_NO_TX) {
                p_spi->SPI_CSR[ul_pcs_ch] &= (~SPI_CSR_CSAAT);
                p_spi->SPI_CSR[ul_pcs_ch] &= (~SPI_CSR_CSNAAT);
        } else if (ul_cs_behavior == SPI_CS_KEEP_LOW) {
                p_spi->SPI_CSR[ul_pcs_ch] |= SPI_CSR_CSAAT;
        }
}

/**
 * \brief Set number of bits per transfer.
 *
 * \param p_spi Pointer to an SPI instance.
 * \param ul_pcs_ch Peripheral Chip Select channel (0~3).
 * \param ul_bits Number of bits (8~16), use the pattern defined
 *        in the device header file.
 */
void spi_set_bits_per_transfer(Spi *p_spi, uint32_t ul_pcs_ch,
                uint32_t ul_bits)
{
        p_spi->SPI_CSR[ul_pcs_ch] &= (~SPI_CSR_BITS_Msk);
        p_spi->SPI_CSR[ul_pcs_ch] |= ul_bits;
}

/**
 * \brief Configure timing for SPI transfer.
 *
 * \param p_spi Pointer to an SPI instance.
 * \param ul_pcs_ch Peripheral Chip Select channel (0~3).
 * \param uc_dlybs Delay before SPCK (in number of MCK clocks).
 * \param uc_dlybct Delay between consecutive transfers (in number of MCK clocks
).
 */
void spi_set_transfer_delay(Spi *p_spi, uint32_t ul_pcs_ch,
                uint8_t uc_dlybs, uint8_t uc_dlybct)
{
        p_spi->SPI_CSR[ul_pcs_ch] &= ~(SPI_CSR_DLYBS_Msk | SPI_CSR_DLYBCT_Msk);
        p_spi->SPI_CSR[ul_pcs_ch] |= SPI_CSR_DLYBS(uc_dlybs)
                        | SPI_CSR_DLYBCT(uc_dlybct);
}

/**
 * \brief Set clock default state.
 *
 * \param p_spi Pointer to an SPI instance.
 * \param ul_pcs_ch Peripheral Chip Select channel (0~3).
 * \param ul_polarity Default clock state is logical one(high)/zero(low).
 */
void spi_set_clock_polarity(Spi *p_spi, uint32_t ul_pcs_ch,
                uint32_t ul_polarity)
{
        if (ul_polarity) {
                p_spi->SPI_CSR[ul_pcs_ch] |= SPI_CSR_CPOL;
        } else {
                p_spi->SPI_CSR[ul_pcs_ch] &= (~SPI_CSR_CPOL);
        }
}

/**
 * \brief Set Data Capture Phase.
 *
 * \param p_spi Pointer to an SPI instance.
 *  \param ul_pcs_ch Peripheral Chip Select channel (0~3).
 *  \param ul_phase Data capture on the rising/falling edge of clock.
 */
void spi_set_clock_phase(Spi *p_spi, uint32_t ul_pcs_ch, uint32_t ul_phase)
{
        if (ul_phase) {
                p_spi->SPI_CSR[ul_pcs_ch] |= SPI_CSR_NCPHA;
        } else {
                p_spi->SPI_CSR[ul_pcs_ch] &= (~SPI_CSR_NCPHA);
        }
}

/** \brief Initialize the SPI in master mode.
 *
 * \param p_spi  Base address of the SPI instance.
 *
 */

void spi_master_init(Spi *p_spi, int ul_cs_pin, int ul_npcs_pin)
{
	static bool init_comms = true;

	if (init_comms)
	{
		PIO_Configure(
			g_APinDescription[SCK_PIN].pPort,
			g_APinDescription[SCK_PIN].ulPinType,
			g_APinDescription[SCK_PIN].ulPin,
			g_APinDescription[SCK_PIN].ulPinConfiguration);

		PIO_Configure(
			g_APinDescription[MOSI_PIN].pPort,
			g_APinDescription[MOSI_PIN].ulPinType,
			g_APinDescription[MOSI_PIN].ulPin,
			g_APinDescription[MOSI_PIN].ulPinConfiguration);

		PIO_Configure(
			g_APinDescription[MISO_PIN].pPort,
			g_APinDescription[MISO_PIN].ulPinType,
			g_APinDescription[MISO_PIN].ulPin,
			g_APinDescription[MISO_PIN].ulPinConfiguration);

		pmc_enable_periph_clk(ID_SPI0);

		spi_reset(p_spi);

		// set master mode, peripheral select, disable fault detection
		SPI_Configure(p_spi, ID_SPI0,
					  SPI_MR_MSTR    | // Master mode
					  SPI_MR_MODFDIS); // Disable mode fault detection
		SPI_Enable(p_spi);

		init_comms = false;
	}

	if (ul_npcs_pin >= 0)
	{
		// Connect the NPCS pin as a PIO peripheral and assert it HIGH
		PIO_Configure(
			g_APinDescription[ul_npcs_pin].pPort,
			g_APinDescription[ul_npcs_pin].ulPinType,
			g_APinDescription[ul_npcs_pin].ulPin,
			g_APinDescription[ul_npcs_pin].ulPinConfiguration);

		WRITE(ul_npcs_pin, HIGH);	
	}

	if (ul_cs_pin >= 0)
	{
		PIO_Configure(
			g_APinDescription[ul_cs_pin].pPort,
			PIO_OUTPUT_1,
			g_APinDescription[ul_cs_pin].ulPin,
			g_APinDescription[ul_cs_pin].ulPinConfiguration);

		WRITE(ul_cs_pin, HIGH);		
	}

#if defined(USE_SAM3X_DMAC)
	pmc_enable_periph_clk(ID_DMAC);
	dmac_disable(DMAC);
	dmac_set_priority_mode(DMAC, DMAC_GCFG_ARB_CFG_FIXED);
	dmac_enable(DMAC);
#endif

	// spi_enable_clock(p_spi);
	// spi_reset(p_spi);
	// spi_set_master_mode(p_spi);
	// spi_disable_mode_fault_detect(p_spi);
	// spi_disable_loopback(p_spi);
	// spi_set_peripheral_chip_select_value(p_spi, DEFAULT_CHIP_ID);
	// spi_set_fixed_peripheral_select(p_spi);
    // spi_disable_peripheral_select_decode(p_spi);
	// spi_set_delay_between_chip_select(p_spi, CONFIG_SPI_MASTER_DELAY_BCS);

	// SPI_Enable(p_spi);
}


/**
 * \brief Calculate the baudrate divider.
 *
 * \param baudrate Baudrate value.
 * \param mck      SPI module input clock frequency (MCK clock, Hz).
 *
 * \return Divider or error code.
 *   \retval > 0  Success.
 *   \retval < 0  Error.
 */
int16_t spi_calc_baudrate_div(uint32_t baud_rate, uint32_t mck)
{
        int16_t baud_div = div_ceil(mck, baud_rate);

        /* The value of baud_div is from 1 to 255 in the SCBR field. */
        if (baud_div <= 0)
			baud_div = 10;
		else if (baud_div > 255)
			baud_div = 255;

        return baud_div;
}

/**
 * \brief Set up an SPI device.
 *
 * The returned device descriptor structure must be passed to the driver
 * whenever that device should be used as current slave device.
 *
 * \param p_spi     Base address of the SPI instance.
 * \param device    Pointer to SPI device struct that should be initialized.
 * \param flags     SPI configuration flags. Common flags for all
 *                  implementations are the SPI modes SPI_MODE_0 ...
 *                  SPI_MODE_3.
 * \param baud_rate Baud rate for communication with slave device in Hz.
 * \param sel_id    Board specific select id.
 */
void spi_master_setup_device(Spi *p_spi, const struct spi_device *device,
		spi_flags_t flags, uint32_t baud_rate, board_spi_select_id_t sel_id)
{
	/* avoid Cppcheck Warning */
	UNUSED(sel_id);
	UNUSED(flags);

	spi_reset(p_spi);
	SPI_Configure(p_spi, ID_SPI0,
				  SPI_MR_MSTR    | // Master mode
				  SPI_MR_MODFDIS); // Disable mode fault detection

	// Set SPI mode 0, clock, select not active after transfer
	// with delay between transfers
	int16_t baud_div = spi_calc_baudrate_div(baud_rate, F_CPU);
	SPI_ConfigureNPCS(p_spi, device->id,
					  SPI_CSR_NCPHA |          // Data capture on rising edge of clock
					  SPI_CSR_CSAAT |          // CS behavior == SPI_CS_KEEP_LOW
					  SPI_CSR_SCBR(baud_div) | // Baud rate
					  device->bits |           // Transfer bit width
					  SPI_CSR_DLYBCT(1));      // Transfer delay
	SPI_Enable(p_spi);

	// spi_set_bits_per_transfer(p_spi, device->id, device->bits);
	// spi_set_transfer_delay(p_spi, device->id, CONFIG_SPI_MASTER_DELAY_BS,
	//					   CONFIG_SPI_MASTER_DELAY_BCT);
	// spi_set_baudrate_div(p_spi, device->id,
	//					 spi_calc_baudrate_div(baud_rate, F_CPU));
	// spi_configure_cs_behavior(p_spi, device->id, SPI_CS_KEEP_LOW);
	// spi_set_clock_polarity(p_spi, device->id, flags >> 1);
	// spi_set_clock_phase(p_spi, device->id, ((flags & 0x1) ^ 0x1));
}

/**
 * \brief Select the given device on the SPI bus.
 *
 * Set device specific setting and call board chip select.
 *
 * \param p_spi   Base address of the SPI instance.
 * \param device  SPI device.
 *
 */
void spi_select_device(Spi *p_spi, const struct spi_device *device)
{
	if (spi_get_peripheral_select_decode_setting(p_spi)) {
		if (device->id < MAX_NUM_WITH_DECODER) {
			spi_set_peripheral_chip_select_value(p_spi, device->id);
		}
	} else {
		if (device->id < MAX_NUM_WITHOUT_DECODER) {
			spi_set_peripheral_chip_select_value(p_spi, (~(1 << device->id)));
		}
    }

	// Enable the CS line
	WRITE(device->cs, LOW);
}

/**
 * \brief Deselect the given device on the SPI bus.
 *
 * Call board chip deselect.
 *
 * \param p_spi   Base address of the SPI instance.
 * \param device  SPI device.
 *
 * \pre SPI device must be selected with spi_select_device() first.
 */
void spi_deselect_device(Spi *p_spi, const struct spi_device *device)
{
	uint32_t timeout = SPI_TIMEOUT;
	while (!spi_is_tx_empty(p_spi)) {
		if (!timeout--) {
			return;
		}
	}

	// Last transfer, so de-assert the current NPCS if CSAAT is set.
	spi_set_lastxfer(p_spi);

	// Disable the CS line
	WRITE(device->cs, HIGH);

	// Assert all lines; no peripheral is selected.
	spi_set_peripheral_chip_select_value(p_spi, NONE_CHIP_SELECT_ID);
}

/**
 * \brief Send a sequence of bytes to an SPI device.
 *
 * Received bytes on the SPI bus are discarded.
 *
 * \param p_spi     Base address of the SPI instance.
 * \param data      Data buffer to write.
 * \param len       Length of data to be written.
 *
 * \pre SPI device must be selected with spi_select_device() first.
 */
status_code_t spi_write_packet(Spi *p_spi, const uint8_t *data, size_t len)
{
	if (len == 0)
		return STATUS_OK;

	for (size_t i = 0; i < len-1; i++)
	{
		uint32_t timeout = SPI_TIMEOUT;
		while (!spi_is_tx_ready(p_spi)) {
			if (!timeout--) {
				return ERR_TMO;
			}
		}

		p_spi->SPI_TDR = (uint32_t)data[i];

		while (!spi_is_rx_ready(p_spi)) {
			if (!timeout--) {
				return ERR_TMO;
			}
		}

		p_spi->SPI_RDR;
	}

	return spi_write_single(p_spi, data[len-1]);
}

/**
 * \brief Receive a sequence of bytes from an SPI device.
 *
 * All bytes sent out on SPI bus are sent as value 0.
 *
 * \param p_spi     Base address of the SPI instance.
 * \param data      Data buffer to read.
 * \param len       Length of data to be read.
 *
 * \pre SPI device must be selected with spi_select_device() first.
 */
status_code_t spi_read_packet(Spi *p_spi, uint8_t *data, size_t len)
{
	if (len-- == 0)
		return STATUS_OK;

	for (uint16_t i = 0; i < len; i++)
	{
		uint32_t timeout = SPI_TIMEOUT;
		while (!spi_is_tx_ready(p_spi)) {
			if (!timeout--) {
				return ERR_TMO;
			}
		}

		p_spi->SPI_TDR = 0x000000FF;

		timeout = SPI_TIMEOUT;
		while (!spi_is_rx_ready(p_spi)) {
			if (!timeout--) {
				return ERR_TMO;
			}
		}

		data[i] = p_spi->SPI_RDR;
	}

	return spi_read_single(p_spi, &data[len]);
}


/**
 * \brief Set Peripheral Chip Select (PCS) value.
 *
 * \param p_spi Pointer to an SPI instance.
 * \param ul_value Peripheral Chip Select value.
 *                 If PCS decode mode is not used, use \ref spi_get_pcs to build
 *                 the value to use.
 *                 On reset the decode mode is not enabled.
 *                 The decode mode can be enabled/disabled by follow functions:
 *                 \ref spi_enable_peripheral_select_decode,
 *                 \ref spi_disable_peripheral_select_decode.
 */
void spi_set_peripheral_chip_select_value(Spi *p_spi, uint32_t ul_value)
{
        p_spi->SPI_MR &= (~SPI_MR_PCS_Msk);
        p_spi->SPI_MR |= SPI_MR_PCS(ul_value);
}

/**
 * \brief Set delay between chip selects (in number of MCK clocks).
 *  If DLYBCS <= 6, 6 MCK clocks will be inserted by default.
 *
 * \param p_spi Pointer to an SPI instance.
 * \param ul_delay Delay between chip selects (in number of MCK clocks).
 */
void spi_set_delay_between_chip_select(Spi *p_spi, uint32_t ul_delay)
{
        p_spi->SPI_MR &= (~SPI_MR_DLYBCS_Msk);
        p_spi->SPI_MR |= SPI_MR_DLYBCS(ul_delay);
}

/**
 * \brief Set Serial Clock Baud Rate divider value (SCBR).
 *
 * \param p_spi Pointer to an SPI instance.
 * \param ul_pcs_ch Peripheral Chip Select channel (0~3).
 * \param uc_baudrate_divider Baudrate divider from MCK.
 */
void spi_set_baudrate_div(Spi *p_spi, uint32_t ul_pcs_ch,
                uint8_t uc_baudrate_divider)
{
        p_spi->SPI_CSR[ul_pcs_ch] &= (~SPI_CSR_SCBR_Msk);
        p_spi->SPI_CSR[ul_pcs_ch] |= SPI_CSR_SCBR(uc_baudrate_divider);
}

#if defined(USE_SAM3X_DMAC)

void spi_start_transmit_dma(Dmac *p_dmac, Spi *p_spi, uint32_t ul_num,
							const void *src, uint32_t nb_bytes)
{
	static uint8_t ff = 0xFF;
	uint32_t cfg, src_incr = DMAC_CTRLB_SRC_INCR_INCREMENTING;
	dma_transfer_descriptor_t desc;

	// Send 0xFF repeatedly if src is NULL
	if (!src) {
		src = &ff;
		src_incr = DMAC_CTRLB_SRC_INCR_FIXED;
	}

	// Disable the DMA channel prior to configuring
	dmac_enable(p_dmac);
	dmac_channel_disable(p_dmac, ul_num);

	cfg = DMAC_CFG_SOD
		| DMAC_CFG_DST_H2SEL
		| DMAC_CFG_DST_PER(SPI_TX_IDX)
		| DMAC_CFG_FIFOCFG_ALAP_CFG;
	dmac_channel_set_configuration(p_dmac, ul_num, cfg);

	// Prepare DMA transfer
	desc.ul_source_addr = (uint32_t)src;
	desc.ul_destination_addr = (uint32_t)&(p_spi->SPI_TDR);
	desc.ul_ctrlA = DMAC_CTRLA_BTSIZE(nb_bytes)
		| DMAC_CTRLA_SRC_WIDTH_BYTE
		| DMAC_CTRLA_DST_WIDTH_BYTE;
	desc.ul_ctrlB = DMAC_CTRLB_SRC_DSCR
		| DMAC_CTRLB_DST_DSCR
		| DMAC_CTRLB_FC_MEM2PER_DMA_FC
		| src_incr
		| DMAC_CTRLB_DST_INCR_FIXED;

	// Next field is ignored, but set it anyway
	desc.ul_descriptor_addr = (uint32_t)NULL;

 	// Finish configuring the transfer
	dmac_channel_single_buf_transfer_init(p_dmac, ul_num, &desc);

	// And now start the DMA transfer
	dmac_channel_enable(p_dmac, ul_num);
}

void spi_start_receive_dma(Dmac *p_dmac, Spi *p_spi, uint32_t ul_num,
						   const void *dest, uint32_t nb_bytes)
{
	uint32_t cfg;
	dma_transfer_descriptor_t desc;

	// clear any potential overrun error
	cfg = p_spi->SPI_SR;

	// Turn the DMA channel off before configuring
	dmac_enable(p_dmac);
	dmac_channel_disable(p_dmac, ul_num);

	cfg = DMAC_CFG_SOD
		| DMAC_CFG_SRC_H2SEL
		| DMAC_CFG_SRC_PER(SPI_RX_IDX)
		| DMAC_CFG_FIFOCFG_ASAP_CFG;
	dmac_channel_set_configuration(p_dmac, ul_num, cfg);

	// Prepare DMA transfer
	desc.ul_source_addr = (uint32_t)&(p_spi->SPI_RDR);
	desc.ul_destination_addr = (uint32_t)dest;
	desc.ul_ctrlA = DMAC_CTRLA_BTSIZE(nb_bytes)
		| DMAC_CTRLA_SRC_WIDTH_BYTE
		| DMAC_CTRLA_DST_WIDTH_BYTE;
	desc.ul_ctrlB = DMAC_CTRLB_SRC_DSCR
		| DMAC_CTRLB_DST_DSCR
		| DMAC_CTRLB_FC_PER2MEM_DMA_FC
		| DMAC_CTRLB_SRC_INCR_FIXED
		| DMAC_CTRLB_DST_INCR_INCREMENTING;

	// This next field is ignored but set it anyway
	desc.ul_descriptor_addr = (uint32_t)NULL;

	// Finish configuring the DMA transfer
	dmac_channel_single_buf_transfer_init(p_dmac, ul_num, &desc);

	// And now allow the DMA transfer to begin
	dmac_channel_enable(p_dmac, ul_num);
}

#endif

//! @}
