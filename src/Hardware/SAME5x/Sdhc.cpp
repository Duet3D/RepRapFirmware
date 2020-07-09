/**
 * \file
 *
 * \brief SAM SDHC HPL
 *
 * Copyright (c) 2016-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

#include <Sdhc.h>
#include <CoreIO.h>

// Define which SDHC controller we are using
Sdhc* const hw = SDHC1;
constexpr uint32_t CONF_BASE_FREQUENCY = 120000000;

#define HSMCI_SLOT_0_SIZE 		4
#define CONF_SDHC_CLK_GEN_SEL	0

//extern void debugPrintf(const char* fmt, ...) noexcept __attribute__ ((format (printf, 1, 2)));

static uint64_t mci_sync_trans_pos;
static uint16_t mci_sync_block_size;
static uint16_t mci_sync_nb_block;

typedef struct
{
    uint16_t                            attribute;
    uint16_t                            length;
    uint32_t                            address;
} SDHC_ADMA_DESCR;

/* ADMA Descriptor Table Attribute Mask */
#define SDHC_DESC_TABLE_ATTR_NO_OP          (0x00 << 4)
#define SDHC_DESC_TABLE_ATTR_RSVD           (0x01 << 4)
#define SDHC_DESC_TABLE_ATTR_XFER_DATA      (0x02 << 4)
#define SDHC_DESC_TABLE_ATTR_LINK_DESC      (0x03 << 4)

#define SDHC_DESC_TABLE_ATTR_VALID          (1 << 0)
#define SDHC_DESC_TABLE_ATTR_END            (1 << 1)
#define SDHC_DESC_TABLE_ATTR_INTR           (1 << 2)

__attribute__((aligned(32))) static SDHC_ADMA_DESCR sdhc1DmaDescrTable[1];

static void hsmci_reset();
static void hsmci_set_speed(uint32_t speed, uint8_t prog_clock_mode);
static bool hsmci_wait_busy();
static bool hsmci_send_cmd_execute(uint32_t cmdr, uint32_t cmd, uint32_t arg);

/**
 * \brief Reset the SDHC interface
 *
 * \param hw The pointer to MCI hardware instance
 */
static void hsmci_reset()
{
	hri_sdhc_set_SRR_SWRSTCMD_bit(hw);
}

/**
 * \brief Set speed of the SDHC clock.
 *
 * \param hw       The pointer to MCI hardware instance
 * \param speed    SDHC clock speed in Hz.
 * \param prog_clock_mode     Use programmable clock mode
 */
static void hsmci_set_speed(uint32_t speed, uint8_t prog_clock_mode)
{
#if 0	//dc42
	// The following is based on the code from Harmony
	uint32_t baseclk_frq = 0;
	uint16_t divider = 0;
	uint32_t clkmul = 0;

	// Disable clock before changing it
	if (hri_sdhc_get_CCR_SDCLKEN_bit(hw))
	{
		// It hangs on this next line because the SDHC_PSR_CMDINHD_CANNOT bit it stuck on
		// If we comment out this line, it doesn't hang but it fails to mount the SD card
		while (hri_sdhc_read_PSR_reg(hw) & (SDHC_PSR_CMDINHC_CANNOT | SDHC_PSR_CMDINHD_CANNOT)) { }
		hri_sdhc_clear_CCR_SDCLKEN_bit(hw);
	}

	// Get the base clock frequency
	baseclk_frq = CONF_BASE_FREQUENCY/2;

	// Use programmable clock mode if it is supported
	clkmul = hri_sdhc_read_CA1R_CLKMULT_bf(hw);
	if (clkmul > 0)
	{
		/* F_SDCLK = F_MULTCLK/(DIV+1), where F_MULTCLK = F_BASECLK x (CLKMULT+1)
		   F_SDCLK = (F_BASECLK x (CLKMULT + 1))/(DIV + 1)
		   For a given F_SDCLK, DIV = [(F_BASECLK x (CLKMULT + 1))/F_SDCLK] - 1
		*/
		divider = (baseclk_frq * (clkmul + 1)) / speed;
		if (divider > 0)
		{
			divider = divider - 1;
		}
		hri_sdhc_set_CCR_CLKGSEL_bit(hw);
	}
	else
	{
		// Programmable clock mode is not supported, so use divided clock mode
		/* F_SDCLK = F_BASECLK/(2 x DIV).
		   For a given F_SDCLK, DIV = F_BASECLK/(2 x F_SDCLK)
		*/
		divider =  baseclk_frq/(2 * speed);
		hri_sdhc_clear_CCR_CLKGSEL_bit(hw);
	}

	if (speed > 25000000)
	{
		// Enable the high speed mode
		hri_sdhc_set_HC1R_HSEN_bit(hw);
	}
	else
	{
		// Clear the high speed mode
		hri_sdhc_clear_HC1R_HSEN_bit(hw);
	}

	if (hri_sdhc_get_HC1R_HSEN_bit(hw) && divider == 0)
	{
		// IP limitation, if high speed mode is active divider must be non zero
		divider = 1;
	}

	// Set the divider
	hri_sdhc_write_CCR_SDCLKFSEL_bf(hw, divider & 0xFF);
	hri_sdhc_write_CCR_USDCLKFSEL_bf(hw, divider >> 8);

	// Enable internal clock
	hri_sdhc_set_CCR_INTCLKEN_bit(hw);

	// Wait for the internal clock to stabilize
	while (hri_sdhc_get_CCR_INTCLKS_bit(hw) == 0) { }

	// Enable the SDCLK
	hri_sdhc_set_CCR_SDCLKEN_bit(hw);
#else
	uint32_t div;
	uint32_t clkbase;
	uint32_t clkmul;

	if (hri_sdhc_get_CCR_SDCLKEN_bit(hw))
	{
		while (hri_sdhc_read_PSR_reg(hw) & (SDHC_PSR_CMDINHC_CANNOT | SDHC_PSR_CMDINHD_CANNOT)) { }
		hri_sdhc_clear_CCR_SDCLKEN_bit(hw);
	}
	//	clkbase = hri_sdhc_read_CA0R_BASECLKF_bf(hw);
	clkbase = CONF_BASE_FREQUENCY;
	clkmul  = hri_sdhc_read_CA1R_CLKMULT_bf(hw);

	/* If programmable clock mode is supported, baseclk is divided by 2 */
	if (clkmul > 0)
	{
		clkbase = clkbase / 2;
	}

	// Calculate required divider, rounded up

	if (prog_clock_mode == 0)
	{
		// Divided clock mode, speed = (div == 0) ? BaseClock : BaseClock/(2 * div)
		hri_sdhc_clear_CCR_CLKGSEL_bit(hw);
		div = (clkbase + speed - 1)/speed;
		div = (div == 1) ? 0 : (div + 1)/2;
	}
	else
	{
		// Programmable clock mode, speed = BaseClock * (clkmul + 1) / (div+1)
		hri_sdhc_set_CCR_CLKGSEL_bit(hw);
		clkbase *= clkmul + 1;
		div = (clkbase + speed - 1)/speed;
		if (div != 0)
		{
			div = div - 1;
		}
	}

	/* Specific constraint for SDHC/SDMMC IP
	The clock divider (DIV) in SDMMC_CCR must be set to a value different from 0 when HSEN is 1. */
	if ((hri_sdhc_get_HC1R_HSEN_bit(hw)) && (div == 0))
	{
		div = 1;
	}

	/* Set clock divider */
	hri_sdhc_write_CCR_SDCLKFSEL_bf(hw, div & 0xFF);
	hri_sdhc_write_CCR_USDCLKFSEL_bf(hw, div >> 8);

	hri_sdhc_set_CCR_INTCLKEN_bit(hw);
	/* Repeat this step until Clock Stable is 1 */
	while (hri_sdhc_get_CCR_INTCLKS_bit(hw) == 0) { }

	/* Output the clock to the card -- Set SD Clock Enable */
	hri_sdhc_set_CCR_SDCLKEN_bit(hw);
#endif
}

// Setup the DMA transfer.
// Each ADMA2 descriptor can transfer 65536 bytes (or 128 blocks) of data.
// For simplicity we use only one descriptor, so numBytes must not exceed 65536.
static void hsmci_dma_setup (const void* buffer, uint32_t numBytes)
{
	hri_sdhc_set_HC1R_DMASEL_bf(hw, SDHC_HC1R_DMASEL_32BIT_Val);

	// Set up the descriptor
	sdhc1DmaDescrTable[0].address = (uint32_t)(buffer);
	sdhc1DmaDescrTable[0].length = numBytes;
	sdhc1DmaDescrTable[0].attribute = (SDHC_DESC_TABLE_ATTR_XFER_DATA | SDHC_DESC_TABLE_ATTR_VALID | SDHC_DESC_TABLE_ATTR_INTR | SDHC_DESC_TABLE_ATTR_END);

	// Set the starting address of the descriptor table
	hw->ASAR[0].reg = (uint32_t)(&sdhc1DmaDescrTable[0]);
}

// Wait for a transfer to complete, returning true if OK, false if it failed
static bool WaitForDmaComplete()
{
	// TODO use the interrupt
	// Note, a read transaction sets the TRFC bit, but a write transaction using DMA doesn't appear to. So use the DMA interrupt bit too.
	while (true)
	{
		uint16_t nistr;
		do
		{
			nistr = hw->NISTR.reg;
		} while ((nistr & (SDHC_NISTR_TRFC | SDHC_NISTR_ERRINT)) == 0);

		uint16_t eistr = hw->EISTR.reg;
		hw->NISTR.reg = nistr;					// clear the interrupt(s)
		hw->EISTR.reg = eistr;					// clear the error status

		if ((nistr & SDHC_NISTR_ERRINT) == 0)
		{
			return true;						// transfer complete or DMA complete
		}

		eistr &= (SDHC_EISTR_DATTEO | SDHC_EISTR_DATCRC | SDHC_EISTR_DATEND | SDHC_EISTR_ADMA);		// get the errors we care about
		if ((nistr & SDHC_NISTR_TRFC) != 0 && (eistr & ~SDHC_EISTR_DATTEO) == 0)
		{
			return true;						// we had transfer complete and timeout - the controller specification says treat this as a successful transfer
		}

		if (eistr != 0)							// if there were errors that we care about, quit
		{
			return false;
		}

		// Else we had an error interrupt but we don't care about that error, so wait again
	}
}

static uint32_t hsmci_get_clock_speed()
{
	uint32_t clkbase = CONF_BASE_FREQUENCY;
	uint32_t clkmul = hri_sdhc_read_CA1R_CLKMULT_bf(hw);

	// If programmable clock mode is supported, baseclk is divided by 2
	if (clkmul > 0)
	{
		clkbase = clkbase / 2;
	}

	uint32_t div = (hri_sdhc_read_CCR_USDCLKFSEL_bf(hw) << 8) | hri_sdhc_read_CCR_SDCLKFSEL_bf(hw);
	if (hri_sdhc_get_CCR_CLKGSEL_bit(hw))				// if programmable clock mode
	{
		return (clkbase * (clkmul + 1))/(div + 1);
	}

	// Divided clock mode
	div = (div == 0) ? 1 : 2 * div;
	return clkbase/div;
}

/**
 * \brief Wait the end of busy signal on data line
 *
 * \param hw       The pointer to MCI hardware instance
 * \return true if success, otherwise false
 */
static bool hsmci_wait_busy()
{
	uint32_t busy_wait = 0xFFFFFFFF;
	uint32_t psr;

	ASSERT(hw);

	do {
		psr = hri_sdhc_read_PSR_reg(hw);

		if (busy_wait-- == 0) {
			hsmci_reset();
			return false;
		}
	} while (!(psr & SDHC_PSR_DATLL(1)));
	return true;
}

/**
 * \brief Send a command
 *
 * \param hw         The pointer to MCI hardware instance
 * \param cmdr       CMDR register bit to use for this command
 * \param cmd        Command definition
 * \param arg        Argument of the command
 *
 * \return true if success, otherwise false
 */
static bool hsmci_send_cmd_execute(uint32_t cmdr, uint32_t cmd, uint32_t arg)
{
	uint32_t sr;
	ASSERT(hw);

	// dc42 clear all bits in NISTR and EISTR before we start
	((Sdhc*)hw)->NISTR.reg = SDHC_NISTR_MASK & 0x7FFF;
	((Sdhc*)hw)->EISTR.reg = SDHC_EISTR_MASK;

	cmdr |= SDHC_CR_CMDIDX(cmd) | SDHC_CR_CMDTYP_NORMAL;

	if (cmd & MCI_RESP_PRESENT) {

		if (cmd & MCI_RESP_136) {
			cmdr |= SDHC_CR_RESPTYP_136_BIT;
		} else if (cmd & MCI_RESP_BUSY) {
			cmdr |= SDHC_CR_RESPTYP_48_BIT_BUSY;
		} else {
			cmdr |= SDHC_CR_RESPTYP_48_BIT;
		}
	}

	if (cmd & MCI_CMD_OPENDRAIN) {
		hri_sdhc_set_MC1R_OPD_bit(hw);
	} else {
		hri_sdhc_clear_MC1R_OPD_bit(hw);
	}

	hri_sdhc_write_ARG1R_reg(hw, arg);
	hri_sdhc_write_CR_reg(hw, cmdr);

	/* Wait end of command */
	do {
		sr = hri_sdhc_read_EISTR_reg(hw);

		if (cmd & MCI_RESP_CRC) {
			if (sr
			    & (SDHC_EISTR_CMDTEO | SDHC_EISTR_CMDEND | SDHC_EISTR_CMDIDX | SDHC_EISTR_DATTEO | SDHC_EISTR_DATEND
			       | SDHC_EISTR_ADMA)) {
				hsmci_reset();
				hri_sdhc_set_EISTR_reg(hw, SDHC_EISTR_MASK);
				return false;
			}
		} else {
			if (sr
			    & (SDHC_EISTR_CMDTEO | SDHC_EISTR_CMDEND | SDHC_EISTR_CMDIDX | SDHC_EISTR_CMDCRC | SDHC_EISTR_DATCRC
			       | SDHC_EISTR_DATTEO | SDHC_EISTR_DATEND | SDHC_EISTR_ADMA)) {
				hsmci_reset();
				hri_sdhc_set_EISTR_reg(hw, SDHC_EISTR_MASK);
				return false;
			}
		}
	} while (!hri_sdhc_get_NISTR_CMDC_bit(hw));
	if (!(cmdr & SDHC_CR_DPSEL_DATA))
	{
		// dc42 only clear the CMDC bit! In particular, don't clear TRFC.
		((Sdhc*)hw)->NISTR.reg = SDHC_NISTR_CMDC;
	}
	if (cmd & MCI_RESP_BUSY) {
		if (!hsmci_wait_busy()) {
			return false;
		}
	}

	return true;
}

/**
 *  \brief Initialize MCI low level driver.
 */
int32_t hsmci_init()
{
	hri_sdhc_set_SRR_SWRSTALL_bit(hw);
	while (hri_sdhc_get_SRR_SWRSTALL_bit(hw)) { }

	/* Set the Data Timeout Register to 2 Mega Cycles */
	hri_sdhc_write_TCR_reg(hw, SDHC_TCR_DTCVAL(0xE));

	/* Set 3v3 power supply */
	hri_sdhc_write_PCR_reg(hw, SDHC_PCR_SDBPWR_ON | SDHC_PCR_SDBVSEL_3V3);

	hri_sdhc_set_NISTER_reg(hw, SDHC_NISTER_MASK);
	hri_sdhc_set_EISTER_reg(hw, SDHC_EISTER_MASK);

	// dc42 The clock divider defaults to 0. Set it to 1 so that M122 doesn't report a too-high interface speed when no card is present.
	hri_sdhc_write_CCR_SDCLKFSEL_bf(hw, 1);

	return ERR_NONE;
}

/**
 *  \brief Select a device and initialize it
 */
void hsmci_select_device(uint8_t slot, uint32_t clock, uint8_t bus_width, bool high_speed)
{
	(void)(slot);

	if (high_speed) {
		hri_sdhc_set_HC1R_HSEN_bit(hw);
	} else {
		hri_sdhc_clear_HC1R_HSEN_bit(hw);
	}

	if (hri_sdhc_get_HC2R_PVALEN_bit(hw) == 0)
	{
		hsmci_set_speed(clock, CONF_SDHC_CLK_GEN_SEL);
	}

	switch (bus_width) {
	case 1:
	default:
		hri_sdhc_clear_HC1R_DW_bit(hw);
		break;

	case 4:
		hri_sdhc_set_HC1R_DW_bit(hw);
		break;
	}
}

/**
 *  \brief Deselect a device by an assigned slot
 */
void hsmci_deselect_device(uint8_t slot)
{
	/* Nothing to do */
	(void)(slot);
}

/**
 *  \brief Get the maximum bus width of a device
 *         by a selected slot
 */
uint8_t hsmci_get_bus_width(uint8_t slot)
{
	switch (slot) {
	case 0:
		return HSMCI_SLOT_0_SIZE;

	default:
		/* Slot number wrong */
		return 0;
	}
}

/**
 *  \brief Get the high speed capability of the device.
 */
bool hsmci_is_high_speed_capable()
{
	return hri_sdhc_get_CA0R_HSSUP_bit(hw);
}

// Get the transfer rate in bytes/sec
uint32_t hsmci_get_speed()
{
	return hsmci_get_clock_speed()/(8/HSMCI_SLOT_0_SIZE);
}

/**
 *  \brief Send 74 clock cycles on the line.
 *   Note: It is required after card plug and before card install.
 */
void hsmci_send_clock()
{
	volatile uint32_t i;
	for (i = 0; i < 5000; i++)
		;
}

/**
 *  \brief Send a command on the selected slot
 */
bool hsmci_send_cmd(uint32_t cmd, uint32_t arg)
{
	/* Check Command Inhibit (CMD) in the Present State register */
	if (hri_sdhc_get_PSR_CMDINHC_bit(hw)) {
		return false;
	}

	return hsmci_send_cmd_execute(0, cmd, arg);
}

/**
 *  \brief Get 32 bits response of the last command.
 */
uint32_t hsmci_get_response()
{
	return hri_sdhc_read_RR_reg(hw, 0);
}

/**
 *  \brief Get 128 bits response of the last command.
 */
void hsmci_get_response_128(uint8_t *response)
{
	uint32_t response_32;

	for (int8_t i = 3; i >= 0; i--) {
		response_32 = hri_sdhc_read_RR_reg(hw, i);
		if (i != 3) {
			*response = (response_32 >> 24) & 0xFF;
			response++;
		}
		*response = (response_32 >> 16) & 0xFF;
		response++;
		*response = (response_32 >> 8) & 0xFF;
		response++;
		*response = (response_32 >> 0) & 0xFF;
		response++;
	}
}

/**
 *  \brief Send an ADTC command on the selected slot
 *         An ADTC (Addressed Data Transfer Commands)
 *         command is used for read/write access.
 */
bool hsmci_adtc_start(uint32_t cmd, uint32_t arg, uint16_t block_size, uint16_t nb_block, const void *dmaAddr)
{
	/* Check Command Inhibit (CMD/DAT) in the Present State register */
	if (hri_sdhc_get_PSR_CMDINHC_bit(hw) || hri_sdhc_get_PSR_CMDINHD_bit(hw)) {
		return false;
	}

	uint32_t tmr;
	if (cmd & MCI_CMD_WRITE) {
		tmr = SDHC_TMR_DTDSEL_WRITE;
	} else {
		tmr = SDHC_TMR_DTDSEL_READ;
	}

	if (cmd & MCI_CMD_SINGLE_BLOCK) {
		tmr |= SDHC_TMR_MSBSEL_SINGLE;
	} else if (cmd & MCI_CMD_MULTI_BLOCK) {
		tmr |= SDHC_TMR_BCEN | SDHC_TMR_MSBSEL_MULTIPLE;
	} else {
		return false;
	}

	hri_sdhc_write_BCR_reg(hw, SDHC_BCR_BCNT(nb_block));
	hri_sdhc_write_BSR_reg(hw, SDHC_BSR_BLOCKSIZE(block_size) | SDHC_BSR_BOUNDARY_4K);

	if (dmaAddr != NULL)
	{
		hsmci_dma_setup(dmaAddr, nb_block * (uint32_t)block_size);
		tmr |= SDHC_TMR_DMAEN_ENABLE;
	}

	hri_sdhc_write_TMR_reg(hw, tmr);

	mci_sync_trans_pos  = 0;
	mci_sync_block_size = block_size;
	mci_sync_nb_block   = nb_block;

	return hsmci_send_cmd_execute(SDHC_CR_DPSEL_DATA, cmd, arg);
}

/**
 *  \brief Send a command to stop an ADTC command on the selected slot.
 */
bool hsmci_adtc_stop(uint32_t cmd, uint32_t arg)
{
	/* Nothing to do */
	(void)(cmd);
	(void)(arg);

	return true;
}

/**
 *  \brief Read a word on the line.
 */
bool hsmci_read_word(uint32_t *value)
{
	uint32_t sr;
	uint8_t  nbytes;

	/* Wait data available */
	nbytes = (mci_sync_block_size * mci_sync_nb_block - mci_sync_trans_pos < 4)
	             ? (mci_sync_block_size % 4)
	             : 4;

	if (mci_sync_trans_pos % mci_sync_block_size == 0)
	{
		do
		{
			sr = hri_sdhc_read_EISTR_reg(hw);

			if (sr & (SDHC_EISTR_DATTEO | SDHC_EISTR_DATCRC | SDHC_EISTR_DATEND))
			{
				hsmci_reset();
				return false;
			}
		} while (!hri_sdhc_get_NISTR_BRDRDY_bit(hw));
		hri_sdhc_set_NISTR_BRDRDY_bit(hw);
	}

	/* Read data */
	if (nbytes == 4)
	{
		*value = hri_sdhc_read_BDPR_reg(hw);
	}
	else
	{
		sr = hri_sdhc_read_BDPR_reg(hw);
		switch (nbytes) {
		case 3:
			value[0] = sr & 0xFFFFFF;
			break;
		case 2:
			value[0] = sr & 0xFFFF;
			break;
		case 1:
			value[0] = sr & 0xFF;
			break;
		}
	}
	mci_sync_trans_pos += nbytes;

	if (((uint64_t)mci_sync_block_size * mci_sync_nb_block) > mci_sync_trans_pos)
	{
		return true;
	}

	/* Wait end of transfer */
	do
	{
		sr = hri_sdhc_read_EISTR_reg(hw);

		if (sr & (SDHC_EISTR_DATTEO | SDHC_EISTR_DATCRC | SDHC_EISTR_DATEND))
		{
			hsmci_reset();
			return false;
		}
	} while (!hri_sdhc_get_NISTR_TRFC_bit(hw));
	hri_sdhc_set_NISTR_TRFC_bit(hw);
	return true;
}

/**
 *  \brief Write a word on the line
 */
bool hsmci_write_word(uint32_t value)
{
	uint32_t sr;
	uint8_t  nbytes;

	/* Wait data available */
	nbytes = 4; //( mci_dev->mci_sync_block_size & 0x3 ) ? 1 : 4;
	if (mci_sync_trans_pos % mci_sync_block_size == 0) {
		do {
			sr = hri_sdhc_read_EISTR_reg(hw);

			if (sr & (SDHC_EISTR_DATTEO | SDHC_EISTR_DATCRC | SDHC_EISTR_DATEND)) {
				hsmci_reset();
				return false;
			}
		} while (!hri_sdhc_get_NISTR_BWRRDY_bit(hw));
		hri_sdhc_set_NISTR_BWRRDY_bit(hw);
	}
	/* Write data */
	hri_sdhc_write_BDPR_reg(hw, value);
	mci_sync_trans_pos += nbytes;

	if (((uint64_t)mci_sync_block_size * mci_sync_nb_block) > mci_sync_trans_pos) {
		return true;
	}

	/* Wait end of transfer */
	do
	{
		sr = hri_sdhc_read_EISTR_reg(hw);

		if (sr & (SDHC_EISTR_DATTEO | SDHC_EISTR_DATCRC | SDHC_EISTR_DATEND))
		{
			hsmci_reset();
			return false;
		}
	} while (!hri_sdhc_get_NISTR_TRFC_bit(hw));
	hri_sdhc_set_NISTR_TRFC_bit(hw);
	return true;
}

/**
 *  \brief Start a read blocks transfer on the line
 *  Note: The driver will use the DMA available to speed up the transfer.
 */
bool hsmci_start_read_blocks(void *dst, uint16_t nb_block)
{
	if (nb_block != 0)
	{
		const bool ok = WaitForDmaComplete();
		if (!ok)
		{
			hsmci_reset();
			return false;
		}
	}

	return true;
}

/**
 *  \brief Start a write blocks transfer on the line
 *  Note: The driver will use the DMA available to speed up the transfer.
 */
bool hsmci_start_write_blocks(const void *src, uint16_t nb_block)
{
	if (nb_block != 0)
	{
		const bool ok = WaitForDmaComplete();
		if (!ok)
		{
			hsmci_reset();
			return false;
		}
	}

	return true;
}

/**
 *  \brief Wait the end of transfer initiated by mci_start_read_blocks()
 */
bool hsmci_wait_end_of_read_blocks()
{
	/* Always return true for sync read blocks */
	return true;
}

/**
 *  \brief Wait the end of transfer initiated by mci_start_write_blocks()
 */
bool hsmci_wait_end_of_write_blocks()
{
	/* Always return true for sync write blocks */
	return true;
}
