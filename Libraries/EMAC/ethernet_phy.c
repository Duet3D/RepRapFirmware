 /**
 * \file
 *
 * \brief API driver for KSZ8051RNL PHY based on driver for DM9161A PHY PHY.
 * Adapted from the ethernet_phy.c file provided as part of LWIP under the license below:
 * Only designed to work with the KSZ8051RNL PHY, not yet fully tested.
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

#include "ethernet_phy.h"
#include "rmii.h"

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

/**
 * \defgroup KSZ8051RNL_ethernet_phy_group PHY component (KSZ8051RNL)
 *
 * Driver for the KSZ8051RNL component. This driver provides access to the main 
 * features of the PHY.
 *
 * \section dependencies Dependencies
 * This driver depends on the following modules:
 * - \ref emac_group Ethernet Media Access Controller (EMAC) module. 
 *
 * @{
 */

/* Max PHY number */
#define ETH_PHY_MAX_ADDR   8

/* Ethernet PHY operation max retry count */
#define ETH_PHY_RETRY_MAX 1000000

/* Ethernet PHY operation timeout */
#define ETH_PHY_TIMEOUT 10


//*****************************AB
/**
 * \brief Read a Register of the PHY.
 *
 * \param p_emac   Pointer to the EMAC instance.
 * \param uc_phy_addr PHY address.
 * \param ul_phy_reg PHY Register to be read
 *
 * Return Register contents
 */
uint8_t ethernet_phy_read_register(Emac *p_emac, uint8_t uc_phy_addr, uint32_t ul_phy_reg, uint32_t *p_ul_reg_cont)
{
	emac_enable_management(p_emac, true);

	uint8_t uc_rc = emac_phy_read(p_emac, uc_phy_addr, ul_phy_reg, p_ul_reg_cont);
	if (uc_rc != EMAC_OK) {
		/* Disable PHY management and start the EMAC transfer */
		emac_enable_management(p_emac, false);
		return uc_rc;
	}
	/* Start the EMAC transfers */
	emac_enable_management(p_emac, false);
	return uc_rc;
}
//*****************************AB


#if 0	// this function is unused in the Duet firmware
/**
 * \brief Find a valid PHY Address ( from addrStart to 31 ).
 *
 * \param p_emac   Pointer to the EMAC instance. 
 * \param uc_phy_addr PHY address.
 * \param uc_start_addr Start address of the PHY to be searched. 
 *
 * \return 0xFF when no valid PHY address is found. 
 */
static uint8_t ethernet_phy_find_valid(Emac *p_emac, uint8_t uc_phy_addr,
		uint8_t addrStart)
{
	uint32_t ul_value = 0;
	uint8_t uc_rc;
	uint8_t uc_cnt;
	uint8_t uc_phy_address = uc_phy_addr;

	emac_enable_management(p_emac, true);

	/* Check the current PHY address */
	uc_rc = uc_phy_address;
	if (emac_phy_read(p_emac, uc_phy_addr, MII_PHYID1, &ul_value) != EMAC_OK) {
	}

	/* Find another one */
	if (ul_value != MII_OUI1) {
		uc_rc = 0xFF;
		for (uc_cnt = addrStart; uc_cnt <= ETH_PHY_MAX_ADDR; uc_cnt++) {
			uc_phy_address = (uc_phy_address + 1) & 0x1F;
			emac_phy_read(p_emac, uc_phy_address, MII_PHYID1, &ul_value);
			if (ul_value == MII_OUI1) {
				uc_rc = uc_phy_address;
				break;
			}
		}
	}

	emac_enable_management(p_emac, false);

	if (uc_rc != 0xFF) {
		emac_phy_read(p_emac, uc_phy_address, MII_OMSS, &ul_value);
	}
	return uc_rc;
}
#endif


/**
 * \brief Perform a HW initialization to the PHY ( via RSTC ) and set up clocks.
 *
 * This should be called only once to initialize the PHY pre-settings.
 * The PHY address is the reset status of CRS, RXD[3:0] (the emacPins' pullups).
 * The COL pin is used to select MII mode on reset (pulled up for Reduced MII).
 * The RXDV pin is used to select test mode on reset (pulled up for test mode).
 * The above pins should be predefined for corresponding settings in resetPins.
 * The EMAC peripheral pins are configured after the reset is done.
 *
 * \param p_emac   Pointer to the EMAC instance. 
 * \param uc_phy_addr PHY address.
 * \param ul_mck EMAC MCK. 
 *
 * Return EMAC_OK if successfully, EMAC_TIMEOUT if timeout.
 */
uint8_t ethernet_phy_init(Emac *p_emac, uint8_t uc_phy_addr, uint32_t mck)
{
	
	uint8_t uc_rc = EMAC_TIMEOUT;

	/* Configure EMAC runtime clock */
	uc_rc = emac_set_clock(p_emac, mck);
	if (uc_rc != EMAC_OK) {	
		return uc_rc;
	}
	/* Check PHY Address  - Not used as we know the PHY address*/
/*
  	uint8_t uc_phy = ethernet_phy_find_valid(p_emac, uc_phy_addr, 0);
	if (uc_phy == 0xFF) {
		return uc_phy;
	}
	if (uc_phy != uc_phy_addr) {
		ethernet_phy_reset(p_emac, uc_phy_addr);
	}
*/
	return uc_rc;
}


/**
 * \brief Get the Link & speed settings, and automatically set up the EMAC with the
 * settings.
 *
 * \param p_emac   Pointer to the EMAC instance. 
 * \param uc_phy_addr PHY address.
 * \param uc_apply_setting_flag Set to 0 to not apply the PHY configurations, else to apply.
 *
 * Return EMAC_OK if successfully, EMAC_TIMEOUT if timeout. 
 */
uint8_t ethernet_phy_set_link(Emac *p_emac, uint8_t uc_phy_addr, uint8_t uc_apply_setting_flag)
{
	emac_enable_management(p_emac, true);
	uint8_t uc_phy_address = uc_phy_addr;

	uint32_t ul_stat1;
	uint8_t uc_rc = emac_phy_read(p_emac, uc_phy_address, MII_BMSR, &ul_stat1);
	if (uc_rc != EMAC_OK) {
		/* Disable PHY management and start the EMAC transfer */
		emac_enable_management(p_emac, false);
		return uc_rc;
	}
	
	if ((ul_stat1 & MII_LINK_STATUS) == 0) {
		/* Disable PHY management and start the EMAC transfer */
		emac_enable_management(p_emac, false);
		return EMAC_INVALID;
	}

	if (uc_apply_setting_flag == 0) {
		/* Disable PHY management and start the EMAC transfer */
		emac_enable_management(p_emac, false);
		return uc_rc;
	}

	// Re-configure Link speed 
	bool uc_speed, uc_fd;
	uint32_t ul_stat2;
	uc_rc = emac_phy_read(p_emac, uc_phy_address, MII_PC1, &ul_stat2);
	if (uc_rc != EMAC_OK) {
		// Disable PHY management and start the EMAC transfer
		emac_enable_management(p_emac, false);
		return uc_rc;
	}

	if ((ul_stat1 & MII_100BASE_TX_FD) && (ul_stat2 & MII_OMI_100_FD)) {
		// Set EMAC for 100BaseTX and Full Duplex 
		uc_speed = true;
		uc_fd = true;
	}
	
	else if ((ul_stat1 & MII_10BASE_T_FD) && (ul_stat2 & MII_OMI_10_FD)) {
		// Set MII for 10BaseT and Full Duplex
		uc_speed = false;
		uc_fd = true;
	}
	
	else if ((ul_stat1 & MII_100BASE_TX_HD) && (ul_stat2 & MII_OMI_100_HD)) {
		// Set MII for 100BaseTX and Half Duplex 
		uc_speed = true;
		uc_fd = false;
	}

	else if ((ul_stat1 & MII_10BASE_T_HD) && (ul_stat2 & MII_OMI_10_HD)) {
		// Set MII for 10BaseT and Half Duplex 
		uc_speed = false;
		uc_fd = false;
	}
	else {
		// not clear what the default should be here (DC)
		uc_speed = false;
		uc_fd = false;
	}
	emac_set_speed(p_emac, uc_speed);
	emac_enable_full_duplex(p_emac, uc_fd);

	/* Start the EMAC transfers */
	emac_enable_management(p_emac, false);
	return uc_rc;
}


/**
 * \brief Issue an auto negotiation of the PHY.
 *
 * \param p_emac   Pointer to the EMAC instance. 
 * \param uc_phy_addr PHY address.
 *
 * Return EMAC_OK if successfully, EMAC_TIMEOUT if timeout (in which case we can keep calling this).
 * This has been refactored as a state machine, because we must not spend much time calling this when
 * no network cable is connected, otherwise it slows down printing.
 */
uint8_t ethernet_phy_auto_negotiate(Emac *p_emac, uint8_t uc_phy_addr)
{
	static uint8_t state = 0;
	static uint8_t ul_retry_count;
	static uint32_t ul_value;
	static uint32_t ul_phy_anar;

	switch(state)
	{
	case 0:
		{
			emac_enable_management(p_emac, true);

			/* Set up control register */
			uint8_t uc_rc = emac_phy_read(p_emac, uc_phy_addr, MII_BMCR, &ul_value);
			if (uc_rc != EMAC_OK)
			{
				emac_enable_management(p_emac, false);
				state = 0;
				return uc_rc;
			}

			++state;
		}
		break;

	case 1:
		{
			ul_value &= ~MII_AUTONEG; /* Remove auto-negotiation enable */
			ul_value &= ~(MII_LOOPBACK | MII_POWER_DOWN);
			ul_value |= MII_ISOLATE; /* Electrically isolate PHY */
			uint8_t uc_rc = emac_phy_write(p_emac, uc_phy_addr, MII_BMCR, ul_value);
			if (uc_rc != EMAC_OK)
			{
				emac_enable_management(p_emac, false);
				state = 0;
				return uc_rc;
			}

			++state;
		}
		break;

	case 2:
		{
			/*
			 * Set the Auto_negotiation Advertisement Register.
			 * MII advertising for Next page.
			 * 100BaseTxFD and HD, 10BaseTFD and HD, IEEE 802.3.
			 */
			ul_phy_anar = MII_TX_FDX | MII_TX_HDX | MII_10_FDX | MII_10_HDX | MII_AN_IEEE_802_3;
			uint8_t uc_rc = emac_phy_write(p_emac, uc_phy_addr, MII_ANAR, ul_phy_anar);
			if (uc_rc != EMAC_OK)
			{
				emac_enable_management(p_emac, false);
				state = 0;
				return uc_rc;
			}

			++state;
		}
		break;

	case 3:
		{
			/* Read & modify control register */
			uint8_t uc_rc = emac_phy_read(p_emac, uc_phy_addr, MII_BMCR, &ul_value);
			if (uc_rc != EMAC_OK)
			{
				emac_enable_management(p_emac, false);
				state = 0;
				return uc_rc;
			}

			++state;
		}
		break;

	case 4:
		{
			ul_value |= MII_SPEED_SELECT | MII_AUTONEG | MII_DUPLEX_MODE;
			uint8_t uc_rc = emac_phy_write(p_emac, uc_phy_addr, MII_BMCR, ul_value);
			if (uc_rc != EMAC_OK)
			{
				emac_enable_management(p_emac, false);
				state = 0;
				return uc_rc;
			}

			++state;
		}
		break;

	case 5:
		{
			/* Restart auto negotiation */
			ul_value |= MII_RESTART_AUTONEG;
			ul_value &= ~MII_ISOLATE;
			uint8_t uc_rc = emac_phy_write(p_emac, uc_phy_addr, MII_BMCR, ul_value);
			if (uc_rc != EMAC_OK)
			{
				emac_enable_management(p_emac, false);
				state = 0;
				return uc_rc;
			}

			ul_retry_count = 0;
			++state;
		}
		break;

	case 6:
		{
			/* Check if auto negotiation is completed */
			uint8_t uc_rc = emac_phy_read(p_emac, uc_phy_addr, MII_BMSR, &ul_value);
			if (uc_rc != EMAC_OK)
			{
				emac_enable_management(p_emac, false);
				state = 0;
				return uc_rc;
			}

			/* Done successfully */
			if (!(ul_value & MII_AUTONEG_COMP))
			{
				++ul_retry_count;
				if (ul_retry_count >= 10000)	// timeout check
				{
					state = 0;					// start again
				}
				return EMAC_TIMEOUT;
			}
			++state;
		}
		break;

	case 7:
		{
			uint32_t ul_phy_analpar;

			/* Get the auto negotiate link partner base page */
			uint8_t uc_rc = emac_phy_read(p_emac, uc_phy_addr, MII_ANLPAR, &ul_phy_analpar);
			if (uc_rc != EMAC_OK)
			{
				emac_enable_management(p_emac, false);
				state = 0;
				return uc_rc;
			}

			// The rest only happens once. so we do it all in one go
			uint8_t uc_fd = false;
			uint8_t uc_speed = false;

			/* Set up the EMAC link speed */
			if ((ul_phy_anar & ul_phy_analpar) & MII_TX_FDX)
			{
				/* Set MII for 100BaseTX and Full Duplex */
				uc_speed = true;
				uc_fd = true;
			}
			else if ((ul_phy_anar & ul_phy_analpar) & MII_10_FDX)
			{
				/* Set MII for 10BaseT and Full Duplex */
				uc_speed = false;
				uc_fd = true;
			}
			else if ((ul_phy_anar & ul_phy_analpar) & MII_TX_HDX)
			{
				/* Set MII for 100BaseTX and half Duplex */
				uc_speed = true;
				uc_fd = false;
			}
			else if ((ul_phy_anar & ul_phy_analpar) & MII_10_HDX)
			{
				/* Set MII for 10BaseT and half Duplex */
				uc_speed = false;
				uc_fd = false;
			}

			emac_set_speed(p_emac, uc_speed);
			emac_enable_full_duplex(p_emac, uc_fd);
			emac_enable_rmii(p_emac, ETH_PHY_MODE);
			emac_enable_transceiver_clock(p_emac, true);
			emac_enable_management(p_emac, false);

			state = 0;			// in case we get called again
			return EMAC_OK;
		}
	}

	return EMAC_TIMEOUT;	// this just means that we needs to be called again
}

/**
 * \brief Issue a SW reset to reset all registers of the PHY.
 *
 * \param p_emac   Pointer to the EMAC instance. 
 * \param uc_phy_addr PHY address.
 *
 * \Return EMAC_OK if successfully, EMAC_TIMEOUT if timeout.
 */
uint8_t ethernet_phy_reset(Emac *p_emac, uint8_t uc_phy_addr)
{
	uint32_t ul_bmcr = MII_RESET;
	uint8_t uc_phy_address = uc_phy_addr;
	uint32_t ul_timeout = ETH_PHY_TIMEOUT;
	uint8_t uc_rc = EMAC_TIMEOUT;

	emac_enable_management(p_emac, true);

	ul_bmcr = MII_RESET;
	emac_phy_write(p_emac, uc_phy_address, MII_BMCR, ul_bmcr);

	do {
		emac_phy_read(p_emac, uc_phy_address, MII_BMCR, &ul_bmcr);
		ul_timeout--;
	} while ((ul_bmcr & MII_RESET) && ul_timeout);

	emac_enable_management(p_emac, false);

	if (!ul_timeout) {
		uc_rc = EMAC_OK;
	}

	return (uc_rc);
}

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond

/**
 * \}
 */
