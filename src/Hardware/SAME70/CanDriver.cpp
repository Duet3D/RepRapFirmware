/**
 * \file
 *
 * \brief SAM Control Area Network (MCAN) Low Level Driver
 *
 * Copyright (c) 2015-2019 Microchip Technology Inc. and its subsidiaries.
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
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#include "CanDriver.h"

#if SUPPORT_CAN_EXPANSION

#include <sam/drivers/pmc/pmc.h>
#include <cstring>

// Values from conf_mcan.h

/*
 * Below is the message RAM setting, it will be stored in the system RAM.
 * Please adjust the message size according to your application.
 */
/** Range: 1..64 */
#define CONF_MCAN0_RX_FIFO_0_NUM         16
/** Range: 1..64 */
#define CONF_MCAN0_RX_FIFO_1_NUM         16
/** Range: 1..64 */
#define CONF_MCAN0_RX_BUFFER_NUM         4
/** Range: 1..16 */
#define CONF_MCAN0_TX_BUFFER_NUM         6
/** Range: 1..16 */
#define CONF_MCAN0_TX_FIFO_QUEUE_NUM     8
/** Range: 1..32 */
#define CONF_MCAN0_TX_EVENT_FIFO         8
/** Range: 1..128 */
#define CONF_MCAN0_RX_STANDARD_ID_FILTER_NUM     1
/** Range: 1..64 */
#define CONF_MCAN0_RX_EXTENDED_ID_FILTER_NUM     3
/** Range: 1..64 */
#define CONF_MCAN1_RX_FIFO_0_NUM         16
/** Range: 1..64 */
#define CONF_MCAN1_RX_FIFO_1_NUM         16
/** Range: 1..64 */
#define CONF_MCAN1_RX_BUFFER_NUM         4
/** Range: 1..16 */
#define CONF_MCAN1_TX_BUFFER_NUM         6
/** Range: 1..16 */
#define CONF_MCAN1_TX_FIFO_QUEUE_NUM     8
/** Range: 1..32 */
#define CONF_MCAN1_TX_EVENT_FIFO         8
/** Range: 1..128 */
#define CONF_MCAN1_RX_STANDARD_ID_FILTER_NUM     1
/** Range: 1..64 */
#define CONF_MCAN1_RX_EXTENDED_ID_FILTER_NUM     3

/**
 * The setting of the nominal bit rate is based on the PCK5 which is 48M which you can
 * change in the conf_clock.h. Below is the default configuration. The
 * time quanta is 48MHz. And each bit is (1 + NTSEG1 + 1 + NTSEG2 + 1) = 48 time
 * quanta which means the bit rate is 48MHz/49 = 1Mbps.
 */
/** Nominal bit Baud Rate Prescaler */
#define CONF_MCAN_NBTP_NBRP_VALUE    (1 - 1)
/** Nominal bit (Re)Synchronization Jump Width */
#define CONF_MCAN_NBTP_NSJW_VALUE    (8 - 1)
/** Nominal bit Time segment before sample point */
#define CONF_MCAN_NBTP_NTSEG1_VALUE (26 - 1)
/** Nominal bit Time segment after sample point */
#define CONF_MCAN_NBTP_NTSEG2_VALUE  (21 - 1)

/*
 * The setting of the data bit rate is based on the PCK5 which is 48M which you can
 * change. Below is the default configuration. The
 * time quanta is 48MHz / (0+1) =  48MHz. And each bit is (1 + FTSEG1 + 1 + FTSEG2 + 1) = 12 time
 * quanta which means the bit rate is 48MHz/12=4MHz.
 */
/** Data bit Baud Rate Prescaler */
#define CONF_MCAN_FBTP_FBRP_VALUE    (1 - 1)
/** Data bit (Re)Synchronization Jump Width */
#define CONF_MCAN_FBTP_FSJW_VALUE    (3 - 1)
/** Data bit Time segment before sample point */
#define CONF_MCAN_FBTP_FTSEG1_VALUE  (8 - 1)
/** Data bit Time segment after sample point */
#define CONF_MCAN_FBTP_FTSEG2_VALUE  (3 - 1)


/* PCK5 ID assigned to MCAN module */
#define PMC_PCK_5               5
/* Get a value of 2 to 15 bit. */
#define BIT_2_TO_15_MASK         0x0000fffc

/* Message ram definition. */
#define CanMemory	__attribute__ ((section (".CanMessage")))
alignas(4) CanMemory static struct mcan_rx_element mcan0_rx_buffer[CONF_MCAN0_RX_BUFFER_NUM];
alignas(4) CanMemory static struct mcan_rx_element mcan0_rx_fifo_0[CONF_MCAN0_RX_FIFO_0_NUM];
alignas(4) CanMemory static struct mcan_rx_element mcan0_rx_fifo_1[CONF_MCAN0_RX_FIFO_1_NUM];
alignas(4) CanMemory static struct mcan_tx_element mcan0_tx_buffer[CONF_MCAN0_TX_BUFFER_NUM + CONF_MCAN0_TX_FIFO_QUEUE_NUM];
alignas(4) CanMemory static struct mcan_tx_event_element mcan0_tx_event_fifo[CONF_MCAN0_TX_EVENT_FIFO];
alignas(4) CanMemory static struct mcan_standard_message_filter_element mcan0_rx_standard_filter[CONF_MCAN0_RX_STANDARD_ID_FILTER_NUM];
alignas(4) CanMemory static struct mcan_extended_message_filter_element mcan0_rx_extended_filter[CONF_MCAN0_RX_EXTENDED_ID_FILTER_NUM];

alignas(4) CanMemory static struct mcan_rx_element mcan1_rx_buffer[CONF_MCAN1_RX_BUFFER_NUM];
alignas(4) CanMemory static struct mcan_rx_element mcan1_rx_fifo_0[CONF_MCAN1_RX_FIFO_0_NUM];
alignas(4) CanMemory static struct mcan_rx_element mcan1_rx_fifo_1[CONF_MCAN1_RX_FIFO_1_NUM];
alignas(4) CanMemory static struct mcan_tx_element mcan1_tx_buffer[CONF_MCAN1_TX_BUFFER_NUM + CONF_MCAN1_TX_FIFO_QUEUE_NUM];
alignas(4) CanMemory static struct mcan_tx_event_element mcan1_tx_event_fifo[CONF_MCAN1_TX_EVENT_FIFO];
alignas(4) CanMemory static struct mcan_standard_message_filter_element mcan1_rx_standard_filter[CONF_MCAN1_RX_STANDARD_ID_FILTER_NUM];
alignas(4) CanMemory static struct mcan_extended_message_filter_element mcan1_rx_extended_filter[CONF_MCAN1_RX_EXTENDED_ID_FILTER_NUM];

static constexpr uint8_t dlc2len[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

/**
 * \brief initialize MCAN memory .
 *
 * \param hw  Base address of the MCAN
 *
 */
static void _mcan_message_memory_init(Mcan *hw) noexcept
{
	if (hw == MCAN0) {
		hw->MCAN_SIDFC = ((uint32_t)mcan0_rx_standard_filter & BIT_2_TO_15_MASK) |
				MCAN_SIDFC_LSS(CONF_MCAN0_RX_STANDARD_ID_FILTER_NUM);
		hw->MCAN_XIDFC = ((uint32_t)mcan0_rx_extended_filter & BIT_2_TO_15_MASK) |
				MCAN_XIDFC_LSE(CONF_MCAN0_RX_EXTENDED_ID_FILTER_NUM);
		hw->MCAN_RXF0C = ((uint32_t)mcan0_rx_fifo_0 & BIT_2_TO_15_MASK) |
				MCAN_RXF0C_F0S(CONF_MCAN0_RX_FIFO_0_NUM);
		hw->MCAN_RXF1C = ((uint32_t)mcan0_rx_fifo_1 & BIT_2_TO_15_MASK) |
				MCAN_RXF1C_F1S(CONF_MCAN0_RX_FIFO_1_NUM);
		hw->MCAN_RXBC = ((uint32_t)mcan0_rx_buffer & BIT_2_TO_15_MASK);
		hw->MCAN_TXBC = ((uint32_t)mcan0_tx_buffer & BIT_2_TO_15_MASK) |
				MCAN_TXBC_NDTB(CONF_MCAN0_TX_BUFFER_NUM) |
				MCAN_TXBC_TFQS(CONF_MCAN0_TX_FIFO_QUEUE_NUM);
		hw->MCAN_TXEFC = ((uint32_t)mcan0_tx_event_fifo & BIT_2_TO_15_MASK) |
				MCAN_TXEFC_EFS(CONF_MCAN0_TX_EVENT_FIFO);
	} else if (hw == MCAN1) {
		hw->MCAN_SIDFC = ((uint32_t)mcan1_rx_standard_filter & BIT_2_TO_15_MASK) |
				MCAN_SIDFC_LSS(CONF_MCAN1_RX_STANDARD_ID_FILTER_NUM);
		hw->MCAN_XIDFC = ((uint32_t)mcan1_rx_extended_filter & BIT_2_TO_15_MASK) |
				MCAN_XIDFC_LSE(CONF_MCAN1_RX_EXTENDED_ID_FILTER_NUM);
		hw->MCAN_RXF0C = ((uint32_t)mcan1_rx_fifo_0 & BIT_2_TO_15_MASK) |
				MCAN_RXF0C_F0S(CONF_MCAN1_RX_FIFO_0_NUM);
		hw->MCAN_RXF1C = ((uint32_t)mcan1_rx_fifo_1 & BIT_2_TO_15_MASK) |
				MCAN_RXF1C_F1S(CONF_MCAN1_RX_FIFO_1_NUM);
		hw->MCAN_RXBC = ((uint32_t)mcan1_rx_buffer & BIT_2_TO_15_MASK);
		hw->MCAN_TXBC = ((uint32_t)mcan1_tx_buffer & BIT_2_TO_15_MASK) |
				MCAN_TXBC_NDTB(CONF_MCAN1_TX_BUFFER_NUM) |
				MCAN_TXBC_TFQS(CONF_MCAN1_TX_FIFO_QUEUE_NUM);
		hw->MCAN_TXEFC = ((uint32_t)mcan1_tx_event_fifo & BIT_2_TO_15_MASK) |
				MCAN_TXEFC_EFS(CONF_MCAN1_TX_EVENT_FIFO);
	}

	/**
	 * The data size in conf_mcan.h should be 8/12/16/20/24/32/48/64,
	 * The corresponding setting value in register is 0/1//2/3/4/5/6/7.
	 * To simplify the calculation, separate to two group 8/12/16/20/24 which
	 * increased with 4 and 32/48/64 which increased with 16.
	 */
	if (CONF_MCAN_ELEMENT_DATA_SIZE <= 24) {
		hw->MCAN_RXESC = MCAN_RXESC_RBDS((CONF_MCAN_ELEMENT_DATA_SIZE - 8) / 4) |
				MCAN_RXESC_F0DS((CONF_MCAN_ELEMENT_DATA_SIZE - 8) / 4) |
				MCAN_RXESC_F1DS((CONF_MCAN_ELEMENT_DATA_SIZE - 8) / 4);
		hw->MCAN_TXESC = MCAN_TXESC_TBDS((CONF_MCAN_ELEMENT_DATA_SIZE - 8) / 4);
	} else {
		hw->MCAN_RXESC = MCAN_RXESC_RBDS((CONF_MCAN_ELEMENT_DATA_SIZE - 32) / 16 + 5) |
				MCAN_RXESC_F0DS((CONF_MCAN_ELEMENT_DATA_SIZE - 32) / 16 + 5) |
				MCAN_RXESC_F1DS((CONF_MCAN_ELEMENT_DATA_SIZE - 32) / 16 + 5);
		hw->MCAN_TXESC = MCAN_TXESC_TBDS((CONF_MCAN_ELEMENT_DATA_SIZE - 32) / 16 + 5);
	}
}

/**
 * \brief set default configuration when initialization.
 *
 * \param hw  Base address of the MCAN
 * \param config  default configuration parameters.
 */
static void _mcan_set_configuration(Mcan *hw, mcan_config *config) noexcept
{
#if (SAMV71B || SAME70B || SAMV70B)
	/* Timing setting for Rev B */
	hw->MCAN_NBTP = MCAN_NBTP_NBRP(CONF_MCAN_NBTP_NBRP_VALUE) |
			MCAN_NBTP_NSJW(CONF_MCAN_NBTP_NSJW_VALUE) |
			MCAN_NBTP_NTSEG1(CONF_MCAN_NBTP_NTSEG1_VALUE) |
			MCAN_NBTP_NTSEG2(CONF_MCAN_NBTP_NTSEG2_VALUE);
	hw->MCAN_DBTP = MCAN_DBTP_DBRP(CONF_MCAN_FBTP_FBRP_VALUE) |
			MCAN_DBTP_DSJW(CONF_MCAN_FBTP_FSJW_VALUE) |
			MCAN_DBTP_DTSEG1(CONF_MCAN_FBTP_FTSEG1_VALUE) |
			MCAN_DBTP_DTSEG2(CONF_MCAN_FBTP_FTSEG2_VALUE);

	hw->MCAN_TDCR = MCAN_TDCR_TDCO(config->delay_compensation_offset) |
		    MCAN_TDCR_TDCF(config->delay_compensation_filter_window_length);

	if (config->tdc_enable) {
		hw->MCAN_DBTP |= MCAN_DBTP_TDC_ENABLED;
	}
#else
	/* Timing setting. */
	hw->MCAN_BTP = MCAN_BTP_BRP(CONF_MCAN_NBTP_NBRP_VALUE) |
			MCAN_BTP_SJW(CONF_MCAN_NBTP_NSJW_VALUE) |
			MCAN_BTP_TSEG1(CONF_MCAN_NBTP_NTSEG1_VALUE) |
			MCAN_BTP_TSEG2(CONF_MCAN_NBTP_NTSEG2_VALUE);
	hw->MCAN_FBTP = MCAN_FBTP_FBRP(CONF_MCAN_FBTP_FBRP_VALUE) |
			MCAN_FBTP_FSJW(CONF_MCAN_FBTP_FSJW_VALUE) |
			MCAN_FBTP_FTSEG1(CONF_MCAN_FBTP_FTSEG1_VALUE) |
			MCAN_FBTP_FTSEG2(CONF_MCAN_FBTP_FTSEG2_VALUE) |
			MCAN_FBTP_TDCO(config->delay_compensation_offset);

	if (config->tdc_enable) {
		hw->MCAN_FBTP |= MCAN_FBTP_TDC_ENABLED;
	}
#endif
	hw->MCAN_RWD |= MCAN_RWD_WDC(config->watchdog_configuration);

	if (config->transmit_pause) {
		hw->MCAN_CCCR |= MCAN_CCCR_TXP;
	}

	if (!config->automatic_retransmission) {
		hw->MCAN_CCCR |= MCAN_CCCR_DAR;
	}

	if (config->clock_stop_request) {
		hw->MCAN_CCCR |= MCAN_CCCR_CSR;
	}

	hw->MCAN_TSCC = MCAN_TSCC_TCP(config->timestamp_prescaler) |
			MCAN_TSCC_TSS_TCP_INC;

	hw->MCAN_TOCC = MCAN_TOCC_TOP(config->timeout_period) |
			config->timeout_mode | config->timeout_enable;

	hw->MCAN_GFC = MCAN_GFC_ANFS(config->nonmatching_frames_action_standard) |
			MCAN_GFC_ANFE(config->nonmatching_frames_action_extended);
	if (config->remote_frames_standard_reject) {
		hw->MCAN_GFC |= MCAN_GFC_RRFS;
	}
	if (config->remote_frames_extended_reject) {
		hw->MCAN_GFC|= MCAN_GFC_RRFE;
	}

	hw->MCAN_XIDAM = config->extended_id_mask;

	if (config->rx_fifo_0_overwrite) {
		hw->MCAN_RXF0C |= MCAN_RXF0C_F0OM;
	}
	hw->MCAN_RXF0C |= MCAN_RXF0C_F0WM(config->rx_fifo_0_watermark);

	if (config->rx_fifo_1_overwrite) {
		hw->MCAN_RXF1C |= MCAN_RXF1C_F1OM;
	}
	hw->MCAN_RXF1C |= MCAN_RXF1C_F1WM(config->rx_fifo_1_watermark);

	if (config->tx_queue_mode) {
		hw->MCAN_TXBC |= MCAN_TXBC_TFQM;
	}

	hw->MCAN_TXEFC |= MCAN_TXEFC_EFWM(config->tx_event_fifo_watermark);
}

/**
 * \brief enable can module clock.
 *
 * \param module_inst  MCAN instance
 *
 */
static void _mcan_enable_peripheral_clock(mcan_module *const module_inst) noexcept
{
	if (module_inst->hw == MCAN0) {
		/* Turn on the digital interface clock. */
		pmc_enable_periph_clk(ID_MCAN0);
	} else if (module_inst->hw == MCAN1) {
		/* Turn on the digital interface clock. */
		pmc_enable_periph_clk(ID_MCAN1);
	}
}

// One-time initialisation. Do not call this to reset after a bus_off condition.
void mcan_init_once(mcan_module *const module_inst, Mcan *hw) noexcept
{
	/* Associate the software module instance with the hardware module */
	module_inst->hw = hw;
	module_inst->taskWaitingOnFifo[0] = module_inst->taskWaitingOnFifo[1] = nullptr;

	pmc_disable_pck(PMC_PCK_5);

	// Use UPLL so the MCAN clock is independent of the CPU clock frequency
	pmc_switch_pck_to_upllck(PMC_PCK_5, PMC_PCK_PRES(9));		// run PCLK5 at 48MHz
	pmc_enable_pck(PMC_PCK_5);

	/* Enable peripheral clock */
	_mcan_enable_peripheral_clock(module_inst);
}

/**
 * \brief initialize can module.
 *
 * \param module_inst  MCAN instance
 * \param hw  Base address of MCAN.
 * \param config default configuration .
 */
void mcan_init(mcan_module *const module_inst, struct mcan_config *config) noexcept
{
	Mcan * const hw = module_inst->hw;

	/* Configuration Change Enable. */
	hw->MCAN_CCCR |= MCAN_CCCR_CCE;

	/* Initialize the message memory address. */
	_mcan_message_memory_init(hw);

	/* Set the configuration. */
	_mcan_set_configuration(hw, config);

	/* Enable the interrupt setting which no need change. */
	hw->MCAN_ILE = MCAN_ILE_EINT0 | MCAN_ILE_EINT1;
	hw->MCAN_TXBTIE = 0xFFFFFFFFul;
	hw->MCAN_TXBCIE = 0xFFFFFFFFul;
}

/**
 * \brief start can module after initialization.
 *
 * \param module_inst  MCAN instance
 *
 */
void mcan_start(mcan_module *const module_inst) noexcept
{
	module_inst->hw->MCAN_CCCR &= ~MCAN_CCCR_INIT;
	/* Wait for the sync. */
	while (module_inst->hw->MCAN_CCCR & MCAN_CCCR_INIT);
}

/**
 * \brief stop mcan module when bus off occurs
 *
 * \param module_inst  MCAN instance
 *
 */
void mcan_stop(mcan_module *const module_inst) noexcept
{
	module_inst->hw->MCAN_CCCR |= MCAN_CCCR_INIT;
	/* Wait for the sync. */
	while (!(module_inst->hw->MCAN_CCCR & MCAN_CCCR_INIT));
}

/**
 * \brief switch mcan module into fd mode.
 *
 * \param module_inst  MCAN instance
 *
 */
void mcan_enable_fd_mode(mcan_module *const module_inst) noexcept
{
	module_inst->hw->MCAN_CCCR |= MCAN_CCCR_INIT;
	/* Wait for the sync. */
	while (!(module_inst->hw->MCAN_CCCR & MCAN_CCCR_INIT));
	module_inst->hw->MCAN_CCCR |= MCAN_CCCR_CCE;
#if (SAMV71B || SAME70B || SAMV70B)
	module_inst->hw->MCAN_CCCR |= (MCAN_CCCR_FDOE | MCAN_CCCR_BRSE);
#else
	module_inst->hw->MCAN_CCCR |= MCAN_CCCR_CME(2);
	module_inst->hw->MCAN_CCCR |= MCAN_CCCR_CMR(2);
#endif
}

/**
 * \brief disable fd mode of mcan module.
 *
 * \param module_inst  MCAN instance
 *
 */
void mcan_disable_fd_mode(mcan_module *const module_inst) noexcept
{
	module_inst->hw->MCAN_CCCR |= MCAN_CCCR_INIT;
	/* Wait for the sync. */
	while (!(module_inst->hw->MCAN_CCCR & MCAN_CCCR_INIT));
	module_inst->hw->MCAN_CCCR |= MCAN_CCCR_CCE;
#if (SAMV71B || SAME70B || SAMV70B)
	module_inst->hw->MCAN_CCCR &= MCAN_CCCR_FDOE;
#else
	module_inst->hw->MCAN_CCCR &= MCAN_CCCR_CME(MCAN_CCCR_CME_ISO11898_1);
#endif
}

/**
 * \brief enable restricted mode of mcan module.
 *
 * \param module_inst  MCAN instance
 *
 */
void mcan_enable_restricted_operation_mode(mcan_module *const module_inst) noexcept
{
	module_inst->hw->MCAN_CCCR |= MCAN_CCCR_INIT;
	/* Wait for the sync. */
	while (!(module_inst->hw->MCAN_CCCR & MCAN_CCCR_INIT));
	module_inst->hw->MCAN_CCCR |= MCAN_CCCR_CCE;

	module_inst->hw->MCAN_CCCR |= MCAN_CCCR_ASM;
}

/**
 * \brief disable restricted mode of mcan module.
 *
 * \param module_inst  MCAN instance
 *
 */
void mcan_disable_restricted_operation_mode(mcan_module *const module_inst) noexcept
{
	module_inst->hw->MCAN_CCCR &= ~MCAN_CCCR_ASM;
}

/**
 * \brief enable bus monitor mode of mcan module.
 *
 * \param module_inst  MCAN instance
 *
 */
void mcan_enable_bus_monitor_mode(mcan_module *const module_inst) noexcept
{
	module_inst->hw->MCAN_CCCR |= MCAN_CCCR_INIT;
	/* Wait for the sync. */
	while (!(module_inst->hw->MCAN_CCCR & MCAN_CCCR_INIT));
	module_inst->hw->MCAN_CCCR |= MCAN_CCCR_CCE;

	module_inst->hw->MCAN_CCCR |= MCAN_CCCR_MON;
}

/**
 * \brief disable bus monitor mode of mcan module.
 *
 * \param module_inst  MCAN instance
 *
 */
void mcan_disable_bus_monitor_mode(mcan_module *const module_inst) noexcept
{
	module_inst->hw->MCAN_CCCR &= ~MCAN_CCCR_MON;
}

/**
 * \brief enable sleep mode of mcan module.
 *
 * \param module_inst  MCAN instance
 *
 */
void mcan_enable_sleep_mode(mcan_module *const module_inst) noexcept
{
	module_inst->hw->MCAN_CCCR |= MCAN_CCCR_CSR;
	/* Wait for the sync. */
	while (!(module_inst->hw->MCAN_CCCR & MCAN_CCCR_INIT));

	while (!(module_inst->hw->MCAN_CCCR & MCAN_CCCR_CSA));
}

/**
 * \brief disable sleep mode of mcan module.
 *
 * \param module_inst  MCAN instance
 *
 */
void mcan_disable_sleep_mode(struct mcan_module *const module_inst) noexcept
{
	/* Enable peripheral clock */
	_mcan_enable_peripheral_clock(module_inst);

	module_inst->hw->MCAN_CCCR &= ~MCAN_CCCR_CSR;
	while ((module_inst->hw->MCAN_CCCR & MCAN_CCCR_CSA));
}

/**
 * \brief enable test mode of mcan module.
 *
 * \param module_inst  MCAN instance
 *
 */
void mcan_enable_test_mode(struct mcan_module *const module_inst) noexcept
{
	module_inst->hw->MCAN_CCCR |= MCAN_CCCR_INIT;
	/* Wait for the sync. */
	while (!(module_inst->hw->MCAN_CCCR & MCAN_CCCR_INIT));
	module_inst->hw->MCAN_CCCR |= MCAN_CCCR_CCE;

	module_inst->hw->MCAN_CCCR |= MCAN_CCCR_TEST;
	module_inst->hw->MCAN_TEST |= MCAN_TEST_LBCK;
}

/**
 * \brief disable test mode of mcan module.
 *
 * \param module_inst  MCAN instance
 *
 */
void mcan_disable_test_mode(struct mcan_module *const module_inst) noexcept
{
	module_inst->hw->MCAN_CCCR &= ~MCAN_CCCR_TEST;
}

/**
 * \brief set standard receive CAN ID.
 *
 * \param module_inst  MCAN instance
 * \param sd_filter  structure of CAN ID
 * \param index  CAN messages memory index for different CAN ID
 *
 * \return status code.
 */
status_code mcan_set_rx_standard_filter(struct mcan_module *const module_inst, struct mcan_standard_message_filter_element *sd_filter, uint32_t index) noexcept
{
	if (module_inst->hw == MCAN0) {
		mcan0_rx_standard_filter[index].S0.reg = sd_filter->S0.reg;
		return STATUS_OK;
	} else if (module_inst->hw == MCAN1) {
		mcan1_rx_standard_filter[index].S0.reg = sd_filter->S0.reg;
		return STATUS_OK;
	}
	return ERR_INVALID_ARG;
}

/**
 * \brief set extended receive CAN ID.
 *
 * \param module_inst  MCAN instance
 * \param sd_filter  structure of extended CAN ID
 * \param index  CAN messages memory index for different CAN ID
 *
 * \return status code.
 */
status_code mcan_set_rx_extended_filter(struct mcan_module *const module_inst, struct mcan_extended_message_filter_element *et_filter, uint32_t index) noexcept
{
	if (module_inst->hw == MCAN0) {
		mcan0_rx_extended_filter[index].F0.reg = et_filter->F0.reg;
		mcan0_rx_extended_filter[index].F1.reg = et_filter->F1.reg;
		return STATUS_OK;
	} else if (module_inst->hw == MCAN1) {
		mcan1_rx_extended_filter[index].F0.reg = et_filter->F0.reg;
		mcan1_rx_extended_filter[index].F1.reg = et_filter->F1.reg;
		return STATUS_OK;
	}
	return ERR_INVALID_ARG;
}

/**
 * \brief get dedicated rx buffer element .
 *
 * \param module_inst  MCAN instance
 * \param rx_element  structure of element
 * \param index  CAN messages memory index for receiving CAN ID
 *
 * \return status code.
 */
status_code mcan_get_rx_buffer_element(struct mcan_module *const module_inst, struct mcan_rx_element *rx_element, uint32_t index) noexcept
{
	if (module_inst->hw == MCAN0) {
		*rx_element = mcan0_rx_buffer[index];
		return STATUS_OK;
	} else if (module_inst->hw == MCAN1) {
		*rx_element = mcan1_rx_buffer[index];
		return STATUS_OK;
	}
	return ERR_INVALID_ARG;
}

/**
 * \brief get FIFO rx buffer element .
 *
 * \param module_inst  MCAN instance
 * \param rx_element  structure of element
 * \param index  CAN messages memory index for receiving CAN ID
 *
 * \return status code.
 */
status_code mcan_get_rx_fifo_0_element(struct mcan_module *const module_inst, struct mcan_rx_element *rx_element, uint32_t index) noexcept
{
	if (module_inst->hw == MCAN0) {
		*rx_element = mcan0_rx_fifo_0[index];
		return STATUS_OK;
	} else if (module_inst->hw == MCAN1) {
		*rx_element = mcan1_rx_fifo_0[index];
		return STATUS_OK;
	}
	return ERR_INVALID_ARG;
}

/**
 * \brief get FIFO rx buffer element .
 *
 * \param module_inst  MCAN instance
 * \param rx_element  structure of element
 * \param index  CAN messages memory index for receiving CAN ID
 *
 * \return status code.
 */
status_code mcan_get_rx_fifo_1_element(struct mcan_module *const module_inst, struct mcan_rx_element *rx_element, uint32_t index) noexcept
{
	if (module_inst->hw == MCAN0) {
		*rx_element = mcan0_rx_fifo_1[index];
		return STATUS_OK;
	} else if (module_inst->hw == MCAN1) {
		*rx_element = mcan1_rx_fifo_1[index];
		return STATUS_OK;
	}
	return ERR_INVALID_ARG;
}

/**
 * \brief set dedicated transmit buffer element .
 *
 * \param module_inst  MCAN instance
 * \param tx_element  structure of element
 * \param index  CAN messages memory index for transmitting CAN ID
 *
 * \return status code.
 */
status_code mcan_set_tx_buffer_element(struct mcan_module *const module_inst, struct mcan_tx_element *tx_element, uint32_t index) noexcept
{
	uint32_t i;
	if (module_inst->hw == MCAN0) {
		mcan0_tx_buffer[index].T0.reg = tx_element->T0.reg;
		mcan0_tx_buffer[index].T1.reg = tx_element->T1.reg;
		for (i = 0; i < CONF_MCAN_ELEMENT_DATA_SIZE; i++) {
			mcan0_tx_buffer[index].data[i] = tx_element->data[i];
		}
		return STATUS_OK;
	} else if (module_inst->hw == MCAN1) {
		mcan1_tx_buffer[index].T0.reg = tx_element->T0.reg;
		mcan1_tx_buffer[index].T1.reg = tx_element->T1.reg;
		for (i = 0; i < CONF_MCAN_ELEMENT_DATA_SIZE; i++) {
			mcan1_tx_buffer[index].data[i] = tx_element->data[i];
		}
		return STATUS_OK;
	}
	return ERR_INVALID_ARG;
}

/**
 * \brief set FIFO transmit buffer element .
 *
 * \param module_inst  MCAN instance
 * \param tx_element  structure of element
 * \param index  CAN messages memory index for transmitting CAN ID
 *
 * \return status code.
 */
status_code mcan_get_tx_event_fifo_element(struct mcan_module *const module_inst, struct mcan_tx_event_element *tx_event_element, uint32_t index) noexcept
{
	if (module_inst->hw == MCAN0) {
		tx_event_element->E0.reg = mcan0_tx_event_fifo[index].E0.reg;
		tx_event_element->E1.reg = mcan0_tx_event_fifo[index].E1.reg;
		return STATUS_OK;
	} else if (module_inst->hw == MCAN1) {
		tx_event_element->E0.reg = mcan1_tx_event_fifo[index].E0.reg;
		tx_event_element->E1.reg = mcan1_tx_event_fifo[index].E1.reg;
		return STATUS_OK;
	}
	return ERR_INVALID_ARG;
}

// Send extended CAN message in FD mode using a dedicated transmit buffer. The transmit buffer must already be free.
status_code mcan_fd_send_ext_message_no_wait(mcan_module *const module_inst, uint32_t id_value, const uint8_t *data, size_t dataLength, uint32_t whichTxBuffer, bool bitRateSwitch) noexcept
{
	const uint32_t dlc = (dataLength <= 8) ? dataLength
							: (dataLength <= 24) ? ((dataLength + 3) >> 2) + 6
								: ((dataLength + 15) >> 4) + 11;
	mcan_tx_element tx_element;
	tx_element.T0.reg = MCAN_TX_ELEMENT_T0_EXTENDED_ID(id_value) | MCAN_TX_ELEMENT_T0_XTD;
	tx_element.T1.reg = MCAN_TX_ELEMENT_T1_DLC(dlc)
						| MCAN_TX_ELEMENT_T1_EFC
						| MCAN_TX_ELEMENT_T1_FDF
						| ((bitRateSwitch) ? MCAN_TX_ELEMENT_T1_BRS : 0);

	memcpy(tx_element.data, data, dataLength);

	// Set any extra data we will be sending to zero
	const size_t roundedUpLength = dlc2len[dlc];
	memset(tx_element.data + dataLength, 0, roundedUpLength - dataLength);

	status_code rc = mcan_set_tx_buffer_element(module_inst, &tx_element, whichTxBuffer);
	if (rc == STATUS_OK)
	{
		rc = mcan_tx_transfer_request(module_inst, (uint32_t)1 << whichTxBuffer);
	}
	return rc;
}

// Wait for a specified buffer to become free. If it's still not free after the timeout, cancel the pending transmission.
// Return true if we cancelled the pending transmission.
bool WaitForTxBufferFree(mcan_module *const module_inst, uint32_t whichTxBuffer, uint32_t maxWait) noexcept
{
	const uint32_t trigMask = (uint32_t)1 << whichTxBuffer;
	if ((module_inst->hw->MCAN_TXBRP & trigMask) != 0)
	{
		// Wait for the timeout period for the message to be sent
		const uint32_t startTime = millis();
		do
		{
			delay(1);
			if ((module_inst->hw->MCAN_TXBRP & trigMask) == 0)
			{
				return false;
			}
		} while (millis() - startTime < maxWait);

		// The last message still hasn't been sent, so cancel it
		module_inst->hw->MCAN_TXBCR = trigMask;
		while ((module_inst->hw->MCAN_TXBRP & trigMask) != 0)
		{
			delay(1);
		}
		return true;
	}
	return false;
}

// Get a message from a FIFO with timeout. Return true if successful, false if we timed out
bool GetMessageFromFifo(mcan_module *const module_inst, CanMessageBuffer *buf, unsigned int fifoNumber, uint32_t timeout) noexcept
{
	volatile uint32_t* const fifoRegisters = &(module_inst->hw->MCAN_RXF0S) + (4 * fifoNumber);	// pointer to FIFO status register followed by FIFO acknowledge register
	module_inst->taskWaitingOnFifo[fifoNumber] = TaskBase::GetCallerTaskHandle();
	while (true)
	{
		const uint32_t status = fifoRegisters[0];									// get FIFO status
		if ((status & MCAN_RXF0S_F0FL_Msk) != 0)									// if there are any messages
		{
			const uint32_t getIndex = (status & MCAN_RXF0S_F0GI_Msk) >> MCAN_RXF0S_F0GI_Pos;
			mcan_rx_element elem;
			if (fifoNumber == 1)
			{
				mcan_get_rx_fifo_1_element(module_inst, &elem, getIndex);			// copy the data (TODO use our own driver, avoid double copying)
			}
			else
			{
				mcan_get_rx_fifo_0_element(module_inst, &elem, getIndex);			// copy the data (TODO use our own driver, avoid double copying)
			}
			fifoRegisters[1] = getIndex;											// acknowledge it, release the FIFO entry

			if (elem.R0.bit.XTD == 1 && elem.R0.bit.RTR != 1)						// if extended address and not a remote frame
			{
				// Copy the message and accompanying data to our buffer
				buf->id.SetReceivedId(elem.R0.bit.ID);
				buf->dataLength = dlc2len[elem.R1.bit.DLC];
				memcpy(buf->msg.raw, elem.data, buf->dataLength);
				module_inst->taskWaitingOnFifo[fifoNumber] = nullptr;
				return true;
			}
		}
		else if (!TaskBase::Take(timeout))
		{
			module_inst->taskWaitingOnFifo[fifoNumber] = nullptr;
			return false;
		}
	}
}

void GetLocalCanTiming(mcan_module *const module_inst, CanTiming& timing) noexcept
{
	const uint32_t nbtp = module_inst->hw->MCAN_NBTP;
	const uint32_t tseg1 = (nbtp & MCAN_NBTP_NTSEG1_Msk) >> MCAN_NBTP_NTSEG1_Pos;
	const uint32_t tseg2 = (nbtp & MCAN_NBTP_NTSEG2_Msk) >> MCAN_NBTP_NTSEG2_Pos;
	const uint32_t jw = (nbtp & MCAN_NBTP_NSJW_Msk) >> MCAN_NBTP_NSJW_Pos;
	const uint32_t brp = (nbtp & MCAN_NBTP_NBRP_Msk) >> MCAN_NBTP_NBRP_Pos;
	timing.period = (tseg1 + tseg2 + 3) * (brp + 1);
	timing.tseg1 = (tseg1 + 1) * (brp + 1);
	timing.jumpWidth = (jw + 1) * (brp + 1);
}

void ChangeLocalCanTiming(mcan_module *const module_inst, const CanTiming& timing) noexcept
{
	// Sort out the bit timing
	uint32_t period = timing.period;
	uint32_t tseg1 = timing.tseg1;
	uint32_t jumpWidth = timing.jumpWidth;
	uint32_t prescaler = 1;						// 48MHz main clock
	uint32_t tseg2;

	for (;;)
	{
		tseg2 = period - tseg1 - 1;
		if (tseg1 <= 32 && tseg2 <= 16 && jumpWidth <= 16)
		{
			break;
		}
		prescaler <<= 1;
		period >>= 1;
		tseg1 >>= 1;
		jumpWidth >>= 1;
	}

	//TODO stop transmissions in an orderly fashion, or postpone initialising CAN until we have the timing data
	module_inst->hw->MCAN_CCCR |= MCAN_CCCR_CCE | MCAN_CCCR_INIT;
	module_inst->hw->MCAN_NBTP =  ((tseg1 - 1) << MCAN_NBTP_NTSEG1_Pos)
							| ((tseg2 - 1) << MCAN_NBTP_NTSEG2_Pos)
							| ((jumpWidth - 1) << MCAN_NBTP_NSJW_Pos)
							| ((prescaler - 1) << MCAN_NBTP_NBRP_Pos);
	module_inst->hw->MCAN_CCCR &= ~MCAN_CCCR_CCE;
}


#endif	// SUPPORT_CAN_EXPANSION

// End

