/**
 * \file
 *
 * \brief SAM4E DMA Controller (DMAC) driver.
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

#include  "dmac.h"

#ifndef DMAC_WPMR_WPKEY_PASSWD
#  define DMAC_WPMR_WPKEY_PASSWD DMAC_WPMR_WPKEY(0x444D41u)
#endif

/**
 * \brief Initialize the DMA controller and disable it.
 *
 * \param[in,out] p_dmac Module hardware register base address pointer
 */
void dmac_init(
		Dmac *p_dmac)
{
	dmac_disable(p_dmac);
}

/**
 * \brief Set the DMA priority mode.
 *
 * \param[in,out] p_dmac Module hardware register base address pointer
 * \param[in] mode       \ref dmac_priority_mode_t "Priority mode"
 */
void dmac_set_priority_mode(
		Dmac *p_dmac,
		dmac_priority_mode_t mode)
{
	/* Validate parameters. */
	Assert(p_dmac);
	
	p_dmac->DMAC_GCFG = (p_dmac->DMAC_GCFG & (~DMAC_GCFG_ARB_CFG)) | mode;
}

/**
 * \brief Enable the DMA Controller.
 *
 * \param[out] p_dmac Module hardware register base address pointer
 */
void dmac_enable(
		Dmac *p_dmac)
{
	/* Validate parameters. */
	Assert(p_dmac);
	
	p_dmac->DMAC_EN = DMAC_EN_ENABLE;
}

/**
 * \brief Disable the DMA Controller.
 *
 * \param[in,out] p_dmac Module hardware register base address pointer
 */
void dmac_disable(
		Dmac *p_dmac)
{
	/* Validate parameters. */
	Assert(p_dmac);
	
	p_dmac->DMAC_EN &= (~DMAC_EN_ENABLE);
}

/**
 * \brief Enable DMAC interrupts.
 *
 * \param[out] p_dmac Module hardware register base address pointer
 * \param[in] ul_mask A bitmask of interrupts to be enabled
 *
 * Where input parameter <i>ul_mask</i> is a bitmask containing one or more
 * of the following:
 * <table>
 * <tr>
 *    <th>Parameter Value</th>
 *    <th>Description</th>
 * </tr>
 *    <tr><td>DMAC_EBCIER_BTC0</td><td>Channel 0 Buffer Transfer Completed</td></tr>
 *    <tr><td>DMAC_EBCIER_BTC1</td><td>Channel 1 Buffer Transfer Completed</td></tr>
 *    <tr><td>DMAC_EBCIER_BTC2</td><td>Channel 2 Buffer Transfer Completed</td></tr>
 *    <tr><td>DMAC_EBCIER_BTC3</td><td>Channel 3 Buffer Transfer Completed</td></tr>
 *    <tr><td>DMAC_EBCIER_CBTC0</td><td>Channel 0 Chained Buffer Transfer Completed</td></tr>
 *    <tr><td>DMAC_EBCIER_CBTC1</td><td>Channel 1 Chained Buffer Transfer Completed</td></tr>
 *    <tr><td>DMAC_EBCIER_CBTC2</td><td>Channel 2 Chained Buffer Transfer Completed</td></tr>
 *    <tr><td>DMAC_EBCIER_CBTC3</td><td>Channel 3 Chained Buffer Transfer Completed</td></tr>
 *    <tr><td>DMAC_EBCIER_ERR0</td><td>Channel 0 Access Error</td></tr>
 *    <tr><td>DMAC_EBCIER_ERR1</td><td>Channel 1 Access Error</td></tr>
 *    <tr><td>DMAC_EBCIER_ERR2</td><td>Channel 2 Access Error</td></tr>
 *    <tr><td>DMAC_EBCIER_ERR3</td><td>Channel 3 Access Error</td></tr>
 * </table>
 */
void dmac_enable_interrupt(
		Dmac *p_dmac,
		uint32_t ul_mask)
{
	/* Validate parameters. */
	Assert(p_dmac);
	
	p_dmac->DMAC_EBCIER = ul_mask;
}

/**
 * \brief Disable DMAC interrupts.
 *
 * \param[out] p_dmac Module hardware register base address pointer
 * \param[in] ul_mask A bitmask of interrupts to be disabled
 *
 * Where input parameter <i>ul_mask</i> is a bitmask containing one or more
 * of the following:
 * <table>
 * <tr>
 *    <th>Parameter Value</th>
 *    <th>Description</th>
 * </tr>
 *    <tr><td>DMAC_EBCIDR_BTC0</td><td>Channel 0 Buffer Transfer Completed</td></tr>
 *    <tr><td>DMAC_EBCIDR_BTC1</td><td>Channel 1 Buffer Transfer Completed</td></tr>
 *    <tr><td>DMAC_EBCIDR_BTC2</td><td>Channel 2 Buffer Transfer Completed</td></tr>
 *    <tr><td>DMAC_EBCIDR_BTC3</td><td>Channel 3 Buffer Transfer Completed</td></tr>
 *    <tr><td>DMAC_EBCIDR_CBTC0</td><td>Channel 0 Chained Buffer Transfer Completed</td></tr>
 *    <tr><td>DMAC_EBCIDR_CBTC1</td><td>Channel 1 Chained Buffer Transfer Completed</td></tr>
 *    <tr><td>DMAC_EBCIDR_CBTC2</td><td>Channel 2 Chained Buffer Transfer Completed</td></tr>
 *    <tr><td>DMAC_EBCIDR_CBTC3</td><td>Channel 3 Chained Buffer Transfer Completed</td></tr>
 *    <tr><td>DMAC_EBCIDR_ERR0</td><td>Channel 0 Access Error</td></tr>
 *    <tr><td>DMAC_EBCIDR_ERR1</td><td>Channel 1 Access Error</td></tr>
 *    <tr><td>DMAC_EBCIDR_ERR2</td><td>Channel 2 Access Error</td></tr>
 *    <tr><td>DMAC_EBCIDR_ERR3</td><td>Channel 3 Access Error</td></tr>
 * </table>
 */
void dmac_disable_interrupt(
		Dmac *p_dmac,
		uint32_t ul_mask)
{
	/* Validate parameters. */
	Assert(p_dmac);
	
	p_dmac->DMAC_EBCIDR = ul_mask;
}

/**
 * \brief Get the DMAC Interrupt Mask.
 *
 * \param[in] p_dmac Module hardware register base address pointer
 *
 * \return DMAC Interrupt mask.
 */
uint32_t dmac_get_interrupt_mask(
		Dmac *p_dmac)
{
	/* Validate parameters. */
	Assert(p_dmac);
	
	return p_dmac->DMAC_EBCIMR;
}

/**
 * \brief Get the DMAC transfer status.
 *
 * \param[in] p_dmac Module hardware register base address pointer
 *
 * \return DMAC transfer status. Refer to section called "DMAC Error,
 * Buffer Transfer and Chained Buffer Transfer Status Register" in the
 * device-specific datasheet for more information.
 */
uint32_t dmac_get_status(
		Dmac *p_dmac)
{
	/* Validate parameters. */
	Assert(p_dmac);
	
	return p_dmac->DMAC_EBCISR;
}

/**
 * \brief Enable the specified DMA Channel.
 *
 * \param[out] p_dmac Module hardware register base address pointer
 * \param[in] ul_num  DMA Channel number (range 0 to 3)
 */
void dmac_channel_enable(
		Dmac *p_dmac,
		uint32_t ul_num)
{
	/* Validate parameters. */
	Assert(p_dmac);
	Assert(ul_num<=3);
	
	p_dmac->DMAC_CHER = DMAC_CHER_ENA0 << ul_num;
}

/**
 * \brief Disable the specified DMA Channel.
 *
 * \param[out] p_dmac Module hardware register base address pointer
 * \param[in] ul_num  DMA Channel number (range 0 to 3)
 */
void dmac_channel_disable(
		Dmac *p_dmac,
		uint32_t ul_num)
{
	/* Validate parameters. */
	Assert(p_dmac);
	Assert(ul_num<=3);
	
	p_dmac->DMAC_CHDR = DMAC_CHDR_DIS0 << ul_num;
}

/**
 * \brief Check if the specified DMA Channel is enabled.
 *
 * \param[in] p_dmac Module hardware register base address pointer
 * \param[in] ul_num DMA Channel number (range 0 to 3)
 *
 * \return The DMA Channel's enable/disable status.
 * \retval 0 DMA Channel is disabled
 * \retval 1 DMA Channel is enabled
 */
uint32_t dmac_channel_is_enable(
		Dmac *p_dmac,
		uint32_t ul_num)
{
	/* Validate parameters. */
	Assert(p_dmac);
	Assert(ul_num<=3);
	
	if (p_dmac->DMAC_CHSR & (DMAC_CHSR_ENA0 << ul_num)) {
		return 1;
	} else {
		return 0;
	}
}

/**
 * \brief Suspend the specified DMA Channel and its current context.
 *
 * \param[out] p_dmac Module hardware register base address pointer
 * \param[in] ul_num  DMA Channel number (range 0 to 3)
 */
void dmac_channel_suspend(
		Dmac *p_dmac,
		uint32_t ul_num)
{
	/* Validate parameters. */
	Assert(p_dmac);
	Assert(ul_num<=3);
	
	p_dmac->DMAC_CHER = DMAC_CHER_SUSP0 << ul_num;
}

/**
 * \brief Resume the specified DMA Channel transfer (restoring its context).
 *
 * \param[out] p_dmac Module hardware register base address pointer
 * \param[in] ul_num  DMA Channel number (range 0 to 3)
 */
void dmac_channel_resume(
		Dmac *p_dmac, 
		uint32_t ul_num)
{
	/* Validate parameters. */
	Assert(p_dmac);
	Assert(ul_num<=3);
	
	p_dmac->DMAC_CHDR = DMAC_CHDR_RES0 << ul_num;
}

/**
 * \brief Resume the specified DMA Channel from an automatic stall state.
 *
 * \param[out] p_dmac Module hardware register base address pointer
 * \param[in] ul_num  DMA Channel number (range 0 to 3)
 */
void dmac_channel_keep(
		Dmac *p_dmac,
		uint32_t ul_num)
{
	/* Validate parameters. */
	Assert(p_dmac);
	Assert(ul_num<=3);
	
	p_dmac->DMAC_CHER = DMAC_CHER_KEEP0 << ul_num;
}

/**
 * \brief Get the DMAC Channel handler status.
 *
 * \param[in] p_dmac Module hardware register base address pointer
 *
 * \return DMAC Channel handler status register. Refer to the section called
 * "DMAC Channel Handler Status Register" in the device-specific datasheet for more
 * information.
 */
uint32_t dmac_channel_get_status(
		Dmac *p_dmac)
{
	/* Validate parameters. */
	Assert(p_dmac);
	
	return p_dmac->DMAC_CHSR;
}

/**
 * \brief Set the DMA source address of the specified DMA Channel.
 *
 * \param[out] p_dmac Module hardware register base address pointer
 * \param[in] ul_num  DMA Channel number (range 0 to 3)
 * \param[in] ul_addr Source address
 *
 * \note This register must be aligned with the source transfer width.
 */
void dmac_channel_set_source_addr(
		Dmac *p_dmac,
		uint32_t ul_num,
		uint32_t ul_addr)
{
	/* Validate parameters. */
	Assert(p_dmac);
	Assert(ul_num<=3);
	
	p_dmac->DMAC_CH_NUM[ul_num].DMAC_SADDR = ul_addr;
}

/**
 * \brief Set the DMA destination address of the specified DMA Channel.
 *
 * \param[out] p_dmac Module hardware register base address pointer
 * \param[in] ul_num  DMA Channel number (range 0 to 3)
 * \param[in] ul_addr Destination address
 *
 * \note This register must be aligned with the source transfer width.
 */
void dmac_channel_set_destination_addr(
		Dmac *p_dmac, 
		uint32_t ul_num,
		uint32_t ul_addr)
{
	/* Validate parameters. */
	Assert(p_dmac);
	Assert(ul_num<=3);
	
	p_dmac->DMAC_CH_NUM[ul_num].DMAC_DADDR = ul_addr;
}

/**
 * \brief Set the DMA descriptor address of the specified DMA Channel.
 *
 * \param[out] p_dmac Module hardware register base address pointer
 * \param[in] ul_num  DMA Channel number (range 0 to 3)
 * \param[in] ul_desc Descriptor address
 */
void dmac_channel_set_descriptor_addr(
		Dmac *p_dmac,
		uint32_t ul_num,
		uint32_t ul_desc)
{
	/* Validate parameters. */
	Assert(p_dmac);
	Assert(ul_num<=3);
	
	p_dmac->DMAC_CH_NUM[ul_num].DMAC_DSCR = ul_desc;
}

/**
 * \brief Set the DMA control A of the specified DMA Channel.
 *
 * \param[out] p_dmac  Module hardware register base address pointer
 * \param[in] ul_num   DMA Channel number (range 0 to 3)
 * \param[in] ul_ctrlA Configuration of control A register
 */
void dmac_channel_set_ctrlA(
		Dmac *p_dmac, 
		uint32_t ul_num, 
		uint32_t ul_ctrlA)
{
	/* Validate parameters. */
	Assert(p_dmac);
	Assert(ul_num<=3);
	
	p_dmac->DMAC_CH_NUM[ul_num].DMAC_CTRLA = ul_ctrlA;
}

/**
 * \brief Set the DMA control B of the specified DMA Channel.
 *
 * \param[out] p_dmac  Module hardware register base address pointer
 * \param[in] ul_num   DMA Channel number (range 0 to 3)
 * \param[in] ul_ctrlB Configuration of control B register
 */
void dmac_channel_set_ctrlB(
		Dmac *p_dmac,
		uint32_t ul_num,
		uint32_t ul_ctrlB)
{
	/* Validate parameters. */
	Assert(p_dmac);
	Assert(ul_num<=3);
	
	p_dmac->DMAC_CH_NUM[ul_num].DMAC_CTRLB = ul_ctrlB;
}

/**
 * \brief Set the DMAC configuration register of the specified DMA Channel.
 *
 * \param[out] p_dmac Module hardware register base address pointer
 * \param[in] ul_num  DMA Channel number (range 0 to 3)
 * \param[in] ul_cfg  Configuration of CFG register
 */
void dmac_channel_set_configuration(
		Dmac *p_dmac,
		uint32_t ul_num,
		uint32_t ul_cfg)
{
	/* Validate parameters. */
	Assert(p_dmac);
	Assert(ul_num<=3);
	
	p_dmac->DMAC_CH_NUM[ul_num].DMAC_CFG = ul_cfg;
}

/**
 * \brief Initialize the DMA Channel for a single buffer transfer.
 *
 * \param[in,out] p_dmac Module hardware register base address pointer
 * \param[in] ul_num     DMA Channel number (range 0 to 3)
 * \param[in] p_desc     Pointer to a \ref dma_transfer_descriptor_t "transfer descriptor"
 */
void dmac_channel_single_buf_transfer_init(
		Dmac *p_dmac,
		uint32_t ul_num,
		dma_transfer_descriptor_t *p_desc)
{
	/* Validate parameters. */
	Assert(p_dmac);
	Assert(ul_num<=3);
	Assert(p_desc);
	
	/* Clear any pending interrupts */
	p_dmac->DMAC_EBCISR;

	dmac_channel_set_source_addr(p_dmac, ul_num, p_desc->ul_source_addr);
	dmac_channel_set_destination_addr(p_dmac, ul_num,
			p_desc->ul_destination_addr);
	dmac_channel_set_descriptor_addr(p_dmac, ul_num, 0);
	dmac_channel_set_ctrlA(p_dmac, ul_num, p_desc->ul_ctrlA);
	dmac_channel_set_ctrlB(p_dmac, ul_num, p_desc->ul_ctrlB);
}

/**
 * \brief Initialize the DMA Channel for a multiple buffer transfer.
 *
 * \param[in,out] p_dmac Module hardware register base address pointer
 * \param[in] ul_num     DMA Channel number (range 0 to 3)
 * \param[in] p_desc     Pointer to a \ref dma_transfer_descriptor_t "transfer descriptor"
 */
void dmac_channel_multi_buf_transfer_init(
		Dmac *p_dmac,
		uint32_t ul_num,
		dma_transfer_descriptor_t *p_desc)
{
	/* Validate parameters. */
	Assert(p_dmac);
	Assert(ul_num<=3);
	Assert(p_desc);
	
	/* Clear any pending interrupts */
	p_dmac->DMAC_EBCISR;

	dmac_channel_set_descriptor_addr(p_dmac, ul_num, (uint32_t)p_desc);
	dmac_channel_set_ctrlB(p_dmac, ul_num, 0);
}

/**
 * \brief Stop a DMA transfer occurring on the specified DMA Channel.
 *
 * \note Under normal operation, the hardware disables a channel on transfer
 * completion by clearing the DMAC_CHSR.ENAx register bit.
 * The recommended way for software to disable a channel without losing data
 * is to use the SUSPx bit in conjunction with the EMPTx bit in the Channel
 * Handler Status Register.
 *
 * \param[in,out] p_dmac Module hardware register base address pointer
 * \param[in] ul_num     DMA Channel number (range 0 to 3)
 */
void dmac_channel_stop_transfer(
		Dmac *p_dmac,
		uint32_t ul_num)
{
	uint32_t status;

	/* Validate parameters. */
	Assert(p_dmac);
	Assert(ul_num<=3);
	
	status = dmac_channel_get_status(p_dmac);
	if (!(status & (DMAC_CHSR_ENA0 << ul_num))) {
		/* The channel is already stopped. */
		return;
	} else {
		/* Suspend channel and the channel FIFO receives no new data. */
		dmac_channel_suspend(p_dmac, ul_num);

		/* Check if the channel FIFO is empty. */
		do {
			status = dmac_channel_get_status(p_dmac);
			if (status & (DMAC_CHSR_EMPT0 << ul_num)) {
				break;
			}
		} while (1);

		/* Disable the channel. */
		dmac_channel_disable(p_dmac, ul_num);
		/* Clear suspend flag. */
		dmac_channel_resume(p_dmac, ul_num);
	}
}

/**
 * \brief Check if the data transfer occurring on the specified DMA Channel is complete.
 * \note This function is used in polling mode.
 *
 * \param[in] p_dmac Module hardware register base address pointer
 * \param[in] ul_num DMA Channel number (range 0 to 3)
 *
 * \return The data transfer status.
 * \retval 0 Data is transferring
 * \retval 1 Data transfer complete
 */
uint32_t dmac_channel_is_transfer_done(
		Dmac *p_dmac,
		uint32_t ul_num)
{
	uint32_t status;

	/* Validate parameters. */
	Assert(p_dmac);
	Assert(ul_num<=3);
	
	status = dmac_channel_get_status(p_dmac);
	if (status & (DMAC_CHSR_ENA0 << ul_num)) {
		return 0;
	} else {
		return 1;
	}
}

/**
 * \brief DMA Channel software single request.
 *
 * \param[in,out] p_dmac Module hardware register base address pointer
 * \param[in] ul_num     DMA Channel number (range 0 to 3)
 * \param[in] ul_src_req Request a source transfer
 * \param[in] ul_dst_req Request a destination transfer
 */
void dmac_soft_single_transfer_request(
		Dmac *p_dmac,
		uint32_t ul_num,
		uint32_t ul_src_req,
		uint32_t ul_dst_req)
{
	uint32_t req;

	/* Validate parameters. */
	Assert(p_dmac);
	Assert(ul_num<=3);
	
	req = ul_src_req ? DMAC_SREQ_SSREQ0 : 0;
	req |= ul_dst_req ? DMAC_SREQ_DSREQ0 : 0;
	p_dmac->DMAC_SREQ |= (req << ul_num);
}

/**
 * \brief DMA Channel software chunk request.
 *
 * \param[in,out] p_dmac Module hardware register base address pointer
 * \param[in] ul_num     DMA Channel number (range 0 to 3)
 * \param[in] ul_src_req Request a source transfer
 * \param[in] ul_dst_req Request a destination transfer
 */
void dmac_soft_chunk_transfer_request(
		Dmac *p_dmac,
		uint32_t ul_num,
		uint32_t ul_src_req,
		uint32_t ul_dst_req)
{
	uint32_t req;

	/* Validate parameters. */
	Assert(p_dmac);
	Assert(ul_num<=3);
	
	req = ul_src_req ? DMAC_CREQ_SCREQ0 : 0;
	req |= ul_dst_req ? DMAC_CREQ_DCREQ0 : 0;
	p_dmac->DMAC_SREQ |= (req << ul_num);
}

/**
 * \brief Set the DMA Channel's last transfer flag.
 *
 * \param[in,out] p_dmac  Module hardware register base address pointer
 * \param[in] ul_num      Channel number
 * \param[in] ul_src_flag Last source transfer flag
 * \param[in] ul_dst_flag Last destination transfer flag
 */
void dmac_soft_set_last_transfer_flag(
		Dmac *p_dmac,
		uint32_t ul_num,
		uint32_t ul_src_flag,
		uint32_t ul_dst_flag)
{
	uint32_t flag;

	/* Validate parameters. */
	Assert(p_dmac);
	Assert(ul_num<=3);
	
	flag = ul_src_flag ? DMAC_LAST_SLAST0 : 0;
	flag |= ul_dst_flag ? DMAC_LAST_DLAST0 : 0;
	p_dmac->DMAC_SREQ |= (flag << ul_num);
}

#if (SAM3XA || SAM4E) || defined(__DOXYGEN__)

#if !defined(__DOXYGEN__)
/** DMAC write protect key */
#define DMAC_WPKEY 0x50494Fu
#endif

/**
 * \brief Enable/Disable the write protect of DMAC registers.
 *
 * \param[out] p_dmac   Module hardware register base address pointer
 * \param[in] ul_enable 1 to enable, 0 to disable
 */
void dmac_set_writeprotect(
		Dmac *p_dmac,
		uint32_t ul_enable)
{
	/* Validate parameters. */
	Assert(p_dmac);
	
	if (ul_enable) {
		p_dmac->DMAC_WPMR = DMAC_WPMR_WPKEY_PASSWD | DMAC_WPMR_WPEN;
	} else {
		p_dmac->DMAC_WPMR = DMAC_WPMR_WPKEY_PASSWD;
	}
}

/**
 * \brief Get the DMAC register's write protect status.
 *
 * \param[in] p_dmac Module hardware register base address pointer
 *
 * \return Write protect status.
 */
uint32_t dmac_get_writeprotect_status(
		Dmac *p_dmac)
{
	/* Validate parameters. */
	Assert(p_dmac);
	
	return p_dmac->DMAC_WPSR;
}
#endif /* (SAM3XA || SAM4E) || defined(__DOXYGEN__) */
