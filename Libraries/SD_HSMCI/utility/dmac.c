/**
 * \file
 *
 * \brief DMA Controller (DMAC) driver for SAM.
 *
 * Copyright (c) 2012 - 2013 Atmel Corporation. All rights reserved.
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

#include  "dmac.h"

/**
 * \brief Initialize DMA controller and disable it.
 *
 * \param p_dmac  Pointer to a DMAC peripheral instance.
 */
void dmac_init(Dmac *p_dmac)
{
	dmac_disable(p_dmac);
}

/**
 * \brief Set DMA priority mode.
 *
 * \param p_dmac Pointer to a DMAC peripheral instance.
 * \param mode   Priority mode.
 */
void dmac_set_priority_mode(Dmac *p_dmac, dmac_priority_mode_t mode)
{
	p_dmac->DMAC_GCFG = (p_dmac->DMAC_GCFG & (~DMAC_GCFG_ARB_CFG)) | mode;
}

/**
 * \brief Enable DMA Controller.
 *
 * \param p_dmac  Pointer to a DMAC peripheral instance.
 */
void dmac_enable(Dmac *p_dmac)
{
	p_dmac->DMAC_EN = DMAC_EN_ENABLE;
}

/**
 * \brief Disable DMA Controller.
 *
 * \param p_dmac Pointer to a DMAC peripheral instance.
 */
void dmac_disable(Dmac *p_dmac)
{
	p_dmac->DMAC_EN &= (~DMAC_EN_ENABLE);
}

/**
 * \brief Enable DMA interrupt.
 *
 * \param p_dmac  Pointer to a DMAC peripheral instance.
 * \param ul_mask Interrupt to be enabled.
 */
void dmac_enable_interrupt(Dmac *p_dmac, uint32_t ul_mask)
{
	p_dmac->DMAC_EBCIER = ul_mask;
}

/**
 * \brief Disable DMA interrupt.
 *
 * \param p_dmac  Pointer to a DMAC peripheral instance.
 * \param ul_mask Interrupt to be disabled.
 */
void dmac_disable_interrupt(Dmac *p_dmac, uint32_t ul_mask)
{
	p_dmac->DMAC_EBCIDR = ul_mask;
}

/**
 * \brief Get DMAC Interrupt Mask.
 *
 * \param p_dmac Pointer to a DMAC peripheral instance.
 *
 * \return DMAC Interrupt mask.
 */
uint32_t dmac_get_interrupt_mask(Dmac *p_dmac)
{
	return (p_dmac->DMAC_EBCIMR);
}

/**
 * \brief Get DMAC transfer status.
 *
 * \param p_dmac Pointer to a DMAC peripheral instance.
 *
 * \return DMAC transfer status.
 */
uint32_t dmac_get_status(Dmac *p_dmac)
{
	return (p_dmac->DMAC_EBCISR);
}

/**
 * \brief Enable the relevant channel.
 *
 * \param p_dmac Pointer to a DMAC peripheral instance.
 * \param ul_num Channel number.
 */
void dmac_channel_enable(Dmac *p_dmac, uint32_t ul_num)
{
	p_dmac->DMAC_CHER = DMAC_CHER_ENA0 << ul_num;
}

/**
 * \brief Disable the relevant channel.
 *
 * \param p_dmac Pointer to a DMAC peripheral instance.
 * \param ul_num Channel number.
 */
void dmac_channel_disable(Dmac *p_dmac, uint32_t ul_num)
{
	p_dmac->DMAC_CHDR = DMAC_CHDR_DIS0 << ul_num;
}

/**
 * \brief Check if the relevant channel is enabled.
 *
 * \param p_dmac Pointer to a DMAC peripheral instance.
 * \param ul_num Channel number.
 *
 * \retval 0: disabled.
 * \retval 1: enabled.
 */
uint32_t dmac_channel_is_enable(Dmac *p_dmac, uint32_t ul_num)
{
	if (p_dmac->DMAC_CHSR & (DMAC_CHSR_ENA0 << ul_num)) {
		return 1;
	} else {
		return 0;
	}
}

/**
 * \brief Suspend the specified channel and its current context.
 *
 * \param p_dmac Pointer to a DMAC peripheral instance.
 * \param ul_num Channel number.
 */
void dmac_channel_suspend(Dmac *p_dmac, uint32_t ul_num)
{
	p_dmac->DMAC_CHER = DMAC_CHER_SUSP0 << ul_num;
}

/**
 * \brief Resume the specified channel transfer (restoring its context).
 *
 * \param p_dmac Pointer to a DMAC peripheral instance.
 * \param ul_num Channel number.
 */
void dmac_channel_resume(Dmac *p_dmac, uint32_t ul_num)
{
	p_dmac->DMAC_CHDR = DMAC_CHDR_RES0 << ul_num;
}

/**
 * \brief Resume the specified channel from an automatic stall state.
 *
 * \param p_dmac Pointer to a DMAC peripheral instance.
 * \param ul_num Channel number.
 */
void dmac_channel_keep(Dmac *p_dmac, uint32_t ul_num)
{
	p_dmac->DMAC_CHER = DMAC_CHER_KEEP0 << ul_num;
}

/**
 * \brief Get DMAC channel handler status.
 *
 * \param p_dmac Pointer to a DMAC peripheral instance.
 *
 * \return DMAC channel handler status register.
 */
uint32_t dmac_channel_get_status(Dmac *p_dmac)
{
	return (p_dmac->DMAC_CHSR);
}

/**
 * \brief Set DMAC source address of the DMAC channel.
 *
 * \param p_dmac      Pointer to a DMAC peripheral instance.
 * \param ul_num      Channel number.
 * \param ul_addr     Source address.
 *
 * \note This register must be aligned with the source transfer width.
 */
void dmac_channel_set_source_addr(Dmac *p_dmac,
		uint32_t ul_num, uint32_t ul_addr)
{
	p_dmac->DMAC_CH_NUM[ul_num].DMAC_SADDR = ul_addr;
}

/**
 * \brief Set DMAC destination address of the DMAC channel.
 *
 * \param p_dmac      Pointer to a DMAC peripheral instance.
 * \param ul_num      Channel number.
 * \param ul_addr     Destination address.
 *
 * \note This register must be aligned with the source transfer width.
 */
void dmac_channel_set_destination_addr(Dmac *p_dmac, uint32_t ul_num,
		uint32_t ul_addr)
{
	p_dmac->DMAC_CH_NUM[ul_num].DMAC_DADDR = ul_addr;
}

/**
 * \brief Set DMAC descriptor address of the DMAC channel.
 *
 * \param p_dmac      Pointer to a DMAC peripheral instance.
 * \param ul_num      Channel number.
 * \param ul_desc     Descriptor address.
 */
void dmac_channel_set_descriptor_addr(Dmac *p_dmac,
		uint32_t ul_num, uint32_t ul_desc)
{
	p_dmac->DMAC_CH_NUM[ul_num].DMAC_DSCR = ul_desc;
}

/**
 * \brief Set DMAC control A of the DMAC channel.
 *
 * \param p_dmac      Pointer to a DMAC peripheral instance.
 * \param ul_num      Channel number.
 * \param ul_ctrlA    Configuration of control A register.
 */
void dmac_channel_set_ctrlA(Dmac *p_dmac, uint32_t ul_num, uint32_t ul_ctrlA)
{
	p_dmac->DMAC_CH_NUM[ul_num].DMAC_CTRLA = ul_ctrlA;
}

/**
 * \brief Set DMAC control B of the DMAC channel.
 *
 * \param p_dmac      Pointer to a DMAC peripheral instance.
 * \param ul_num      Channel number.
 * \param ul_ctrlB    Configuration of control B register.
 */
void dmac_channel_set_ctrlB(Dmac *p_dmac, uint32_t ul_num, uint32_t ul_ctrlB)
{
	p_dmac->DMAC_CH_NUM[ul_num].DMAC_CTRLB = ul_ctrlB;
}

/**
 * \brief Set DMAC configuration register of the DMAC channel.
 *
 * \param p_dmac      Pointer to a DMAC peripheral instance.
 * \param ul_num      Channel number.
 * \param ul_cfg      Configuration of CFG register.
 */
void dmac_channel_set_configuration(Dmac *p_dmac, uint32_t ul_num,
		uint32_t ul_cfg)
{
	p_dmac->DMAC_CH_NUM[ul_num].DMAC_CFG = ul_cfg;
}

/**
 * \brief Initialize DMAC channel of single buffer transfer.
 *
 * \param p_dmac      Pointer to a DMAC peripheral instance.
 * \param ul_num      Channel number.
 * \param p_desc      Pointer to a transfer descriptor.
 */
void dmac_channel_single_buf_transfer_init(Dmac *p_dmac,
		uint32_t ul_num, dma_transfer_descriptor_t *p_desc)
{
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
 * \brief Initialize DMAC channel of multiple buffer transfer.
 *
 * \param p_dmac      Pointer to a DMAC peripheral instance.
 * \param ul_num      Channel number.
 * \param p_desc      Pointer to a transfer descriptor.
 */
void dmac_channel_multi_buf_transfer_init(Dmac *p_dmac,
		uint32_t ul_num, dma_transfer_descriptor_t *p_desc)
{
	/* Clear any pending interrupts */
	p_dmac->DMAC_EBCISR;

	dmac_channel_set_descriptor_addr(p_dmac, ul_num, (uint32_t)p_desc);
	dmac_channel_set_ctrlB(p_dmac, ul_num, 0);
}

/**
 * \brief Stop DMA transfer of the DMAC channel.
 *
 * \note Under normal operation, the hardware disables a channel on transfer
 * completion by clearing the DMAC_CHSR.ENAx register bit.
 * The recommended way for software to disable a channel without losing data
 * is to use the SUSPx bit in conjunction with the EMPTx bit in the Channel
 * Handler Status Register.
 *
 * \param p_dmac      Pointer to a DMAC peripheral instance.
 * \param ul_num      Channel number.
 */
void dmac_channel_stop_transfer(Dmac *p_dmac, uint32_t ul_num)
{
	uint32_t status;

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
 * \brief Check if the transfer of the DMAC channel is done.
 * This function is used for polling mode.
 *
 * \param p_dmac      Pointer to a DMAC peripheral instance.
 * \param ul_num      Channel number.
 *
 * \retval 0 - Transferring.
 * \retval 1 - Transfer is done.
 */
uint32_t dmac_channel_is_transfer_done(Dmac *p_dmac, uint32_t ul_num)
{
	uint32_t status;

	status = dmac_channel_get_status(p_dmac);
	if (status & (DMAC_CHSR_ENA0 << ul_num)) {
		return 0;
	} else {
		return 1;
	}
}

/**
 * \brief DMAC software single request.
 *
 * \param p_dmac      Pointer to a DMAC peripheral instance.
 * \param ul_num      Channel number.
 * \param ul_src_req  Request a source transfer.
 * \param ul_dst_req  Request a destination transfer.
 */
void dmac_soft_single_transfer_request(Dmac *p_dmac,
		uint32_t ul_num, uint32_t ul_src_req, uint32_t ul_dst_req)
{
	uint32_t req;

	req = ul_src_req ? DMAC_SREQ_SSREQ0 : 0;
	req |= ul_dst_req ? DMAC_SREQ_DSREQ0 : 0;
	p_dmac->DMAC_SREQ |= (req << ul_num);
}

/**
 * \brief DMAC software chunk request.
 *
 * \param p_dmac      Pointer to a DMAC peripheral instance.
 * \param ul_num      Channel number.
 * \param ul_src_req  Request a source transfer.
 * \param ul_dst_req  Request a destination transfer.
 */
void dmac_soft_chunk_transfer_request(Dmac *p_dmac,
		uint32_t ul_num, uint32_t ul_src_req, uint32_t ul_dst_req)
{
	uint32_t req;

	req = ul_src_req ? DMAC_CREQ_SCREQ0 : 0;
	req |= ul_dst_req ? DMAC_CREQ_DCREQ0 : 0;
	p_dmac->DMAC_SREQ |= (req << ul_num);
}

/**
 * \brief Set DMAC last transfer flag.
 *
 * \param p_dmac       Pointer to a DMAC peripheral instance.
 * \param ul_num       Channel number.
 * \param ul_src_flag  Last source transfer flag.
 * \param ul_dst_flag  Last destination transfer flag.
 */
void dmac_soft_set_last_transfer_flag(Dmac *p_dmac,
		uint32_t ul_num, uint32_t ul_src_flag, uint32_t ul_dst_flag)
{
	uint32_t flag;

	flag = ul_src_flag ? DMAC_LAST_SLAST0 : 0;
	flag |= ul_dst_flag ? DMAC_LAST_DLAST0 : 0;
	p_dmac->DMAC_SREQ |= (flag << ul_num);
}

#if (SAM3XA_SERIES || SAM4E)

/** DMAC write protect key */
#define DMAC_WPKEY 0x50494Fu

/**
 * \brief Enable/Disable write protect of DMAC registers.
 *
 * \param p_dmac    Pointer to a DMAC peripheral instance.
 * \param ul_enable 1 to enable, 0 to disable.
 */
void dmac_set_writeprotect(Dmac *p_dmac, uint32_t ul_enable)
{
	if (ul_enable) {
		p_dmac->DMAC_WPMR = DMAC_WPMR_WPKEY(DMAC_WPKEY) |
				DMAC_WPMR_WPEN;
	} else {
		p_dmac->DMAC_WPMR = DMAC_WPMR_WPKEY(DMAC_WPKEY);
	}
}

/**
 * \brief Get write protect status.
 *
 * \param p_dmac    Pointer to a DMAC peripheral instance.
 *
 * \return Write protect status.
 */
uint32_t dmac_get_writeprotect_status(Dmac *p_dmac)
{
	return (p_dmac->DMAC_WPSR);
}

#endif
