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

#ifndef DMAC_H_INCLUDED
#define DMAC_H_INCLUDED

/**
 * \defgroup asfdoc_sam_drivers_dmac_group SAM3A/3U/3X/4E DMA Controller (DMAC) Driver
 *
 * This driver for Atmel&reg; | SMART ARM&reg;-based microcontrollers 
 * provides an interface for the configuration and management of the 
 * device's Direct Memory Access DMA Controller (DMAC) functionality.
 *
 * The DMAC is an AHB-central DMA controller core that
 * transfers data from a source peripheral to a destination peripheral
 * over one or more AMBA buses. This is a driver for the configuration,
 * enabling, disabling, and use of the DMAC peripheral.
 *
 * Devices from the following series can use this module:
 * - Atmel | SMART SAM3A
 * - Atmel | SMART SAM3U
 * - Atmel | SMART SAM3X
 * - Atmel | SMART SAM4E
 *
 * The outline of this documentation is as follows:
 *  - \ref asfdoc_sam_drivers_dmac_prerequisites
 *  - \ref asfdoc_sam_drivers_dmac_module_overview
 *  - \ref asfdoc_sam_drivers_dmac_special_considerations
 *  - \ref asfdoc_sam_drivers_dmac_extra_info
 *  - \ref asfdoc_sam_drivers_dmac_examples
 *  - \ref asfdoc_sam_drivers_dmac_api_overview
 *
 *
 * \section asfdoc_sam_drivers_dmac_prerequisites Prerequisites
 *
 * There are no prerequisites for this module.
 *
 *
 * \section asfdoc_sam_drivers_dmac_module_overview Module Overview
 * The DMA Controller (DMAC) is an AHB-central DMA controller core that transfers
 * data from a source peripheral to a destination peripheral over one or more AMBA
 * buses. One channel is required for each source/destination pair. In the
 * most basic configuration, the DMAC has one master interface and one channel. The
 * master interface reads the data from a source and writes it to a destination.
 * Two AMBA transfers are required for each DMAC data transfer. This is also known
 * as a dual-access transfer.
 *
 *
 * \section asfdoc_sam_drivers_dmac_special_considerations Special Considerations
 * There are no special considerations for this module.
 *
 *
 * \section asfdoc_sam_drivers_dmac_extra_info Extra Information
 *
 * For extra information, see \ref asfdoc_sam_drivers_dmac_extra. This includes:
 *  - \ref asfdoc_sam_drivers_dmac_extra_acronyms
 *  - \ref asfdoc_sam_drivers_dmac_extra_dependencies
 *  - \ref asfdoc_sam_drivers_dmac_extra_errata
 *  - \ref asfdoc_sam_drivers_dmac_extra_history
 *
 * \section asfdoc_sam_drivers_dmac_examples Examples
 *
 * For a list of examples related to this driver, see
 * \ref asfdoc_sam_drivers_dmac_exqsg.
 *
 *
 * \section asfdoc_sam_drivers_dmac_api_overview API Overview
 * @{
 */

#include  <compiler.h>

/** @cond */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/** @endcond */


/** \brief DMAC priority mode */
typedef enum {
#if (SAM3U)
	/** Fixed priority arbiter */
	DMAC_PRIORITY_FIXED       = 0,
	/** Modified round robin arbiter */
	DMAC_PRIORITY_ROUND_ROBIN = DMAC_GCFG_ARB_CFG
#else
	/** Fixed priority arbiter */
	DMAC_PRIORITY_FIXED       = DMAC_GCFG_ARB_CFG_FIXED,
	/** Modified round robin arbiter */
	DMAC_PRIORITY_ROUND_ROBIN = DMAC_GCFG_ARB_CFG_ROUND_ROBIN
#endif
} dmac_priority_mode_t;

/** DMA transfer descriptor structure, otherwise known as a Linked List Item (LLI). */
typedef struct {
	uint32_t ul_source_addr;      /**< Source buffer address */
	uint32_t ul_destination_addr; /**< Destination buffer address */
	uint32_t ul_ctrlA;            /**< Control A register settings */
	uint32_t ul_ctrlB;            /**< Control B register settings */
	uint32_t ul_descriptor_addr;  /**< Next descriptor address */
} dma_transfer_descriptor_t;

#if !defined(__DOXYGEN__)
#define DMA_MAX_LENGTH 0xFFFu
#endif /* !defined(__DOXYGEN__) */

void dmac_init(Dmac *p_dmac);
void dmac_set_priority_mode(Dmac *p_dmac, dmac_priority_mode_t mode);
void dmac_enable(Dmac *p_dmac);
void dmac_disable(Dmac *p_dmac);
void dmac_enable_interrupt(Dmac *p_dmac, uint32_t ul_mask);
void dmac_disable_interrupt(Dmac *p_dmac, uint32_t ul_mask);
uint32_t dmac_get_interrupt_mask(Dmac *p_dmac);
uint32_t dmac_get_status(Dmac *p_dmac);

void dmac_channel_enable(Dmac *p_dmac, uint32_t ul_num);
void dmac_channel_disable(Dmac *p_dmac, uint32_t ul_num);
uint32_t dmac_channel_is_enable(Dmac *p_dmac, uint32_t ul_num);
void dmac_channel_suspend(Dmac *p_dmac, uint32_t ul_num);
void dmac_channel_resume(Dmac *p_dmac, uint32_t ul_num);
void dmac_channel_keep(Dmac *p_dmac, uint32_t ul_num);
uint32_t dmac_channel_get_status(Dmac *p_dmac);
void dmac_channel_set_source_addr(Dmac *p_dmac,
		uint32_t ul_num, uint32_t ul_addr);
void dmac_channel_set_destination_addr(Dmac *p_dmac,
		uint32_t ul_num, uint32_t ul_addr);
void dmac_channel_set_descriptor_addr(Dmac *p_dmac,
		uint32_t ul_num, uint32_t ul_desc);
void dmac_channel_set_ctrlA(Dmac *p_dmac, uint32_t ul_num, uint32_t ul_ctrlA);
void dmac_channel_set_ctrlB(Dmac *p_dmac, uint32_t ul_num, uint32_t ul_ctrlB);
void dmac_channel_set_configuration(Dmac *p_dmac, uint32_t ul_num,
		uint32_t ul_cfg);
void dmac_channel_single_buf_transfer_init(Dmac *p_dmac,
		uint32_t ul_num, dma_transfer_descriptor_t *p_desc);
void dmac_channel_multi_buf_transfer_init(Dmac *p_dmac,
		uint32_t ul_num, dma_transfer_descriptor_t *p_desc);
void dmac_channel_stop_transfer(Dmac *p_dmac, uint32_t ul_num);
uint32_t dmac_channel_is_transfer_done(Dmac *p_dmac, uint32_t ul_num);

void dmac_soft_single_transfer_request(Dmac *p_dmac,
		uint32_t ul_num, uint32_t ul_src_req, uint32_t ul_dst_req);
void dmac_soft_chunk_transfer_request(Dmac *p_dmac,
		uint32_t ul_num, uint32_t ul_src_req, uint32_t ul_dst_req);
void dmac_soft_set_last_transfer_flag(Dmac *p_dmac,
		uint32_t ul_num, uint32_t ul_src_flag, uint32_t ul_dst_flag);

#if (SAM3XA || SAM4E) || defined(__DOXYGEN__)
void dmac_set_writeprotect(Dmac *p_dmac, uint32_t ul_enable);
uint32_t dmac_get_writeprotect_status(Dmac *p_dmac);
#endif

/** @cond */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/** @endcond */

 /** @} */

/**
 * \page asfdoc_sam_drivers_dmac_extra Extra Information for Direct Memory Access Controller Driver
 *
 * \section asfdoc_sam_drivers_dmac_extra_acronyms Acronyms
 * Below is a table listing the acronyms used in this module, along with their
 * intended meanings.
 *
 * <table>
 *  <tr>
 *      <th>Acronym</th>
 *      <th>Definition</th>
 *  </tr>
 *  <tr>
 *      <td>AHB</td>
 *      <td>AMBA High-performance Bus</td>
 * </tr>
 *  <tr>
 *      <td>AMBA</td>
 *      <td>Advanced Microcontroller Bus Architecture</td>
 * </tr>
 *  <tr>
 *      <td>FIFO</td>
 *      <td>First In First Out</td>
 * </tr>
 *  <tr>
 *      <td>LLI</td>
 *      <td>Linked List Item</td>
 * </tr>
 *  <tr>
 *      <td>QSG</td>
 *      <td>Quick Start Guide</td>
 * </tr>
 * </table>
 *
 *
 * \section asfdoc_sam_drivers_dmac_extra_dependencies Dependencies
 * This driver has the following dependencies:
 *
 *  - None
 *
 *
 * \section asfdoc_sam_drivers_dmac_extra_errata Errata
 * There are no errata related to this driver.
 *
 *
 * \section asfdoc_sam_drivers_dmac_extra_history Module History
 * An overview of the module history is presented in the table below, with
 * details on the enhancements and fixes made to the module since its first
 * release. The current version of this corresponds to the newest version in
 * the table.
 *
 * <table>
 *	<tr>
 *		<th>Changelog</th>
 *	</tr>
 *	<tr>
 *		<td>Initial document release</td>
 *	</tr>
 * </table>
 */
 
/**
 * \page asfdoc_sam_drivers_dmac_exqsg Examples for Direct Memory Access Controller Driver
 *
 * This is a list of the available Quick Start Guides (QSGs) and example
 * applications for \ref asfdoc_sam_drivers_dmac_group. QSGs are simple examples with
 * step-by-step instructions to configure and use this driver in a selection of
 * use cases. Note that a QSG can be compiled as a standalone application or be
 * added to the user application.
 *
 *  - \subpage asfdoc_sam_drivers_dmac_qsg
 *  - \subpage asfdoc_sam_drivers_dmac_example
 *
 * \page asfdoc_sam_drivers_dmac_document_revision_history Document Revision History
 *
 * <table>
 *	<tr>
 *		<th>Doc. Rev.</td>
 *		<th>Date</td>
 *		<th>Comments</td>
 *	</tr>
 *	<tr>
 *		<td>42291B</td>
 *		<td>07/2015</td>
 *		<td>Updated title of application note and added list of supported devices</td>
 *	</tr>
 *	<tr>
 *		<td>42291A</td>
 *		<td>05/2014</td>
 *		<td>Initial document release</td>
 *	</tr>
 * </table>
 *
 */

  /**
 * \page asfdoc_sam_drivers_dmac_qsg Quick Start Guide for the DMAC driver
 *
 * This is the quick start guide for the \ref asfdoc_sam_drivers_dmac_group, with
 * step-by-step instructions on how to configure and use the driver for
 * a specific use case. The code examples can be
 * copied into the main application loop or any other function that will need 
 * to control the DMAC module.
 *
 * \section asfdoc_sam_drivers_dmac_qsg_use_cases Use Cases
 * - \ref asfdoc_sam_drivers_dmac_qsg_basic
 *
 * \section asfdoc_sam_drivers_dmac_qsg_basic DMAC Basic Usage
 *
 * This use case will demonstrate how to initialize the DMAC module to
 * perform a single memory to memory transfer.
 *
 *
 * \section asfdoc_sam_drivers_dmac_qsg_basic_setup Setup Steps
 *
 * \subsection asfdoc_sam_drivers_dmac_qsg_basic_prereq Prerequisites
 *
 * This module requires the following service
 * - \ref clk_group "System Clock Management (sysclock)"
 *
 * \subsection asfdoc_sam_drivers_dmac_qsg_basic_setup_code Setup Code
 *
 * Add these macros and global variable to the top of your application's C-file:
 * \snippet dmac_example.c dmac_define_channel
 * \snippet dmac_example.c dmac_define_buffer
 *
 * Add this to the main loop or a setup function:
 * \snippet dmac_example.c dmac_init_clock
 *
 * \subsection asfdoc_sam_drivers_dmac_qsg_basic_setup_workflow Workflow
 *
 * -# Define the variables needed, in order to perform a data transfer:
 * \snippet dmac_example.c dmac_define_vars
 * -# Prepare the data buffer to be transferred:
 * \snippet dmac_example.c dmac_define_prepare_buffer
 * -# Initialize the DMAC module:
 * \snippet dmac_example.c dmac_init_module
 * -# Set the priority to round-robin:
 * \snippet dmac_example.c dmac_set_priority
 * -# Enable the DMAC module:
 * \snippet dmac_example.c dmac_enable_module
 * -# Configure the channel for:
 *    - Enable stop on done
 *    - Enable AHB protection
 *    - Set the FIFO so that largest defined length AHB burst is performed
 * \snippet dmac_example.c dmac_configure_channel
 *
 * \section asfdoc_sam_drivers_dmac_qsg_basic_usage Usage Steps
 *
 * \subsection asfdoc_sam_drivers_dmac_qsg_basic_usage_code Usage Code
 * Configure the DMA source and destination buffer addresses:
 * \snippet dmac_example.c dmac_configure_for_single_transfer_1
 *
 * Configure DMA CTRLA:
 *    - Set the buffer transfer size to DMA_BUF_SIZE
 *    - Set the source transfer width to 32-bit
 *    - Set the destination transfer width to 32-bit
 * \snippet dmac_example.c dmac_configure_for_single_transfer_2
 *
 * Configure DMA CTRLB:
 *    - Disable source buffer descriptor fetch
 *    - Disable destination buffer descriptor Fetch
 *    - Enable memory-to-memory transfer
 *    - Increment the source address
 *    - Increment the  destination address
 * \snippet dmac_example.c dmac_configure_for_single_transfer_3
 *
 * Initialize the DMA transfer:
 * \snippet dmac_example.c dmac_configure_for_single_transfer_4
 *
 * Start the DMA transfer:
 * \snippet dmac_example.c dmac_start_transfer
 *
 * Finally, poll for the DMA transfer to complete:
 * \snippet dmac_example.c dmac_wait_for_done
 */

#endif /* DMAC_H_INCLUDED */
