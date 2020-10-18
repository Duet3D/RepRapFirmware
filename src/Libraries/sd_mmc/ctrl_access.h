/*****************************************************************************
 *
 * \file
 *
 * \brief Abstraction layer for memory interfaces.
 *
 * This module contains the interfaces:
 *   - MEM <-> USB;
 *   - MEM <-> RAM;
 *   - MEM <-> MEM.
 *
 * This module may be configured and expanded to support the following features:
 *   - write-protected globals;
 *   - password-protected data;
 *   - specific features;
 *   - etc.
 *
 * Copyright (c) 2009-2015 Atmel Corporation. All rights reserved.
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
 ******************************************************************************/
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */


#ifndef _CTRL_ACCESS_H_
#define _CTRL_ACCESS_H_

#include <Core.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \defgroup group_common_services_storage_ctrl_access Memory Control Access
 *
 * Common abstraction layer for memory interfaces. It provides interfaces between:
 * Memory and USB, Memory and RAM, Memory and Memory. Common API for XMEGA and UC3.
 *
 * \{
 */

#ifndef SECTOR_SIZE
# define SECTOR_SIZE  512
#endif

//! Status returned by CTRL_ACCESS interfaces.
typedef enum
{
  CTRL_GOOD       = 0,		//!< Success, memory ready.
  CTRL_FAIL       = 1,		//!< An error occurred.
  CTRL_NO_PRESENT = 2,		//!< Memory unplugged.
  CTRL_BUSY       = 3		//!< Memory not initialized or changed.
} Ctrl_status;

#define MAX_LUN         2

/*! \name Control Interface
 */
//! @{

/*! \brief Tests the memory state and initializes the memory if required.
 *
 * The TEST UNIT READY SCSI primary command allows an application client to poll
 * a LUN until it is ready without having to allocate memory for returned data.
 *
 * This command may be used to check the media status of LUNs with removable
 * media.
 *
 * \param lun Logical Unit Number.
 *
 * \return Status.
 */
extern Ctrl_status mem_test_unit_ready(uint8_t lun) noexcept;

/*! \brief Returns the address of the last valid sector (512 bytes) in the
 *         memory.
 *
 * \param lun           Logical Unit Number.
 * \param u32_nb_sector Pointer to the address of the last valid sector.
 *
 * \return Status.
 */
extern Ctrl_status mem_read_capacity(uint8_t lun, uint32_t *u32_nb_sector) noexcept;

/*! \brief Returns the size of the physical sector.
 *
 * \param lun Logical Unit Number.
 *
 * \return Sector size (unit: 512 bytes).
 */
extern uint8_t mem_sector_size(uint8_t lun) noexcept;

/*! \brief Returns the write-protection state of the memory.
 *
 * \param lun Logical Unit Number.
 *
 * \return \c true if the memory is write-protected, else \c false.
 *
 * \note Only used by removable memories with hardware-specific write
 *       protection.
 */
extern bool mem_wr_protect(uint8_t lun) noexcept;

/*! \brief Copies 1 data sector from the memory to RAM.
 *
 * \param lun   Logical Unit Number.
 * \param addr  Address of first memory sector to read.
 * \param ram   Pointer to RAM buffer to write.
 *
 * \return Status.
 */
extern Ctrl_status memory_2_ram(uint8_t lun, uint32_t addr, void *ram, uint32_t numBlocks) noexcept;

/*! \brief Copies 1 data sector from RAM to the memory.
 *
 * \param lun   Logical Unit Number.
 * \param addr  Address of first memory sector to write.
 * \param ram   Pointer to RAM buffer to read.
 *
 * \return Status.
 */
extern Ctrl_status ram_2_memory(uint8_t lun, uint32_t addr, const void *ram, uint32_t numBlocks) noexcept;

//! @}


/**
 * \}
 */

#ifdef __cplusplus
}
#endif

#endif  // _CTRL_ACCESS_H_
