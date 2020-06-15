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

#include "compiler.h"
#include "conf_access.h"

#ifndef SECTOR_SIZE
#define SECTOR_SIZE  512
#endif

//! Status returned by CTRL_ACCESS interfaces.
typedef enum
{
  CTRL_GOOD       = PASS,     //!< Success, memory ready.
  CTRL_FAIL       = FAIL,     //!< An error occurred.
  CTRL_NO_PRESENT = FAIL + 1, //!< Memory unplugged.
  CTRL_BUSY       = FAIL + 2  //!< Memory not initialized or changed.
} Ctrl_status;


// FYI: Each Logical Unit Number (LUN) corresponds to a memory.

// Check LUN defines.
#ifndef LUN_0
  #error LUN_0 must be defined as ENABLE or DISABLE in conf_access.h
#endif
#ifndef LUN_1
  #error LUN_1 must be defined as ENABLE or DISABLE in conf_access.h
#endif
#ifndef LUN_2
  #error LUN_2 must be defined as ENABLE or DISABLE in conf_access.h
#endif
#ifndef LUN_3
  #error LUN_3 must be defined as ENABLE or DISABLE in conf_access.h
#endif
#ifndef LUN_4
  #error LUN_4 must be defined as ENABLE or DISABLE in conf_access.h
#endif
#ifndef LUN_5
  #error LUN_5 must be defined as ENABLE or DISABLE in conf_access.h
#endif
#ifndef LUN_6
  #error LUN_6 must be defined as ENABLE or DISABLE in conf_access.h
#endif
#ifndef LUN_7
  #error LUN_7 must be defined as ENABLE or DISABLE in conf_access.h
#endif

/*! \name LUN IDs
 */
//! @{
#define LUN_ID_0        (0)                 //!< First static LUN.
#define LUN_ID_1        (LUN_ID_0 + LUN_0)
#define LUN_ID_2        (LUN_ID_1 + LUN_1)
#define LUN_ID_3        (LUN_ID_2 + LUN_2)
#define LUN_ID_4        (LUN_ID_3 + LUN_3)
#define LUN_ID_5        (LUN_ID_4 + LUN_4)
#define LUN_ID_6        (LUN_ID_5 + LUN_5)
#define LUN_ID_7        (LUN_ID_6 + LUN_6)
#define MAX_LUN         (LUN_ID_7 + LUN_7)  //!< Number of static LUNs.
#define LUN_ID_USB      (MAX_LUN)           //!< First dynamic LUN (USB host mass storage).
//! @}


// Include LUN header files.
#if LUN_0 == ENABLE
  #include LUN_0_INCLUDE
#endif
#if LUN_1 == ENABLE
  #include LUN_1_INCLUDE
#endif
#if LUN_2 == ENABLE
  #include LUN_2_INCLUDE
#endif
#if LUN_3 == ENABLE
  #include LUN_3_INCLUDE
#endif
#if LUN_4 == ENABLE
  #include LUN_4_INCLUDE
#endif
#if LUN_5 == ENABLE
  #include LUN_5_INCLUDE
#endif
#if LUN_6 == ENABLE
  #include LUN_6_INCLUDE
#endif
#if LUN_7 == ENABLE
  #include LUN_7_INCLUDE
#endif


// Check the configuration of write protection in conf_access.h.
#ifndef GLOBAL_WR_PROTECT
  #error GLOBAL_WR_PROTECT must be defined as true or false in conf_access.h
#endif



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
extern Ctrl_status mem_test_unit_ready(U8 lun) noexcept;

/*! \brief Returns the address of the last valid sector (512 bytes) in the
 *         memory.
 *
 * \param lun           Logical Unit Number.
 * \param u32_nb_sector Pointer to the address of the last valid sector.
 *
 * \return Status.
 */
extern Ctrl_status mem_read_capacity(U8 lun, U32 *u32_nb_sector) noexcept;

/*! \brief Returns the size of the physical sector.
 *
 * \param lun Logical Unit Number.
 *
 * \return Sector size (unit: 512 bytes).
 */
extern U8 mem_sector_size(U8 lun) noexcept;

/*! \brief Returns the write-protection state of the memory.
 *
 * \param lun Logical Unit Number.
 *
 * \return \c true if the memory is write-protected, else \c false.
 *
 * \note Only used by removable memories with hardware-specific write
 *       protection.
 */
extern bool mem_wr_protect(U8 lun) noexcept;

/*! \brief Copies 1 data sector from the memory to RAM.
 *
 * \param lun   Logical Unit Number.
 * \param addr  Address of first memory sector to read.
 * \param ram   Pointer to RAM buffer to write.
 *
 * \return Status.
 */
extern Ctrl_status memory_2_ram(U8 lun, U32 addr, void *ram, uint32_t numBlocks) noexcept;

/*! \brief Copies 1 data sector from RAM to the memory.
 *
 * \param lun   Logical Unit Number.
 * \param addr  Address of first memory sector to write.
 * \param ram   Pointer to RAM buffer to read.
 *
 * \return Status.
 */
extern Ctrl_status ram_2_memory(U8 lun, U32 addr, const void *ram, uint32_t numBlocks) noexcept;

//! @}


/**
 * \}
 */

#ifdef __cplusplus
}
#endif

#endif  // _CTRL_ACCESS_H_
