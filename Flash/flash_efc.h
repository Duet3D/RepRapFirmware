/**
 * \file
 *
 * \brief Embedded Flash service for SAM.
 *
 * Copyright (c) 2011-2013 Atmel Corporation. All rights reserved.
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

#ifndef FLASH_H_INCLUDED
#define FLASH_H_INCLUDED

#include <Arduino.h>
#include "efc.h"

/* Internal Flash 0 base address. */
#define IFLASH_ADDR     IFLASH0_ADDR
	/* Internal flash page size. */
#define IFLASH_PAGE_SIZE     IFLASH0_PAGE_SIZE

/* Last page start address. */
#define IFLASH_LAST_PAGE_ADDRESS (IFLASH1_ADDR + IFLASH1_SIZE - IFLASH1_PAGE_SIZE)

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
 extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

/*! \name Flash driver return codes */
//! @{
typedef enum flash_rc {
	FLASH_RC_OK = 0,        //!< Operation OK
	FLASH_RC_YES = 0,       //!< Yes
	FLASH_RC_NO = 1,        //!< No
	FLASH_RC_ERROR = 0x10,  //!< General error
	FLASH_RC_INVALID,       //!< Invalid argument input
	FLASH_RC_NOT_SUPPORT = 0xFFFFFFFF    //!< Operation is not supported
} flash_rc_t;
//! @}

/*! \name Flash erase page num in FARG[1:0] 
  \note The erase page commands should be cautiouly used as EPA4/EPA32 will not
  take effect according to errata and EPA8/EPA16 must module 8/16 page addresses.*/
//! @{
typedef enum flash_farg_page_num {
	/* 4 of pages to be erased with EPA command*/
	IFLASH_ERASE_PAGES_4=0,
	/* 8 of pages to be erased with EPA command*/
	IFLASH_ERASE_PAGES_8,
	/* 16 of pages to be erased with EPA command*/
	IFLASH_ERASE_PAGES_16,
	/* 32 of pages to be erased with EPA command*/
	IFLASH_ERASE_PAGES_32,
	/* Parameter is not support */
	IFLASH_ERASE_PAGES_INVALID,
}flash_farg_page_num_t;
//! @}

/*! \name Flash access mode */
//! @{
#define FLASH_ACCESS_MODE_128    EFC_ACCESS_MODE_128
#define FLASH_ACCESS_MODE_64     EFC_ACCESS_MODE_64
//! @}

uint32_t flash_init(uint32_t ul_mode, uint32_t ul_fws);
uint32_t flash_set_wait_state(uint32_t ul_address, uint32_t ul_fws);
uint32_t flash_set_wait_state_adaptively(uint32_t ul_address);
uint32_t flash_get_wait_state(uint32_t ul_address);
uint32_t flash_get_descriptor(uint32_t ul_address,
		uint32_t *pul_flash_descriptor,	uint32_t ul_size);
uint32_t flash_get_page_count(const uint32_t *pul_flash_descriptor);
uint32_t flash_get_page_count_per_region(const uint32_t *pul_flash_descriptor);
uint32_t flash_get_region_count(const uint32_t *pul_flash_descriptor);
uint32_t flash_erase_all(uint32_t ul_address);

#if SAM3SD8
uint32_t flash_erase_plane(uint32_t ul_address);
#endif

#if (SAM4S || SAM4E)
uint32_t flash_erase_page(uint32_t ul_address, uint8_t uc_page_num);
uint32_t flash_erase_sector(uint32_t ul_address);
#endif

uint32_t flash_write(uint32_t ul_address, const void *p_buffer,
		uint32_t ul_size, uint32_t ul_erase_flag);
uint32_t flash_lock(uint32_t ul_start, uint32_t ul_end,
		uint32_t *pul_actual_start, uint32_t *pul_actual_end);
uint32_t flash_unlock(uint32_t ul_start, uint32_t ul_end,
		uint32_t *pul_actual_start, uint32_t *pul_actual_end);
uint32_t flash_is_locked(uint32_t ul_start, uint32_t ul_end);
uint32_t flash_set_gpnvm(uint32_t ul_gpnvm);
uint32_t flash_clear_gpnvm(uint32_t ul_gpnvm);
uint32_t flash_is_gpnvm_set(uint32_t ul_gpnvm);
uint32_t flash_enable_security_bit(void);
uint32_t flash_is_security_bit_enabled(void);
uint32_t flash_read_unique_id(uint32_t *pul_data, uint32_t ul_size);

#if (SAM4S || SAM4E)
uint32_t flash_read_user_signature(uint32_t *p_data, uint32_t ul_size);
uint32_t flash_write_user_signature(uint32_t ul_address, const void *p_buffer,
		uint32_t ul_size);
uint32_t flash_erase_user_signature(void);
#endif

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond

#endif /* FLASH_H_INCLUDED */
