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

#include <string.h>
#include <assert.h>
#include "flash_efc.h"

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

/**
 * \defgroup sam_services_flash_efc_group Embedded Flash Service
 *
 * The Embedded Flash service provides functions for internal flash operations.
 *
 * @{
 */

#if SAM4E
/* User signature size */
# define FLASH_USER_SIG_SIZE   (512)
#endif

#if SAM4S
/* Internal Flash Controller 0. */
# define EFC     EFC0
/* User signature size */
# define FLASH_USER_SIG_SIZE   (512)
/* Internal Flash 0 base address. */
# define IFLASH_ADDR     IFLASH0_ADDR
/* Internal flash page size. */
# define IFLASH_PAGE_SIZE     IFLASH0_PAGE_SIZE
/* Internal flash lock region size. */
# define IFLASH_LOCK_REGION_SIZE     IFLASH0_LOCK_REGION_SIZE
#endif

/* Internal Flash Controller 0. */
# define EFC     EFC0
/* The max GPNVM number. */
# define GPNVM_NUM_MAX        3
/* Internal Flash 0 base address. */
# define IFLASH_ADDR     IFLASH0_ADDR
/* Internal flash page size. */
# define IFLASH_PAGE_SIZE     IFLASH0_PAGE_SIZE
/* Internal flash lock region size. */
# define IFLASH_LOCK_REGION_SIZE     IFLASH0_LOCK_REGION_SIZE

/* Flash page buffer for alignment */
static uint32_t gs_ul_page_buffer[IFLASH_PAGE_SIZE / sizeof(uint32_t)];

/**
 * \brief Translate the given flash address to page and offset values.
 * \note pus_page and pus_offset must not be null in order to store the
 * corresponding values.
 *
 * \param pp_efc Pointer to an EFC pointer.
 * \param ul_addr Address to translate.
 * \param pus_page The first page accessed.
 * \param pus_offset Byte offset in the first page.
 */
static void translate_address(Efc **pp_efc, uint32_t ul_addr,
		uint16_t *pus_page, uint16_t *pus_offset)
{
	Efc *p_efc;
	uint16_t us_page;
	uint16_t us_offset;

	if (ul_addr >= IFLASH1_ADDR) {
		p_efc = EFC1;
		us_page = (ul_addr - IFLASH1_ADDR) / IFLASH1_PAGE_SIZE;
		us_offset = (ul_addr - IFLASH1_ADDR) % IFLASH1_PAGE_SIZE;
	} else {
		p_efc = EFC0;
		us_page = (ul_addr - IFLASH0_ADDR) / IFLASH0_PAGE_SIZE;
		us_offset = (ul_addr - IFLASH0_ADDR) % IFLASH0_PAGE_SIZE;
	}

	/* Store values */
	if (pp_efc) {
		*pp_efc = p_efc;
	}

	if (pus_page) {
		*pus_page = us_page;
	}

	if (pus_offset) {
		*pus_offset = us_offset;
	}
}

/**
 * \brief Compute the address of a flash by the given page and offset.
 *
 * \param p_efc Pointer to an EFC instance.
 * \param us_page Page number.
 * \param us_offset Byte offset inside page.
 * \param pul_addr Computed address (optional).
 */
static void compute_address(Efc *p_efc, uint16_t us_page, uint16_t us_offset,
		uint32_t *pul_addr)
{
	uint32_t ul_addr;

/* Dual bank flash */
#ifdef EFC1
	/* Compute address */
	ul_addr = (p_efc == EFC0) ?
			IFLASH0_ADDR + us_page * IFLASH_PAGE_SIZE + us_offset :
			IFLASH1_ADDR + us_page * IFLASH_PAGE_SIZE + us_offset;

/* One bank flash */
#else
	/* avoid Cppcheck Warning */
	UNUSED(p_efc);
	/* Compute address */
	ul_addr = IFLASH_ADDR + us_page * IFLASH_PAGE_SIZE + us_offset;
#endif

	/* Store result */
	if (pul_addr != NULL) {
		*pul_addr = ul_addr;
	}
}

/**
 * \brief Compute the lock range associated with the given address range.
 *
 * \param ul_start Start address of lock range.
 * \param ul_end End address of lock range.
 * \param pul_actual_start Actual start address of lock range.
 * \param pul_actual_end Actual end address of lock range.
 */
static void compute_lock_range(uint32_t ul_start, uint32_t ul_end,
		uint32_t *pul_actual_start, uint32_t *pul_actual_end)
{
	uint32_t ul_actual_start, ul_actual_end;

	ul_actual_start = ul_start - (ul_start % IFLASH_LOCK_REGION_SIZE);
	ul_actual_end = ul_end - (ul_end % IFLASH_LOCK_REGION_SIZE) +
			IFLASH_LOCK_REGION_SIZE - 1;

	if (pul_actual_start) {
		*pul_actual_start = ul_actual_start;
	}

	if (pul_actual_end) {
		*pul_actual_end = ul_actual_end;
	}
}

/**
 * \brief Initialize the flash service.
 *
 * \param ul_mode FLASH_ACCESS_MODE_128 or FLASH_ACCESS_MODE_64.
 * \param ul_fws The number of wait states in cycle (no shift).
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_init(uint32_t ul_mode, uint32_t ul_fws)
{
	efc_init(EFC, ul_mode, ul_fws);

#ifdef EFC1
	efc_init(EFC1, ul_mode, ul_fws);
#endif

	return FLASH_RC_OK;
}

/**
 * \brief Set flash wait state.
 *
 * \param ul_address Flash bank start address.
 * \param ul_fws The number of wait states in cycle (no shift).
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_set_wait_state(uint32_t ul_address, uint32_t ul_fws)
{
	Efc *p_efc;

	translate_address(&p_efc, ul_address, NULL, NULL);
	efc_set_wait_state(p_efc, ul_fws);

	return FLASH_RC_OK;
}

/**
 * \brief Set flash wait state.
 *
 * \param ul_address Flash bank start address.
 * \param ul_fws The number of wait states in cycle (no shift).
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_set_wait_state_adaptively(uint32_t ul_address)
{
	Efc *p_efc;
	uint32_t clock = SystemCoreClock;

	translate_address(&p_efc, ul_address, NULL, NULL);

	/* Set FWS for embedded Flash access according to operating frequency */
	if (clock < CHIP_FREQ_FWS_0) {
		efc_set_wait_state(p_efc, 0);
	} else if (clock < CHIP_FREQ_FWS_1) {
		efc_set_wait_state(p_efc, 1);
	} else if (clock < CHIP_FREQ_FWS_2) {
		efc_set_wait_state(p_efc, 2);
	} else if (clock < CHIP_FREQ_FWS_3) {
		efc_set_wait_state(p_efc, 3);
	} else {
		efc_set_wait_state(p_efc, 4);
	}
	return FLASH_RC_OK;
}

/**
 * \brief Get flash wait state.
 *
 * \param ul_address Flash bank start address.
 *
 * \return The number of wait states in cycle (no shift).
 */
uint32_t flash_get_wait_state(uint32_t ul_address)
{
	Efc *p_efc;

	translate_address(&p_efc, ul_address, NULL, NULL);
	return efc_get_wait_state(p_efc);
}

/**
 * \brief Get flash descriptor.
 *
 * \param ul_address Flash bank start address.
 * \param pul_flash_descriptor Pointer to a data buffer to store flash descriptor.
 * \param ul_size Data buffer size in DWORD.
 *
 * \return The actual descriptor length.
 */
uint32_t flash_get_descriptor(uint32_t ul_address,
		uint32_t *pul_flash_descriptor, uint32_t ul_size)
{
	Efc *p_efc;
	uint32_t ul_tmp;
	uint32_t ul_cnt;

	translate_address(&p_efc, ul_address, NULL, NULL);

	/* Command fails */
	if (FLASH_RC_OK != efc_perform_command(p_efc, EFC_FCMD_GETD, 0)) {
		return 0;
	} else {
		/* Read until no result */
		for (ul_cnt = 0;; ul_cnt++) {
			ul_tmp = efc_get_result(p_efc);
			if ((ul_size > ul_cnt) && (ul_tmp != 0)) {
				*pul_flash_descriptor++ = ul_tmp;
			} else {
				break;
			}
		}
	}

	return ul_cnt;
}

/**
 * \brief Get flash total page count for the specified bank.
 *
 * \note The flash descriptor must be fetched from flash_get_descriptor
 * function first.
 *
 * \param pul_flash_descriptor Pointer to a flash descriptor.
 *
 * \return The flash total page count.
 */
uint32_t flash_get_page_count(const uint32_t *pul_flash_descriptor)
{
	return (pul_flash_descriptor[1] / pul_flash_descriptor[2]);
}

/**
 * \brief Get flash page count per region (plane) for the specified bank.
 *
 * \note The flash descriptor must be fetched from flash_get_descriptor
 * function first.
 *
 * \param pul_flash_descriptor Pointer to a flash descriptor.
 *
 * \return The flash page count per region (plane).
 */
uint32_t flash_get_page_count_per_region(const uint32_t *pul_flash_descriptor)
{
	return (pul_flash_descriptor[4] / pul_flash_descriptor[2]);
}

/**
 * \brief Get flash region (plane) count for the specified bank.
 *
 * \note The flash descriptor must be fetched from flash_get_descriptor
 * function first.
 *
 * \param pul_flash_descriptor Pointer to a flash descriptor.
 *
 * \return The flash region (plane) count.
 */
uint32_t flash_get_region_count(const uint32_t *pul_flash_descriptor)
{
	return (pul_flash_descriptor[3]);
}

/**
 * \brief Erase the entire flash.
 *
 * \note Only the flash bank including ul_address will be erased. If there are
 * two flash banks, we need to call this function twice with each bank start
 * address.
 *
 * \param ul_address  Flash bank start address.
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_erase_all(uint32_t ul_address)
{
	Efc *p_efc;

	translate_address(&p_efc, ul_address, NULL, NULL);

	if (EFC_RC_OK != efc_perform_command(p_efc, EFC_FCMD_EA, 0)) {
		return FLASH_RC_ERROR;
	}

	return FLASH_RC_OK;
}

#if SAM3SD8
/**
 * \brief Erase the flash by plane.
 *
 * \param ul_address Flash plane start address.
 *
 * \note Erase plane command needs a page number parameter which belongs to
 * the plane to be erased.
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_erase_plane(uint32_t ul_address)
{
	Efc *p_efc;
	uint16_t us_page;

	translate_address(&p_efc, ul_address, &us_page, NULL);

	if (EFC_RC_OK != efc_perform_command(p_efc, EFC_FCMD_EPL, us_page)) {
		return FLASH_RC_ERROR;
	}

	return FLASH_RC_OK;
}
#endif

#if (SAM4S || SAM4E)
/**
 * \brief Erase the specified pages of flash.
 *
 * \param ul_address Flash bank start address.
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_erase_page(uint32_t ul_address, uint8_t uc_page_num)
{
	Efc *p_efc;
	uint16_t us_page;

	if (uc_page_num >= IFLASH_ERASE_PAGES_INVALID) {
		return FLASH_RC_INVALID;
	}

	if (ul_address & (IFLASH_PAGE_SIZE - 1)) {
		return FLASH_RC_INVALID;
	}

	translate_address(&p_efc, ul_address, &us_page, NULL);

	if (EFC_RC_OK != efc_perform_command(p_efc, EFC_FCMD_EPA,
					(us_page | uc_page_num))) {
		return FLASH_RC_ERROR;
	}

	return FLASH_RC_OK;
}

/**
 * \brief Erase the flash sector.
 *
 * \note Erase sector command needs a page number parameter which belongs to
 * the sector to be erased.
 *
 * \param ul_address Flash sector start address.
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_erase_sector(uint32_t ul_address)
{
	Efc *p_efc;
	uint16_t us_page;

	translate_address(&p_efc, ul_address, &us_page, NULL);

	if (EFC_RC_OK != efc_perform_command(p_efc, EFC_FCMD_ES, us_page)) {
		return FLASH_RC_ERROR;
	}

	return FLASH_RC_OK;
}
#endif

/**
 * \brief Write a data buffer on flash.
 *
 * \note This function works in polling mode, and thus only returns when the
 * data has been effectively written.
 * \note For dual bank flash, this function doesn't support cross write from
 * bank 0 to bank 1. In this case, flash_write must be called twice (ie for
 * each bank).
 *
 * \param ul_address Write address.
 * \param p_buffer Data buffer.
 * \param ul_size Size of data buffer in bytes.
 * \param ul_erase_flag Flag to set if erase first.
 *
 * \return 0 if successful, otherwise returns an error code.
 */
uint32_t flash_write(uint32_t ul_address, const void *p_buffer,
		uint32_t ul_size, uint32_t ul_erase_flag)
{
	Efc *p_efc;
	uint32_t ul_fws_temp;
	uint16_t us_page;
	uint16_t us_offset;
	uint32_t writeSize;
	uint32_t ul_page_addr;
	uint16_t us_padding;
	uint32_t ul_error;
	uint32_t ul_idx;
	uint32_t *p_aligned_dest;
	uint8_t *puc_page_buffer = (uint8_t *) gs_ul_page_buffer;

	translate_address(&p_efc, ul_address, &us_page, &us_offset);

	/* According to the errata, set the wait state value to 6. */
	ul_fws_temp = efc_get_wait_state(p_efc);
	efc_set_wait_state(p_efc, 6);

	/* Write all pages */
	while (ul_size > 0) {
		/* Copy data in temporary buffer to avoid alignment problems. */
		writeSize = Min((uint32_t) IFLASH_PAGE_SIZE - us_offset,
				ul_size);
		compute_address(p_efc, us_page, 0, &ul_page_addr);
		us_padding = IFLASH_PAGE_SIZE - us_offset - writeSize;

		/* Pre-buffer data */
		memcpy(puc_page_buffer, (void *)ul_page_addr, us_offset);

		/* Buffer data */
		memcpy(puc_page_buffer + us_offset, p_buffer, writeSize);

		/* Post-buffer data */
		memcpy(puc_page_buffer + us_offset + writeSize,
				(void *)(ul_page_addr + us_offset + writeSize),
				us_padding);

		/* Write page.
		 * Writing 8-bit and 16-bit data is not allowed and may lead to
		 * unpredictable data corruption.
		 */
		p_aligned_dest = (uint32_t *) ul_page_addr;
		for (ul_idx = 0; ul_idx < (IFLASH_PAGE_SIZE / sizeof(uint32_t));
				++ul_idx) {
			*p_aligned_dest++ = gs_ul_page_buffer[ul_idx];
		}

		if (ul_erase_flag) {
			ul_error = efc_perform_command(p_efc, EFC_FCMD_EWP,
					us_page);
		} else {
			ul_error = efc_perform_command(p_efc, EFC_FCMD_WP,
					us_page);
		}

		if (ul_error) {
			return ul_error;
		}

		/* Progression */
		p_buffer = (void *)((uint32_t) p_buffer + writeSize);
		ul_size -= writeSize;
		us_page++;
		us_offset = 0;
	}

	/* According to the errata, restore the wait state value. */
	efc_set_wait_state(p_efc, ul_fws_temp);

	return FLASH_RC_OK;
}


/**
 * \brief Lock all the regions in the given address range. The actual lock
 * range is reported through two output parameters.
 *
 * \param ul_start Start address of lock range.
 * \param ul_end End address of lock range.
 * \param pul_actual_start Start address of the actual lock range (optional).
 * \param pul_actual_end End address of the actual lock range (optional).
 *
 * \return 0 if successful, otherwise returns an error code.
 */
uint32_t flash_lock(uint32_t ul_start, uint32_t ul_end,
		uint32_t *pul_actual_start, uint32_t *pul_actual_end)
{
	Efc *p_efc;
	uint32_t ul_actual_start, ul_actual_end;
	uint16_t us_start_page, us_end_page;
	uint32_t ul_error;
	uint16_t us_num_pages_in_region =
			IFLASH_LOCK_REGION_SIZE / IFLASH_PAGE_SIZE;

	/* Compute actual lock range and store it */
	compute_lock_range(ul_start, ul_end, &ul_actual_start, &ul_actual_end);

	if (pul_actual_start != NULL) {
		*pul_actual_start = ul_actual_start;
	}

	if (pul_actual_end != NULL) {
		*pul_actual_end = ul_actual_end;
	}

	/* Compute page numbers */
	translate_address(&p_efc, ul_actual_start, &us_start_page, 0);
	translate_address(0, ul_actual_end, &us_end_page, 0);

	/* Lock all pages */
	while (us_start_page < us_end_page) {
		ul_error = efc_perform_command(p_efc, EFC_FCMD_SLB, us_start_page);

		if (ul_error) {
			return ul_error;
		}
		us_start_page += us_num_pages_in_region;
	}

	return FLASH_RC_OK;
}

/**
 * \brief Unlock all the regions in the given address range. The actual unlock
 * range is reported through two output parameters.
 *
 * \param ul_start Start address of unlock range.
 * \param ul_end End address of unlock range.
 * \param pul_actual_start Start address of the actual unlock range (optional).
 * \param pul_actual_end End address of the actual unlock range (optional).
 *
 * \return 0 if successful, otherwise returns an error code.
 */
uint32_t flash_unlock(uint32_t ul_start, uint32_t ul_end,
		uint32_t *pul_actual_start, uint32_t *pul_actual_end)
{
	Efc *p_efc;
	uint32_t ul_actual_start, ul_actual_end;
	uint16_t us_start_page, us_end_page;
	uint32_t ul_error;
	uint16_t us_num_pages_in_region =
			IFLASH_LOCK_REGION_SIZE / IFLASH_PAGE_SIZE;

	/* Compute actual unlock range and store it */
	compute_lock_range(ul_start, ul_end, &ul_actual_start, &ul_actual_end);
	if (pul_actual_start != NULL) {
		*pul_actual_start = ul_actual_start;
	}
	if (pul_actual_end != NULL) {
		*pul_actual_end = ul_actual_end;
	}

	/* Compute page numbers */
	translate_address(&p_efc, ul_actual_start, &us_start_page, 0);
	translate_address(0, ul_actual_end, &us_end_page, 0);

	/* Unlock all pages */
	while (us_start_page < us_end_page) {
		ul_error = efc_perform_command(p_efc, EFC_FCMD_CLB,
				us_start_page);
		if (ul_error) {
			return ul_error;
		}
		us_start_page += us_num_pages_in_region;
	}

	return FLASH_RC_OK;
}

/**
 * \brief Get the number of locked regions inside the given address range.
 *
 * \param ul_start Start address of range
 * \param ul_end End address of range.
 *
 * \return The number of locked regions inside the given address range.
 */
uint32_t flash_is_locked(uint32_t ul_start, uint32_t ul_end)
{
	Efc *p_efc;
	uint16_t us_start_page, us_end_page;
	uint8_t uc_start_region, uc_end_region;
	uint16_t us_num_pages_in_region;
	uint32_t ul_status;
	uint32_t ul_num_locked_regions = 0;
	uint32_t ul_count = 0;
	uint32_t ul_bit = 0;

	/* Compute page numbers */
	translate_address(&p_efc, ul_start, &us_start_page, 0);
	translate_address(0, ul_end, &us_end_page, 0);

	/* Compute region numbers */
	us_num_pages_in_region = IFLASH_LOCK_REGION_SIZE / IFLASH_PAGE_SIZE;
	uc_start_region = us_start_page / us_num_pages_in_region;
	uc_end_region = us_end_page / us_num_pages_in_region;

	/* Retrieve lock status */
	efc_perform_command(p_efc, EFC_FCMD_GLB, 0);

	/* Skip unrequested regions (if necessary) */
	ul_status = efc_get_result(p_efc);
	while (!(ul_count <= uc_start_region &&
			uc_start_region < (ul_count + 32))) {
		ul_status = efc_get_result(p_efc);
		ul_count += 32;
	}

	/* Check status of each involved region */
	ul_bit = uc_start_region - ul_count;

	/* Number of region to check (must be > 0) */
	ul_count = uc_end_region - uc_start_region + 1;

	while (ul_count > 0) {
		if (ul_status & (1 << (ul_bit))) {
			ul_num_locked_regions++;
		}

		ul_count -= 1;
		ul_bit += 1;
		if (ul_bit == 32) {
			ul_status = efc_get_result(p_efc);
			ul_bit = 0;
		}
	}

	return ul_num_locked_regions;
}

/**
 * \brief Set the given GPNVM bit.
 *
 * \param ul_gpnvm GPNVM bit index.
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_set_gpnvm(uint32_t ul_gpnvm)
{
	if (ul_gpnvm >= GPNVM_NUM_MAX) {
		return FLASH_RC_INVALID;
	}

	if (FLASH_RC_YES == flash_is_gpnvm_set(ul_gpnvm)) {
		return FLASH_RC_OK;
	}

	if (EFC_RC_OK == efc_perform_command(EFC, EFC_FCMD_SGPB, ul_gpnvm)) {
		return FLASH_RC_OK;
	}

	return FLASH_RC_ERROR;
}

/**
 * \brief Clear the given GPNVM bit.
 *
 * \param ul_gpnvm GPNVM bit index.
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_clear_gpnvm(uint32_t ul_gpnvm)
{
	if (ul_gpnvm >= GPNVM_NUM_MAX) {
		return FLASH_RC_INVALID;
	}

	if (FLASH_RC_NO == flash_is_gpnvm_set(ul_gpnvm)) {
		return FLASH_RC_OK;
	}

	if (EFC_RC_OK == efc_perform_command(EFC, EFC_FCMD_CGPB, ul_gpnvm)) {
		return FLASH_RC_OK;
	}

	return FLASH_RC_ERROR;
}

/**
 * \brief Check if the given GPNVM bit is set or not.
 *
 * \param ul_gpnvm GPNVM bit index.
 *
 * \retval 1 If the given GPNVM bit is currently set.
 * \retval 0 If the given GPNVM bit is currently cleared.
 */
uint32_t flash_is_gpnvm_set(uint32_t ul_gpnvm)
{
	uint32_t ul_gpnvm_bits;

	if (ul_gpnvm >= GPNVM_NUM_MAX) {
		return FLASH_RC_INVALID;
	}

	if (EFC_RC_OK != efc_perform_command(EFC, EFC_FCMD_GGPB, 0)) {
		return FLASH_RC_ERROR;
	}

	ul_gpnvm_bits = efc_get_result(EFC);
	if (ul_gpnvm_bits & (1 << ul_gpnvm)) {
		return FLASH_RC_YES;
	}

	return FLASH_RC_NO;
}

/**
 * \brief Set security bit.
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_enable_security_bit(void)
{
	return flash_set_gpnvm(0);
}

/**
 * \brief Check if the security bit is set or not.
 *
 * \retval 1 If the security bit is currently set.
 * \retval 0 If the security bit is currently cleared.
 */
uint32_t flash_is_security_bit_enabled(void)
{
	return flash_is_gpnvm_set(0);
}

/**
 * \brief Read the flash unique ID.
 *
 * \param pul_data Pointer to a data buffer to store 128-bit unique ID.
 * \param ul_size Data buffer size in DWORD.
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_read_unique_id(uint32_t *pul_data, uint32_t ul_size)
{
	uint32_t uid_buf[4];
	uint32_t ul_idx;

	if (FLASH_RC_OK != efc_perform_read_sequence(EFC, EFC_FCMD_STUI,
			EFC_FCMD_SPUI, uid_buf, 4)) {
		return FLASH_RC_ERROR;
	}

	if (ul_size > 4) {
		/* Only 4 dword to store unique ID */
		ul_size = 4;
	}

	for (ul_idx = 0; ul_idx < ul_size; ul_idx++) {
		pul_data[ul_idx] = uid_buf[ul_idx];
	}

	return FLASH_RC_OK;
}

#if (SAM4S || SAM4E)
/**
 * \brief Read the flash user signature.
 *
 * \param p_data Pointer to a data buffer to store 512 bytes of user signature.
 * \param ul_size Data buffer size.
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_read_user_signature(uint32_t *p_data, uint32_t ul_size)
{
	if (ul_size > FLASH_USER_SIG_SIZE) {
		/* Only 512 byte to store unique ID */
		ul_size = FLASH_USER_SIG_SIZE;
	}

	/* Send the read user signature commands */
	if (FLASH_RC_OK != efc_perform_read_sequence(EFC, EFC_FCMD_STUS,
			EFC_FCMD_SPUS, p_data, ul_size)) {
		return FLASH_RC_ERROR;
	}

	return FLASH_RC_OK;
}

/**
 * \brief Write the flash user signature.
 *
 * \param ul_address Write address.
 * \param p_data Pointer to a data buffer to store 512 bytes of user signature.
 * \param ul_size Data buffer size.
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_write_user_signature(uint32_t ul_address, const void *p_buffer,
		uint32_t ul_size)
{
	/* The user signature should be no longer than 512 bytes */
	if (ul_size > FLASH_USER_SIG_SIZE) {
		return FLASH_RC_INVALID;
	}

	/* Write the full page */
	flash_write(ul_address,  p_buffer, ul_size, 0);

	/* Send the write signature command */
	if (FLASH_RC_OK != efc_perform_command(EFC, EFC_FCMD_WUS, 0)) {
		return FLASH_RC_ERROR;
	}

	return FLASH_RC_OK;
}

/**
 * \brief Erase the flash user signature.
 *
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t flash_erase_user_signature(void)
{
	/* Perform the erase user signature command */
	return efc_perform_command(EFC, EFC_FCMD_EUS, 0);
}
#endif

//@}

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond
