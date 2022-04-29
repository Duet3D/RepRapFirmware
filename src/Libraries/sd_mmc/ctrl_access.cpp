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


//_____ I N C L U D E S ____________________________________________________

#include "ctrl_access.h"
#include "sd_mmc_mem.h"


Ctrl_status mem_test_unit_ready(uint8_t lun) noexcept
{
	return (lun < MAX_LUN) ? sd_mmc_test_unit_ready(lun) : CTRL_FAIL;
}

Ctrl_status mem_read_capacity(uint8_t lun, uint32_t *u32_nb_sector) noexcept
{
	return (lun < MAX_LUN) ? sd_mmc_read_capacity(lun, u32_nb_sector) : CTRL_FAIL;
}

uint8_t mem_sector_size(uint8_t lun) noexcept
{
	return 1;
}

bool mem_wr_protect(uint8_t lun) noexcept
{
#if SUPPORT_WRITE_PROTECT
	return (lun >= MAX_LUN) || sd_mmc_wr_protect(lun)e;
#else
	return false;
#endif
}

Ctrl_status memory_2_ram(uint8_t lun, uint32_t addr, void *ram, uint32_t numBlocks) noexcept
{
	return (lun < MAX_LUN) ? sd_mmc_mem_2_ram(lun, addr, ram, numBlocks) : CTRL_FAIL;
}

Ctrl_status ram_2_memory(uint8_t lun, uint32_t addr, const void *ram, uint32_t numBlocks) noexcept
{
	return (lun < MAX_LUN) ? sd_mmc_ram_2_mem(lun, addr, ram, numBlocks) : CTRL_FAIL;
}


//! @}
