/*
 * Cache.cpp
 *
 *  Created on: 22 Nov 2019
 *      Author: David
 */

#include <Hardware/Cache.h>

#if USE_CACHE

#if SAME70
# include <core_cm7.h>

extern uint32_t _nocache_ram_start;
extern uint32_t _nocache_ram_end;

# if USE_MPU
#  include <mpu_armv7.h>

// Macro ARM_MPU_RASR_EX is incorrectly defined in CMSIS 5.4.0, see https://github.com/ARM-software/CMSIS_5/releases. Redefine it here.

#  undef ARM_MPU_RASR_EX

/**
* MPU Region Attribute and Size Register Value
*
* \param DisableExec       Instruction access disable bit, 1= disable instruction fetches.
* \param AccessPermission  Data access permissions, allows you to configure read/write access for User and Privileged mode.
* \param AccessAttributes  Memory access attribution, see \ref ARM_MPU_ACCESS_.
* \param SubRegionDisable  Sub-region disable field.
* \param Size              Region size of the region to be configured, for example 4K, 8K.
*/
#  define ARM_MPU_RASR_EX(DisableExec, AccessPermission, AccessAttributes, SubRegionDisable, Size)    \
  ((((DisableExec)      << MPU_RASR_XN_Pos)   & MPU_RASR_XN_Msk)                                  | \
   (((AccessPermission) << MPU_RASR_AP_Pos)   & MPU_RASR_AP_Msk)                                  | \
   (((AccessAttributes) & (MPU_RASR_TEX_Msk | MPU_RASR_S_Msk | MPU_RASR_C_Msk | MPU_RASR_B_Msk))) | \
   (((SubRegionDisable) << MPU_RASR_SRD_Pos)  & MPU_RASR_SRD_Msk)                                 | \
   (((Size)             << MPU_RASR_SIZE_Pos) & MPU_RASR_SIZE_Msk)                                | \
   (((MPU_RASR_ENABLE_Msk))))

# endif

#endif

#if SAM4E
# include <cmcc/cmcc.h>
#endif

static bool enabled = false;

void Cache::Init()
{
#if SAME70

# if USE_MPU
	// Set up the MPU so that we can have a non-cacheable RAM region, and so that we can trap accesses to non-existent memory
	// Where regions overlap, the region with the highest region number takes priority
	constexpr ARM_MPU_Region_t regionTable[] =
	{
		// Flash memory: read-only, execute allowed, cacheable
		{
			ARM_MPU_RBAR(0, IFLASH_ADDR),
			ARM_MPU_RASR_EX(0u, ARM_MPU_AP_RO, ARM_MPU_ACCESS_NORMAL(ARM_MPU_CACHEP_WB_WRA, ARM_MPU_CACHEP_WB_WRA, 1u), 0u, ARM_MPU_REGION_SIZE_1MB)
		},
		// First 256kb RAM, read-write, cacheable, execute disabled. Parts of this are overridden later.
		{
			ARM_MPU_RBAR(1, IRAM_ADDR),
			ARM_MPU_RASR_EX(1u, ARM_MPU_AP_FULL, ARM_MPU_ACCESS_NORMAL(ARM_MPU_CACHEP_WB_WRA, ARM_MPU_CACHEP_WB_WRA, 1u), 0u, ARM_MPU_REGION_SIZE_256KB)
		},
		// Final 128kb RAM, read-write, cacheable, execute disabled
		{
			ARM_MPU_RBAR(2, IRAM_ADDR + 0x00040000),
			ARM_MPU_RASR_EX(1u, ARM_MPU_AP_FULL, ARM_MPU_ACCESS_NORMAL(ARM_MPU_CACHEP_WB_WRA, ARM_MPU_CACHEP_WB_WRA, 1u), 0u, ARM_MPU_REGION_SIZE_128KB)
		},
		// Non-cachable RAM. This must be before normal RAM because it includes CAN buffers which must be within first 64kb.
		// Read write, execute disabled, non-cacheable
		{
			ARM_MPU_RBAR(3, IRAM_ADDR),
			ARM_MPU_RASR_EX(1u, ARM_MPU_AP_FULL, ARM_MPU_ACCESS_ORDERED, 0, ARM_MPU_REGION_SIZE_64KB)
		},
		// RAMFUNC memory. Read-only (the code has already been written to it), execution allowed. The initialised data memory follows, so it must be RW.
		// 256 bytes is enough at present (check the linker memory map if adding more RAMFUNCs).
		{
			ARM_MPU_RBAR(4, IRAM_ADDR + 0x00010000),
			ARM_MPU_RASR_EX(0u, ARM_MPU_AP_FULL, ARM_MPU_ACCESS_NORMAL(ARM_MPU_CACHEP_WB_WRA, ARM_MPU_CACHEP_WB_WRA, 1u), 0u, ARM_MPU_REGION_SIZE_256B)
		},
		// Peripherals
		{
			ARM_MPU_RBAR(5, 0x40000000),
			ARM_MPU_RASR_EX(1u, ARM_MPU_AP_FULL, ARM_MPU_ACCESS_DEVICE(1u), 0u, ARM_MPU_REGION_SIZE_16MB)
		},
		// USBHS
		{
			ARM_MPU_RBAR(6, 0xA0100000),
			ARM_MPU_RASR_EX(1u, ARM_MPU_AP_FULL, ARM_MPU_ACCESS_DEVICE(1u), 0u, ARM_MPU_REGION_SIZE_1MB)
		},
		// ROM
		{
			ARM_MPU_RBAR(7, IROM_ADDR),
			ARM_MPU_RASR_EX(0u, ARM_MPU_AP_RO, ARM_MPU_ACCESS_NORMAL(ARM_MPU_CACHEP_WB_WRA, ARM_MPU_CACHEP_WB_WRA, 1u), 0u, ARM_MPU_REGION_SIZE_4MB)
		},
		// ARM Private Peripheral Bus
		{
			ARM_MPU_RBAR(8, 0xE0000000),
			ARM_MPU_RASR_EX(1u, ARM_MPU_AP_FULL, ARM_MPU_ACCESS_ORDERED, 0u, ARM_MPU_REGION_SIZE_1MB)
		}
	};

	// Ensure MPU is disabled
	ARM_MPU_Disable();

	// Clear all regions
	const uint32_t numRegions = (MPU->TYPE & MPU_TYPE_DREGION_Msk) >> MPU_TYPE_DREGION_Pos;
	for (unsigned int region = 0; region < numRegions; ++region)
	{
		ARM_MPU_ClrRegion(region);
	}

	// Load regions from our table
	ARM_MPU_Load(regionTable, ARRAY_SIZE(regionTable));

	// Enable the MPU, disabling the default map but allowing exception handlers to use it
	ARM_MPU_Enable(0x01);
# endif

#elif SAM4E
	cmcc_config g_cmcc_cfg;
	cmcc_get_config_defaults(&g_cmcc_cfg);
	cmcc_init(CMCC, &g_cmcc_cfg);
#endif
}

void Cache::Enable()
{
	if (!enabled)
	{
		enabled = true;
#if SAME70
		SCB_EnableICache();
		SCB_EnableDCache();

#elif SAM4E
		cmcc_invalidate_all(CMCC);
		cmcc_enable(CMCC);
#endif
	}
}

void Cache::Disable()
{
	if (enabled)
	{
#if SAME70
		SCB_DisableICache();
		SCB_DisableDCache();
#elif SAM4E
		cmcc_disable(CMCC);
#endif
		enabled = false;
	}
}

#if SAME70

void Cache::Flush(const volatile void *start, size_t length)
{
	if (enabled)
	{
		// We assume that the DMA buffer is entirely inside or entirely outside the non-cached RAM area
		if (start < (void*)&_nocache_ram_start || start >= (void*)&_nocache_ram_end)
		{
			const uint32_t startAddr = reinterpret_cast<uint32_t>(start);
			SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t*>(startAddr & ~3), length + (startAddr & 3));
		}
	}
}

#endif

void Cache::Invalidate(const volatile void *start, size_t length)
{
	if (enabled)
	{
#if SAME70
		// We assume that the DMA buffer is entirely inside or entirely outside the non-cached RAM area
		if (start < (void*)&_nocache_ram_start || start >= (void*)&_nocache_ram_end)
		{
			// Caution! if any part of the cache line is dirty, the written data will be lost!
			const uint32_t startAddr = reinterpret_cast<uint32_t>(start);
			SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(startAddr & ~3), length + (startAddr & 3));
		}
#elif SAM4E
		// The cache is only 2kb on the SAM4E so we just invalidate the whole cache
		cmcc_invalidate_all(CMCC);
#endif
	}
}

#if SAM4E

uint32_t Cache::GetHitCount()
{
	return cmcc_get_monitor_cnt(CMCC);
}

#endif

#endif

// Entry points that can be called from ASF C code
void CacheFlushBeforeDMAReceive(const volatile void *start, size_t length) { Cache::FlushBeforeDMAReceive(start, length); }
void CacheInvalidateAfterDMAReceive(const volatile void *start, size_t length) { Cache::InvalidateAfterDMAReceive(start, length); }
void CacheFlushBeforeDMASend(const volatile void *start, size_t length) { Cache::FlushBeforeDMASend(start, length); }

// End
