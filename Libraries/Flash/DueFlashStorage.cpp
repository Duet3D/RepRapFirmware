#include "DueFlashStorage.h"
#include <cstring>

void DueFlashStorage::read(uint32_t address, void *data, uint32_t dataLength)
{
	memcpy(data, FLASH_START + address, dataLength);
}

bool DueFlashStorage::write(uint32_t address, const void *data, uint32_t dataLength)
{
	if ((uint32_t)FLASH_START + address < IFLASH1_ADDR)
	{
		FLASH_DEBUG("Flash write address too low");
		return false;
	}

	if ((uint32_t)FLASH_START + address + dataLength > IFLASH1_ADDR + IFLASH1_SIZE)
	{
		FLASH_DEBUG("Flash write address too high");
		return false;
	}

	if ((((uint32_t)FLASH_START + address) & 3) != 0)
	{
		FLASH_DEBUG("Flash start address must be on 4-byte boundary\n");
		return false;
	}

	// The flash management code in the ASF is fragile and has a tendency to fail to return. Help it by disabling interrupts.
	irqflags_t flags = cpu_irq_save();

	// Unlock page
	uint32_t retCode = flash_unlock((uint32_t)FLASH_START + address, (uint32_t)FLASH_START + address + dataLength - 1, NULL, NULL);
	if (retCode != FLASH_RC_OK)
	{
		FLASH_DEBUG("Failed to unlock flash for write");
	}
	else
	{
		// Write data
		retCode = flash_write((uint32_t)FLASH_START + address, data, dataLength, 1);
		if (retCode != FLASH_RC_OK)
		{
			FLASH_DEBUG("Flash write failed");
		}
		else
		{
			// Lock page
			retCode = flash_lock((uint32_t)FLASH_START + address, (uint32_t)FLASH_START + address + dataLength - 1, NULL, NULL);
			if (retCode != FLASH_RC_OK)
			{
				FLASH_DEBUG("Failed to lock flash page");
			}
		}
	}

	cpu_irq_restore(flags);
	return retCode == FLASH_RC_OK;
}

// End
