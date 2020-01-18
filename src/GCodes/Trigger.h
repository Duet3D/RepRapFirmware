/*
 * Trigger.h
 *
 *  Created on: 6 Jun 2019
 *      Author: David
 */

#ifndef SRC_GCODES_TRIGGER_H_
#define SRC_GCODES_TRIGGER_H_

#include "RepRapFirmware.h"
#include "Hardware/IoPorts.h"

typedef Bitmap<uint32_t> TriggerNumbersBitmap;					// Bitmap of trigger numbers
typedef Bitmap<uint16_t> TriggerInputStatesBitmap;				// Bitmap of input states
static_assert(MaxTriggers <= TriggerNumbersBitmap::MaxBits(), "need larger TriggerNumbersBitmap type");
static_assert(MaxPortsPerTrigger <= TriggerInputStatesBitmap::MaxBits(), "need larger TriggerInputStatesBitmap");

struct Trigger
{
	Trigger() noexcept;

	void Init() noexcept;

	// Return true if this trigger is unused, i.e. it doesn't watch any pins
	bool IsUnused() const noexcept;

	// Check whether this trigger is active and update the input states
	bool Check() noexcept;

	IoPort ports[MaxPortsPerTrigger];
	TriggerInputStatesBitmap inputStates;
	uint8_t condition;
};


#endif /* SRC_GCODES_TRIGGER_H_ */
