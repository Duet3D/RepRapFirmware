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

typedef uint32_t TriggerNumbersBitmap;					// Bitmap of trigger numbers
typedef uint16_t TriggerInputStatesBitmap;				// Bitmap of input states
static_assert(MaxTriggers <= sizeof(TriggerNumbersBitmap) * CHAR_BIT, "need larger TriggerNumbersBitmap type");
static_assert(MaxPortsPerTrigger <= sizeof(TriggerInputStatesBitmap) * CHAR_BIT, "need larger TriggerInputStatesBitmap");

struct Trigger
{
	Trigger();

	void Init();

	// Return true if this trigger is unused, i.e. it doesn't watch any pins
	bool IsUnused() const;

	// Check whether this trigger is active and update the input states
	bool Check();

	IoPort ports[MaxPortsPerTrigger];
	TriggerInputStatesBitmap inputStates;
	uint8_t condition;
};


#endif /* SRC_GCODES_TRIGGER_H_ */
