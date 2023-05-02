/*
 * RemoteLedStrip.cpp
 *
 *  Created on: 30 Apr 2023
 *      Author: David
 */

#include "RemoteLedStrip.h"

#if SUPPORT_LED_STRIPS && SUPPORT_CAN_EXPANSION

// Constructor
RemoteLedStrip::RemoteLedStrip(LedStripType p_type, size_t p_slotNumber, CanAddress p_boardNumber) noexcept
	: LedStripBase(p_type), slotNumber(p_slotNumber), boardNumber(p_boardNumber)
{
}

// Configure this strip
GCodeResult RemoteLedStrip::Configure(GCodeBuffer& gb, const StringRef& reply, const char *_ecv_array pinName) THROWS(GCodeException)
{
	//TODO
	return GCodeResult::errorNotSupported;
}

// Send a M150 command to this strip
GCodeResult RemoteLedStrip::HandleM150(GCodeBuffer &gb, const StringRef &reply) THROWS(GCodeException)
{
	//TODO
	return GCodeResult::errorNotSupported;
}

// Delete the remote strip
void RemoteLedStrip::DeleteRemote() noexcept
{
	//TODO
}

#endif

// End
