/*
 * RemoteLedStrip.cpp
 *
 *  Created on: 30 Apr 2023
 *      Author: David
 */

#include "RemoteLedStrip.h"

#if SUPPORT_LED_STRIPS

RemoteLedStrip::RemoteLedStrip() noexcept
{
	// TODO Auto-generated constructor stub

}

GCodeResult RemoteLedStrip::HandleM150(GCodeBuffer &gb, const StringRef &reply) THROWS(GCodeException)
{
	//TODO
	return GCodeResult::errorNotSupported;
}

#endif

// End
