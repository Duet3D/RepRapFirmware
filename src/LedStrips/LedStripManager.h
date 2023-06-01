/*
 * LedStripManager.h
 *
 *  Created on: 30 Apr 2023
 *      Author: David
 */

#ifndef SRC_LEDSTRIPS_LEDSTRIPMANAGER_H_
#define SRC_LEDSTRIPS_LEDSTRIPMANAGER_H_

#include <ObjectModel/ObjectModel.h>

#if SUPPORT_LED_STRIPS

#include "LedStripBase.h"

#if SUPPORT_REMOTE_COMMANDS
class CanMessageGeneric;
#endif

class LedStripManager
{
public:
	LedStripManager() noexcept;

	GCodeResult CreateStrip(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult HandleM150(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	bool MustStopMovement(GCodeBuffer& gb) noexcept;			// test whether this strip requires motion to be stopped before sending a command

	size_t GetNumLedStrips() const noexcept;					// called by RepRap.cpp object model entry
	const LedStripBase *GetLedStrip(size_t index) const noexcept;		// called by RepRap.cpp object model entry

#if SUPPORT_REMOTE_COMMANDS
	GCodeResult HandleM950Led(const CanMessageGeneric &msg, const StringRef& reply, uint8_t& extra) noexcept;
	GCodeResult HandleLedSetColours(const CanMessageGeneric &msg, const StringRef& reply) noexcept;
#endif

	static ReadWriteLock ledLock;								// needs to be public and static because RepRap.cpp needs to get it for the object model

private:
	LedStripBase *strips[MaxLedStrips];
};

#endif

#endif /* SRC_LEDSTRIPS_LEDSTRIPMANAGER_H_ */
