/*
 * RemoteLedStrip.h
 *
 *  Created on: 30 Apr 2023
 *      Author: David
 */

#ifndef SRC_LEDSTRIPS_REMOTELEDSTRIP_H_
#define SRC_LEDSTRIPS_REMOTELEDSTRIP_H_

#include "LedStripBase.h"

#if SUPPORT_LED_STRIPS

class RemoteLedStrip : public LedStripBase
{
public:
	RemoteLedStrip() noexcept;

	GCodeResult HandleM150(GCodeBuffer& gb, const StringRef& reply) override THROWS(GCodeException);

private:
	CanAddress boardNumber;
};

#endif

#endif /* SRC_LEDSTRIPS_REMOTELEDSTRIP_H_ */
