/*
 * DotStarLed.h
 *
 *  Created on: 18 Jul 2018
 *      Author: David
 */

#ifndef SRC_FANS_LEDSTRIPDRIVER_H_
#define SRC_FANS_LEDSTRIPDRIVER_H_

#include <RepRapFirmware.h>

class GCodeBuffer;

namespace LedStripDriver
{
	void Init() noexcept;
	GCodeResult SetColours(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);		// handle M150
};

#endif /* SRC_FANS_LEDSTRIPDRIVER_H_ */
