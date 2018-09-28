/*
 * DotStarLed.h
 *
 *  Created on: 18 Jul 2018
 *      Author: David
 */

#ifndef SRC_FANS_DOTSTARLED_H_
#define SRC_FANS_DOTSTARLED_H_

#include "RepRapFirmware.h"
#include "GCodes/GCodeResult.h"

class GCodeBuffer;

namespace DotStarLed
{
	void Init();
	GCodeResult SetColours(GCodeBuffer& gb, const StringRef& reply);
};

#endif /* SRC_FANS_DOTSTARLED_H_ */
