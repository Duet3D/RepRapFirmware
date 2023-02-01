/*
 * KeepoutZone.h
 *
 *  Created on: 1 Feb 2023
 *      Author: David
 */

#ifndef SRC_GCODES_KEEPOUTZONE_H_
#define SRC_GCODES_KEEPOUTZONE_H_

#include <RepRapFirmware.h>

#if SUPPORT_KEEPOUT_ZONES

#include "GCodeException.h"
#include <General/FreelistManager.h>

class KeepoutZone
{
public:
	// Constructor
	KeepoutZone() noexcept : active(false) { }			// the BitMap default constructor will clear axesChecked

	// Configure or report this zone
	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

	// Check that a point is outside the keepout zone, returning true if it is
	bool CheckPointIsOutside(const GCodeBuffer& gb, const float *pointCoords) noexcept;

	// Check that a straight line move lies fully outside the keepout zone, returning true if it is
	bool CheckLineIsOutside(const GCodeBuffer& gb, const float *startCoords, const float *endCoords) noexcept;

private:
	AxesBitmap axesChecked;
	float coords[MaxAxes][2];
	bool active;
};

#endif

#endif /* SRC_GCODES_KEEPOUTZONE_H_ */
