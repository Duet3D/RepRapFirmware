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
	// Set the limits of one axis
	void SetAxisLimits(size_t axis, float min, float max) noexcept;

	// Configure or report this zone
	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

	// Check that a point is outside the keepout zone, throwing if it is inside
	void CheckPointIsOutside(const GCodeBuffer& gb, const float *coords) THROWS(GCodeException);

	// Check that a straight move lies fully outside the keepout zone, throwing if it encroaches it
	void CheckLineIsOutside(const GCodeBuffer& gb, const float *startCoords, const float *endCoords) THROWS(GCodeException);

private:
	AxesBitmap axesChecked;
	float lowCoords[MaxAxes];
	float highCoords[MaxAxes];
	bool active;
};

#endif

#endif /* SRC_GCODES_KEEPOUTZONE_H_ */
