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

#if 0	// not used
	// Check whether a point is inside the keepout zone, returning true if it is
	bool IsPointInside(const GCodeBuffer& gb, const float *pointCoords) noexcept;
#endif

	// Check whether a straight line move intrudes into the keepout zone, returning true if it does
	bool DoesLineIntrude(const float startCoords[MaxAxes], const float endCoords[MaxAxes]) noexcept;

	// Check whether an arc move intrudes into the keepout zone, returning true if it does
	bool DoesArcIntrude(const float startCoords[MaxAxes], const float endCoords[MaxAxes],
							float startAngle, float endAngle,
							const float arcCentres[MaxAxes], float arcRadius,
							AxesBitmap cosineAxes, AxesBitmap sineAxes,
							bool clockwise, bool wholeCircle) noexcept;

private:
	AxesBitmap axesChecked;
	float coords[MaxAxes][2];
	bool active;
};

#endif

#endif /* SRC_GCODES_KEEPOUTZONE_H_ */
