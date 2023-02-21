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

#include <ObjectModel/ObjectModel.h>
#include "GCodeException.h"
#include <General/FreelistManager.h>

// This class represents a keepout zone in the form of a hypercuboid
class KeepoutZone INHERIT_OBJECT_MODEL
{
public:
	// Constructor
	KeepoutZone() noexcept : active(false) { }			// the BitMap default constructor will clear axesChecked

	bool IsDefined() const noexcept { return !axesChecked.IsEmpty(); }

	// Configure or report this zone
	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

#if 0	// not used
	// Check whether a point is inside the keepout zone, returning true if it is
	bool IsPointInside(const GCodeBuffer& gb, const float *pointCoords) const noexcept;
#endif

	// Check whether a straight line move intrudes into the keepout zone, returning true if it does
	bool DoesLineIntrude(const float startCoords[MaxAxes], const float endCoords[MaxAxes]) const noexcept;

	// Check whether an arc move intrudes into the keepout zone, returning true if it does
	bool DoesArcIntrude(const float startCoords[MaxAxes], const float endCoords[MaxAxes],
							float startAngle, float endAngle,
							const float arcCentres[MaxAxes], float arcRadius,
							AxesBitmap cosineAxes, AxesBitmap sineAxes,
							bool clockwise, bool wholeCircle) const noexcept;

protected:
	DECLARE_OBJECT_MODEL_WITH_ARRAYS

private:
	AxesBitmap axesChecked;				// which axes have defined limits
	float coords[MaxAxes][2];			// the lower and upper limits for each axis, lower < upper
	bool active;
};

#endif

#endif /* SRC_GCODES_KEEPOUTZONE_H_ */
