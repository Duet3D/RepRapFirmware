/*
 * RandomProbePointSet.h
 *
 *  Created on: 6 May 2017
 *      Author: David
 */

#ifndef SRC_MOVEMENT_RANDOMPROBEPOINTSET_H_
#define SRC_MOVEMENT_RANDOMPROBEPOINTSET_H_

#include "RepRapFirmware.h"
#include "ObjectModel/ObjectModel.h"

class RandomProbePointSet INHERIT_OBJECT_MODEL
{
public:
	RandomProbePointSet() noexcept;

	unsigned int GetNumBedCompensationPoints() const noexcept { return numBedCompensationPoints; }

	float GetZHeight(size_t index) const noexcept
	pre(index < numPoints) { return zBedProbePoints[index]; }

	float GetXCoord(size_t index) const noexcept
	pre(index < numPoints) { return xBedProbePoints[index]; }

	float GetYCoord(size_t index) const noexcept
	pre(index < numPoints) { return yBedProbePoints[index]; }

	bool PointWasCorrected(size_t index) const noexcept
	pre(index < numPoints) { return (probePointSet[index] & xyCorrected) != 0; }

	size_t NumberOfProbePoints() const noexcept;								// Return the number of points probed

	void SetXYBedProbePoint(size_t index, float x, float y) noexcept;			// Record the X and Y coordinates of a probe point
    void SetZBedProbePoint(size_t index, float z, bool wasXyCorrected, bool wasError) noexcept; // Record the Z coordinate of a probe point

	void ClearProbeHeights() noexcept;											// Clear out the Z heights so that we don't re-use old points
	void SetIdentity() noexcept { numBedCompensationPoints = 0; }				// Set identity transform

	bool GoodProbePoints(size_t numPoints) const noexcept;						// Check whether the specified set of points has been successfully defined and probed
	void ReportProbeHeights(size_t numPoints, const StringRef& reply) const noexcept;	// Print out the probe heights and any errors
	void DebugPrint(size_t numPoints) const noexcept;

protected:
	DECLARE_OBJECT_MODEL

private:
	// Enumeration to record what has been set
	enum PointCoordinateSet
	{
		unset = 0,
		xySet = 1,
		zSet = 2,
		xyCorrected = 4,
		probeError = 8
	};

	uint32_t numBedCompensationPoints;									// The number of points we are actually using for bed compensation, 0 means identity bed transform

	// Variables used to report what has been probed
	float xBedProbePoints[MaxProbePoints];								// The X coordinates of the points on the bed at which to probe
	float yBedProbePoints[MaxProbePoints];								// The Y coordinates of the points on the bed at which to probe
	float zBedProbePoints[MaxProbePoints];								// The Z coordinates of the points on the bed that were probed
	uint8_t probePointSet[MaxProbePoints];								// Has the XY of this point been set? Has the Z been probed?

	// Variables used to do 3-point compensation
	float aX, aY, aC; 													// Bed plane explicit equation z' = z + aX*x + aY*y + aC

	// Variables used to do 4-point compensation
	float xRectangle, yRectangle;										// The side lengths of the rectangle used for second-degree bed compensation
};

#endif /* SRC_MOVEMENT_RANDOMPROBEPOINTSET_H_ */
