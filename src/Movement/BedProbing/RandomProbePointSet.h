/*
 * RandomProbePointSet.h
 *
 *  Created on: 6 May 2017
 *      Author: David
 */

#ifndef SRC_MOVEMENT_RANDOMPROBEPOINTSET_H_
#define SRC_MOVEMENT_RANDOMPROBEPOINTSET_H_

#include "RepRapFirmware.h"

class RandomProbePointSet
{
public:
	RandomProbePointSet();

	unsigned int GetNumBedCompensationPoints() const { return numBedCompensationPoints; }

	float GetZHeight(size_t index) const
	pre(index < numPoints) { return zBedProbePoints[index]; }

	float GetXCoord(size_t index) const
	pre(index < numPoints) { return xBedProbePoints[index]; }

	float GetYCoord(size_t index) const
	pre(index < numPoints) { return yBedProbePoints[index]; }

	bool PointWasCorrected(size_t index) const
	pre(index < numPoints) { return (probePointSet[index] & xyCorrected) != 0; }

	size_t NumberOfProbePoints() const;									// Return the number of points probed

	void SetXYBedProbePoint(size_t index, float x, float y);			// Record the X and Y coordinates of a probe point
    void SetZBedProbePoint(size_t index, float z, bool wasXyCorrected, bool wasError); // Record the Z coordinate of a probe point

	void ClearProbeHeights();											// Clear out the Z heights so that we don't re-use old points
	bool SetProbedBedEquation(size_t numPoints, const StringRef& reply);		// When we have a full set of probed points, work out the bed's equation
	void SetIdentity() { numBedCompensationPoints = 0; }				// Set identity transform

	float GetInterpolatedHeightError(float x, float y) const;			// Compute the interpolated height error at the specified point

	bool GoodProbePoints(size_t numPoints) const;						// Check whether the specified set of points has been successfully defined and probed
	void ReportProbeHeights(size_t numPoints, const StringRef& reply) const;	// Print out the probe heights and any errors
	void DebugPrint(size_t numPoints) const;

private:
	bool GoodProbePointOrdering(size_t numPoints) const;				// Check that the probe points are in the right order
    float SecondDegreeTransformZ(float x, float y) const;				// Used for second degree bed equation
	float TriangleZ(float x, float y) const;							// Interpolate onto a triangular grid
	void BarycentricCoordinates(size_t p0, size_t p1,   				// Compute the barycentric coordinates of a point in a triangle
			size_t p2, float x, float y, float& l1,     				// (see http://en.wikipedia.org/wiki/Barycentric_coordinate_system).
			float& l2, float& l3) const;

	// Enumeration to record what has been set
	enum PointCoordinateSet
	{
		unset = 0,
		xySet = 1,
		zSet = 2,
		xyCorrected = 4,
		probeError = 8
	};

	unsigned int numBedCompensationPoints;								// The number of points we are actually using for bed compensation, 0 means identity bed transform

	// Variables used to report what has been probed
	float xBedProbePoints[MaxProbePoints];								// The X coordinates of the points on the bed at which to probe
	float yBedProbePoints[MaxProbePoints];								// The Y coordinates of the points on the bed at which to probe
	float zBedProbePoints[MaxProbePoints];								// The Z coordinates of the points on the bed that were probed
	uint8_t probePointSet[MaxProbePoints];								// Has the XY of this point been set? Has the Z been probed?

	// Variables used to do 3-point compensation
	float aX, aY, aC; 													// Bed plane explicit equation z' = z + aX*x + aY*y + aC

	// Variables used to do 4-point compensation
	float xRectangle, yRectangle;										// The side lengths of the rectangle used for second-degree bed compensation

	// Variables used to do 5-point compensation
	float baryXBedProbePoints[5];										// The X coordinates of the triangle corner points
	float baryYBedProbePoints[5];										// The Y coordinates of the triangle corner points
	float baryZBedProbePoints[5];										// The Z coordinates of the triangle corner points
};

#endif /* SRC_MOVEMENT_RANDOMPROBEPOINTSET_H_ */
