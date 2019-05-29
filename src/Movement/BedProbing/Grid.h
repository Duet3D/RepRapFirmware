/*
 * Grid.h
 *
 *  Created on: 18 Nov 2016
 *      Author: David
 */

#ifndef SRC_MOVEMENT_GRID_H_
#define SRC_MOVEMENT_GRID_H_

#include "RepRapFirmware.h"
#include "ObjectModel/ObjectModel.h"

// This class defines the bed probing grid
class GridDefinition INHERIT_OBJECT_MODEL
{
public:
	friend class HeightMap;

	GridDefinition();

	uint32_t NumXpoints() const { return numX; }
	uint32_t NumYpoints() const { return numY; }
	uint32_t NumPoints() const { return numX * numY; }
	float GetXCoordinate(unsigned int xIndex) const;
	float GetYCoordinate(unsigned int yIndex) const;
	bool IsInRadius(float x, float y) const;
	bool IsValid() const { return isValid; }

	bool Set(const float xRange[2], const float yRange[2], float pRadius, const float pSpacings[2]);
	void PrintParameters(const StringRef& r) const;
	void WriteHeadingAndParameters(const StringRef& r) const;
	static int CheckHeading(const StringRef& s);
	bool ReadParameters(const StringRef& s, int version);

	void PrintError(float originalXrange, float originalYrange, const StringRef& r) const
	pre(!IsValid());

protected:
	DECLARE_OBJECT_MODEL

private:
	void CheckValidity();

	static constexpr float MinSpacing = 0.1;						// The minimum point spacing allowed
	static constexpr float MinRange = 1.0;							// The minimum X and Y range allowed
	static const char * const HeightMapLabelLines[];				// The line we write to the height map file listing the parameter names

	// Primary parameters
	float xMin, xMax, yMin, yMax;									// The edges of the grid for G29 probing
	float radius;													// The grid radius to probe
	float xSpacing, ySpacing;										// The spacing of the grid probe points

	// Derived parameters
	uint32_t numX, numY;
	float recipXspacing, recipYspacing;
	bool isValid;
};

// Class to represent the height map
class HeightMap
{
public:
	HeightMap();

	const GridDefinition& GetGrid() const { return def; }
	void SetGrid(const GridDefinition& gd);

	float GetInterpolatedHeightError(float x, float y) const;		// Compute the interpolated height error at the specified point
	void ClearGridHeights();										// Clear all grid height corrections
	void SetGridHeight(size_t xIndex, size_t yIndex, float height);	// Set the height of a grid point

	bool SaveToFile(FileStore *f, float zOffset) const				// Save the grid to file returning true if an error occurred
	pre(IsValid());

	bool LoadFromFile(FileStore *f, const StringRef& r);			// Load the grid from file returning true if an error occurred

	unsigned int GetMinimumSegments(float deltaX, float deltaY) const;	// Return the minimum number of segments for a move by this X or Y amount

	bool UseHeightMap(bool b);
	bool UsingHeightMap() const { return useMap; }

	unsigned int GetStatistics(float& mean, float& deviation, float& minError, float& maxError) const;
																	// Return number of points probed, mean and RMS deviation, min and max error
	void ExtrapolateMissing();										// Extrapolate missing points to ensure consistency

private:
	static const char * const HeightMapComment;						// The start of the comment we write at the start of the height map file

	GridDefinition def;
	float gridHeights[MaxGridProbePoints];							// The Z coordinates of the points on the bed that were probed
	uint32_t gridHeightSet[(MaxGridProbePoints + 31)/32];			// Bitmap of which heights are set
	bool useMap;													// True to do bed compensation

	uint32_t GetMapIndex(uint32_t xIndex, uint32_t yIndex) const { return (yIndex * def.NumXpoints()) + xIndex; }
	bool IsHeightSet(uint32_t index) const { return (gridHeightSet[index/32] & (1 << (index & 31))) != 0; }

	float InterpolateXY(uint32_t xIndex, uint32_t yIndex, float xFrac, float yFrac) const;
};

#endif /* SRC_MOVEMENT_GRID_H_ */
