/*
 * Grid.h
 *
 *  Created on: 18 Nov 2016
 *      Author: David
 */

#ifndef SRC_MOVEMENT_GRID_H_
#define SRC_MOVEMENT_GRID_H_

#include <cstdint>
#include "RepRapFirmware.h"
#include "Libraries/General/StringRef.h"

// This class defines the bed probing grid
class GridDefinition
{
public:
	friend class HeightMap;

	GridDefinition();
	GridDefinition(const float xRange[2], const float yRange[2], float pRadius, float pSpacing);

	uint32_t NumXpoints() const { return numX; }
	uint32_t NumYpoints() const { return numY; }
	uint32_t NumPoints() const { return numX * numY; }
	float GetXCoordinate(unsigned int xIndex) const;
	float GetYCoordinate(unsigned int yIndex) const;
	bool IsInRadius(float x, float y) const;
	bool IsValid() const { return isValid; }

	void PrintParameters(StringRef& r) const;
	void WriteHeadingAndParameters(StringRef& r) const;
	static bool CheckHeading(const StringRef& s);
	bool ReadParameters(const StringRef& s);

	void PrintError(StringRef& r) const
	pre(!IsValid());

private:
	void CheckValidity();

	static constexpr float MinSpacing = 0.1;						// The minimum point spacing allowed
	static constexpr float MinRange = 1.0;							// The minimum X and Y range allowed
	static const char *HeightMapLabelLine;							// The line we write to the height map file listing the parameter names

	// Primary parameters
	float xMin, xMax, yMin, yMax;									// The edges of the grid for G29 probing
	float radius;													// The grid radius to probe
	float spacing;													// The spacing of the grid probe points

	// Derived parameters
	uint32_t numX, numY;
	float recipSpacing;
	bool isValid;

};

// Class to represent the height map
class HeightMap
{
public:
	HeightMap(float *heightStorage);

	const GridDefinition& GetGrid() const { return def; }
	void SetGrid(const GridDefinition& gd);

	float GetInterpolatedHeightError(float x, float y) const;		// Compute the interpolated height error at the specified point
	void ClearGridHeights();										// Clear all grid height corrections
	void SetGridHeight(size_t xIndex, size_t yIndex, float height);	// Set the height of a grid point

	bool SaveToFile(FileStore *f) const								// Save the grid to file returning true if an error occurred
	pre(IsValid());

	bool LoadFromFile(FileStore *f, StringRef& r);					// Load the grid from file returning true if an error occurred

	unsigned int GetMinimumSegments(float distance) const;			// Return the minimum number of segments for a move by this X or Y amount

	void UseHeightMap(bool b);
	bool UsingHeightMap() const { return useMap; }

	unsigned int GetStatistics(float& mean, float& deviation) const; // Return number of points probed, mean and RMS deviation

	void ExtrapolateMissing();										//extrapolate missing points to ensure consistency

private:
	static const char *HeightMapComment;							// The start of the comment we write at the start of the height map file

	GridDefinition def;
	float *gridHeights;												// The map of grid heights, must have at least MaxGridProbePoints entries
	uint32_t gridHeightSet[MaxGridProbePoints/32];					// Bitmap of which heights are set
	bool useMap;													// True to do bed compensation

	uint32_t GetMapIndex(uint32_t xIndex, uint32_t yIndex) const { return (yIndex * def.NumXpoints()) + xIndex; }
	bool IsHeightSet(uint32_t index) const { return (gridHeightSet[index/32] & (1 << (index & 31))) != 0; }

	float InterpolateXY(uint32_t xIndex, uint32_t yIndex, float xFrac, float yFrac) const;
};

#endif /* SRC_MOVEMENT_GRID_H_ */
