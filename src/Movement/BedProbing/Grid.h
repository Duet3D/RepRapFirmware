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

class DataTransfer;
class Deviation;

// This class defines the bed probing grid
class GridDefinition
{
public:
	friend class DataTransfer;
	friend class HeightMap;

	GridDefinition() noexcept;

	uint32_t NumXpoints() const noexcept { return numX; }
	uint32_t NumYpoints() const noexcept { return numY; }
	uint32_t NumPoints() const noexcept { return numX * numY; }
	float GetXCoordinate(unsigned int xIndex) const noexcept;
	float GetYCoordinate(unsigned int yIndex) const noexcept;
	bool IsInRadius(float x, float y) const noexcept;
	bool IsValid() const noexcept { return isValid; }

	bool Set(const float xRange[2], const float yRange[2], float pRadius, const float pSpacings[2]) noexcept;
	void PrintParameters(const StringRef& r) const noexcept;
	void WriteHeadingAndParameters(const StringRef& r) const noexcept;
	static int CheckHeading(const StringRef& s) noexcept;
	bool ReadParameters(const StringRef& s, int version) noexcept;

	void PrintError(float originalXrange, float originalYrange, const StringRef& r) const noexcept
	pre(!IsValid());

private:
	void CheckValidity() noexcept;

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
	HeightMap() noexcept;

	const GridDefinition& GetGrid() const noexcept { return def; }
	void SetGrid(const GridDefinition& gd) noexcept;

	float GetInterpolatedHeightError(float x, float y) const noexcept;			// Compute the interpolated height error at the specified point
	void ClearGridHeights() noexcept;											// Clear all grid height corrections
	void SetGridHeight(size_t xIndex, size_t yIndex, float height) noexcept;	// Set the height of a grid point
	void SetGridHeight(size_t index, float height) noexcept;					// Set the height of a grid point

#if HAS_MASS_STORAGE
	bool SaveToFile(FileStore *f, const char *fname, float zOffset) noexcept	// Save the grid to file returning true if an error occurred
	pre(IsValid());
	bool LoadFromFile(FileStore *f, const char *fname, const StringRef& r) noexcept;	// Load the grid from file returning true if an error occurred
	const char *GetFileName() const noexcept { return fileName.c_str(); }
#endif

#if HAS_LINUX_INTERFACE
	void SaveToArray(float *array, float zOffset) const noexcept				// Save the grid Z coordinates to an array
	pre(IsValid());
#endif

	unsigned int GetMinimumSegments(float deltaX, float deltaY) const noexcept;	// Return the minimum number of segments for a move by this X or Y amount

	bool UseHeightMap(bool b) noexcept;
	bool UsingHeightMap() const noexcept { return useMap; }

	unsigned int GetStatistics(Deviation& deviation, float& minError, float& maxError) const noexcept;
																	// Return number of points probed, mean and RMS deviation, min and max error
	void ExtrapolateMissing() noexcept;								// Extrapolate missing points to ensure consistency

private:
	static const char * const HeightMapComment;						// The start of the comment we write at the start of the height map file

	GridDefinition def;
	float gridHeights[MaxGridProbePoints];							// The Z coordinates of the points on the bed that were probed
	LargeBitmap<MaxGridProbePoints> gridHeightSet;					// Bitmap of which heights are set
#if HAS_MASS_STORAGE
	String<MaxFilenameLength> fileName;								// The name of the file that this height map was loaded from or saved to
#endif
	bool useMap;													// True to do bed compensation

	uint32_t GetMapIndex(uint32_t xIndex, uint32_t yIndex) const noexcept { return (yIndex * def.NumXpoints()) + xIndex; }

	float InterpolateXY(uint32_t xIndex, uint32_t yIndex, float xFrac, float yFrac) const noexcept;
};

#endif /* SRC_MOVEMENT_GRID_H_ */
