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
class GridDefinition INHERIT_OBJECT_MODEL
{
public:
	friend class DataTransfer;
	friend class HeightMap;

	GridDefinition() noexcept;

	char GetAxisLetter(size_t axis) const noexcept pre(axis < 2) { return letters[axis]; }
	uint8_t GetAxisNumber(size_t axis) const noexcept pre(axis < 2) { return axisNumbers[axis]; }
	float GetMin(size_t axis) const noexcept pre(axis < 2) { return mins[axis]; }
	float GetMax(size_t axis) const noexcept pre(axis < 2) { return maxs[axis]; }
	float GetSpacing(size_t axis) const noexcept pre(axis < 2) { return spacings[axis]; }

	uint32_t NumAxisPoints(size_t axis) const noexcept pre(axis < 2) { return nums[axis]; }
	uint32_t NumPoints() const noexcept { return nums[0] * nums[1]; }
	float GetCoordinate(size_t axis, size_t coordinateIndex) pre(axis < 2) const noexcept { return mins[axis] + (coordinateIndex * spacings[axis]); }
	bool IsInRadius(float x, float y) const noexcept;
	bool IsValid() const noexcept { return isValid; }

	bool Set(const char axisLetters[2], const float axis0Range[2], const float axis1Range[2], float pRadius, const float pSpacings[2]) noexcept;
	void PrintParameters(const StringRef& r) const noexcept;
	void WriteHeadingAndParameters(const StringRef& r) const noexcept;
	static int CheckHeading(const StringRef& s) noexcept;
	bool ReadParameters(const StringRef& s, int version) noexcept;

	void PrintError(float originalXrange, float originalYrange, const StringRef& r) const noexcept
	pre(!IsValid());

protected:
	DECLARE_OBJECT_MODEL

private:
	void CheckValidity(bool setNum0Num1) noexcept;

	static constexpr float MinSpacing = 0.1;						// The minimum point spacing allowed
	static constexpr float MinRange = 1.0;							// The minimum X and Y range allowed
	static const char * const HeightMapLabelLines[];				// The line we write to the height map file listing the parameter names

	// Primary parameters
	char letters[2];												// Axis letters for this grid
	float mins[2], maxs[2];											// The edges of the grid for G29 probing
	float radius;													// The grid radius to probe
	float spacings[2];												// The spacings of the grid probe points

	// Derived parameters
	uint8_t axisNumbers[2];											// Axis numbers for this grid
	uint32_t nums[2];												// Number of probe points in each direction
	float recipAxisSpacings[2];										// Reciprocals of the axis spacings
	bool isValid;
};

// Class to represent the height map
class HeightMap
{
public:
	HeightMap() noexcept;

	const GridDefinition& GetGrid() const noexcept { return def; }
	void SetGrid(const GridDefinition& gd) noexcept;

	float GetInterpolatedHeightError(float axis0, float axis1) const noexcept;			// Compute the interpolated height error at the specified point
	void ClearGridHeights() noexcept;													// Clear all grid height corrections
	void SetGridHeight(size_t axis0Index, size_t axis1Index, float height) noexcept;	// Set the height of a grid point
	void SetGridHeight(size_t index, float height) noexcept;							// Set the height of a grid point

#if HAS_MASS_STORAGE
	bool SaveToFile(FileStore *f, const char *fname, float zOffset) noexcept	// Save the grid to file returning true if an error occurred
	pre(IsValid());
	bool LoadFromFile(FileStore *f, const char *fname, const StringRef& r) noexcept;	// Load the grid from file returning true if an error occurred
#endif

#if HAS_MASS_STORAGE || HAS_LINUX_INTERFACE
	const char *GetFileName() const noexcept { return fileName.c_str(); }
#endif

#if HAS_LINUX_INTERFACE
	void SetFileName(const char *name) noexcept;								// Update the filename
	void SaveToArray(float *array, float zOffset) const noexcept				// Save the grid Z coordinates to an array
	pre(IsValid());
#endif

	unsigned int GetMinimumSegments(float deltaAxis0, float deltaAxis1) const noexcept;	// Return the minimum number of segments for a move by this X or Y amount

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
#if HAS_MASS_STORAGE || HAS_LINUX_INTERFACE
	String<MaxFilenameLength> fileName;								// The name of the file that this height map was loaded from or saved to
#endif
	bool useMap;													// True to do bed compensation

	uint32_t GetMapIndex(uint32_t axis0Index, uint32_t axis1Index) const noexcept { return (axis1Index * def.NumAxisPoints(0)) + axis0Index; }

	float InterpolateAxis0Axis1(uint32_t axis0Index, uint32_t axis1Index, float axis0Frac, float axis1Frac) const noexcept;
};

#endif /* SRC_MOVEMENT_GRID_H_ */
