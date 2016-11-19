/*
 * Grid.h
 *
 *  Created on: 18 Nov 2016
 *      Author: David
 */

#ifndef SRC_MOVEMENT_GRID_H_
#define SRC_MOVEMENT_GRID_H_

#include <cstdint>
#include "ecv.h"
#include "Libraries/General/StringRef.h"

// This class defines the bed probing grid
class GridDefinition
{
public:
	GridDefinition();
	GridDefinition(const float xRange[2], const float yRange[2], float pRadius, float pSpacing);
	void SetStorage(const float *heightStorage, const uint32_t *heightSetStorage);

	uint32_t NumXpoints() const { return numX; }
	uint32_t NumYpoints() const { return numY; }
	uint32_t NumPoints() const { return numX * numY; }
	float GetXCoordinate(unsigned int xIndex) const;
	float GetYCoordinate(unsigned int yIndex) const;
	bool IsInRadius(float x, float y) const;
	bool IsValid() const { return isValid; }

	void PrintParameters(StringRef& r) const;
	void PrintError(StringRef& r) const
	pre(!IsValid());

	float ComputeHeightError(float x, float y) const		// Compute the height error at the specified point
	pre(IsValid(); gridHeights != nullptr; gridHeights.upb >= NumPoints());

private:
	static constexpr float MinSpacing = 0.1;			// The minimum point spacing allowed
	static constexpr float MinRange = 1.0;				// The minimum X and Y range allowed

	// Primary parameters
	float xMin, xMax, yMin, yMax;						// The edges of the grid for G29 probing
	float radius;										// The grid radius to probe
	float spacing;										// The spacing of the grid probe points
	const float *gridHeights;							// The map of grid heights
	const uint32_t *gridHeightSet;							// Bitmap of which heights are set

	// Derived parameters
	uint32_t numX, numY;
	float recipSpacing;
	bool isValid;

	uint32_t GetMapIndex(uint32_t xIndex, uint32_t yIndex) const { return (yIndex * numX) + xIndex; }
	bool IsHeightSet(uint32_t index) const { return (gridHeightSet[index/32] & (1 << (index & 31))) != 0; }
	float GetHeightError(uint32_t xIndex, uint32_t yIndex) const;
	float InterpolateX(uint32_t xIndex, uint32_t yIndex, float xFrac) const;
	float InterpolateY(uint32_t xIndex, uint32_t yIndex, float yFrac) const;
	float InterpolateXY(uint32_t xIndex, uint32_t yIndex, float xFrac, float yFrac) const;
	float Interpolate2(uint32_t index1, uint32_t index2, float frac) const;
	float DiagonalInterpolate(uint32_t index1, uint32_t index2, float xFrac, float yFrac) const;
	float Interpolate3(uint32_t cornerIndex, uint32_t indexX, uint32_t indexY, float xFrac, float yFrac) const;
	float Interpolate4(uint32_t index1, uint32_t index2, uint32_t index3, uint32_t index4, float xFrac, float yFrac) const;
};

#endif /* SRC_MOVEMENT_GRID_H_ */
