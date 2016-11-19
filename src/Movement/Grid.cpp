/*
 * Grid.cpp
 *
 *  Created on: 18 Nov 2016
 *      Author: David
 */

#include "Grid.h"
#include "Configuration.h"
#include <cmath>

// Initialise the grid to be invalid
GridDefinition::GridDefinition()
	: xMin(0.0), xMax(-1.0), yMin(0.0), yMax(-1.0), radius(-1.0), spacing(DefaultGridSpacing), gridHeights(nullptr),
	  numX(0), numY(0), recipSpacing(1.0/spacing), isValid(false)
{
}

GridDefinition::GridDefinition(const float xRange[2], const float yRange[2], float pRadius, float pSpacing)
	: xMin(xRange[0]), xMax(xRange[1]), yMin(yRange[0]), yMax(yRange[1]), radius(pRadius), spacing(pSpacing)
{
	numX = (xMax - xMin >= MinRange && spacing >= MinSpacing) ? (uint32_t)((xMax - xMin)/spacing) + 1 : 0;
	numY = (yMax - yMin >= MinRange && spacing >= MinSpacing) ? (uint32_t)((xMax - xMin)/spacing) + 1 : 0;
	isValid = NumPoints() != 0 && NumPoints() <= MaxGridProbePoints && (radius < 0.0 || radius >= 1.0);
}

void GridDefinition::SetStorage(const float *heightStorage, const uint32_t *heightSetStorage)
{
	gridHeights = heightStorage;
	gridHeightSet = heightSetStorage;
}

float GridDefinition::GetXCoordinate(unsigned int xIndex) const
{
	return xMin + (xIndex * spacing);
}

float GridDefinition::GetYCoordinate(unsigned int yIndex) const
{
	return yMin + (yIndex * spacing);
}

bool GridDefinition::IsInRadius(float x, float y) const
{
	return radius < 0.0 || x * x + y * y < radius * radius;
}

// Append the grid parameters to the end of a string
void GridDefinition::PrintParameters(StringRef& r) const
{
	r.catf("X%.1f:%.1f, Y%.1f:%.1f, radius %.1f, spacing %.1f, %d points", xMin, xMax, yMin, yMax, radius, spacing, NumPoints());
}

// Print what is wrong with the grid
void GridDefinition::PrintError(StringRef& r) const
{
	if (spacing < MinSpacing)
	{
		r.cat("Spacing too small");
	}
	else if (NumXpoints() == 0)
	{
		r.cat("X range too small");
	}
	else if (NumYpoints() == 0)
	{
		r.cat("Y range too small");
	}
	else if (NumPoints() > MaxGridProbePoints)
	{
		r.catf("Too many grid points (maximum %d, needed %d)", MaxGridProbePoints, NumPoints());
	}
	else
	{
		// The only thing left is a bad radius
		r.cat("Bad radius");
	}
}

// Compute the height error at the specified point
float GridDefinition::ComputeHeightError(float x, float y) const
{
	const float xf = (x - xMin) * recipSpacing;
	const float xFloor = floor(xf);
	const int32_t xIndex = (int32_t)xFloor;
	const float yf = (y - yMin) * recipSpacing;
	const float yFloor = floor(yf);
	const int32_t yIndex = (int32_t)yFloor;

	if (xIndex < 0)
	{
		if (yIndex < 0)
		{
			// We are off the bottom left corner of the grid
			return GetHeightError(0, 0);
		}
		else if (yIndex >= (int)NumYpoints())
		{
			return GetHeightError(0, NumYpoints());
		}
		else
		{
			return InterpolateY(0, yIndex, yf - yFloor);
		}
	}
	else if (xIndex >= (int)NumXpoints())
	{
		if (yIndex < 0)
		{
			// We are off the bottom left corner of the grid
			return GetHeightError(NumXpoints(), 0);
		}
		else if (yIndex >= (int)NumYpoints())
		{
			return GetHeightError(NumXpoints(), NumYpoints());
		}
		else
		{
			return InterpolateY(NumXpoints(), yIndex, yf - yFloor);
		}
	}
	else
	{
		if (yIndex < 0)
		{
			// We are off the bottom left corner of the grid
			return InterpolateX(xIndex, 0, xf - xFloor);
		}
		else if (yIndex >= (int)NumYpoints())
		{
			return InterpolateX(xIndex, NumYpoints(), xf - xFloor);
		}
		else
		{
			return InterpolateXY(xIndex, yIndex, xf - xFloor, yf - yFloor);
		}
	}
}

float GridDefinition::GetHeightError(uint32_t xIndex, uint32_t yIndex) const
{
	const uint32_t index = GetMapIndex(xIndex, yIndex);
	return (IsHeightSet(index)) ? gridHeights[index] : 0.0;
}

float GridDefinition::InterpolateX(uint32_t xIndex, uint32_t yIndex, float xFrac) const
{
	const uint32_t index1 = GetMapIndex(xIndex, yIndex);
	return Interpolate2(index1, index1 + 1, xFrac);
}

float GridDefinition::InterpolateY(uint32_t xIndex, uint32_t yIndex, float yFrac) const
{
	const uint32_t index1 = GetMapIndex(xIndex, yIndex);
	return Interpolate2(index1, index1 + numX, yFrac);
}

float GridDefinition::Interpolate2(uint32_t index1, uint32_t index2, float frac) const
{
	const bool b1 = IsHeightSet(index1);
	const bool b2 = IsHeightSet(index2);
	return (b1 && b2) ? (frac * gridHeights[index1]) + ((1.0 - frac) * gridHeights[index2])
			: (b1) ? gridHeights[index1]
			: (b2) ? gridHeights[index2]
			: 0.0;
}

float GridDefinition::InterpolateXY(uint32_t xIndex, uint32_t yIndex, float xFrac, float yFrac) const
{
	const uint32_t indexX0Y0 = GetMapIndex(xIndex, yIndex);			// (X0,Y0)
	const uint32_t indexX1Y0 = indexX0Y0 + 1;						// (X1,Y0)
	const uint32_t indexX0Y1 = indexX0Y0 + numX;					// (X0 Y1)
	const uint32_t indexX1Y1 = indexX0Y1 + 1;						// (X1,Y1)
	const unsigned int cc = ((unsigned int)IsHeightSet(indexX0Y0) << 0)
							+ ((unsigned int)IsHeightSet(indexX1Y0) << 1)
							+ ((unsigned int)IsHeightSet(indexX0Y1) << 2)
							+ ((unsigned int)IsHeightSet(indexX1Y1) << 3);
	switch(cc)
	{
	case 0:		// no points defined
	default:
		return 0.0;
	case 1:		// (X0,Y0) defined
		return gridHeights[indexX0Y0];
	case 2:		// (X1,Y0) defined
		return gridHeights[indexX1Y0];
	case 3:		// (X0,Y0) and (X1,Y0) defined
		return Interpolate2(indexX0Y0, indexX1Y0, xFrac);
	case 4:		// (X0,Y1) defined
		return gridHeights[indexX0Y1];
	case 5:		// (X0,Y0) and (X0,Y1) defined
		return Interpolate2(indexX0Y0, indexX0Y1, yFrac);
	case 6:		// (X1,Y0) and (X0,Y1) defined - diagonal interpolation
		return DiagonalInterpolate(indexX1Y0, indexX0Y1, 1.0 - xFrac, yFrac);
	case 7:		// (X0,Y0), (X1,Y0) and (X0,Y1) defined - 3-way interpolation
		return Interpolate3(indexX0Y0, indexX1Y0, indexX0Y1, xFrac, yFrac);
	case 8:		// (X1,Y1) defined
		return gridHeights[indexX1Y1];
	case 9:		// (X0,Y0) and (X1,Y1) defined - diagonal interpolation
		return DiagonalInterpolate(indexX0Y0, indexX1Y1, xFrac, yFrac);
	case 10:	// (X1,Y0) and (X1,Y1) defined
		return Interpolate2(indexX1Y0, indexX1Y1, yFrac);
	case 11:	// (X0,Y0), (X1,Y0) and (X1,Y1) defined - 3-way interpolation
		return Interpolate3(indexX1Y0, indexX0Y0, indexX1Y1, xFrac, yFrac);
	case 12:	// (X0,Y1) and (X1,Y1) defined
		return Interpolate2(indexX0Y1, indexX1Y1, yFrac);
	case 13:	// (X0,Y0), (X0,Y1) and (X1,Y1) defined - 3-way interpolation
		return Interpolate3(indexX0Y1, indexX1Y1, indexX0Y0, xFrac, 1.0 - yFrac);
	case 14:	// (X1,Y0), (X0,Y1) and (X1,Y1) defined - 3-way interpolation
		return Interpolate3(indexX1Y1, indexX0Y1, indexX1Y0, 1.0 - xFrac, 1.0 - yFrac);
	case 15:	// All points defined
		return Interpolate4(indexX0Y0, indexX1Y0, indexX0Y1, indexX1Y1, xFrac, yFrac);
	}
}

// End
