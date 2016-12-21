/*
 * Grid.cpp
 *
 *  Created on: 18 Nov 2016
 *      Author: David
 */

#include "Grid.h"
#include "RepRapFirmware.h"
#include <cmath>

const char *GridDefinition::HeightMapLabelLine = "xmin,xmax,ymin,ymax,radius,spacing,xnum,ynum";

// Initialise the grid to be invalid
GridDefinition::GridDefinition()
	: xMin(0.0), xMax(-1.0), yMin(0.0), yMax(-1.0), radius(-1.0), spacing(DefaultGridSpacing),
	  numX(0), numY(0), recipSpacing(1.0/spacing), isValid(false)
{
}

GridDefinition::GridDefinition(const float xRange[2], const float yRange[2], float pRadius, float pSpacing)
	: xMin(xRange[0]), xMax(xRange[1]), yMin(yRange[0]), yMax(yRange[1]), radius(pRadius), spacing(pSpacing), recipSpacing(1.0/spacing)
{
	numX = (xMax - xMin >= MinRange && spacing >= MinSpacing) ? (uint32_t)((xMax - xMin) * recipSpacing) + 1 : 0;
	numY = (yMax - yMin >= MinRange && spacing >= MinSpacing) ? (uint32_t)((yMax - yMin) * recipSpacing) + 1 : 0;
	CheckValidity();

}

void GridDefinition::CheckValidity()
{
	isValid = NumPoints() != 0 && NumPoints() <= MaxGridProbePoints && (radius < 0.0 || radius >= 1.0);
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
void GridDefinition::PrintParameters(StringRef& s) const
{
	s.catf("X%.1f:%.1f, Y%.1f:%.1f, radius %.1f, spacing %.1f, %d points", xMin, xMax, yMin, yMax, radius, spacing, NumPoints());
}

// Write the parameter label line to a string
void GridDefinition::WriteHeadingAndParameters(StringRef& s) const
{
	s.printf("%s\n%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%u\n", HeightMapLabelLine, xMin, xMax, yMin, yMax, radius, spacing, numX, numY);
}

// Check the parameter label line returning true if correct
/*static*/ bool GridDefinition::CheckHeading(const StringRef& s)
{
	return StringStartsWith(s.Pointer(), HeightMapLabelLine);
}

// Read the grid parameters from a string returning true if success
bool GridDefinition::ReadParameters(const StringRef& s)
{
	bool ok = (sscanf(s.Pointer(), "%f,%f,%f,%f,%f,%f,%lu,%lu", &xMin, &xMax, &yMin, &yMax, &radius, &spacing, &numX, &numY) == 8);
	if (ok)
	{
		CheckValidity();
	}
	else
	{
		isValid = false;
	}
	return ok;
}

// Print what is wrong with the grid, appending it to the existing string
void GridDefinition::PrintError(StringRef& r) const
{
	if (spacing < MinSpacing)
	{
		r.cat("Spacing too small");
	}
	else if (numX == 0)
	{
		r.cat("X range too small");
	}
	else if (numY == 0)
	{
		r.cat("Y range too small");
	}
	else if (numX > MaxGridProbePoints || numY > MaxGridProbePoints || NumPoints() > MaxGridProbePoints)	// check X and Y individually in case X*Y overflows
	{
		r.catf("Too many grid points (maximum %d, needed %d)", MaxGridProbePoints, NumPoints());
	}
	else
	{
		// The only thing left is a bad radius
		r.cat("Bad radius");
	}
}

// Increase the version number in the following string whenever we change the format of the height map file.
const char *HeightMap::HeightMapComment = "RepRapFirmware height map file v1";

HeightMap::HeightMap(float *heightStorage) : gridHeights(heightStorage), useMap(false) { }

void HeightMap::SetGrid(const GridDefinition& gd)
{
	useMap = false;
	def = gd;
	ClearGridHeights();
}

void HeightMap::ClearGridHeights()
{
	for (size_t i = 0; i < MaxGridProbePoints/32; ++i)
	{
		gridHeightSet[i] = 0;
	}
}

// Set the height of a grid point
void HeightMap::SetGridHeight(size_t xIndex, size_t yIndex, float height)
{
	size_t index = yIndex * def.numX + xIndex;
	if (index < MaxGridProbePoints)
	{
		gridHeights[index] = height;
		gridHeightSet[index/32] |= 1u << (index & 31u);
	}
}

// Return the minimum number of segments for a move by this X or Y amount
unsigned int HeightMap::GetMinimumSegments(float distance) const
{
	return (distance > 0.0) ? (unsigned int)(distance * def.recipSpacing + 0.4) : 1;
}

// Save the grid to file returning true if an error occurred
bool HeightMap::SaveToFile(FileStore *f) const
{
	char bufferSpace[500];
	StringRef buf(bufferSpace, ARRAY_SIZE(bufferSpace));

	// Write the header comment
	buf.copy(HeightMapComment);
	if (reprap.GetPlatform()->IsDateTimeSet())
	{
		time_t timeNow = reprap.GetPlatform()->GetDateTime();
		const struct tm * const timeInfo = gmtime(&timeNow);
		buf.catf(" generated at %04u-%02u-%02u %02u:%02u",
						timeInfo->tm_year + 1900, timeInfo->tm_mon, timeInfo->tm_mday, timeInfo->tm_hour, timeInfo->tm_min);
	}
	float mean, deviation;
	(void)GetStatistics(mean, deviation);
	buf.catf(", mean error %.2f, deviation %.2f\n", mean, deviation);
	if (!f->Write(buf.Pointer()))
	{
		return true;
	}

	// Write the grid parameters
	def.WriteHeadingAndParameters(buf);
	if (!f->Write(buf.Pointer()))
	{
		return true;
	}

	// Write the grid heights. We use a fixed field with of 6 characters to make is easier to view.
	uint32_t index = 0;
	for (uint32_t i = 0; i < def.numY; ++i)
	{
		buf.Clear();
		for (uint32_t j = 0; j < def.numX; ++j)
		{
			if (j != 0)
			{
				buf.cat(',');
			}
			if (IsHeightSet(index))
			{
				buf.catf("%7.3f%", gridHeights[index]);
			}
			else
			{
				buf.cat("      0");				// write 0 with no decimal point where we didn't probe, so we can tell when we reload it
			}
			++index;
		}
		buf.cat('\n');
		if (!f->Write(buf.Pointer()))
		{
			return true;
		}
	}

	return false;
}

// Load the grid from file, returning true if an error occurred with the error reason appended to the buffer
bool HeightMap::LoadFromFile(FileStore *f, StringRef& r)
{
	const size_t MaxLineLength = 200;						// maximum length of a line in the height map file
	const char* const readFailureText = "failed to read line from file";
	char buffer[MaxLineLength + 1];
	StringRef s(buffer, ARRAY_SIZE(buffer));

	ClearGridHeights();
	GridDefinition newGrid;

	if (f->ReadLine(buffer, sizeof(buffer)) <= 0)
	{
		r.cat(readFailureText);
	}
	else if (!StringStartsWith(buffer, HeightMapComment))	// check the version line is as expected
	{
		r.cat("bad header line or wrong version header");
	}
	else if (f->ReadLine(buffer, sizeof(buffer)) <= 0)
	{
		r.cat(readFailureText);
	}
	else if (!GridDefinition::CheckHeading(s))				// check the label line is as expected
	{
		r.cat("bad label line");
	}
	else if (f->ReadLine(buffer, sizeof(buffer)) <= 0)		// read the height map parameters
	{
		r.cat(readFailureText);
	}
	else if (!newGrid.ReadParameters(s))
	{
		r.cat("failed to parse grid parameters");
	}
	else if (!newGrid.IsValid())
	{
		r.cat("invalid grid");
	}
	else
	{
		SetGrid(newGrid);
		for (uint32_t row = 0; row < def.numY; ++row)		// read the grid a row at a time
		{
			if (f->ReadLine(buffer, sizeof(buffer)) <= 0)
			{
				r.cat(readFailureText);
				return true;								// failed to read a line
			}
			const char *p = buffer;
			for (uint32_t col = 0; col < def.numX; ++col)
			{
				if (*p == '0' && (p[1] == ',' || p[1] == 0))
				{
					// Values of 0 with no decimal places in un-probed values, so leave the point set as not valid
					++p;
				}
				else
				{
					char* np = nullptr;
					const float f = strtod(p, &np);
					if (np == p)
					{
						r.catf("number expected at line %u column %d", row + 3, (p - buffer) + 1);
						return true;						// failed to read a number
					}
					SetGridHeight(col, row, f);
					p = np;
				}
				if (*p == ',')
				{
					++p;
				}
			}
		}
		return false;										// success!
	}
	return true;											// an error occurred
}

// Return number of points probed, mean and RMS deviation
unsigned int HeightMap::GetStatistics(float& mean, float& deviation) const
{
	double heightSum = 0.0, heightSquaredSum = 0.0;
	unsigned int numProbed = 0;
	for (uint32_t i = 0; i < def.NumPoints(); ++i)
	{
		if (IsHeightSet(i))
		{
			++numProbed;
			const double heightError = (double)gridHeights[i];
			heightSum += heightError;
			heightSquaredSum += heightError * heightError;
		}
	}
	if (numProbed == 0)
	{
		mean = deviation = 0.0;
	}
	else
	{
		mean = (float)(heightSum/numProbed);
		deviation = (float)sqrt(((heightSquaredSum * numProbed) - (heightSum * heightSum)))/numProbed;
	}
	return numProbed;
}

void HeightMap::UseHeightMap(bool b)
{
	useMap = b && def.IsValid();
}

// Compute the height error at the specified point
float HeightMap::GetInterpolatedHeightError(float x, float y) const
{
	if (!useMap)
	{
		return 0.0;
	}

	const float xf = (x - def.xMin) * def.recipSpacing;
	const float xFloor = floor(xf);
	const int32_t xIndex = (int32_t)xFloor;
	const float yf = (y - def.yMin) * def.recipSpacing;
	const float yFloor = floor(yf);
	const int32_t yIndex = (int32_t)yFloor;

	if (xIndex < 0)
	{
		if (yIndex < 0)
		{
			// We are off the bottom left corner of the grid
			return GetHeightError(0, 0);
		}
		else if (yIndex >= (int)def.numY)
		{
			return GetHeightError(0, def.numY);
		}
		else
		{
			return InterpolateY(0, yIndex, yf - yFloor);
		}
	}
	else if (xIndex >= (int)def.numX)
	{
		if (yIndex < 0)
		{
			// We are off the bottom left corner of the grid
			return GetHeightError(def.numX, 0);
		}
		else if (yIndex >= (int)def.numY)
		{
			return GetHeightError(def.numX, def.numY);
		}
		else
		{
			return InterpolateY(def.numX, yIndex, yf - yFloor);
		}
	}
	else
	{
		if (yIndex < 0)
		{
			// We are off the bottom left corner of the grid
			return InterpolateX(xIndex, 0, xf - xFloor);
		}
		else if (yIndex >= (int)def.numY)
		{
			return InterpolateX(xIndex, def.numY, xf - xFloor);
		}
		else
		{
			return InterpolateXY(xIndex, yIndex, xf - xFloor, yf - yFloor);
		}
	}
}

float HeightMap::GetHeightError(uint32_t xIndex, uint32_t yIndex) const
{
	const uint32_t index = GetMapIndex(xIndex, yIndex);
	return (IsHeightSet(index)) ? gridHeights[index] : 0.0;
}

float HeightMap::InterpolateX(uint32_t xIndex, uint32_t yIndex, float xFrac) const
{
	const uint32_t index1 = GetMapIndex(xIndex, yIndex);
	return Interpolate2(index1, index1 + 1, xFrac);
}

float HeightMap::InterpolateY(uint32_t xIndex, uint32_t yIndex, float yFrac) const
{
	const uint32_t index1 = GetMapIndex(xIndex, yIndex);
	return Interpolate2(index1, index1 + def.numX, yFrac);
}

float HeightMap::Interpolate2(uint32_t index1, uint32_t index2, float frac) const
{
	const bool b1 = IsHeightSet(index1);
	const bool b2 = IsHeightSet(index2);
	return (b1 && b2) ? (frac * gridHeights[index1]) + ((1.0 - frac) * gridHeights[index2])
			: (b1) ? gridHeights[index1]
			: (b2) ? gridHeights[index2]
			: 0.0;
}

float HeightMap::InterpolateXY(uint32_t xIndex, uint32_t yIndex, float xFrac, float yFrac) const
{
	const uint32_t indexX0Y0 = GetMapIndex(xIndex, yIndex);			// (X0,Y0)
	const uint32_t indexX1Y0 = indexX0Y0 + 1;						// (X1,Y0)
	const uint32_t indexX0Y1 = indexX0Y0 + def.numX;				// (X0 Y1)
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
		return (xFrac * gridHeights[indexX1Y0]) + ((1.0 - xFrac) * gridHeights[indexX0Y0]);
	case 4:		// (X0,Y1) defined
		return gridHeights[indexX0Y1];
	case 5:		// (X0,Y0) and (X0,Y1) defined
		return (yFrac * gridHeights[indexX0Y1]) + ((1.0 - yFrac) * gridHeights[indexX0Y0]);
	case 6:		// (X1,Y0) and (X0,Y1) defined - diagonal interpolation
		return (((xFrac + 1.0 - yFrac) * gridHeights[indexX1Y0]) + ((yFrac + 1.0 - xFrac) * gridHeights[indexX0Y1]))/2;
	case 7:		// (X0,Y0), (X1,Y0) and (X0,Y1) defined - 3-way interpolation
		return InterpolateCorner(indexX0Y0, indexX1Y0, indexX0Y1, xFrac, yFrac);
	case 8:		// (X1,Y1) defined
		return gridHeights[indexX1Y1];
	case 9:		// (X0,Y0) and (X1,Y1) defined - diagonal interpolation
		return ((xFrac + yFrac) * gridHeights[indexX1Y1]) + ((2.0 - (xFrac + yFrac)) * gridHeights[indexX0Y0])/2;
	case 10:	// (X1,Y0) and (X1,Y1) defined
		return (yFrac * gridHeights[indexX1Y1]) + ((1.0 - yFrac) * gridHeights[indexX1Y0]);
	case 11:	// (X0,Y0), (X1,Y0) and (X1,Y1) defined - 3-way interpolation
		return InterpolateCorner(indexX1Y0, indexX0Y0, indexX1Y1, xFrac, yFrac);
	case 12:	// (X0,Y1) and (X1,Y1) defined
		return (xFrac * gridHeights[indexX1Y1]) + ((1.0 - xFrac) * gridHeights[indexX0Y1]);
	case 13:	// (X0,Y0), (X0,Y1) and (X1,Y1) defined - 3-way interpolation
		return InterpolateCorner(indexX0Y1, indexX1Y1, indexX0Y0, xFrac, 1.0 - yFrac);
	case 14:	// (X1,Y0), (X0,Y1) and (X1,Y1) defined - 3-way interpolation
		return InterpolateCorner(indexX1Y1, indexX0Y1, indexX1Y0, 1.0 - xFrac, 1.0 - yFrac);
	case 15:	// All points defined
		{
			const float xyFrac = xFrac * yFrac;
			return (gridHeights[indexX0Y0] * (1.0 - xFrac - yFrac + xyFrac))
				 + (gridHeights[indexX1Y0] * (xFrac - xyFrac))
				 + (gridHeights[indexX0Y1] * (yFrac - xyFrac))
				 + (gridHeights[indexX1Y1] * xyFrac);
		}
	}
}

float HeightMap::InterpolateCorner(uint32_t cornerIndex, uint32_t indexX, uint32_t indexY, float xFrac, float yFrac) const
{
	return ((xFrac * gridHeights[indexX]) + (yFrac * gridHeights[indexY]) + ((2.0 - xFrac - yFrac) * gridHeights[cornerIndex]))/2;
}

// End
