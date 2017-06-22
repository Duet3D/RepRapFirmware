/*
 * Grid.cpp
 *
 *  Created on: 18 Nov 2016
 *      Author: David
 */

#include "Grid.h"
#include "Platform.h"
#include "RepRap.h"
#include "Storage/FileStore.h"
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
	isValid = NumPoints() != 0 && NumPoints() <= MaxGridProbePoints
			&& (radius < 0.0 || radius >= 1.0)
			&& NumXpoints() <= MaxXGridPoints;
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
		recipSpacing = 1.0 / spacing;
	}
	else
	{
		isValid = false;
	}
	return ok;
}

// Print what is wrong with the grid, appending it to the existing string
void GridDefinition::PrintError(float originalXrange, float originalYrange, StringRef& r) const
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
	else if (   numX > MaxXGridPoints
			 || numX > MaxGridProbePoints || numY > MaxGridProbePoints		// check X and Y individually in case X*Y overflows
			 || NumPoints() > MaxGridProbePoints
			)
	{
		const float totalRange = originalXrange + originalYrange;
		const float area = originalXrange * originalYrange;
		const float minSpacing = (totalRange + sqrtf(fsquare(totalRange) + 4.0 * (MaxGridProbePoints - 1) * area))/(2.0 * (MaxGridProbePoints - 1));
		const float minXspacing = originalXrange/(MaxXGridPoints - 1);
		r.catf("Too many grid points; suggest increase spacing to %.1fmm", max<float>(minSpacing, minXspacing));
	}
	else
	{
		// The only thing left is a bad radius
		r.cat("Bad radius");
	}
}

// Increase the version number in the following string whenever we change the format of the height map file.
const char *HeightMap::HeightMapComment = "RepRapFirmware height map file v1";

HeightMap::HeightMap() : useMap(false) { }

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
// Note that deltaX and deltaY may be negative
unsigned int HeightMap::GetMinimumSegments(float deltaX, float deltaY) const
{
	float distance = max<float>(fabs(deltaX), fabs(deltaY));
	return (distance > 0.0) ? (unsigned int)(distance * def.recipSpacing + 0.4) : 1;
}

// Save the grid to file returning true if an error occurred
bool HeightMap::SaveToFile(FileStore *f) const
{
	char bufferSpace[500];
	StringRef buf(bufferSpace, ARRAY_SIZE(bufferSpace));

	// Write the header comment
	buf.copy(HeightMapComment);
	if (reprap.GetPlatform().IsDateTimeSet())
	{
		time_t timeNow = reprap.GetPlatform().GetDateTime();
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
	const size_t MaxLineLength = (MaxXGridPoints * 8) + 2;						// maximum length of a line in the height map file, need 8 characters per grid point
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
		ExtrapolateMissing();
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

// Try to turn mesh compensation on or off and report the state achieved
bool HeightMap::UseHeightMap(bool b)
{
	useMap = b && def.IsValid();
	return useMap;
}

// Compute the height error at the specified point
float HeightMap::GetInterpolatedHeightError(float x, float y) const
{
	if (!useMap)
	{
		return 0.0;
	}

	// Last grid point
	const float xLast = def.xMin + (def.numX-1)*def.spacing;
	const float yLast = def.yMin + (def.numY-1)*def.spacing;

	// Clamp to rectangle so InterpolateXY will always have valid parameters
	const float fEPSILON = 0.01;
	if (x < def.xMin) { x = def.xMin; }
	if (y < def.yMin) {	y = def.yMin; }
	if (x > xLast -fEPSILON) { x = xLast -fEPSILON; }
	if (y > yLast -fEPSILON) { y = yLast -fEPSILON; }


	const float xf = (x - def.xMin) * def.recipSpacing;
	const float xFloor = floor(xf);
	const int32_t xIndex = (int32_t)xFloor;
	const float yf = (y - def.yMin) * def.recipSpacing;
	const float yFloor = floor(yf);
	const int32_t yIndex = (int32_t)yFloor;

	return InterpolateXY(xIndex, yIndex, xf - xFloor, yf - yFloor);
}

float HeightMap::InterpolateXY(uint32_t xIndex, uint32_t yIndex, float xFrac, float yFrac) const
{
	const uint32_t indexX0Y0 = GetMapIndex(xIndex, yIndex);			// (X0,Y0)
	const uint32_t indexX1Y0 = indexX0Y0 + 1;						// (X1,Y0)
	const uint32_t indexX0Y1 = indexX0Y0 + def.numX;				// (X0 Y1)
	const uint32_t indexX1Y1 = indexX0Y1 + 1;						// (X1,Y1)

	const float xyFrac = xFrac * yFrac;
	return (gridHeights[indexX0Y0] * (1.0 - xFrac - yFrac + xyFrac))
			+ (gridHeights[indexX1Y0] * (xFrac - xyFrac))
			+ (gridHeights[indexX0Y1] * (yFrac - xyFrac))
			+ (gridHeights[indexX1Y1] * xyFrac);
}

void HeightMap::ExtrapolateMissing()
{
	//1: calculating the bed plane by least squares fit
	//2: filling in missing points

	//algorithm: http://www.ilikebigbits.com/blog/2015/3/2/plane-from-points
	float sumX = 0, sumY = 0, sumZ = 0;
	int n = 0;
	for (uint32_t iY = 0; iY < def.numY; iY++)
	{
		for (uint32_t iX = 0; iX < def.numX; iX++)
		{
			const uint32_t index = GetMapIndex(iX, iY);
			if (IsHeightSet(index))
			{
				const float fX = (def.spacing * iX) + def.xMin;
				const float fY = (def.spacing * iY) + def.yMin;
				const float fZ = gridHeights[index];

				n++;
				sumX += fX; sumY += fY; sumZ += fZ;
			}
		}
	}

	const float invN = 1.0 / float(n);
	const float centX = sumX * invN, centY = sumY * invN, centZ = sumZ * invN;

	// Calc full 3x3 covariance matrix, excluding symmetries:
	float xx = 0.0; float xy = 0.0; float xz = 0.0;
	float yy = 0.0; float yz = 0.0; float zz = 0.0;

	for (uint32_t iY = 0; iY < def.numY; iY++)
	{
		for (uint32_t iX = 0; iX < def.numX; iX++)
		{
			const uint32_t index = GetMapIndex(iX, iY);
			if (IsHeightSet(index))
			{
				const float fX = (def.spacing * iX) + def.xMin;
				const float fY = (def.spacing * iY) + def.yMin;
				const float fZ = gridHeights[index];

				const float rX = fX - centX;
				const float rY = fY - centY;
				const float rZ = fZ - centZ;

				xx += rX * rX;
				xy += rX * rY;
				xz += rX * rZ;
				yy += rY * rY;
				yz += rY * rZ;
				zz += rZ * rZ;
			}
		}
	}

	const float detZ = xx*yy - xy*xy;
	if (detZ <= 0)
	{
		// Not a valid plane (or a vertical one)
		return;
	}

	// Plane equation: ax+by+cz=d -> z = (d-(ax+by))/c
	float a = (yz*xy - xz*yy) / detZ;
	float b = (xz*xy - yz*xx) / detZ;
	const float invC = sqrtf(a*a + b*b + 1.0);
	const float normLenInv = 1.0 / invC;
	a *= normLenInv;
	b *= normLenInv;
	const float c = normLenInv;
	const float d = centX*a + centY*b + centZ*c;

	// Fill in the blanks
	for (uint32_t iY = 0; iY < def.numY; iY++)
	{
		for (uint32_t iX = 0; iX < def.numX; iX++)
		{
			const uint32_t index = GetMapIndex(iX, iY);
			if (!IsHeightSet(index))
			{
				const float fX = (def.spacing * iX) + def.xMin;
				const float fY = (def.spacing * iY) + def.yMin;
				const float fZ = (d - (a * fX + b * fY)) * invC;
				gridHeights[index] = fZ;	// fill in Z but don't mark it as set so we can always differentiate between measured and extrapolated
			}
		}
	}
}

// End
