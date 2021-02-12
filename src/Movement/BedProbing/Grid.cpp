/*
 * Grid.cpp
 *
 *  Created on: 18 Nov 2016
 *      Author: David
 */

#include "Grid.h"
#include "Platform.h"
#include "RepRap.h"
#include <GCodes/GCodes.h>
#include "Storage/FileStore.h"
#include <Math/Deviation.h>

#include <cmath>

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(GridDefinition, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(...) OBJECT_MODEL_FUNC_IF_BODY(GridDefinition, __VA_ARGS__)

constexpr ObjectModelTableEntry GridDefinition::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. GridDefinition members
	{ "axis0Letter",	OBJECT_MODEL_FUNC(self->axis0Letter),			ObjectModelEntryFlags::none },
	{ "axis0Max",		OBJECT_MODEL_FUNC(self->axis0Max, 1),			ObjectModelEntryFlags::none },
	{ "axis0Min",		OBJECT_MODEL_FUNC(self->axis0Min, 1),			ObjectModelEntryFlags::none },
	{ "axis0Number",	OBJECT_MODEL_FUNC(self->axis0Number, 1),		ObjectModelEntryFlags::none },
	{ "axis0Spacing",	OBJECT_MODEL_FUNC(self->axis0Spacing, 1),		ObjectModelEntryFlags::none },
	{ "axis1Letter",	OBJECT_MODEL_FUNC(self->axis1Letter),			ObjectModelEntryFlags::none },
	{ "axis1Max",		OBJECT_MODEL_FUNC(self->axis1Max, 1),			ObjectModelEntryFlags::none },
	{ "axis1Min",		OBJECT_MODEL_FUNC(self->axis1Min, 1),			ObjectModelEntryFlags::none },
	{ "axis1Number",	OBJECT_MODEL_FUNC(self->axis1Number, 1),		ObjectModelEntryFlags::none },
	{ "axis1Spacing",	OBJECT_MODEL_FUNC(self->axis1Spacing, 1),		ObjectModelEntryFlags::none },
	{ "radius",			OBJECT_MODEL_FUNC(self->radius, 1),				ObjectModelEntryFlags::none },
};

constexpr uint8_t GridDefinition::objectModelTableDescriptor[] = { 1, 11 };

DEFINE_GET_OBJECT_MODEL_TABLE(GridDefinition)

#endif

const char * const GridDefinition::HeightMapLabelLines[] =
{
	"xmin,xmax,ymin,ymax,radius,spacing,xnum,ynum",																		// old version label line
	"xmin,xmax,ymin,ymax,radius,xspacing,yspacing,xnum,ynum",															// label line until 3.2
	"axis0Number,axis0Letter,axis0letter,axis1letter,axis0min,axis0max,axis1min,axis1max,radius,axis0spacing,axis1spacing,axis0num,axis1num",	// label line from 3.3
};

// Initialise the grid to be invalid
GridDefinition::GridDefinition() noexcept
	: axis0Number(X_AXIS), axis1Number(Y_AXIS), axis0Letter('X'), axis1Letter('Y'), axis0Min(0.0), axis0Max(-1.0), axis1Min(0.0), axis1Max(-1.0), radius(-1.0), axis0Spacing(0.0), axis1Spacing(0.0)
{
	CheckValidity();		// will flag the grid as invalid
}

// Set the grid parameters ands return true if it is now valid
bool GridDefinition::Set(const uint8_t axesNumbers[2], const char axesLetters[2], const float axis0Range[2], const float axis1Range[2], float pRadius, const float pSpacings[2]) noexcept
{
	axis0Number = axesNumbers[0];
	axis1Number = axesNumbers[1];
	axis0Letter = axesLetters[0];
	axis1Letter = axesLetters[1];
	axis0Min = axis0Range[0];
	axis0Max = axis0Range[1];
	axis1Min = axis1Range[0];
	axis1Max = axis1Range[1];
	radius = pRadius;
	axis0Spacing = pSpacings[0];
	axis1Spacing = pSpacings[1];
	CheckValidity();
	return isValid;
}

// Set up internal variables and check validity of the grid.
// numAxis0, numAxis1 are always set up, but recipAxis0spacing, recipAxis1spacing only if the grid is valid
void GridDefinition::CheckValidity() noexcept
{
	numAxis0 = (axis0Max - axis0Min >= MinRange && axis0Spacing >= MinSpacing) ? (uint32_t)((axis0Max - axis0Min)/axis0Spacing) + 1 : 0;
	numAxis1 = (axis1Max - axis1Min >= MinRange && axis1Spacing >= MinSpacing) ? (uint32_t)((axis1Max - axis1Min)/axis1Spacing) + 1 : 0;

	const size_t axis0NumForLetter = reprap.GetGCodes().GetAxisNumberForLetter(axis0Letter);
	const size_t axis1NumForLetter = reprap.GetGCodes().GetAxisNumberForLetter(axis1Letter);
	const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();

	// TODO: possibly check for hidden axes
	isValid = NumPoints() != 0 && NumPoints() <= MaxGridProbePoints
			&& (radius < 0.0 || radius >= 1.0)
			&& NumAxis0points() <= MaxAxis0GridPoints
			&& axis0Letter != axis1Letter
			&& axis0Number == axis0NumForLetter
			&& axis1Number == axis1NumForLetter
			&& axis0Number < numVisibleAxes
			&& axis1Number < numVisibleAxes;

	if (isValid)
	{
		recipAxis0spacing = 1.0/axis0Spacing;
		recipAxis1spacing = 1.0/axis1Spacing;
	}
}

float GridDefinition::GetAxis0Coordinate(unsigned int axis0Index) const noexcept
{
	return axis0Min + (axis0Index * axis0Spacing);
}

float GridDefinition::GetAxis1Coordinate(unsigned int axis1Index) const noexcept
{
	return axis1Min + (axis1Index * axis1Spacing);
}

bool GridDefinition::IsInRadius(float axis0, float y) const noexcept
{
	return radius < 0.0 || axis0 * axis0 + y * y < radius * radius;
}

// Append the grid parameters to the end of a string
void GridDefinition::PrintParameters(const StringRef& s) const noexcept
{
	s.catf("%c%.1f:%.1f, %c%.1f:%.1f, radius %.1f, %c spacing %.1f, %c spacing %.1f, %" PRIu32 " points",
			axis0Letter, (double)axis0Min, (double)axis0Max, axis1Letter, (double)axis1Min, (double)axis1Max, (double)radius, axis0Letter, (double)axis0Spacing, axis1Letter, (double)axis1Spacing, NumPoints());
}

// Write the parameter label line to a string
void GridDefinition::WriteHeadingAndParameters(const StringRef& s) const noexcept
{
	s.printf("%s\n%d,%d,%c,%c,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%" PRIi32 ",%" PRIi32 "\n",
				HeightMapLabelLines[ARRAY_UPB(HeightMapLabelLines)], axis0Number, axis1Number, axis0Letter, axis1Letter, (double)axis0Min, (double)axis0Max, (double)axis1Min, (double)axis1Max, (double)radius, (double)axis0Spacing, (double)axis1Spacing, numAxis0, numAxis1);
}

// Check the parameter label line, returning -1 if not recognised, else the version we found
/*static*/ int GridDefinition::CheckHeading(const StringRef& s) noexcept
{
	for (size_t i = 0; i < ARRAY_SIZE(HeightMapLabelLines); ++i)
	{
		if (StringStartsWith(s.c_str(), HeightMapLabelLines[i]))
		{
			return (int)i;
		}
	}
	return -1;
}

// Read the grid parameters from a string returning true if success
bool GridDefinition::ReadParameters(const StringRef& s, int version) noexcept
{
	// 2018-04-08: rewrote this not to use sscanf because that function isn't thread safe
	isValid = false;						// assume failure
	const char *p = s.c_str();
	const char *q;

	if (version < 2)
	{
		axis0Number = X_AXIS;
		axis1Number = Y_AXIS;
		axis0Letter = 'X';
		axis1Letter = 'Y';
	}
	else
	{
		axis0Number = StrToU32(p, &q);
		if (p == q || *q != ',')
		{
			return false;
		}
		p = q + 1;

		axis1Number = StrToU32(p, &q);
		if (p == q || *q != ',')
		{
			return false;
		}
		p = q + 1;

		axis0Letter = *p;
		++p;
		if (*p != ',')
		{
			return false;
		}
		++p;
		axis1Letter = *p;
		++p;
		if (*p != ',')
		{
			return false;
		}
		++p;
	}

	axis0Min = SafeStrtof(p, &q);
	if (p == q || *q != ',')
	{
		return false;
	}
	p = q + 1;

	axis0Max = SafeStrtof(p, &q);
	if (p == q || *q != ',')
	{
		return false;
	}
	p = q + 1;

	axis1Min = SafeStrtof(p, &q);
	if (p == q || *q != ',')
	{
		return false;
	}
	p = q + 1;

	axis1Max = SafeStrtof(p, &q);
	if (p == q || *q != ',')
	{
		return false;
	}
	p = q + 1;

	radius = SafeStrtof(p, &q);
	if (p == q || *q != ',')
	{
		return false;
	}
	p = q + 1;

	axis0Spacing = SafeStrtof(p, &q);
	if (p == q || *q != ',')
	{
		return false;
	}
	p = q + 1;

	if (version == 0)
	{
		axis1Spacing = axis0Spacing;
	}
	else
	{
		axis1Spacing = SafeStrtof(p, &q);
		if (p == q || *q != ',')
		{
			return false;
		}
		p = q + 1;
	}

	numAxis0 = StrToU32(p, &q);
	if (p == q || *q != ',')
	{
		return false;
	}
	p = q + 1;

	numAxis1 = StrToU32(p, &q);
	if (p == q)
	{
		return false;
	}

	CheckValidity();
	return true;
}

// Print what is wrong with the grid, appending it to the existing string
void GridDefinition::PrintError(float originalAxis0range, float originalAxis1range, const StringRef& r) const noexcept
{
	if (axis0Spacing < MinSpacing || axis1Spacing < MinSpacing)
	{
		r.cat("Spacing too small");
	}
	else if (numAxis0 == 0)
	{
		r.cat("X range too small");
	}
	else if (numAxis1 == 0)
	{
		r.cat("Y range too small");
	}
	else if (   numAxis0 > MaxAxis0GridPoints
			 || numAxis0 > MaxGridProbePoints || numAxis1 > MaxGridProbePoints		// check X and Y individually in case X*Y overflows
			 || NumPoints() > MaxGridProbePoints
			)
	{
		const float totalRange = originalAxis0range + originalAxis1range;
		const float area = originalAxis0range * originalAxis1range;
		const float minSpacing = (totalRange + sqrtf(fsquare(totalRange) + 4.0 * (MaxGridProbePoints - 1) * area))/(2.0 * (MaxGridProbePoints - 1));
		const float minXspacing = originalAxis0range/(MaxAxis0GridPoints - 1);
		r.catf("Too many grid points; suggest increase spacing to %.1fmm", (double)max<float>(minSpacing, minXspacing));
	}
	else
	{
		// The only thing left is a bad radius
		r.cat("Bad radius");
	}
}

// Increase the version number in the following string whenever we change the format of the height map file.
const char * const HeightMap::HeightMapComment = "RepRapFirmware height map file v3";

HeightMap::HeightMap() noexcept : useMap(false) { }

void HeightMap::SetGrid(const GridDefinition& gd) noexcept
{
	useMap = false;
	def = gd;
	ClearGridHeights();
}

void HeightMap::ClearGridHeights() noexcept
{
	gridHeightSet.ClearAll();
#if HAS_MASS_STORAGE
	fileName.Clear();
#endif
}

// Set the height of a grid point
void HeightMap::SetGridHeight(size_t axis0Index, size_t axis1Index, float height) noexcept
{
	SetGridHeight(axis1Index * def.numAxis0 + axis0Index, height);
}

void HeightMap::SetGridHeight(size_t index, float height) noexcept
{
	if (index < MaxGridProbePoints)
	{
		gridHeights[index] = height;
		gridHeightSet.SetBit(index);
	}
}

// Return the minimum number of segments for a move by this X or Y amount
// Note that deltaAxis0 and deltaAxis1 may be negative
unsigned int HeightMap::GetMinimumSegments(float deltaAxis0, float deltaAxis1) const noexcept
{
	const float axis0Distance = fabsf(deltaAxis0);
	unsigned int axis0Segments = (axis0Distance > 0.0) ? (unsigned int)(axis0Distance * def.recipAxis0spacing + 0.4) : 1;

	const float axis1Distance = fabsf(deltaAxis1);
	unsigned int axis1Segments = (axis1Distance > 0.0) ? (unsigned int)(axis1Distance * def.recipAxis1spacing + 0.4) : 1;

	return max<unsigned int>(axis0Segments, axis1Segments);
}

#if HAS_MASS_STORAGE

// Save the grid to file returning true if an error occurred
bool HeightMap::SaveToFile(FileStore *f, const char *fname, float zOffset) noexcept
{
	String<StringLength500> bufferSpace;
	const StringRef buf = bufferSpace.GetRef();

	// Write the header comment
	buf.copy(HeightMapComment);
	tm timeInfo;
	if (reprap.GetPlatform().GetDateTime(timeInfo))
	{
		buf.catf(" generated at %04u-%02u-%02u %02u:%02u",
						timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min);
	}

	Deviation deviation;
	float minError, maxError;
	(void)GetStatistics(deviation, minError, maxError);
	buf.catf(", min error %.3f, max error %.3f, mean %.3f, deviation %.3f\n",
				(double)(minError + zOffset), (double)(maxError + zOffset), (double)(deviation.GetMean() + zOffset), (double)deviation.GetDeviationFromMean());
	if (!f->Write(buf.c_str()))
	{
		return true;
	}

	// Write the grid parameters
	def.WriteHeadingAndParameters(buf);
	if (!f->Write(buf.c_str()))
	{
		return true;
	}

	// Write the grid heights. We use a fixed field with of 6 characters to make is easier to view.
	uint32_t index = 0;
	for (uint32_t i = 0; i < def.numAxis1; ++i)
	{
		buf.Clear();
		for (uint32_t j = 0; j < def.numAxis0; ++j)
		{
			if (j != 0)
			{
				buf.cat(',');
			}
			if (gridHeightSet.IsBitSet(index))
			{
				buf.catf("%7.3f", (double)(gridHeights[index] + zOffset));
			}
			else
			{
				buf.cat("      0");				// write 0 with no decimal point where we didn't probe, so we can tell when we reload it
			}
			++index;
		}
		buf.cat('\n');
		if (!f->Write(buf.c_str()))
		{
			return true;
		}
	}

	fileName.copy(fname);
	return false;
}

// Load the grid from file, returning true if an error occurred with the error reason appended to the buffer
bool HeightMap::LoadFromFile(FileStore *f, const char *fname, const StringRef& r) noexcept
{
	const size_t MaxLineLength = (MaxAxis0GridPoints * 8) + 2;					// maximum length of a line in the height map file, need 8 characters per grid point
	const char* const readFailureText = "failed to read line from file";
	char buffer[MaxLineLength + 1];
	StringRef s(buffer, ARRAY_SIZE(buffer));

	ClearGridHeights();															// this also clears the filename
	GridDefinition newGrid;
	int gridVersion;

	if (f->ReadLine(buffer, sizeof(buffer)) <= 0)
	{
		r.cat(readFailureText);
	}
	else if (!StringStartsWith(buffer, HeightMapComment))						// check the version line is as expected
	{
		r.cat("bad header line or wrong version header");
	}
	else if (f->ReadLine(buffer, sizeof(buffer)) <= 0)
	{
		r.cat(readFailureText);
	}
	else if ((gridVersion = GridDefinition::CheckHeading(s)) < 0)				// check the label line is as expected
	{
		r.cat("bad label line");
	}
	else if (f->ReadLine(buffer, sizeof(buffer)) <= 0)							// read the height map parameters
	{
		r.cat(readFailureText);
	}
	else if (!newGrid.ReadParameters(s, gridVersion))
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
		for (uint32_t row = 0; row < def.numAxis1; ++row)		// read the grid a row at a time
		{
			if (f->ReadLine(buffer, sizeof(buffer)) <= 0)
			{
				r.cat(readFailureText);
				return true;								// failed to read a line
			}
			const char *p = buffer;
			for (uint32_t col = 0; col < def.numAxis0; ++col)
			{
				while (*p == ' ' || *p == '\t')
				{
					++p;
				}
				if (*p == '0' && (p[1] == ',' || p[1] == 0))
				{
					// Values of 0 with no decimal places in un-probed values, so leave the point set as not valid
					++p;
				}
				else
				{
					const char* np;
					const float f = SafeStrtof(p, &np);
					if (np == p)
					{
						r.catf("number expected at line %" PRIu32 " column %d", row + 3, (p - buffer) + 1);
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
		fileName.copy(fname);
		return false;										// success!
	}
	return true;											// an error occurred
}

#endif

#if HAS_LINUX_INTERFACE

// Update the filename
void HeightMap::SetFileName(const char *name) noexcept
{
	fileName.copy(name);
}

// Save the grid to a sequential array in the same way as to a regular CSV file
void HeightMap::SaveToArray(float *arr, float zOffset) const noexcept
{
	size_t index = 0;
	for (size_t i = 0; i < def.numAxis1; ++i)
	{
		for (size_t j = 0; j < def.numAxis0; ++j)
		{
			arr[index] = gridHeightSet.IsBitSet(index) ? (gridHeights[index] + zOffset) : std::numeric_limits<float>::quiet_NaN();
			index++;
		}
	}
}

#endif

// Return number of points probed, mean and RMS deviation, min and max error
unsigned int HeightMap::GetStatistics(Deviation& deviation, float& minError, float& maxError) const noexcept
{
	double heightSum = 0.0, heightSquaredSum = 0.0;
	minError = 9999.0;
	maxError = -9999.0;
	unsigned int numProbed = 0;
	for (uint32_t i = 0; i < def.NumPoints(); ++i)
	{
		if (gridHeightSet.IsBitSet(i))
		{
			++numProbed;
			const float fHeightError = gridHeights[i];
			if (fHeightError > maxError)
			{
				maxError = fHeightError;
			}
			if (fHeightError < minError)
			{
				minError = fHeightError;
			}
			const double dHeightError = (double)fHeightError;
			heightSum += dHeightError;
			heightSquaredSum += dsquare(dHeightError);
		}
	}

	deviation.Set(heightSquaredSum, heightSum, numProbed);
	return numProbed;
}

// Try to turn mesh compensation on or off and report the state achieved
bool HeightMap::UseHeightMap(bool b) noexcept
{
	useMap = b && def.IsValid();
	return useMap;
}

// Compute the height error at the specified point
float HeightMap::GetInterpolatedHeightError(float axis0, float axis1) const noexcept
{
	if (!useMap)
	{
		return 0.0;
	}

	// Last grid point
	const float xLast = def.axis0Min + (def.numAxis0-1)*def.axis0Spacing;
	const float yLast = def.axis1Min + (def.numAxis1-1)*def.axis1Spacing;

	// Clamp to rectangle so InterpolateXY will always have valid parameters
	const float fEPSILON = 0.01;
	if (axis0 < def.axis0Min) { axis0 = def.axis0Min; }
	if (axis1 < def.axis1Min) {	axis1 = def.axis1Min; }
	if (axis0 > xLast -fEPSILON) { axis0 = xLast -fEPSILON; }
	if (axis1 > yLast -fEPSILON) { axis1 = yLast -fEPSILON; }


	const float xf = (axis0 - def.axis0Min) * def.recipAxis0spacing;
	const float xFloor = floor(xf);
	const int32_t xIndex = (int32_t)xFloor;
	const float yf = (axis1 - def.axis1Min) * def.recipAxis1spacing;
	const float yFloor = floor(yf);
	const int32_t yIndex = (int32_t)yFloor;

	return InterpolateAxis0Axis1(xIndex, yIndex, xf - xFloor, yf - yFloor);
}

float HeightMap::InterpolateAxis0Axis1(uint32_t axis0Index, uint32_t axis1Index, float axis0Frac, float axis1Frac) const noexcept
{
	const uint32_t indexX0Y0 = GetMapIndex(axis0Index, axis1Index);	// (X0,Y0)
	const uint32_t indexX1Y0 = indexX0Y0 + 1;						// (X1,Y0)
	const uint32_t indexX0Y1 = indexX0Y0 + def.numAxis0;			// (X0 Y1)
	const uint32_t indexX1Y1 = indexX0Y1 + 1;						// (X1,Y1)

	const float xyFrac = axis0Frac * axis1Frac;
	return (gridHeights[indexX0Y0] * (1.0 - axis0Frac - axis1Frac + xyFrac))
			+ (gridHeights[indexX1Y0] * (axis0Frac - xyFrac))
			+ (gridHeights[indexX0Y1] * (axis1Frac - xyFrac))
			+ (gridHeights[indexX1Y1] * xyFrac);
}

void HeightMap::ExtrapolateMissing() noexcept
{
	//1: calculating the bed plane by least squares fit
	//2: filling in missing points

	//algorithm: http://www.ilikebigbits.com/blog/2015/3/2/plane-from-points
	float sumAxis0 = 0, sumAxis1 = 0, sumZ = 0;
	int n = 0;
	for (uint32_t iAxis1 = 0; iAxis1 < def.numAxis1; iAxis1++)
	{
		for (uint32_t iAxis0 = 0; iAxis0 < def.numAxis0; iAxis0++)
		{
			const uint32_t index = GetMapIndex(iAxis0, iAxis1);
			if (gridHeightSet.IsBitSet(index))
			{
				const float fAxis0 = (def.axis0Spacing * iAxis0) + def.axis0Min;
				const float fAxis1 = (def.axis1Spacing * iAxis1) + def.axis1Min;
				const float fZ = gridHeights[index];

				n++;
				sumAxis0 += fAxis0; sumAxis1 += fAxis1; sumZ += fZ;
			}
		}
	}

	const float invN = 1.0 / float(n);
	const float centAxis0 = sumAxis0 * invN, centAxis1 = sumAxis1 * invN, centZ = sumZ * invN;

	// Calculate full 3x3 covariance matrix, excluding symmetries
	float axis0Axis0 = 0.0; float axis0Axis1 = 0.0; float axis0z = 0.0;
	float axis1Axis1 = 0.0; float axis1z = 0.0; float zz = 0.0;

	for (uint32_t iAxis1 = 0; iAxis1 < def.numAxis1; iAxis1++)
	{
		for (uint32_t iAxis0 = 0; iAxis0 < def.numAxis0; iAxis0++)
		{
			const uint32_t index = GetMapIndex(iAxis0, iAxis1);
			if (gridHeightSet.IsBitSet(index))
			{
				const float fAxis0 = (def.axis0Spacing * iAxis0) + def.axis0Min;
				const float fAxis1 = (def.axis1Spacing * iAxis1) + def.axis1Min;
				const float fZ = gridHeights[index];

				const float rAxis0 = fAxis0 - centAxis0;
				const float rAxis1 = fAxis1 - centAxis1;
				const float rZ = fZ - centZ;

				axis0Axis0 += rAxis0 * rAxis0;
				axis0Axis1 += rAxis0 * rAxis1;
				axis0z += rAxis0 * rZ;
				axis1Axis1 += rAxis1 * rAxis1;
				axis1z += rAxis1 * rZ;
				zz += rZ * rZ;
			}
		}
	}

	const float detZ = axis0Axis0*axis1Axis1 - axis0Axis1*axis0Axis1;
	if (detZ <= 0)
	{
		// Not a valid plane (or a vertical one)
		return;
	}

	// Plane equation: ax+by+cz=d -> z = (d-(ax+by))/c
	float a = (axis1z*axis0Axis1 - axis0z*axis1Axis1) / detZ;
	float b = (axis0z*axis0Axis1 - axis1z*axis0Axis0) / detZ;
	const float invC = sqrtf(a*a + b*b + 1.0);
	const float normLenInv = 1.0 / invC;
	a *= normLenInv;
	b *= normLenInv;
	const float c = normLenInv;
	const float d = centAxis0*a + centAxis1*b + centZ*c;

	// Fill in the blanks
	for (uint32_t iAxis1 = 0; iAxis1 < def.numAxis1; iAxis1++)
	{
		for (uint32_t iAxis0 = 0; iAxis0 < def.numAxis0; iAxis0++)
		{
			const uint32_t index = GetMapIndex(iAxis0, iAxis1);
			if (!gridHeightSet.IsBitSet(index))
			{
				const float fAxis0 = (def.axis0Spacing * iAxis0) + def.axis0Min;
				const float fAxis1 = (def.axis1Spacing * iAxis1) + def.axis1Min;
				const float fZ = (d - (a * fAxis0 + b * fAxis1)) * invC;
				gridHeights[index] = fZ;	// fill in Z but don't mark it as set so we can always differentiate between measured and extrapolated
			}
		}
	}
}

// End
