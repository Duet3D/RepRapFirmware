/*
 * Grid.cpp
 *
 *  Created on: 18 Nov 2016
 *      Author: David
 */

#include "Grid.h"
#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <GCodes/GCodes.h>
#include <Storage/FileStore.h>
#include <Math/Deviation.h>

#include <cmath>

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(GridDefinition, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(...) OBJECT_MODEL_FUNC_IF_BODY(GridDefinition, __VA_ARGS__)

static constexpr ObjectModelArrayDescriptor axisArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return 2; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((const GridDefinition*)self)->GetAxisLetter(context.GetLastIndex())); }
};

static constexpr ObjectModelArrayDescriptor maxsArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return 2; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((const GridDefinition*)self)->GetMax(context.GetLastIndex()), 1); }
};

static constexpr ObjectModelArrayDescriptor minsArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return 2; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((const GridDefinition*)self)->GetMin(context.GetLastIndex()), 1); }
};

static constexpr ObjectModelArrayDescriptor spacingsArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return 2; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((const GridDefinition*)self)->GetSpacing(context.GetLastIndex()), 1); }
};

constexpr ObjectModelTableEntry GridDefinition::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. GridDefinition members
	{ "axes",		OBJECT_MODEL_FUNC_NOSELF(&axisArrayDescriptor), 		ObjectModelEntryFlags::none },
	{ "maxs",		OBJECT_MODEL_FUNC_NOSELF(&maxsArrayDescriptor), 		ObjectModelEntryFlags::none },
	{ "mins",		OBJECT_MODEL_FUNC_NOSELF(&minsArrayDescriptor), 		ObjectModelEntryFlags::none },
	{ "radius",		OBJECT_MODEL_FUNC(self->radius, 1),						ObjectModelEntryFlags::none },
	{ "spacings",	OBJECT_MODEL_FUNC_NOSELF(&spacingsArrayDescriptor), 	ObjectModelEntryFlags::none },

	{ "xMax",		OBJECT_MODEL_FUNC(self->maxs[0], 1),					ObjectModelEntryFlags::obsolete },
	{ "xMin",		OBJECT_MODEL_FUNC(self->mins[0], 1),					ObjectModelEntryFlags::obsolete },
	{ "xSpacing",	OBJECT_MODEL_FUNC(self->spacings[0], 1),				ObjectModelEntryFlags::obsolete },
	{ "yMax",		OBJECT_MODEL_FUNC(self->maxs[1], 1),					ObjectModelEntryFlags::obsolete },
	{ "yMin",		OBJECT_MODEL_FUNC(self->mins[1], 1),					ObjectModelEntryFlags::obsolete },
	{ "ySpacing",	OBJECT_MODEL_FUNC(self->spacings[1], 1),				ObjectModelEntryFlags::obsolete },
};

constexpr uint8_t GridDefinition::objectModelTableDescriptor[] = { 1, 11 };

DEFINE_GET_OBJECT_MODEL_TABLE(GridDefinition)

#endif

const char * const GridDefinition::HeightMapLabelLines[] =
{
	"xmin,xmax,ymin,ymax,radius,spacing,xnum,ynum",														// old version label line
	"xmin,xmax,ymin,ymax,radius,xspacing,yspacing,xnum,ynum",											// label line until 3.3-beta1
	"axis0,axis1,min0,max0,min1,max1,radius,spacing0,spacing1,num0,num1",								// label line from 3.3-beta2
};

// Initialise the grid to be invalid
GridDefinition::GridDefinition() noexcept : radius(-1.0), isValid(false)
{
	letters[0] = 'X';
	letters[1] = 'Y';
	mins[0] = mins[1] = 0.0;
	maxs[0] = maxs[1] = -1.0;
	spacings[0] = spacings[1] = 0.0;

	nums[0] = nums[1] = 0;
	recipAxisSpacings[0] = recipAxisSpacings[1] = 0.0;
}

// Set the grid parameters ands return true if it is now valid
bool GridDefinition::Set(const char axisLetters[2], const float axis0Range[2], const float axis1Range[2], float pRadius, const float pSpacings[2]) noexcept
{
	letters[0] = axisLetters[0];
	letters[1] = axisLetters[1];
	mins[0] = axis0Range[0];
	maxs[0] = axis0Range[1];
	mins[1] = axis1Range[0];
	maxs[1] = axis1Range[1];
	radius = pRadius;
	spacings[0] = pSpacings[0];
	spacings[1] = pSpacings[1];
	CheckValidity(true);
	return isValid;
}

// Set up internal variables and check validity of the grid.
// If setNum0Num1 is true then set up numAxis0 and numAxis1, otherwise leave them alone if the grid is valid
// If the grid is valid then set up recipAxis0spacing and recipAxis1spacing
void GridDefinition::CheckValidity(bool setNum0Num1) noexcept
{
	if (maxs[0] - mins[0] < MinRange || spacings[0] < MinSpacing || maxs[1] - mins[1] < MinRange || spacings[1] < MinSpacing)
	{
		isValid = false;
		if (setNum0Num1)
		{
			nums[0] = nums[1] = 0;
		}
	}
	else
	{
		if (setNum0Num1)
		{
			nums[0] = (uint32_t)((maxs[0] - mins[0])/spacings[0]) + 1;
			nums[1] = (uint32_t)((maxs[1] - mins[1])/spacings[1]) + 1;
		}

		const size_t axis0NumForLetter = reprap.GetGCodes().GetAxisNumberForLetter(letters[0]);
		const size_t axis1NumForLetter = reprap.GetGCodes().GetAxisNumberForLetter(letters[1]);
		const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();

		isValid = NumPoints() != 0 && NumPoints() <= MaxGridProbePoints
				&& (radius < 0.0 || radius >= 1.0)
				&& nums[0] <= MaxAxis0GridPoints
				&& letters[0] != letters[1]
				&& axis0NumForLetter < numVisibleAxes
				&& axis1NumForLetter < numVisibleAxes;

		if (isValid)
		{
			axisNumbers[0] = axis0NumForLetter;
			axisNumbers[1] = axis1NumForLetter;
			recipAxisSpacings[0] = 1.0/spacings[0];
			recipAxisSpacings[1] = 1.0/spacings[1];
		}
	}
}

bool GridDefinition::IsInRadius(float axis0, float y) const noexcept
{
	return radius < 0.0 || fsquare(axis0) + fsquare(y) < fsquare(radius);
}

// Append the grid parameters to the end of a string
void GridDefinition::PrintParameters(const StringRef& s) const noexcept
{
	s.catf("%c%.1f:%.1f, %c%.1f:%.1f, radius %.1f, %c spacing %.1f, %c spacing %.1f, %" PRIu32 " points",
			letters[0], (double)mins[0], (double)maxs[0], letters[1], (double)mins[1], (double)maxs[1], (double)radius, letters[0], (double)spacings[0], letters[1], (double)spacings[1], NumPoints());
}

// Write the parameter label line to a string
void GridDefinition::WriteHeadingAndParameters(const StringRef& s) const noexcept
{
	s.printf("%s\n%c,%c,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%" PRIi32 ",%" PRIi32 "\n",
				HeightMapLabelLines[ARRAY_UPB(HeightMapLabelLines)], letters[0], letters[1], (double)mins[0], (double)maxs[0], (double)mins[1], (double)maxs[1], (double)radius, (double)spacings[0], (double)spacings[1], nums[0], nums[1]);
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
		letters[0] = 'X';
		letters[1] = 'Y';
	}
	else
	{
		letters[0] = *p;
		++p;
		if (*p != ',')
		{
			return false;
		}
		++p;
		letters[1] = *p;
		++p;
		if (*p != ',')
		{
			return false;
		}
		++p;
	}

	mins[0] = SafeStrtof(p, &q);
	if (p == q || *q != ',')
	{
		return false;
	}
	p = q + 1;

	maxs[0] = SafeStrtof(p, &q);
	if (p == q || *q != ',')
	{
		return false;
	}
	p = q + 1;

	mins[1] = SafeStrtof(p, &q);
	if (p == q || *q != ',')
	{
		return false;
	}
	p = q + 1;

	maxs[1] = SafeStrtof(p, &q);
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

	spacings[0] = SafeStrtof(p, &q);
	if (p == q || *q != ',')
	{
		return false;
	}
	p = q + 1;

	if (version == 0)
	{
		spacings[1] = spacings[0];
	}
	else
	{
		spacings[1] = SafeStrtof(p, &q);
		if (p == q || *q != ',')
		{
			return false;
		}
		p = q + 1;
	}

	nums[0] = StrToU32(p, &q);
	if (p == q || *q != ',')
	{
		return false;
	}
	p = q + 1;

	nums[1] = StrToU32(p, &q);
	if (p == q)
	{
		return false;
	}

	CheckValidity(false);
	return true;
}

// Print what is wrong with the grid, appending it to the existing string
void GridDefinition::PrintError(float originalAxis0range, float originalAxis1range, const StringRef& r) const noexcept
{
	if (spacings[0] < MinSpacing || spacings[1] < MinSpacing)
	{
		r.cat("Spacing too small");
	}
	else if (nums[0] == 0)
	{
		r.catf("%c range too small", letters[0]);
	}
	else if (nums[1] == 0)
	{
		r.catf("%c range too small", letters[1]);
	}
	else if (   nums[0] > MaxAxis0GridPoints
			 || nums[0] > MaxGridProbePoints || nums[1] > MaxGridProbePoints		// check X and Y individually in case X*Y overflows
			 || NumPoints() > MaxGridProbePoints
			)
	{
		const float totalRange = originalAxis0range + originalAxis1range;
		const float area = originalAxis0range * originalAxis1range;
		const float minSpacing = (totalRange + fastSqrtf(fsquare(totalRange) + 4.0 * (MaxGridProbePoints - 1) * area))/(2.0 * (MaxGridProbePoints - 1));
		const float minXspacing = originalAxis0range/(MaxAxis0GridPoints - 1);
		r.catf("Too many grid points; suggest increase spacing to %.1fmm", (double)max<float>(minSpacing, minXspacing));
	}
	else
	{
		// The only thing left is a bad radius
		r.cat("Bad radius");
	}
}

// Increase the version number in the following string whenever we change the format of the height map file significantly.
// Adding more fields to the header row can be handled in GridDefinition::ReadParameters(), though.
const char * const HeightMap::HeightMapComment = "RepRapFirmware height map file v2";

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
	SetGridHeight(axis1Index * def.nums[0] + axis0Index, height);
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
	unsigned int axis0Segments = (axis0Distance > 0.0) ? (unsigned int)(axis0Distance * def.recipAxisSpacings[0] + 0.4) : 1;

	const float axis1Distance = fabsf(deltaAxis1);
	unsigned int axis1Segments = (axis1Distance > 0.0) ? (unsigned int)(axis1Distance * def.recipAxisSpacings[1] + 0.4) : 1;

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
	for (uint32_t i = 0; i < def.nums[1]; ++i)
	{
		buf.Clear();
		for (uint32_t j = 0; j < def.nums[0]; ++j)
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
		for (uint32_t row = 0; row < def.nums[1]; ++row)	// read the grid a row at a time
		{
			if (f->ReadLine(buffer, sizeof(buffer)) <= 0)
			{
				r.cat(readFailureText);
				return true;								// failed to read a line
			}
			const char *p = buffer;
			for (uint32_t col = 0; col < def.nums[0]; ++col)
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
	for (size_t i = 0; i < def.nums[1]; ++i)
	{
		for (size_t j = 0; j < def.nums[0]; ++j)
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
	const float xLast = def.mins[0] + (def.nums[0]-1)*def.spacings[0];
	const float yLast = def.mins[1] + (def.nums[1]-1)*def.spacings[1];

	// Clamp to rectangle so InterpolateXY will always have valid parameters
	const float fEPSILON = 0.01;
	if (axis0 < def.mins[0]) { axis0 = def.mins[0]; }
	if (axis1 < def.mins[1]) { axis1 = def.mins[1]; }
	if (axis0 > xLast -fEPSILON) { axis0 = xLast -fEPSILON; }
	if (axis1 > yLast -fEPSILON) { axis1 = yLast -fEPSILON; }


	const float xf = (axis0 - def.mins[0]) * def.recipAxisSpacings[0];
	const float xFloor = floor(xf);
	const int32_t xIndex = (int32_t)xFloor;
	const float yf = (axis1 - def.mins[1]) * def.recipAxisSpacings[1];
	const float yFloor = floor(yf);
	const int32_t yIndex = (int32_t)yFloor;

	return InterpolateAxis0Axis1(xIndex, yIndex, xf - xFloor, yf - yFloor);
}

float HeightMap::InterpolateAxis0Axis1(uint32_t axis0Index, uint32_t axis1Index, float axis0Frac, float axis1Frac) const noexcept
{
	const uint32_t indexX0Y0 = GetMapIndex(axis0Index, axis1Index);	// (X0,Y0)
	const uint32_t indexX1Y0 = indexX0Y0 + 1;						// (X1,Y0)
	const uint32_t indexX0Y1 = indexX0Y0 + def.nums[0];				// (X0 Y1)
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
	for (uint32_t iAxis1 = 0; iAxis1 < def.nums[1]; iAxis1++)
	{
		for (uint32_t iAxis0 = 0; iAxis0 < def.nums[0]; iAxis0++)
		{
			const uint32_t index = GetMapIndex(iAxis0, iAxis1);
			if (gridHeightSet.IsBitSet(index))
			{
				const float fAxis0 = (def.spacings[0] * iAxis0) + def.mins[0];
				const float fAxis1 = (def.spacings[1] * iAxis1) + def.mins[1];
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

	for (uint32_t iAxis1 = 0; iAxis1 < def.nums[1]; iAxis1++)
	{
		for (uint32_t iAxis0 = 0; iAxis0 < def.nums[0]; iAxis0++)
		{
			const uint32_t index = GetMapIndex(iAxis0, iAxis1);
			if (gridHeightSet.IsBitSet(index))
			{
				const float fAxis0 = (def.spacings[0] * iAxis0) + def.mins[0];
				const float fAxis1 = (def.spacings[1] * iAxis1) + def.mins[1];
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
	const float invC = fastSqrtf(a*a + b*b + 1.0);
	const float normLenInv = 1.0 / invC;
	a *= normLenInv;
	b *= normLenInv;
	const float c = normLenInv;
	const float d = centAxis0*a + centAxis1*b + centZ*c;

	// Fill in the blanks
	for (uint32_t iAxis1 = 0; iAxis1 < def.nums[1]; iAxis1++)
	{
		for (uint32_t iAxis0 = 0; iAxis0 < def.nums[0]; iAxis0++)
		{
			const uint32_t index = GetMapIndex(iAxis0, iAxis1);
			if (!gridHeightSet.IsBitSet(index))
			{
				const float fAxis0 = (def.spacings[0] * iAxis0) + def.mins[0];
				const float fAxis1 = (def.spacings[1] * iAxis1) + def.mins[1];
				const float fZ = (d - (a * fAxis0 + b * fAxis1)) * invC;
				gridHeights[index] = fZ;	// fill in Z but don't mark it as set so we can always differentiate between measured and extrapolated
			}
		}
	}
}

// End
