/*
 * ScrewMap.cpp
 *
 *  Created on: 11 Dec 2020
 *      Author: MarkZ
 */
#include "ScrewMap.h"
#include "GCodes/GCodes.h"
#include "../Move.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"

/* Screw Mapping is a well-known adjustment mechanism for screw nonlinearities in axes
   Unlike 'real' screw mapping, this allows one axis to independently affect multiple axes for physical correction
   The usedAxes property is the list of axes that get adjusted for each specific input axis
   This must therefore be set after the drives and axes are all defined
   
   MapTable is an array of values, one row per adjusted axis, with delta values in each entry
     MapTable[axis][] == {0,0...} is no changes
     MapTable[X][] = {-1,-1,...} would shift all positions to the left one unit

	Transform algorithm -> 
		for each visible axis
			given an input coordinate, calculate the (float) table index for the coordinate
	  		for each transformed (sub)axis:
				a) if index is less than min use the min adjustment
				b) if index is greater than max use the max adjustment
				c) if index is between i, i+1 entries linearly interpolate for the adjustment
		the assumption is delta values are small,
		so when X, for example, changes Y we can use the new Y' as source for adjustment
	Inverse Transform Algorithm
		the reverse algorithm finds the (float) index by walking through the adjustment table
		but then linearly interpolates to return the original values to floating point accuracy

	This uses 4 M-codes. 

	-- Enable/disable all screw mapping. Does not clear tables.
	-- if no S prints report
	M640 S[0|1]

	-- create a screw table for srcaxis with one row per destaxis
	-- if no count prints report
	M641 R"srcaxis" A"destaxes" Sstart Iinterval Ncount

	-- set table entries (may be called multiple times for shorter gcode lines)
	-- offset value lets a subset of the table be set
	-- if no data prints contents of table
	M642 R"srcaxis" [Ooffset] Xf0:f1:f2:f3... Yf0:f1:f2...

	-- selftest (for debugging)
	[M643]
*/

// set this to zero to not include the selftest code, though M810 is still mapped in gcodes
#define TESTING_SCREW_MAP 0

static float* FindTableRow(const ScrewMapInfo& smi, unsigned int findAxis) noexcept;

// having these be global saves space and lets translate be const
float LastPosn[MaxAxes];		// single position cache for speed
float LastTranslate[MaxAxes];

ScrewMap::ScrewMap()
{
	// initial hard set of null pointers (disable the tables)
	for (size_t i=0; i<MaxAxes; i++)
	{
		screwInfos[i].mapTable = nullptr;
	}
	IsMapEnabled = false;
	InvalidateCache();
}

ScrewMap::~ScrewMap()
{
	ClearScrewMaps();
}

const char* ScrewMap::GetEnabledString() const noexcept
{
	return IsEnabled() ? "enabled" : "disabled";
}

// enable/disable the entire map process
bool ScrewMap::SetEnabled(bool newEnable) noexcept
{
	bool oldV = IsMapEnabled;
	IsMapEnabled = newEnable;
	UpdatePositions();
	return oldV;
}

// the hokey cache is to speed up inverse transform and make it float exact
// since in my watching it always happens right after a transform
void ScrewMap::InvalidateCache() noexcept
{
	LastTranslate[0] = -1232.321;		// random not useful
}

// find axis letter in list, return axis number
static int AxisLetterLookup(char axis)
{
	int numAxis = -1;
	char uaxis = toupper(axis);
	for (size_t i=0; i<MaxAxes; i++)
	{
		if (uaxis == reprap.GetGCodes().GetAxisLetters()[i])
		{
			numAxis = i;
			break;
		}
	}
	return numAxis;
}

GCodeResult  ScrewMap::ParseEnable(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	if (gb.Seen('S'))
	{
		bool doEnable = (gb.GetUIValue() > 0);		// S0=disable or S1=enable
		SetEnabled(doEnable);
	}
	reply.printf("Screw mapping is %s", GetEnabledString());
	return GCodeResult::ok;
}

// given a create request from gcode, parse and execute it
GCodeResult ScrewMap::ParseCreate(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	float start = 0;
	float increment = 1;
	int count = -1;
	char axisbuf[3] = {'\0'};
	char destlistbuf[3] = {'\0'};
	char destnumbersbuf[10] = {'\0'};
	StringRef axis(axisbuf, ARRAY_SIZE(axisbuf));
	StringRef destlist(destlistbuf, ARRAY_SIZE(destlistbuf));
	StringRef destnumbers(destnumbersbuf, ARRAY_SIZE(destnumbersbuf));

	// R:src_axis S:start I:increment N:count
	if (gb.Seen('R'))
	{
		gb.GetReducedString(axis);
		if (gb.Seen('S'))
		{
			start = gb.ConvertDistance(gb.GetFValue());
		}
		if (gb.Seen('I'))
		{	// if we're in a different unit of measure we need to convert
			increment = gb.ConvertDistance(gb.GetFValue());
		}
		if (gb.Seen('N'))
		{
			count = gb.GetUIValue();
		}
		if (gb.Seen('A'))
		{
			gb.GetReducedString(destlist);
		}
	}
	// check for source axis input
	if (axis.strlen() == 0)
	{
		String<StringLength100> outLine;
		// report only
		reply.cat("Screw maps\n");
		for (size_t i=0; i<MaxAxes; i++)
		{
			char drv = reprap.GetGCodes().GetAxisLetters()[i];
			if (drv != '\0')
			{
				if ( screwInfos[i].IsEnabled())
				{
					ScrewMapInfo& smi = screwInfos[i];
					char axisList[10] = {'\0'};
					smi.usedAxes.Iterate( [&axisList] (unsigned int subaxis, unsigned int counter) noexcept
					{
						axisList[counter] = reprap.GetGCodes().GetAxisLetters()[subaxis];
					});
					outLine.printf(" %c - start=%f increment=%f count=%d dest axes=%s\n",
							drv, (double)gb.InverseConvertDistance(smi.start), 
							(double)gb.InverseConvertDistance(smi.increment), smi.count, axisList );
				}
				else
				{
					outLine.printf(" %c - disabled\n", drv);
				}
				reply.cat(outLine.c_str());
			}
		}
		return GCodeResult::ok;
	}

	int srcAxis = AxisLetterLookup(axis[0]);
	if (srcAxis == -1)
	{
		reply.printf("ScrewMap R value %c is not a valid axis.", axis[0]);
		return GCodeResult::error;
	}

	if (count == -1)
	{
		reply.printf("ScrewMap C count value required. Use 0 to delete the mapping.\n");
		return GCodeResult::error;
	}

	if (count == 0)
	{
		reply.printf("Clearing %c screw map", axis[0]);
		ClearMap(&screwInfos[srcAxis]);
		return GCodeResult::ok;
	}

	AxesBitmap mxa = AxesBitmap::MakeFromBits(0);
	int numAxes = 0;
	if ( !destlist.IsEmpty())
	{
		size_t destlen = destlist.strlen();
		for (size_t i=0; i<destlen; i++)
		{
			int iax = AxisLetterLookup(destlist[i]);
			if (iax >= 0 )
			{
				const char *digits = "0123456789";
				mxa.SetBit(iax);
				if (numAxes > 0)
				{
					destnumbers.cat(LIST_SEPARATOR);
				}
				// cheap itoa
				if (iax > 10)
				{
					destnumbers.cat('1');
					iax -= 10;
				}
				destnumbers.cat(digits[iax]);
			}
		}
	}

	if (numAxes == 0)
	{
		// no destination axes specified
		numAxes = 1;
		mxa.SetBit(srcAxis);
	}

	SetScrewMap(srcAxis, start, increment, count, mxa);

	reply.printf ("Creating screw map axis '%c'[%d], start %f, increment %f, count %d, dest axes '%s'[%s]",
		axis[0], srcAxis,
		(double)gb.InverseConvertDistance(start), (double)gb.InverseConvertDistance(increment),
		count, destlist.c_str(), destnumbers.c_str());

	return GCodeResult::ok;
}

GCodeResult ScrewMap::ParseTable(GCodeBuffer& gb, const StringRef& reply, OutputBuffer*& buf) THROWS(GCodeException)
{
	char axisbuf[3] = {'\0'};
	size_t dataOffset = 0;
	StringRef axis(axisbuf, ARRAY_SIZE(axisbuf));
	// R:src_axis S:start I:increment N:count
	if (gb.Seen('R'))
	{
		gb.GetReducedString(axis);
	}
		// check for source axis input
	if (axis.strlen() == 0)
	{
		reply.printf("ScrewMap R is required. Syntax R:srcAxisLetter [X:f1:f2:...] [Y:f1:f2:...] ...");
		return GCodeResult::error;
	}

	int srcAxis = AxisLetterLookup(axis[0]);
	if (srcAxis == -1)
	{
		reply.printf("ScrewMap R value %c is not a valid axis.", axis[0]);
		return GCodeResult::error;
	}

	if (! screwInfos[srcAxis].IsEnabled())
	{
		reply.printf("Axis %c has no screw map defined.", axis[0]);
		return GCodeResult::error;
	}

	if (gb.Seen('O'))
	{
		dataOffset = gb.GetUIValue();
	}

	if (!OutputBuffer::Allocate(buf))
	{
		reply.copy("No output buffer");
		return GCodeResult::error;
	}
		// check for source axis input
	// find tables of data
	buf->catf("ScrewMap for %c Axis\n", axis[0]);
	ScrewMapInfo& smi = screwInfos[srcAxis];
	for (size_t i=0; i<MaxAxes; i++)
	{
		char drv = reprap.GetGCodes().GetAxisLetters()[i];
		if (drv == '\0')
			break;
		bool bMapped = smi.usedAxes.IsBitSet(i);
		if (gb.Seen(drv))
		{
			if (bMapped)
			{
				float values[100] = {0.0f};
				size_t valsize = ARRAY_SIZE(values);
				gb.GetFloatArray( values, valsize, false);
				// if we're in a different unit of measure we need to convert
				for (size_t i=0; i<valsize; i++)
				{
					values[i] = gb.ConvertDistance(values[i]);	// metric conversion if necessary
				}
				SetScrewAxis(srcAxis, i, values, valsize, dataOffset);
				buf->cat('*');
			}
			else
			{
				buf->catf("Destination axis %c is not mapped.\n", drv);
			}
		}
		if (bMapped)
		{
			buf->catf(" %c=", drv);
			float* values = FindTableRow(smi, i);
			for (uint8_t j=0; j<smi.count; j++)
			{
				if (j > 0)
				{
					buf->cat(LIST_SEPARATOR);
				}
				buf->catf( "%.4f", (double)values[j]);
			}
			buf->cat("\n");
		}
	}

	return GCodeResult::ok;
}


void ScrewMap::RunSelfTest() noexcept
{
	debugPrintf("SCREWMAP: Starting screwmap test\n");
	TestScrewMap();
}

void ScrewMap::ClearScrewMaps() noexcept
{
	for (size_t i=0; i<MaxAxes; i++)
	{
		ClearMap(&screwInfos[i]);
	}
}

void ScrewMap::ClearMap(ScrewMapInfo* psmi) noexcept
{
	if (psmi->mapTable != nullptr)
	{
		delete[] psmi->mapTable;
		psmi->mapTable = nullptr;
	}
}

// for a mapped axis, find the table row corresponding to findAxis
static float* FindTableRow(const ScrewMapInfo& smi, unsigned int findAxis) noexcept
{
	// axis is disable from mapping
	if (smi.mapTable == nullptr)
		return nullptr;

	volatile int foundAxis = -1;
	smi.usedAxes.IterateWhile( [&foundAxis, findAxis](unsigned int subaxis, unsigned int counter) noexcept
	{
		if (findAxis == subaxis)
		{
			foundAxis = (int)counter;
			return false;
		}
		return true;
	});
	return (foundAxis == -1) ? nullptr : &smi.mapTable[foundAxis * smi.count];
}

// set the screw map properties and allocate the map table area
bool ScrewMap::SetScrewMap(unsigned int axis, float start, float increment, unsigned int count, const AxesBitmap& axes) noexcept
{
	ScrewMapInfo& smi = screwInfos[axis];
	ClearMap(&smi);
	InvalidateCache();

	smi.start = start;
	smi.increment = increment;
	smi.count = count;
	smi.usedAxes.SetFromRaw(axes.GetRaw());

	// we have one list per altered axis
	int numBits = smi.usedAxes.CountSetBits();

	// initialize the map table to all zeros (default)
	smi.mapTable = (count > 0 && numBits > 0) ? (new float[count * numBits]) : nullptr;

	// did the init work?
	return (count == 0) || smi.mapTable != nullptr;
}

bool ScrewMap::SetScrewAxis(unsigned int srcAxis, unsigned int destAxis, float* deltas, size_t count, size_t dataOffset) noexcept
{
	ScrewMapInfo& smi = screwInfos[srcAxis];
	InvalidateCache();
	if((count + dataOffset) > smi.count)
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Screw map has %d list entries, trying to set %d\n", smi.count, count + dataOffset );
		if ( dataOffset >= smi.count)
			return false;
		count = smi.count - dataOffset;		// most possible
	}

	// copy the list into the destination map table
	float* destTable = FindTableRow(smi, destAxis);
	if (destTable != nullptr)
	{
		size_t i;
		for (i=0; i<count; i++)
		{
			destTable[i + dataOffset] = deltas[i];
		}
		// now validate monotonicity
		bool isMonotone = true;
		for (i=1; i<smi.count; i++)
		{
			isMonotone = (destTable[i] + deltas[i] + smi.increment) > (destTable[i-1] + deltas[i-1]);
			if(!isMonotone)
				break;
		}
		if(!isMonotone)
		{
			reprap.GetPlatform().MessageF(WarningMessage, "Screw map set is not monotonically increasing after set at %d\n", dataOffset);
		}
	}
	else
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Attempt to set unused screw map row %d:%d with map %d\n", srcAxis, destAxis, smi.usedAxes.GetRaw() );
	}

	UpdatePositions();
	// did we copy data?
	return destTable != nullptr;
}

// apply screw mapping transform to input coord
void ScrewMap::TransformAxis(float xyzCoord[MaxAxes], const ScrewMapInfo& smi, unsigned int axis) noexcept
{
	// start by figuring out where source coord is in the table
	float fdelta = xyzCoord[axis] - smi.start;
	const int idx = floor(fdelta / smi.increment);	// index of axis coord within table
	const float remainder = (fdelta - idx * smi.increment)/smi.increment;			// delta between coords

	// now adjust all coords in usedAxes with their map tables
	smi.usedAxes.Iterate( [smi, xyzCoord, remainder, idx](unsigned int subaxis, unsigned int counter) noexcept
	{
		const float* table = &smi.mapTable[counter * smi.count];		// offset to the subaxis table
		float adjust = 0.0f;
		if (idx < 0 || smi.count == 1)
			adjust = table[0];					// use constant delta if less than min
		else if (idx >= (smi.count-1))
			adjust = table[smi.count-1];		// use constant delta if greater than max
		else
		{
			// in range of the corrections so linear interpolate
			adjust = table[idx] + remainder * (table[idx+1]-table[idx]);
		}
		xyzCoord[subaxis] += adjust;
	});

}

// Find original coord given screw-mapped coord
// For the inverse transform to work the total screw map (pos+delta) must be monotonic increasing
void ScrewMap::InverseTransformAxis(float xyzCoord[MaxAxes], const ScrewMapInfo& smi, unsigned int axis) noexcept
{
	int idx = 0;
	float remainder = 0.0f;
	float* table = FindTableRow(smi, axis);	// get the source axis translate table

	if (table == nullptr)
	{
		// it's being used but has no translate table for itself so it's an identity
		float fdelta = xyzCoord[axis] - smi.start;
		idx = floor(fdelta / smi.increment);	// index of axis coord within table
		remainder = (fdelta - idx * smi.increment)/smi.increment;			// delta between coords
	}
	else
	{
		// find the bucket (idx,remainder) that xyzCoord[axis] fits into
		// find the indices we are between by a simple search
		float coord = xyzCoord[axis] - smi.start;
		if (coord <= table[0] || smi.count == 1)
		{
			idx = -1;
		}
		else if (coord >= (table[smi.count-1] + (smi.count-1) * smi.increment))
		{
			idx = smi.count;
		}
		else
		{
			// Data is between two indices; find the indices coord is between
			for (idx = 1; coord > (table[idx] + idx * smi.increment); idx++)
			{
				// Nothing to do here, we're just finding
			}
			idx--;
			// get remainder = 0...1 ratio of coord between the two endvalues
			remainder = (coord - (table[idx] + idx * smi.increment))/(table[idx+1] + smi.increment - table[idx]);
		}
	}

	// now iterate over all the axes that need changing
	smi.usedAxes.Iterate([smi, xyzCoord, idx, remainder](unsigned int subaxis, unsigned int counter) noexcept
	{
		float adjust = 0.0f;
		float* subtable = &smi.mapTable[counter * smi.count];		// offset to the axis table

		// less than min or greater than max are simple constant adjustments
		if (idx < 0)
		{
			adjust = subtable[0];
		}
		else if (idx >= (smi.count-1))
		{
			adjust = subtable[smi.count - 1];
		}
		else
		{
			// linear interpolate
			adjust = subtable[idx] + remainder * (subtable[idx+1] - subtable[idx]);
		}
		xyzCoord[subaxis] -= adjust;
	});
}

static void printDebugCoord(const char* msg, const float* xyzCoord)
{
	if (reprap.Debug(moduleMove))
	{
		debugPrintf(msg, (double)xyzCoord[0], (double)xyzCoord[1], (double)xyzCoord[2]);
	}
}

// if we've changed mapping, update the displayed position
// without moving the head
void ScrewMap::UpdatePositions() noexcept
{
	// this seems to work...along with the UpdateCurrentUserPosition call in GCodes2
	// Adjust the motor endpoints to allow for the change to endstop adjustments
	float vs[MaxAxes] = {0.0f};
	InvalidateCache();
	reprap.GetMove().AdjustMotorPositions(vs, MaxAxes);	// steps -> machine pos
	reprap.MoveUpdated();	// ? does this help
}

// transform the coordinates for all axes
bool ScrewMap::ScrewMapTransform(float xyzCoord[]) const noexcept
{
	if( !IsEnabled())
		return false;
		
	printDebugCoord("SM: Move to coord: %f,%f,%f\n", xyzCoord);
	const size_t numTotalAxes = reprap.GetGCodes().GetTotalAxes();
	size_t i;
	for (i=0; i<numTotalAxes; i++)
	{
		LastPosn[i] = xyzCoord[i];
	}
	for (unsigned int axis = 0; axis < numTotalAxes; axis++)
	{
		const ScrewMapInfo& smi = screwInfos[axis];
		// transform in place so multiple axes compound
		if (smi.IsEnabled())
			TransformAxis(xyzCoord, smi, axis);
	}
	for (i=0; i<numTotalAxes; i++)
	{
		LastTranslate[i] = xyzCoord[i];
	}

	printDebugCoord("SM: a.Mapped to coord: %f,%f,%f\n", xyzCoord);
	return true;
}

// inverse transform the coordinates for all axes
bool ScrewMap::ScrewMapInverseTransform(float xyzCoord[]) const noexcept
{
	if( !IsEnabled())
		return false;

	printDebugCoord("SM: InvMove to : %f,%f,%f\n", xyzCoord);
	const size_t numTotalAxes = reprap.GetGCodes().GetTotalAxes();

	// if the cached value is the current value, just use it for speed
	size_t i = -1;
	for (i=0; i<numTotalAxes; i++)
	{
		if ( LastTranslate[i] != xyzCoord[i])
			break;
	}
	if (i == numTotalAxes)
	{
		for (i=0; i<numTotalAxes; i++)
		{
			xyzCoord[i] = LastPosn[i];
		}
		printDebugCoord("SM: CacheInv to : %f,%f,%f\n", xyzCoord);
		return true;
	}

	// do the inverse transform in reverse order
	for (int axis = numTotalAxes-1; axis >=0; axis--)
	{
		const ScrewMapInfo& smi = screwInfos[axis];
		// reverse transform in place so multiple axes de-compound
		if (smi.IsEnabled())
			InverseTransformAxis(xyzCoord, smi, axis);
	}

	printDebugCoord("SM: inv.Mapped to : %f,%f,%f\n", xyzCoord);
	return true;
}

#if TESTING_SCREW_MAP
// some simple self-test stuff
// random number we won't use to mark pair endings
#define PAIR_END -132.0f

// simple set array values for testing
static void SetFloatArray(float* fltArray, float start, float increment)
{
	for (size_t i=0; i<10; i++)
	{
		fltArray[i] = i * increment + start;
	}
}

static void PrintAssert(bool bTest, const char* fmt, char axis, float f1, float f2)
{
	if (!bTest)
	{
		debugPrintf(fmt, axis, (double)f1, (double)f2, double(f1-f2));
	}
}

static bool ApproxEq(float f1, float f2)
{
	if (f1 != f2)
	{
		return (abs(f1-f2) < .00001f);
	}
	return true;
}

// ensure the map and inverse agree with our expectation for X and Y
static void AxisCompare(const ScrewMap* myMap, float* calc, float* expected)
{
	float xyzCopy[MaxAxes];
	size_t i;
	for (i=0; i<MaxAxes; i++)
	{
		xyzCopy[i] = calc[i];		// copy the input data
	}
	myMap->ScrewMapTransform(calc);
	const char* axisLetters = reprap.GetGCodes().GetAxisLetters();
	for (i=0; i<MaxAxes && axisLetters[i]; i++)
	{
		if (calc[i] != 0 || expected[i] != 0)
			PrintAssert(ApproxEq(calc[i], expected[i]), "Invalid %c map. Expected %f got %f err=%f.\n", axisLetters[i], expected[i], calc[i]);
	}
	myMap->InvalidateCache();
	myMap->ScrewMapInverseTransform(calc);
	for (i=0; i<MaxAxes && axisLetters[i]; i++)
	{
		if (calc[i] != 0 || xyzCopy[i] != 0)
			PrintAssert(ApproxEq(calc[i], xyzCopy[i]), "Invalid %c inverse map. Expected %f got %f err=%f.\n", axisLetters[i], xyzCopy[i], calc[i]);
	}
}

static void BasicTestPair(const ScrewMap* myMap, float calc, float expected, int inaxis, int outaxis)
{
	float xyzCoord[MaxAxes];		// x,y,z maps
	float xyzExpect[MaxAxes];		// x,y,z maps
	for (size_t inc=0; inc<MaxAxes; inc++)
	{
		xyzCoord[inc] = 0.0f;
		xyzExpect[inc] = 0.0f;
	}
	xyzCoord[inaxis]	 = calc;
	if (outaxis != inaxis)
	{
		xyzExpect[inaxis] = calc;	// no change in X
	}
	xyzExpect[outaxis] = expected;
	AxisCompare(myMap, xyzCoord, xyzExpect);
}

static void BasicTest(const ScrewMap* myMap, const float* matval, int inaxis, int outaxis)
{
	while (PAIR_END != *matval)
	{
		BasicTestPair( myMap, *matval, *(1+matval), inaxis, outaxis);
		matval += 2;
	}
}

void ScrewMap::TestScrewMap() noexcept
{
	float fsets[10];

	// axis, start, increment, count, AxesBitmap& axes
	// test A::: X->X with stretch of 1.0f
	AxesBitmap mapx = AxesBitmap::MakeFromBits(X_AXIS);
	AxesBitmap mapxy = AxesBitmap::MakeFromBits(X_AXIS, Y_AXIS);
	AxesBitmap mapy = AxesBitmap::MakeFromBits(Y_AXIS);

	debugPrintf ("Debugging Test Screw Map\n");
	debugPrintf ("----------------------------\n");

	debugPrintf("\nSCREW SEQUENCE: Single Axis Diagnostics\n");
	debugPrintf("\nSCREW TEST: Single value offset\n");
	ClearScrewMaps();
	SetScrewMap(X_AXIS, 0.0f, 1.0f, 1, mapx);
	SetFloatArray(fsets, 1.0f, 0.0f);	// shift single right
	SetScrewAxis(X_AXIS, X_AXIS, fsets, 1);
	const float SinglePairs[][2] = {{-0.1f, 0.9f}, {0.14f, 1.14f}, {PAIR_END, 0.0f}};
	BasicTest(this, &SinglePairs[0][0], 0, 0);

	debugPrintf("\nSCREW TEST: Dual value offset\n");
	SetScrewMap(X_AXIS, 0.0f, 1.0f, 2, mapx);
	SetFloatArray(fsets, -1.0f, 2.0f);	// -1,1
	SetScrewAxis(X_AXIS, X_AXIS, fsets, 2);
	const float DualPairs[][2] = {{-0.1f, -1.1f}, {0.25f, -0.25f}, {0.5f, 0.5f}, {1.0f, 2.0f}, {1.1f, 2.1f}, {PAIR_END, 0.0f}};
	BasicTest(this, &DualPairs[0][0], 0, 0);

	debugPrintf("\nSCREW TEST: 10 stretch test\n");
	SetScrewMap(X_AXIS, 0.0f, 1.0f, 10, mapx);
	SetFloatArray(fsets, 0.0f, 1.0f);	// 0,1,2,3....
	SetScrewAxis(X_AXIS, X_AXIS, fsets, 10);
	const float StretchPairs[][2] = {{0.0f, 0.0f}, {0.5f, 1.0f}, {1.0f, 2.0f}, {1.5f, 3.0f}, {1.6f, 3.2f}, {-1.0f, -1.0f}, {12.0f, 21.0f}, {PAIR_END, 0.0f}};
	BasicTest(this, &StretchPairs[0][0], 0, 0);

	debugPrintf("\nSCREW TEST: offset test\n");
	SetScrewMap(X_AXIS, 350.0f, 50.0f, 6, mapx);
	SetFloatArray(fsets, 0.0f, 1.0f);	// 0,1,2,3....
	float offsetFloats[] = {0.0f, 12.0f, 12.0f, 24.0f, 24.0f, 0.0f};
	SetScrewAxis(X_AXIS, X_AXIS, offsetFloats, 6);
	const float OffsetPairs[][2] = {{349.0f, 349.0f}, {350.0f, 350.0f}, {375.0f, 381.0f}, {400.0f, 412.0f}, 
				{410.0f, 422.0f}, {450.0f, 462.0f}, {475.0f, 493.0f}, {600.0f, 600.0f}, {PAIR_END, 0.0f}};
	BasicTest(this, &OffsetPairs[0][0], 0, 0);

	debugPrintf("\nSCREW TEST: only Y test\n");
	ClearScrewMaps();	// remove the x mapping
	SetScrewMap(Y_AXIS, 0.0f, 1.0f, 10, mapy);
	SetFloatArray(fsets, 0.0f, 1.0f);	// stretch a lot 0,1,2,3....
	SetScrewAxis(Y_AXIS, Y_AXIS, fsets, 10);
	const float TestYX0Pairs[][2] = {{0.0f, 0.0f}, {0.5f, 1.0f}, {1.0f, 2.0f}, {1.5f, 3.0f}, {1.6f, 3.2f}, {-1.0f, -1.0f}, {12.0f, 21.0f}, {PAIR_END, 0.0f}};
	BasicTest(this, &TestYX0Pairs[0][0], 1, 1);

	// now adjust Y from X
	debugPrintf("\nSCREW SEQUENCE: Two Axis Diagnostics\n");
	debugPrintf("\nSCREW TEST: X to Y test\n");
	ClearScrewMaps();	// remove the x mapping
	SetScrewMap(X_AXIS, 0.0f, 1.0f, 10, mapy);
	SetFloatArray(fsets, 0.0f, 1.0f);	// 0,1,2,3,4,5,6,7,8,9
	SetScrewAxis(X_AXIS, Y_AXIS, fsets, 10);
	const float TestYPairs[][2] = {{0.0f, 0.0f}, {0.5f, 0.5f}, {1.0f, 1.0f}, {1.5f, 1.5f}, {1.6f, 1.6f}, {-1.0f, 0.0f}, {12.0f, 9.0f}, {PAIR_END, 0.0f}};
	BasicTest(this, &TestYPairs[0][0], 0, 1);

	debugPrintf("\nSCREW TEST: X0, Y test\n");
	ClearScrewMaps();	// remove the xy mapping
	SetScrewMap(X_AXIS, 0.0f, 1.0f, 10, mapxy);
	SetFloatArray(fsets, 0.0f, 0.0f);	// no X adjustment using zeros
	SetScrewAxis(X_AXIS, X_AXIS, fsets, 10);
	SetFloatArray(fsets, 0.0f, 1.0f);	// stretch a lot 0,1,2,3....
	SetScrewAxis(X_AXIS, Y_AXIS, fsets, 10);
	const float TestX0YPairs[][2] = {{0.0f, 0.0f}, {0.5f, 0.5f}, {1.0f, 1.0f}, {1.5f, 1.5f}, {1.6f, 1.6f}, {-1.0f, 0.0f}, {12.0f, 9.0f}, {PAIR_END, 0.0f}};
	BasicTest(this, &TestX0YPairs[0][0], 0, 1);

	debugPrintf("\nSCREW TEST: Y to X test\n");
	ClearScrewMaps();	// remove the x mapping
	SetScrewMap(Y_AXIS, 0.0f, 1.0f, 10, mapx);
	SetFloatArray(fsets, 0.0f, 1.0f);	// stretch a lot 0,1,2,3....
	SetScrewAxis(Y_AXIS, X_AXIS, fsets, 10);
	const float TestYXPairs[][2] = {{0.0f, 0.0f}, {0.5f, 0.5f}, {1.0f, 1.0f}, {1.5f, 1.5f}, {1.6f, 1.6f}, {-1.0f, 0.0f}, {12.0f, 9.0f}, {PAIR_END, 0.0f}};
	BasicTest(this, &TestYXPairs[0][0], 1, 0);

	ClearScrewMaps();
}
#else
void ScrewMap::TestScrewMap() noexcept
{
}
#endif
