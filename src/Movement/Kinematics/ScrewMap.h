/*
 * ScrewMap.h
 *
 *  Created on: 11 Dec 2020
 *      Author: MarkZ
 */

#ifndef SCREWMAP_H_
#define SCREWMAP_H_

#include "RepRapFirmware.h"
#include "GCodes/GCodeResult.h"

// each axis can change a set of axes. Usually this is X,Y,Z
typedef struct tagScrewMapInfo
{
	uint8_t count;				// # of entries
	float start;				// Starting coordinate
	float increment;			// Grid size
	AxesBitmap usedAxes;		// set of axes that are altered by this axis
	float* mapTable = nullptr;	// Allocated #floats == count * NumbitsSet(usedAxes)
	inline bool IsEnabled() const noexcept { return mapTable != nullptr; }
} ScrewMapInfo;

 class ScrewMap
 {
 public:
	ScrewMap();
	virtual ~ScrewMap();
	GCodeResult ParseEnable(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult ParseCreate(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult ParseTable(GCodeBuffer& gb, const StringRef& reply, OutputBuffer*& buf) THROWS(GCodeException);
	bool IsEnabled() const noexcept { return IsMapEnabled; }
	const char* GetEnabledString() const noexcept;
	bool SetEnabled(bool newEnable) noexcept;
	void ClearScrewMaps() noexcept;
	bool SetScrewMap(unsigned int axis, float start, float increment, unsigned int count, const AxesBitmap& axes) noexcept;		// allocate the map table
	bool SetScrewAxis(unsigned int srcAxis, unsigned int destAxis, float* deltas, size_t count, size_t dataOffset=0) noexcept;
	bool ScrewMapTransform(float xyzCoord[MaxAxes]) const noexcept;			// transform all axes
	bool ScrewMapInverseTransform(float xyzCoord[MaxAxes]) const noexcept;	// inverse transform all axes
	void RunSelfTest() noexcept;
	static void InvalidateCache() noexcept;	// cache is global

 private:
	static void UpdatePositions() noexcept;	// update global pos
	static void ClearMap(ScrewMapInfo* psmi) noexcept;
	static void TransformAxis(float xyzCoord[MaxAxes], const ScrewMapInfo& smi, unsigned int axis) noexcept;
	static void InverseTransformAxis(float xyzCoord[MaxAxes], const ScrewMapInfo& smi, unsigned int axis) noexcept;
	void TestScrewMap() noexcept;

	bool IsMapEnabled;
	ScrewMapInfo screwInfos[MaxAxes];
 };

 #endif
