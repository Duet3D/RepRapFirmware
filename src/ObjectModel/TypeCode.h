/*
 * TypeCode.h
 *
 *  Created on: 27 Sept 2022
 *      Author: David
 */

#ifndef SRC_OBJECTMODEL_TYPECODE_H_
#define SRC_OBJECTMODEL_TYPECODE_H_

#include <cstdint>

#define SUPPORT_BITMAP64	(SAME70 || SAME5x)		// we don't have any 64-bit bitmaps in the OM on Duet 2

// Type codes to indicate what type of expression we have and how it is represented.
// The "Special" type is for items that we have to evaluate when we are ready to write them out, in particular strings whose storage might disappear.
enum class TypeCode : uint8_t
{
	None = 0,
	Bool,
	Char,
	Uint32,
	Int32,
	Uint64,				// only 56 bits actually available
	Float,
	Bitmap16,
	Bitmap32,
#if SUPPORT_BITMAP64
	Bitmap64,			// only 56 bits actually available
#else
	Bitmap64_unused,
#endif
	Enum32,
	ObjectModel_tc,		// renamed for eCv to avoid clash with class ObjectModel
	CString,
	HeapString,
	HeapArray,
	IPAddress_tc,		// renamed for eCv to avoid clash with class IPAddress in RRFLibraries
	ObjectModelArray,
	DateTime_tc,		// renamed for eCv to avoid clash with class DateTime
	DriverId_tc,		// renamed for eCv to avoid clash with class DriverId
	MacAddress_tc,		// renamed for eCv to avoid clash with class MacAddress
	Special,
	Port,
	UniqueId_tc,
	Duration,			// a duration represented an unsigned number of seconds (used by the 12864 LCD code)
#if SUPPORT_CAN_EXPANSION
	CanExpansionBoardDetails
#endif
};

#endif /* SRC_OBJECTMODEL_TYPECODE_H_ */
