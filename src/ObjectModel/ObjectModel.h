/*
 * ObjectModel.h
 *
 *  Created on: 27 Aug 2018
 *      Author: David
 */

#ifndef SRC_OBJECTMODEL_OBJECTMODEL_H_
#define SRC_OBJECTMODEL_OBJECTMODEL_H_

#include "RepRapFirmware.h"

#if SUPPORT_OBJECT_MODEL

#include <General/IPAddress.h>
#include <RTOSIface/RTOSIface.h>

typedef uint32_t ObjectModelFilterFlags;
typedef uint8_t TypeCode;
constexpr TypeCode NoType = 0;							// code for an invalid or unknown type

// Dummy types, used to define type codes
class Bitmap32;
class Enum32;
class ObjectModel;					// forward declaration
class ObjectModelArrayDescriptor;	// forward declaration

// Function template used to get constexpr type IDs
// Each type must return a unique type code in the range 1 to 127
template<class T> constexpr TypeCode TypeOf() noexcept;

template<> constexpr TypeCode TypeOf<bool>() noexcept { return 1; }
template<> constexpr TypeCode TypeOf<uint32_t>() noexcept { return 2; }
template<> constexpr TypeCode TypeOf<int32_t>() noexcept { return 3; }
template<> constexpr TypeCode TypeOf<float>() noexcept { return 4; }
template<> constexpr TypeCode TypeOf<Bitmap32>() noexcept { return 5; }
template<> constexpr TypeCode TypeOf<Enum32>() noexcept { return 6; }
template<> constexpr TypeCode TypeOf<const ObjectModel*>() noexcept { return 7; }
template<> constexpr TypeCode TypeOf<const char*>() noexcept { return 8; }
template<> constexpr TypeCode TypeOf<IPAddress>() noexcept { return 9; }
template<> constexpr TypeCode TypeOf<const ObjectModelArrayDescriptor*>() noexcept { return 10; }

#define TYPE_OF(_t) (TypeOf<_t>())

// Function used for compile-time check for the correct number of entries in an object model table
static inline constexpr size_t ArraySum(const uint8_t *arr, size_t numEntries)
{
	return (numEntries == 0) ? 0 : arr[0] + ArraySum(arr + 1, numEntries - 1);
}

// Forward declarations
class ObjectModelTableEntry;
class ObjectModel;
class StringParser;

struct ExpressionValue
{
	TypeCode type;
	uint8_t param;
	union
	{
		bool bVal;
		float fVal;
		int32_t iVal;
		uint32_t uVal;				// used for enumerations, bitmaps and IP addresses (not for integers, we always use int32_t for those)
		const char *sVal;
		const ObjectModel *omVal;
		const ObjectModelArrayDescriptor *omadVal;
	};

	ExpressionValue() noexcept : type(NoType) { }
	explicit constexpr ExpressionValue(bool b) noexcept : type(TYPE_OF(bool)), param(0), bVal(b) { }
	explicit constexpr ExpressionValue(float f) noexcept : type(TYPE_OF(float)), param(1), fVal(f) { }
	constexpr ExpressionValue(float f, uint8_t numDecimalPlaces) noexcept : type(TYPE_OF(float)), param(numDecimalPlaces), fVal(f) { }
	explicit constexpr ExpressionValue(int32_t i) noexcept : type(TYPE_OF(int32_t)), param(0), iVal(i) { }
	explicit constexpr ExpressionValue(const ObjectModel *om) noexcept : type(TYPE_OF(const ObjectModel*)), param(0), omVal(om) { }
	constexpr ExpressionValue(const ObjectModel *om, uint8_t tableNumber) noexcept : type(TYPE_OF(const ObjectModel*)), param(tableNumber), omVal(om) { }
	explicit constexpr ExpressionValue(const char *s) noexcept : type(TYPE_OF(const char*)), param(0), sVal(s) { }
	explicit constexpr ExpressionValue(const ObjectModelArrayDescriptor *omad) noexcept : type(TYPE_OF(const ObjectModelArrayDescriptor*)), param(0), omadVal(omad) { }
	explicit constexpr ExpressionValue(IPAddress ip) noexcept : type(TYPE_OF(IPAddress)), param(0), uVal(ip.GetV4LittleEndian()) { }
	explicit constexpr ExpressionValue(nullptr_t dummy) noexcept : type(NoType), param(0), uVal(0) { }

	void Set(bool b) noexcept { type = TYPE_OF(bool); bVal = b; }
	void Set(int32_t i) noexcept { type = TYPE_OF(int32_t); iVal = i; }
	void Set(int i) noexcept { type = TYPE_OF(int32_t); iVal = i; }
	void Set(float f) noexcept { type = TYPE_OF(float); fVal = f; param = 1; }
};

// Entry to describe an array. These should be brace-initializable in flash memory.
class ObjectModelArrayDescriptor
{
public:
	size_t (*GetNumElements)(const ObjectModel*) noexcept;
	ExpressionValue (*GetElement)(const ObjectModel*, size_t) noexcept;
};

class ObjectModel
{
public:
	enum ReportFlags : uint16_t
	{
		flagsNone,
		flagShortForm = 1
	};

	ObjectModel() noexcept;

	// Construct a JSON representation of those parts of the object model requested by the user
	bool ReportAsJson(OutputBuffer *buf, uint8_t tableNumber, const char *filter, ReportFlags rflags) const noexcept;

	// Report an entire array as JSON
	bool ReportArrayAsJson(OutputBuffer *buf, const ObjectModelArrayDescriptor *omad, const char *filter, ReportFlags rflags) const noexcept;

	// Function to report a value or object as JSON
	bool ReportItemAsJson(ExpressionValue val, OutputBuffer *buf, const char *filter, ObjectModel::ReportFlags flags) const noexcept;

	// Get the value of an object when we don't know what its type is
	ExpressionValue GetObjectValue(const StringParser& sp, uint8_t tableNumber, const char *idString) const THROWS_PARSE_ERROR;

	// Get the object model table entry for the current level object in the query
	const ObjectModelTableEntry *FindObjectModelTableEntry(uint8_t tableNumber, const char *idString) const noexcept;

	// Skip the current element in the ID or filter string
	static const char* GetNextElement(const char *id) noexcept;

protected:
	virtual const ObjectModelTableEntry *GetObjectModelTable(const uint8_t*& descriptor) const noexcept = 0;

private:
	// Get pointers to various types from the object model, returning null if failed
//	template<class T> T* GetObjectPointer(const char* idString) noexcept;

//	const char **GetStringObjectPointer(const char *idString) noexcept;
//	uint32_t *GetShortEnumObjectPointer(const char *idString) noexcept;
//	uint32_t *GetBitmapObjectPointer(const char *idString) noexcept;
};

// Flags field of a table entry
enum class ObjectModelEntryFlags : uint8_t
{
	none = 0,				// nothing special
	live = 1,				// fast changing data, included in common status response
	canAlter = 2,			// we can alter this value
};

// Object model table entry
// It must be possible to construct these in the form of initialised data in flash memory, to avoid using large amounts of RAM.
// Therefore we can't use a class hierarchy to represent different types of entry.
class ObjectModelTableEntry
{
public:
	// Type declarations
	// Type of the function pointer in the table entry, that returns the data
	typedef ExpressionValue(*DataFetchPtr_t)(const ObjectModel*) noexcept;

	// Member data. This must be public so that we can brace-initialise table entries.
	const char * name;				// name of this field
	DataFetchPtr_t func;			// function that yields this value
	ObjectModelEntryFlags flags;	// information about this value

	// Member functions. These must all be 'const'.

	// Return true if this object table entry matches a filter or query
	bool Matches(const char *filter, ObjectModelFilterFlags flags) const noexcept;

	// See whether we should add the value of this element to the buffer, returning true if it matched the filter and we did add it
	bool ReportAsJson(OutputBuffer* buf, const ObjectModel *self, const char* filter, ObjectModel::ReportFlags flags) const noexcept;

	// Return the name of this field
	const char* GetName() const noexcept { return name; }

	// Compare the name of this field with the filter string that we are trying to match
	int IdCompare(const char *id) const noexcept;
};

// Use this macro to inherit form ObjectModel
#define INHERIT_OBJECT_MODEL	: public ObjectModel

// Use this macro in the 'protected' section of every class declaration that derived from ObjectModel
#define DECLARE_OBJECT_MODEL \
	const ObjectModelTableEntry *GetObjectModelTable(const uint8_t*& descriptor) const noexcept override; \
	static const ObjectModelTableEntry objectModelTable[]; \
	static const uint8_t objectModelTableDescriptor[];

#define DEFINE_GET_OBJECT_MODEL_TABLE(_class) \
	const ObjectModelTableEntry *_class::GetObjectModelTable(const uint8_t*& descriptor) const noexcept \
	{ \
		static_assert(ARRAY_SIZE(_class::objectModelTableDescriptor) == _class::objectModelTableDescriptor[0] + 1, "Bad descriptor length"); \
		static_assert(ARRAY_SIZE(_class::objectModelTable) == ArraySum(_class::objectModelTableDescriptor + 1, ARRAY_SIZE(_class::objectModelTableDescriptor) - 1), \
				"Mismatched object model table and descriptor"); \
		descriptor = objectModelTableDescriptor; \
		return objectModelTable; \
	}

#define OBJECT_MODEL_FUNC_BODY(_class,...) [] (const ObjectModel* arg) noexcept { const _class * const self = static_cast<const _class*>(arg); return ExpressionValue(__VA_ARGS__); }
#define OBJECT_MODEL_FUNC_NOSELF(...) [] (const ObjectModel* arg) noexcept { return ExpressionValue(__VA_ARGS__); }

#else

#define INHERIT_OBJECT_MODEL			// nothing
#define DECLARE_OBJECT_MODEL			// nothing
#define DEFINE_GET_OBJECT_MODEL_TABLE	// nothing

#endif

#endif /* SRC_OBJECTMODEL_OBJECTMODEL_H_ */
