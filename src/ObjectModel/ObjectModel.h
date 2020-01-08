/*
 * ObjectModel.h
 *
 *  Created on: 27 Aug 2018
 *      Author: David
 */

#ifndef SRC_OBJECTMODEL_OBJECTMODEL_H_
#define SRC_OBJECTMODEL_OBJECTMODEL_H_

#include <RepRapFirmware.h>
#include <GCodes/GCodeException.h>

#if SUPPORT_OBJECT_MODEL

#include <General/IPAddress.h>
#include <RTOSIface/RTOSIface.h>

typedef uint8_t TypeCode;
constexpr TypeCode NoType = 0;							// code for an invalid or unknown type

// Dummy types, used to define type codes
class Bitmap32;
class Enum32;
class ObjectModel;					// forward declaration
class ObjectModelArrayDescriptor;	// forward declaration

// Function template used to get constexpr type codes
// Each type must return a unique type code in the range 1 to 127 (0 is NoType)
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

// Struct used to hold the expressions with polymorphic types
struct ExpressionValue
{
	TypeCode type;									// what type is stored in the union
	uint8_t param;									// additional parameter, e.g. number of usual displayed decimal places for a float, or table # for an ObjectModel
	union
	{
		bool bVal;
		float fVal;
		int32_t iVal;
		uint32_t uVal;								// used for enumerations, bitmaps and IP addresses (not for integers, we always use int32_t for those)
		const char *sVal;
		const ObjectModel *omVal;					// object of some class derived form ObkectModel
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
	void Set(const char *s) noexcept { type = TYPE_OF(const char*); sVal = s; }
};

enum class ObjectModelReportFlags : uint16_t
{
	none = 0,
	shortForm = 1
};

// Flags field of a table entry
enum class ObjectModelEntryFlags : uint8_t
{
	none = 0,				// nothing special
	live = 1,				// fast changing data, included in common status response
	canAlter = 2,			// we can alter this value
};

// Context passed to object model functions
class ObjectExplorationContext
{
public:
	ObjectExplorationContext(ObjectModelReportFlags rf, ObjectModelEntryFlags ff) noexcept : numIndices(0), reportFlags(rf), filterFlags(ff) { }

	void AddIndex(unsigned int index) THROWS_GCODE_EXCEPTION;
	void RemoveIndex() THROWS_GCODE_EXCEPTION;
	unsigned int GetIndex(size_t n) const THROWS_GCODE_EXCEPTION;
	size_t GetNumIndices() const noexcept { return numIndices; }
	ObjectModelReportFlags GetReportFlags() const noexcept { return reportFlags; }
	ObjectModelEntryFlags GetFilterFlags() const noexcept { return filterFlags; }
	bool ShortFormReport() const noexcept { return ((uint16_t)reportFlags & (uint16_t)ObjectModelReportFlags::shortForm) != 0; }

private:
	static constexpr size_t MaxIndices = 4;			// max depth of array nesting

	size_t numIndices;								// the number of indices stored
	unsigned int indices[MaxIndices];
	ObjectModelReportFlags reportFlags;
	ObjectModelEntryFlags filterFlags;
};

// Entry to describe an array of objects or values. These must be brace-initializable into flash memory.
class ObjectModelArrayDescriptor
{
public:
	size_t (*GetNumElements)(const ObjectModel*, const ObjectExplorationContext&) noexcept;
	ExpressionValue (*GetElement)(const ObjectModel*, ObjectExplorationContext&) noexcept;
};

// Class from which other classes that represent part of the object model are derived
class ObjectModel
{
public:
	ObjectModel() noexcept;

	// Construct a JSON representation of those parts of the object model requested by the user. This version is called on the root of the tree.
	bool ReportAsJson(OutputBuffer *buf, const char *filter, ObjectModelReportFlags rf, ObjectModelEntryFlags ff) const THROWS_GCODE_EXCEPTION;

	// Get the value of an object. This version is called on the root of the tree.
	ExpressionValue GetObjectValue(const StringParser& sp, const char *idString) const THROWS_GCODE_EXCEPTION;

	// Function to report a value or object as JSON
	bool ReportItemAsJson(OutputBuffer *buf, ObjectExplorationContext& context, ExpressionValue val, const char *filter) const THROWS_GCODE_EXCEPTION;

protected:
	// Construct a JSON representation of those parts of the object model requested by the user
	bool ReportAsJson(OutputBuffer *buf, ObjectExplorationContext& context, uint8_t tableNumber, const char *filter) const THROWS_GCODE_EXCEPTION;

	// Report an entire array as JSON
	bool ReportArrayAsJson(OutputBuffer *buf, ObjectExplorationContext& context, const ObjectModelArrayDescriptor *omad, const char *filter) const THROWS_GCODE_EXCEPTION;

	// Get the value of an object
	ExpressionValue GetObjectValue(const StringParser& sp, ObjectExplorationContext& context, uint8_t tableNumber, const char *idString) const THROWS_GCODE_EXCEPTION;

	// Get the object model table entry for the current level object in the query
	const ObjectModelTableEntry *FindObjectModelTableEntry(uint8_t tableNumber, const char *idString) const noexcept;

	// Skip the current element in the ID or filter string
	static const char* GetNextElement(const char *id) noexcept;

	virtual const ObjectModelTableEntry *GetObjectModelTable(const uint8_t*& descriptor) const noexcept = 0;
};

// Object model table entry
// It must be possible to construct these in the form of initialised data in flash memory, to avoid using large amounts of RAM.
// Therefore we can't use a class hierarchy to represent different types of entry.
class ObjectModelTableEntry
{
public:
	// Type declarations
	// Type of the function pointer in the table entry, that returns the data
	typedef ExpressionValue(*DataFetchPtr_t)(const ObjectModel*, ObjectExplorationContext&) noexcept;

	// Member data. This must be public so that we can brace-initialise table entries.
	const char * name;				// name of this field
	DataFetchPtr_t func;			// function that yields this value
	ObjectModelEntryFlags flags;	// information about this value

	// Member functions. These must all be 'const'.

	// Return true if this object table entry matches a filter or query
	bool Matches(const char *filter, const ObjectExplorationContext& context) const noexcept;

	// See whether we should add the value of this element to the buffer, returning true if it matched the filter and we did add it
	bool ReportAsJson(OutputBuffer* buf, ObjectExplorationContext& context, const ObjectModel *self, const char* filter) const noexcept;

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

#define OBJECT_MODEL_FUNC_BODY(_class,...) [] (const ObjectModel* arg, ObjectExplorationContext& context) noexcept { const _class * const self = static_cast<const _class*>(arg); return ExpressionValue(__VA_ARGS__); }
#define OBJECT_MODEL_FUNC_NOSELF(...) [] (const ObjectModel* arg, ObjectExplorationContext& context) noexcept { return ExpressionValue(__VA_ARGS__); }

#else

#define INHERIT_OBJECT_MODEL			// nothing
#define DECLARE_OBJECT_MODEL			// nothing
#define DEFINE_GET_OBJECT_MODEL_TABLE	// nothing

#endif

#endif /* SRC_OBJECTMODEL_OBJECTMODEL_H_ */
