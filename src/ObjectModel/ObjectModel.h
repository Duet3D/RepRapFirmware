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
#include <General/Bitmap.h>
#include <RTOSIface/RTOSIface.h>
#include <Networking/NetworkDefs.h>

typedef uint8_t TypeCode;
constexpr TypeCode NoType = 0;							// code for an invalid or unknown type

// Dummy types, used to define type codes
class Bitmap32;
class Bitmap64;
class Enum32;

#if SUPPORT_CAN_EXPANSION

class CanExpansionBoardDetails;

enum class ExpansionDetail : uint32_t
{
	shortName, firmwareVersion, firmwareFileName
};

#endif

class ObjectModel;					// forward declaration
class ObjectModelArrayDescriptor;	// forward declaration

// Encapsulated time_t, used to facilitate overloading the ExpressionValue constructor
struct DateTime
{
	DateTime(time_t t) : tim(t) { }

	time_t tim;
};

// Function template used to get constexpr type codes
// Each type must return a unique type code in the range 1 to 127 (0 is NoType)
template<class T> constexpr TypeCode TypeOf() noexcept;

template<> constexpr TypeCode TypeOf<bool>								() noexcept { return 1; }
template<> constexpr TypeCode TypeOf<char>								() noexcept { return 2; }
template<> constexpr TypeCode TypeOf<uint32_t>							() noexcept { return 3; }
template<> constexpr TypeCode TypeOf<int32_t>							() noexcept { return 4; }
template<> constexpr TypeCode TypeOf<float>								() noexcept { return 5; }
template<> constexpr TypeCode TypeOf<Bitmap<uint16_t>>					() noexcept { return 6; }
template<> constexpr TypeCode TypeOf<Bitmap<uint32_t>>					() noexcept { return 7; }
template<> constexpr TypeCode TypeOf<Bitmap<uint64_t>>					() noexcept { return 8; }
template<> constexpr TypeCode TypeOf<Enum32>							() noexcept { return 9; }
template<> constexpr TypeCode TypeOf<const ObjectModel*>				() noexcept { return 10; }
template<> constexpr TypeCode TypeOf<const char*>						() noexcept { return 11; }
template<> constexpr TypeCode TypeOf<IPAddress>							() noexcept { return 12; }
template<> constexpr TypeCode TypeOf<const ObjectModelArrayDescriptor*>	() noexcept { return 13; }
template<> constexpr TypeCode TypeOf<DateTime>							() noexcept	{ return 14; }
template<> constexpr TypeCode TypeOf<DriverId>							() noexcept { return 15; }
template<> constexpr TypeCode TypeOf<MacAddress>						() noexcept { return 16; }

#if SUPPORT_CAN_EXPANSION
template<> constexpr TypeCode TypeOf<CanExpansionBoardDetails>			() noexcept { return 17; }
#endif

#define TYPE_OF(_t) (TypeOf<_t>())

// Forward declarations
class ObjectModelTableEntry;
class ObjectModel;

// Struct used to hold the expressions with polymorphic types
struct ExpressionValue
{
	uint32_t type : 8,								// what type is stored in the union
			 param : 24;							// additional parameter, e.g. number of usual displayed decimal places for a float,
													// or table # for an ObjectModel, or 24 extra bits for a date/time or a long bitmap
	union
	{
		bool bVal;
		char cVal;
		float fVal;
		int32_t iVal;
		uint32_t uVal;								// used for enumerations, bitmaps and IP addresses (not for integers, we always use int32_t for those)
		const char *sVal;
		const ObjectModel *omVal;					// object of some class derived form ObjectModel
		const ObjectModelArrayDescriptor *omadVal;
	};

	ExpressionValue() noexcept : type(NoType) { }
	explicit constexpr ExpressionValue(bool b) noexcept : type(TYPE_OF(bool)), param(0), bVal(b) { }
	explicit constexpr ExpressionValue(char c) noexcept : type(TYPE_OF(char)), param(0), cVal(c) { }
	explicit constexpr ExpressionValue(float f) noexcept : type(TYPE_OF(float)), param(MaxFloatDigitsDisplayedAfterPoint), fVal(f) { }
	constexpr ExpressionValue(float f, uint8_t numDecimalPlaces) noexcept : type(TYPE_OF(float)), param(numDecimalPlaces), fVal(f) { }
	explicit constexpr ExpressionValue(int32_t i) noexcept : type(TYPE_OF(int32_t)), param(0), iVal(i) { }
	explicit constexpr ExpressionValue(const ObjectModel *om) noexcept : type(TYPE_OF(const ObjectModel*)), param(0), omVal(om) { }
	constexpr ExpressionValue(const ObjectModel *om, uint8_t tableNumber) noexcept : type(TYPE_OF(const ObjectModel*)), param(tableNumber), omVal(om) { }
	explicit constexpr ExpressionValue(const char *s) noexcept : type(TYPE_OF(const char*)), param(0), sVal(s) { }
	explicit constexpr ExpressionValue(const ObjectModelArrayDescriptor *omad) noexcept : type(TYPE_OF(const ObjectModelArrayDescriptor*)), param(0), omadVal(omad) { }
	explicit constexpr ExpressionValue(IPAddress ip) noexcept : type(TYPE_OF(IPAddress)), param(0), uVal(ip.GetV4LittleEndian()) { }
	explicit constexpr ExpressionValue(nullptr_t dummy) noexcept : type(NoType), param(0), uVal(0) { }
	explicit ExpressionValue(DateTime t) noexcept : type((t.tim == 0) ? NoType : TYPE_OF(DateTime)), param(t.tim >> 32), uVal((uint32_t)t.tim) { }
	explicit ExpressionValue(DriverId id) noexcept : type(TYPE_OF(DriverId)), param(0), uVal(id.AsU32()) { }
	explicit ExpressionValue(Bitmap<uint16_t> bm) noexcept : type(TYPE_OF(Bitmap<uint16_t>)), param(0), uVal(bm.GetRaw()) { }
	explicit ExpressionValue(Bitmap<uint32_t> bm) noexcept : type(TYPE_OF(Bitmap<uint32_t>)), param(0), uVal(bm.GetRaw()) { }
	explicit ExpressionValue(Bitmap<uint64_t> bm) noexcept : type(TYPE_OF(Bitmap<uint64_t>)), param(bm.GetRaw() >> 32), uVal((uint32_t)bm.GetRaw()) { }
	explicit ExpressionValue(const MacAddress& mac) noexcept;
#if SUPPORT_CAN_EXPANSION
	ExpressionValue(const char*s, ExpansionDetail p) noexcept : type(TYPE_OF(CanExpansionBoardDetails)), param((uint32_t)p), sVal(s) { }
#endif

	void Set(bool b) noexcept { type = TYPE_OF(bool); bVal = b; }
	void Set(char c) noexcept { type = TYPE_OF(char); cVal = c; }
	void Set(int32_t i) noexcept { type = TYPE_OF(int32_t); iVal = i; }
	void Set(float f) noexcept { type = TYPE_OF(float); fVal = f; param = 1; }
	void Set(const char *s) noexcept { type = TYPE_OF(const char*); sVal = s; }

	// Extract a 56-bit value that we have stored. Used to retrieve date/times and large bitmaps.
	uint64_t Get56BitValue() const noexcept { return ((uint64_t)param << 32) | uVal; }

	// Get the format string to use assuming this is a floating point number
	const char *GetFloatFormatString() const noexcept { return ::GetFloatFormatString(param); }

#if SUPPORT_CAN_EXPANSION
	void ExtractRequestedPart(const StringRef& rslt) pre(type == TYPE_OF(CanExpansionBoardDetails));
#endif
};

// Flags field of a table entry
enum class ObjectModelEntryFlags : uint8_t
{
	// none, live and verbose are alternatives occupying the bottom 2 bits
	none = 0,				// nothing special
	live = 1,				// fast changing data, included in common status response
	verbose = 2,			// omit reporting this value by default

	// canAlter can be or'ed in
	canAlter = 4,			// we can alter this value
	liveCanAlter = 5,		// we can alter this value
};

// Context passed to object model functions
class ObjectExplorationContext
{
public:
	ObjectExplorationContext(const char *reportFlags, bool wal, unsigned int initialMaxDepth, int p_line = -1, int p_col = -1) noexcept;

	void SetMaxDepth(unsigned int d) noexcept { maxDepth = d; }
	bool IncreaseDepth() noexcept { if (currentDepth < maxDepth) { ++currentDepth; return true; } return false; }
	void DecreaseDepth() noexcept { --currentDepth; }
	void AddIndex(int32_t index) THROWS_GCODE_EXCEPTION;
	void AddIndex() THROWS_GCODE_EXCEPTION;
	void RemoveIndex() THROWS_GCODE_EXCEPTION;
	void ProvideIndex(int32_t index) THROWS_GCODE_EXCEPTION;
	int32_t GetIndex(size_t n) const THROWS_GCODE_EXCEPTION;
	int32_t GetLastIndex() const THROWS_GCODE_EXCEPTION;
	size_t GetNumIndicesCounted() const noexcept { return numIndicesCounted; }
	bool ShortFormReport() const noexcept { return shortForm; }
	bool ShouldReport(const ObjectModelEntryFlags f) const noexcept;
	bool WantArrayLength() const noexcept { return wantArrayLength; }
	bool ShouldIncludeNulls() const noexcept { return includeNulls; }

	GCodeException ConstructParseException(const char *msg) const noexcept;

private:
	static constexpr size_t MaxIndices = 4;			// max depth of array nesting

	unsigned int maxDepth;
	unsigned int currentDepth;
	size_t numIndicesProvided;						// the number of indices provided, when we are doing a value lookup
	size_t numIndicesCounted;						// the number of indices passed in the search string
	int32_t indices[MaxIndices];
	int line;
	int column;
	bool shortForm;
	bool onlyLive;
	bool includeVerbose;
	bool wantArrayLength;
	bool includeNulls;
};

// Entry to describe an array of objects or values. These must be brace-initializable into flash memory.
class ObjectModelArrayDescriptor
{
public:
	ReadWriteLock *lockPointer;
	size_t (*GetNumElements)(const ObjectModel*, const ObjectExplorationContext&) noexcept;
	ExpressionValue (*GetElement)(const ObjectModel*, ObjectExplorationContext&) noexcept;
};

// Class from which other classes that represent part of the object model are derived
class ObjectModel
{
public:
	ObjectModel() noexcept;
	virtual ~ObjectModel() { }

	// Construct a JSON representation of those parts of the object model requested by the user. This version is called on the root of the tree.
	void ReportAsJson(OutputBuffer *buf, const char *filter, const char *reportFlags, bool wantArrayLength) const THROWS_GCODE_EXCEPTION;

	// Get the value of an object via the table
	ExpressionValue GetObjectValue(ObjectExplorationContext& context, const char *idString, uint8_t tableNumber = 0) const THROWS_GCODE_EXCEPTION;

	// Function to report a value or object as JSON
	void ReportItemAsJson(OutputBuffer *buf, ObjectExplorationContext& context, ExpressionValue val, const char *filter) const THROWS_GCODE_EXCEPTION;

	// Skip the current element in the ID or filter string
	static const char* GetNextElement(const char *id) noexcept;

protected:
	// Construct a JSON representation of those parts of the object model requested by the user
	void ReportAsJson(OutputBuffer *buf, ObjectExplorationContext& context, uint8_t tableNumber, const char *filter) const THROWS_GCODE_EXCEPTION;

	// Report an entire array as JSON
	void ReportArrayAsJson(OutputBuffer *buf, ObjectExplorationContext& context, const ObjectModelArrayDescriptor *omad, const char *filter) const THROWS_GCODE_EXCEPTION;

	// Get the value of an object that we hold
	ExpressionValue GetObjectValue(ObjectExplorationContext& context, ExpressionValue val, const char *idString) const THROWS_GCODE_EXCEPTION;

	// Get the object model table entry for the current level object in the query
	const ObjectModelTableEntry *FindObjectModelTableEntry(uint8_t tableNumber, const char *idString) const noexcept;

	virtual const ObjectModelTableEntry *GetObjectModelTable(const uint8_t*& descriptor) const noexcept = 0;
};

// Function used for compile-time check for the correct number of entries in an object model table
static inline constexpr size_t ArraySum(const uint8_t *arr, size_t numEntries) noexcept
{
	return (numEntries == 0) ? 0 : arr[0] + ArraySum(arr + 1, numEntries - 1);
}

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
	bool ReportAsJson(OutputBuffer* buf, ObjectExplorationContext& context, const ObjectModel *self, const char* filter, bool first) const noexcept;

	// Return the name of this field
	const char* GetName() const noexcept { return name; }

	// Compare the name of this field with the filter string that we are trying to match
	int IdCompare(const char *id) const noexcept;

	// Return true if a section of the OMT is ordered
	static inline constexpr bool IsOrdered(const ObjectModelTableEntry *omt, size_t len) noexcept
	{
		return len <= 1 || (strcmp(omt[1].name, omt[0].name) == 1 && IsOrdered(omt + 1, len - 1));
	}

	// Return true if a section of the OMT specified by the descriptor is ordered
	static inline constexpr bool IsOrdered(uint8_t sectionsLeft, const uint8_t *descriptorSection, const ObjectModelTableEntry *omt) noexcept
	{
		return sectionsLeft == 0 || (IsOrdered(omt, *descriptorSection) && IsOrdered(sectionsLeft - 1, descriptorSection + 1, omt + *descriptorSection));
	}

	// Return true if the whole OMT is ordered
	static inline constexpr bool IsOrdered(const uint8_t *descriptor, const ObjectModelTableEntry *omt) noexcept
	{
		return IsOrdered(descriptor[0], descriptor + 1, omt);
	}
};

// Use this macro to inherit form ObjectModel
#define INHERIT_OBJECT_MODEL	: public ObjectModel

// Use this macro in the 'protected' section of every class declaration that derived from ObjectModel
#define DECLARE_OBJECT_MODEL \
	const ObjectModelTableEntry *GetObjectModelTable(const uint8_t*& descriptor) const noexcept override; \
	static const ObjectModelTableEntry objectModelTable[]; \
	static const uint8_t objectModelTableDescriptor[];

#define DECLARE_OBJECT_MODEL_VIRTUAL \
	virtual const ObjectModelTableEntry *GetObjectModelTable(const uint8_t*& descriptor) const noexcept override = 0;

#define DESCRIPTOR_OK(_class) 	(ARRAY_SIZE(_class::objectModelTableDescriptor) == _class::objectModelTableDescriptor[0] + 1)
#define OMT_SIZE_OK(_class)		(ARRAY_SIZE(_class::objectModelTable) == ArraySum(_class::objectModelTableDescriptor + 1, ARRAY_SIZE(_class::objectModelTableDescriptor) - 1))
#define OMT_ORDERING_OK(_class)	(ObjectModelTableEntry::IsOrdered(_class::objectModelTableDescriptor, _class::objectModelTable))

#define DEFINE_GET_OBJECT_MODEL_TABLE(_class) \
	const ObjectModelTableEntry *_class::GetObjectModelTable(const uint8_t*& descriptor) const noexcept \
	{ \
		static_assert(DESCRIPTOR_OK(_class), "Bad descriptor length"); \
		static_assert(!DESCRIPTOR_OK(_class) || OMT_SIZE_OK(_class), "Mismatched object model table and descriptor"); \
		static_assert(!DESCRIPTOR_OK(_class) || !OMT_SIZE_OK(_class) || OMT_ORDERING_OK(_class), "Object model table must be ordered"); \
		descriptor = objectModelTableDescriptor; \
		return objectModelTable; \
	}

#define OBJECT_MODEL_FUNC_BODY(_class,...) [] (const ObjectModel* arg, ObjectExplorationContext& context) noexcept \
	{ const _class * const self = static_cast<const _class*>(arg); return ExpressionValue(__VA_ARGS__); }
#define OBJECT_MODEL_FUNC_IF_BODY(_class,_condition,...) [] (const ObjectModel* arg, ObjectExplorationContext& context) noexcept \
	{ const _class * const self = static_cast<const _class*>(arg); return (_condition) ? ExpressionValue(__VA_ARGS__) : ExpressionValue(nullptr); }
#define OBJECT_MODEL_FUNC_NOSELF(...) [] (const ObjectModel* arg, ObjectExplorationContext& context) noexcept { return ExpressionValue(__VA_ARGS__); }
#define OBJECT_MODEL_ARRAY(_name)	static const ObjectModelArrayDescriptor _name ## ArrayDescriptor;

#else

#define INHERIT_OBJECT_MODEL			// nothing
#define DECLARE_OBJECT_MODEL			// nothing
#define DECLARE_OBJECT_MODEL_VIRTUAL	// nothing
#define DEFINE_GET_OBJECT_MODEL_TABLE	// nothing
#define OBJECT_MODEL_ARRAY(_name)		// nothing

#endif

#endif /* SRC_OBJECTMODEL_OBJECTMODEL_H_ */
