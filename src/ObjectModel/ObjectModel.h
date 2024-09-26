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
#include <Platform/StringHandle.h>
#include <Platform/ArrayHandle.h>

#include "TypeCode.h"
#include <General/IPAddress.h>
#include <General/Bitmap.h>
#include <RTOSIface/RTOSIface.h>
#include <Networking/NetworkDefs.h>
#include <Platform/OutputMemory.h>

#if SUPPORT_CAN_EXPANSION

enum class ExpansionDetail : uint32_t
{
	shortName, firmwareVersion, firmwareFileNameBin, firmwareFileNameUf2, firmwareDate, longName
};

#endif

// Forward declarations
class ObjectModel;
class ObjectModelArrayTableEntry;
class ObjectModelTableEntry;
class IoPort;
class UniqueId;

// Encapsulated time_t, used to facilitate overloading the ExpressionValue constructor
struct DateTime
{
	explicit DateTime(time_t t) noexcept : tim(t) { }

	time_t tim;
};

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
		const char *_ecv_array sVal;
		const ObjectModel *_ecv_from _ecv_null omVal;	// object of some class derived from ObjectModel. Object model arrays may include null objects, so allow nullptr here.
		StringHandle shVal;
		ArrayHandle ahVal;
		const IoPort *iopVal;
		const UniqueId *uniqueIdVal;
		uint32_t whole;								// a member we can use to copy the whole thing safely, at least as big as all the others. Assumes all other members are trivially copyable.
	};

	static_assert(sizeof(whole) >= sizeof(shVal));
	static_assert(sizeof(whole) >= sizeof(ahVal));
	static_assert(sizeof(whole) >= sizeof(omVal));
	static_assert(sizeof(whole) >= sizeof(fVal));

	enum class SpecialType : uint32_t
	{
		sysDir = 0,
	};

	ExpressionValue() noexcept : type((uint32_t)TypeCode::None) { }
	explicit constexpr ExpressionValue(bool b) noexcept : type((uint32_t)TypeCode::Bool), param(0), bVal(b) { }
	explicit constexpr ExpressionValue(char c) noexcept : type((uint32_t)TypeCode::Char), param(0), cVal(c) { }
	explicit constexpr ExpressionValue(float f) noexcept : type((uint32_t)TypeCode::Float), param(MaxFloatDigitsDisplayedAfterPoint), fVal(f) { }
	constexpr ExpressionValue(float f, uint8_t numDecimalPlaces) noexcept : type((uint32_t)TypeCode::Float), param(numDecimalPlaces), fVal(f) { }
	explicit constexpr ExpressionValue(int32_t i) noexcept : type((uint32_t)TypeCode::Int32), param(0), iVal(i) { }
	explicit constexpr ExpressionValue(FilePosition p) noexcept : type((uint32_t)TypeCode::Uint32), param(0), uVal(p) { }
	explicit ExpressionValue(uint64_t u) noexcept : type((uint32_t)TypeCode::Uint64) { Set56BitValue(u); }
	explicit constexpr ExpressionValue(const ObjectModel *_ecv_from _ecv_null om) noexcept : type((om == nullptr) ? (uint32_t)TypeCode::None : (uint32_t)TypeCode::ObjectModel_tc), param(0), omVal(om) { }
	constexpr ExpressionValue(const ObjectModel *_ecv_from _ecv_null om, uint8_t tableNumber) noexcept : type((om == nullptr) ? (uint32_t)TypeCode::None : (uint32_t)TypeCode::ObjectModel_tc), param(tableNumber), omVal(om) { }
	constexpr ExpressionValue(const ObjectModel *_ecv_from om, uint8_t arrayNumber, bool dummy) noexcept : type((uint32_t)TypeCode::ObjectModelArray), param(arrayNumber), omVal(om) { }
	explicit constexpr ExpressionValue(const char *_ecv_array s) noexcept : type((uint32_t)TypeCode::CString), param(0), sVal(s) { }
	explicit constexpr ExpressionValue(IPAddress ip) noexcept : type((uint32_t)TypeCode::IPAddress_tc), param(0), uVal(ip.GetV4LittleEndian()) { }
	explicit constexpr ExpressionValue(std::nullptr_t dummy) noexcept : type((uint32_t)TypeCode::None), param(0), uVal(0) { }
	explicit ExpressionValue(DateTime t) noexcept : type((t.tim == 0) ? (uint32_t)TypeCode::None : (uint32_t)TypeCode::DateTime_tc) { Set56BitValue(t.tim); }

	explicit ExpressionValue(DriverId id) noexcept
		: type((uint32_t)TypeCode::DriverId_tc),
#if SUPPORT_CAN_EXPANSION
		  param(id.boardAddress),
#else
		  param(0),
#endif
		  uVal(id.localDriver)
	{
	}

	explicit ExpressionValue(Bitmap<uint16_t> bm) noexcept : type((uint32_t)TypeCode::Bitmap16), param(0), uVal(bm.GetRaw()) { }
	explicit ExpressionValue(Bitmap<uint32_t> bm) noexcept : type((uint32_t)TypeCode::Bitmap32), param(0), uVal(bm.GetRaw()) { }
#if SUPPORT_BITMAP64
	explicit ExpressionValue(Bitmap<uint64_t> bm) noexcept : type((uint32_t)TypeCode::Bitmap64) { Set56BitValue(bm.GetRaw()); }
#endif
	explicit ExpressionValue(const MacAddress& mac) noexcept;
	ExpressionValue(SpecialType s, uint32_t u) noexcept : type((uint32_t)TypeCode::Special), param((uint32_t)s), uVal(u) { }

	// NOTE: When using these two constructors, the reference count in the handle must already be high enough to account for this expression referring to it.
	// If the handle is newly created, that will normally be the case. If it already exists, the reference count will need to be incremented.
	explicit ExpressionValue(StringHandle h) noexcept : type((uint32_t)TypeCode::HeapString), param(0), shVal(h) { }
	explicit ExpressionValue(AutoStringHandle h) noexcept : type((uint32_t)TypeCode::HeapString), param(0), shVal(h.IncreaseRefCount()) { }
	explicit ExpressionValue(ArrayHandle h) noexcept : type((uint32_t)TypeCode::HeapArray), param(0), ahVal(h) { }
	// End note

	explicit ExpressionValue(const IoPort& p) noexcept : type((uint32_t)TypeCode::Port), param(0), iopVal(&p) { }
	explicit ExpressionValue(const UniqueId& id) noexcept : type((uint32_t)TypeCode::UniqueId_tc), param(0), uniqueIdVal(&id) { }
#if SUPPORT_CAN_EXPANSION
	ExpressionValue(const char *_ecv_array s, ExpansionDetail p) noexcept : type((uint32_t)TypeCode::CanExpansionBoardDetails), param((uint32_t)p), sVal(s) { }
#endif

	bool operator==(const ExpressionValue& other) const noexcept;
	bool operator!=(const ExpressionValue& other) const noexcept { return !operator==(other); }

	ExpressionValue(const ExpressionValue& other) noexcept;
	ExpressionValue(ExpressionValue&& other) noexcept;
	~ExpressionValue();
	ExpressionValue& operator=(const ExpressionValue& other) noexcept;
	void Release() noexcept;					// release any associated storage

	TypeCode GetType() const noexcept { return (TypeCode)type; }
	bool IsStringType() const noexcept { return type == (uint32_t)TypeCode::CString || type == (uint32_t)TypeCode::HeapString; }
	bool IsHeapStringArrayType() const noexcept;

	void SetBool(bool b) noexcept;
	void SetInt(int32_t i) noexcept;
	void SetChar(char c) noexcept;
	void SetFloat(float f, uint32_t digits) noexcept;
	void SetFloat(float f) noexcept { SetFloat(f, MaxFloatDigitsDisplayedAfterPoint); }
	void SetCString(const char *_ecv_array s) noexcept { Release(); type = (uint32_t)TypeCode::CString; sVal = s; }
	void SetIPAddress(IPAddress ip) noexcept { Release(); type = (uint32_t)TypeCode::IPAddress_tc; uVal = ip.GetV4LittleEndian(); }
	void SetDriverId(DriverId did) noexcept;

	void SetStringHandle(StringHandle sh) noexcept { Release(); type = (uint32_t)TypeCode::HeapString; shVal = sh; }
	void SetArrayHandle(ArrayHandle ah) noexcept { Release(); type = (uint32_t)TypeCode::HeapArray; ahVal = ah; }
	void SetNull(std::nullptr_t dummy) noexcept { Release();  type = (uint32_t)TypeCode::None; }
	void SetDateTime(time_t t) noexcept { Release(); type = (uint32_t)TypeCode::DateTime_tc; Set56BitValue(t); }

	void SetUnsigned(uint32_t u) noexcept { Release(); type = (uint32_t)TypeCode::Uint32; uVal = u; }
	void SetDuration(uint32_t u) noexcept { Release(); type = (uint32_t)TypeCode::Duration; uVal = u; }

	// Store a 56-bit value
	void Set56BitValue(uint64_t v) { param = (uint32_t)(v >> 32) & 0x00FFFFFFu; uVal = (uint32_t)v; }

	// Extract a 56-bit value that we have stored. Used to retrieve date/times and large bitmaps.
	uint64_t Get56BitValue() const noexcept { return ((uint64_t)param << 32) | uVal; }

	// Extract a driver ID value
	DriverId GetDriverIdValue() const noexcept
#if SUPPORT_CAN_EXPANSION
	{ return DriverId(param, uVal); }
#else
	{ return DriverId(uVal); }
#endif

	// Get the format string to use assuming this is a floating point number
	const char *_ecv_array GetFloatFormatString() const noexcept { return ::GetFloatFormatString(fVal, param); }

	// Append a string representation of this value to a string
	void AppendAsString(const StringRef& str) const noexcept;

#if SUPPORT_CAN_EXPANSION
	void ExtractRequestedPart(const StringRef& rslt) const noexcept pre(type == TYPE_OF(CanExpansionBoardDetails));
#endif
};

// Flags field of a table entry
enum class ObjectModelEntryFlags : uint8_t
{
	// none, live and verbose are alternatives occupying the bottom 2 bits
	none = 0,					// nothing special
	live = 1,					// fast changing data, included in common status response
	important = 2,				// important when it is present, so include in unsolicited responses to PanelDue
	liveOrImportantMask = 3,	// mask to select values flagged as either live or important
	verbose = 4,				// omit reporting this value by default
	obsolete = 8				// entry is deprecated and should not be used any more
};

// Context passed to object model functions
class ObjectExplorationContext
{
public:
	// Constructor used when reporting the OM as JSON
	ObjectExplorationContext(const GCodeBuffer *_ecv_null gbp, bool wal, const char *reportFlags, unsigned int initialMaxDepth, size_t initialBufferOffset) noexcept;

	// Constructor used when evaluating expressions
	ObjectExplorationContext(const GCodeBuffer *_ecv_null gbp, bool wal, bool wex, int p_line, int p_col) noexcept;

	// Constructor used for generating a context to pass array indices
	ObjectExplorationContext() noexcept;

	const GCodeBuffer *_ecv_null GetGCodeBuffer() const noexcept { return gb; }
	void SetMaxDepth(unsigned int d) noexcept { maxDepth = d; }
	bool IncreaseDepth() noexcept { if (currentDepth < maxDepth) { ++currentDepth; return true; } return false; }
	void DecreaseDepth() noexcept { --currentDepth; }
	void AddIndex(int32_t index) THROWS(GCodeException);
	void AddIndex() THROWS(GCodeException);
	void RemoveIndex() THROWS(GCodeException);
	void ProvideIndex(int32_t index) THROWS(GCodeException);
	int32_t GetIndex(size_t n) const noexcept;
	int32_t GetLastIndex() const noexcept;
	size_t GetNumIndicesCounted() const noexcept { return numIndicesCounted; }
	unsigned int GetStartElement() const noexcept { return startElement; }
	void SetNextElement(int arg) noexcept { nextElement = arg; }
	int GetNextElement() const noexcept { return nextElement; }
	bool ShortFormReport() const noexcept { return shortForm; }
	bool ShouldReport(const ObjectModelEntryFlags f) const noexcept;
	bool WantArrayLength() const noexcept { return wantArrayLength; }
	bool WantExists() const noexcept { return wantExists; }
	bool ShouldIncludeNulls() const noexcept { return includeNulls; }
	bool ShouldIncludeImportant() const noexcept { return includeImportant; }
	bool TruncateLongArrays() const noexcept { return truncateLongArrays; }
	uint64_t GetStartMillis() const { return startMillis; }
	size_t GetInitialBufferOffset() const noexcept { return initialBufOffset; }

	bool ObsoleteFieldQueried() const noexcept { return obsoleteFieldQueried; }
	void SetObsoleteFieldQueried() noexcept { obsoleteFieldQueried = true; }

	GCodeException ConstructParseException(const char *msg) const noexcept;
	GCodeException ConstructParseException(const char *msg, const char *sparam) const noexcept;
	void CheckStack(uint32_t calledFunctionStackUsage) const THROWS(GCodeException);

private:
	uint64_t startMillis;							// the milliseconds counter when we started exploring the OM. Stored so that upTime and msUpTime are consistent.
	size_t initialBufOffset;
	unsigned int maxDepth;
	unsigned int currentDepth;
	unsigned int startElement;
	int nextElement;
	size_t numIndicesProvided;						// the number of indices provided, when we are doing a value lookup
	size_t numIndicesCounted;						// the number of indices passed in the search string
	int32_t indices[MaxExpressionArrayIndices];
	int line;
	int column;
	const GCodeBuffer *_ecv_null gb;
	unsigned int shortForm : 1,
				wantArrayLength : 1,
				wantExists : 1,
				includeNonLive : 1,
				includeImportant : 1,
				includeNulls : 1,
				excludeVerbose : 1,
				excludeObsolete : 1,
				obsoleteFieldQueried : 1,
				truncateLongArrays : 1;
};

// Entry to describe an array of objects or values. These must be brace-initializable into flash memory.
class ObjectModelArrayTableEntry
{
public:
	ReadWriteLock *null lockPointer;
	size_t (*GetNumElements)(const ObjectModel *_ecv_from, const ObjectExplorationContext&) noexcept;
	ExpressionValue (*GetElement)(const ObjectModel *_ecv_from, ObjectExplorationContext&) noexcept;
};

struct ObjectModelClassDescriptor;

// Class from which other classes that represent part of the object model are derived
class ObjectModel
{
public:
	ObjectModel() noexcept;
	virtual ~ObjectModel() { }

	// Forwarding function so that we can make GetObjectModelArrayEntry() protected
	const ObjectModelArrayTableEntry *FindObjectModelArrayEntry(unsigned int index) const noexcept { return GetObjectModelArrayEntry(index); }

	// Construct a JSON representation of those parts of the object model requested by the user. This version is called only on the root of the tree.
	void ReportAsJson(const GCodeBuffer *_ecv_null gb, OutputBuffer *buf, const char *_ecv_array filter, const char *_ecv_array reportFlags, bool wantArrayLength) const THROWS(GCodeException);

	// Get the value of an object via the table
	ExpressionValue GetObjectValueUsingTableNumber(ObjectExplorationContext& context, const ObjectModelClassDescriptor * null classDescriptor, const char *_ecv_array idString, uint8_t tableNumber) const THROWS(GCodeException);

	// Function to report a value or object as JSON. This does not need to handle 'var' or 'global' because those are checked for before this is called.
	void ReportItemAsJson(OutputBuffer *buf, ObjectExplorationContext& context, const ObjectModelClassDescriptor *_ecv_null classDescriptor,
							const ExpressionValue& val, const char *_ecv_array filter) const THROWS(GCodeException);

	// Skip the current element in the ID or filter string
	static const char *_ecv_array GetNextElement(const char *_ecv_array id) noexcept;

protected:
	// Construct a JSON representation of those parts of the object model requested by the user
	// Overridden in class GlobalVariables
	virtual void ReportAsJson(OutputBuffer *buf, ObjectExplorationContext& context, const ObjectModelClassDescriptor * null classDescriptor, uint8_t tableNumber, const char *_ecv_array filter) const THROWS(GCodeException);

	// Report an entire array as JSON
	void ReportObjectModelArrayAsJson(OutputBuffer *buf, ObjectExplorationContext& context, const ObjectModelClassDescriptor *null classDescriptor, const ObjectModelArrayTableEntry *entry, const char *_ecv_array filter) const THROWS(GCodeException);

	// Report an entire array as JSON
	void ReportHeapArrayAsJson(OutputBuffer *buf, ObjectExplorationContext& context, const ObjectModelClassDescriptor *null classDescriptor, ArrayHandle ah, const char *_ecv_array filter) const THROWS(GCodeException);

	// Get the value of an object that we hold
	ExpressionValue GetObjectValue(ObjectExplorationContext& context, const ObjectModelClassDescriptor *classDescriptor, const ExpressionValue& val, const char *_ecv_array idString) const THROWS(GCodeException);

	// Get the object model table entry for the current level object in the query
	const ObjectModelTableEntry *FindObjectModelTableEntry(const ObjectModelClassDescriptor *classDescriptor, uint8_t tableNumber, const char *_ecv_array idString) const noexcept;

	virtual const ObjectModelClassDescriptor *_ecv_null GetObjectModelClassDescriptor() const noexcept = 0;

	// Get the requested entry in the array table
	virtual const ObjectModelArrayTableEntry *_ecv_null GetObjectModelArrayEntry(unsigned int index) const noexcept { return nullptr; }

	// Return the address of the ReadWriteLock (if any) that we need to acquire before querying or reporting on this object
	// Override this default implementation in classes that need to be locked. If the returned lock belongs to the current object
	// then it must be declared 'mutable' because this function is const, like the querying and reporting functions
	virtual ReadWriteLock *_ecv_null GetObjectLock(unsigned int tableNumber) const noexcept { return nullptr; }

private:
	// These functions have been separated from ReportItemAsJson to avoid high stack usage in the recursive functions, therefore they must not be inlined
	// Report on a single item
	__attribute__ ((noinline)) void ReportItemAsJsonFull(OutputBuffer *buf, ObjectExplorationContext& context, const ObjectModelClassDescriptor *_ecv_null classDescriptor,
															const ExpressionValue& val, const char *_ecv_array filter) const THROWS(GCodeException);
	__attribute__ ((noinline)) void ReportArrayLengthAsJson(OutputBuffer *buf, ObjectExplorationContext& context, const ExpressionValue& val) const noexcept;
	__attribute__ ((noinline)) static void ReportDateTime(OutputBuffer *buf, const ExpressionValue& val) noexcept;
	__attribute__ ((noinline)) static void ReportFloat(OutputBuffer *buf, const ExpressionValue& val) noexcept;
	__attribute__ ((noinline)) static void ReportBitmap1632Long(OutputBuffer *buf, const ExpressionValue& val) noexcept;
	__attribute__ ((noinline)) static void ReportBitmap64Long(OutputBuffer *buf, const ExpressionValue& val) noexcept;
	__attribute__ ((noinline)) static void ReportPinNameAsJson(OutputBuffer *buf, const ExpressionValue& val) noexcept;

#if SUPPORT_CAN_EXPANSION
	__attribute__ ((noinline)) static void ReportExpansionBoardDetail(OutputBuffer *buf, const ExpressionValue& val) noexcept;
	__attribute__ ((noinline)) static ExpressionValue GetExpansionBoardDetailLength(const ExpressionValue& val) noexcept;
#endif

};

// Function to report a value or object as JSON
// This function is recursive, so keep its stack usage low.
// Most recursive calls are for non-array object values, so handle object values inline to reduce stack usage.
// This saves about 240 bytes of stack space but costs 272 bytes of flash memory.
inline void ObjectModel::ReportItemAsJson(OutputBuffer *buf, ObjectExplorationContext& context, const ObjectModelClassDescriptor *_ecv_null classDescriptor,
											const ExpressionValue& val, const char *_ecv_array filter) const THROWS(GCodeException)
{
	if (context.WantArrayLength() && *filter == 0)
	{
		ReportArrayLengthAsJson(buf, context, val);
	}
	else if (val.GetType() == TypeCode::ObjectModel_tc)
	{
		if (  (*filter != '.' && *filter != 0)		// we should have reached the end of the filter or a '.', error if not
			|| val.omVal == nullptr					// OM arrays may contain null entries, so we need to handle them here
		   )
		{
			buf->cat("null");
		}
		else
		{
			if (*filter == '.')
			{
				++filter;
			}
			not_null(val.omVal)->ReportAsJson(buf, context, (val.omVal == this) ? classDescriptor : nullptr, val.param, filter);
		}
	}
	else
	{
		ReportItemAsJsonFull(buf, context, classDescriptor, val, filter);
	}
}

// Function used for compile-time check for the correct number of entries in an object model table
static inline constexpr size_t ArraySum(const uint8_t *_ecv_array arr, size_t numEntries) noexcept
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
	typedef ExpressionValue(*DataFetchPtr_t)(const ObjectModel *_ecv_from, ObjectExplorationContext&) noexcept;

	// Member data. This must be public so that we can brace-initialise table entries.
	const char *_ecv_array name;		// name of this field
	DataFetchPtr_t func;				// function that yields this value
	ObjectModelEntryFlags flags;		// information about this value

	// Member functions. These must all be 'const'.

	// Return true if this object table entry matches a filter or query
	bool Matches(const char *filter, const ObjectExplorationContext& context) const noexcept;

	// Check if the queried field is obsolete
	bool IsObsolete() const noexcept { return ((uint8_t)flags & (uint8_t)ObjectModelEntryFlags::obsolete) != 0; }

	// See whether we should add the value of this element to the buffer, returning true if it matched the filter and we did add it
	bool ReportAsJson(OutputBuffer* buf, ObjectExplorationContext& context, const ObjectModelClassDescriptor *_ecv_null classDescriptor, const ObjectModel *_ecv_from self, const char *_ecv_array filter, bool first) const THROWS(GCodeException);

	// Return the name of this field
	const char *_ecv_array  GetName() const noexcept { return name; }

	// Compare the name of this field with the filter string that we are trying to match
	int IdCompare(const char *id) const noexcept;

	// Return true if a section of the OMT is ordered
	static inline constexpr bool IsOrdered(const ObjectModelTableEntry *_ecv_array omt, size_t len) noexcept
	{
		return len <= 1 || (strcmp(omt[1].name, omt[0].name) == 1 && IsOrdered(omt + 1, len - 1));
	}

	// Return true if a section of the OMT specified by the descriptor is ordered
	static inline constexpr bool IsOrdered(uint8_t sectionsLeft, const uint8_t *_ecv_array descriptorSection, const ObjectModelTableEntry *_ecv_array omt) noexcept
	{
		return sectionsLeft == 0 || (IsOrdered(omt, *descriptorSection) && IsOrdered(sectionsLeft - 1, descriptorSection + 1, omt + *descriptorSection));
	}

	// Return true if the whole OMT is ordered
	static inline constexpr bool IsOrdered(const uint8_t *_ecv_array descriptor, const ObjectModelTableEntry *_ecv_array omt) noexcept
	{
		return IsOrdered(descriptor[0], descriptor + 1, omt);
	}
};

struct ObjectModelClassDescriptor
{
	const ObjectModelTableEntry *_ecv_array omt;
	const uint8_t *_ecv_array omd;
	const ObjectModelClassDescriptor *_ecv_null parent;
};

// Use this macro to inherit from ObjectModel
#define INHERIT_OBJECT_MODEL	: public ObjectModel

// Use this macro in the 'protected' section of every class declaration that derived from ObjectModel
#define DECLARE_OBJECT_MODEL \
	const ObjectModelClassDescriptor *_ecv_null GetObjectModelClassDescriptor() const noexcept override; \
	static const ObjectModelTableEntry objectModelTable[]; \
	static const uint8_t objectModelTableDescriptor[]; \
	static const ObjectModelClassDescriptor objectModelClassDescriptor;

#define DECLARE_OBJECT_MODEL_WITH_ARRAYS \
	DECLARE_OBJECT_MODEL \
	static const ObjectModelArrayTableEntry objectModelArrayTable[]; \
	const ObjectModelArrayTableEntry *_ecv_null GetObjectModelArrayEntry(unsigned int index) const noexcept override;

#define DECLARE_OBJECT_MODEL_VIRTUAL \
	virtual const ObjectModelClassDescriptor *GetObjectModelClassDescriptor() const noexcept override = 0;

#define DESCRIPTOR_OK(_class) 	(ARRAY_SIZE(_class::objectModelTableDescriptor) == _class::objectModelTableDescriptor[0] + 1)
#define OMT_SIZE_OK(_class)		(ARRAY_SIZE(_class::objectModelTable) == ArraySum(_class::objectModelTableDescriptor + 1, ARRAY_SIZE(_class::objectModelTableDescriptor) - 1))
#define OMT_ORDERING_OK(_class)	(ObjectModelTableEntry::IsOrdered(_class::objectModelTableDescriptor, _class::objectModelTable))

#define DEFINE_GET_OBJECT_MODEL_TABLE(_class) \
	const ObjectModelClassDescriptor _class::objectModelClassDescriptor = { _class::objectModelTable, _class::objectModelTableDescriptor, nullptr }; \
	const ObjectModelClassDescriptor *_ecv_null _class::GetObjectModelClassDescriptor() const noexcept \
	{ \
		static_assert(DESCRIPTOR_OK(_class), "Bad descriptor length"); \
		static_assert(!DESCRIPTOR_OK(_class) || OMT_SIZE_OK(_class), "Mismatched object model table and descriptor"); \
		static_assert(!DESCRIPTOR_OK(_class) || !OMT_SIZE_OK(_class) || OMT_ORDERING_OK(_class), "Object model table must be ordered"); \
		return &objectModelClassDescriptor; \
	}

#define DEFINE_GET_OBJECT_MODEL_TABLE_WITH_PARENT(_class, _parent) \
	const ObjectModelClassDescriptor _class::objectModelClassDescriptor = { _class::objectModelTable, _class::objectModelTableDescriptor, &_parent::objectModelClassDescriptor }; \
	const ObjectModelClassDescriptor *_ecv_null _class::GetObjectModelClassDescriptor() const noexcept \
	{ \
		static_assert(DESCRIPTOR_OK(_class), "Bad descriptor length"); \
		static_assert(!DESCRIPTOR_OK(_class) || OMT_SIZE_OK(_class), "Mismatched object model table and descriptor"); \
		static_assert(!DESCRIPTOR_OK(_class) || !OMT_SIZE_OK(_class) || OMT_ORDERING_OK(_class), "Object model table must be ordered"); \
		return &objectModelClassDescriptor; \
	}

#define DEFINE_GET_OBJECT_MODEL_ARRAY_TABLE(_class) \
	constexpr unsigned int ArrayIndexOffset = 0; \
	const ObjectModelArrayTableEntry *_ecv_null _class::GetObjectModelArrayEntry(unsigned int index) const noexcept \
	{ \
		if (index < ARRAY_SIZE(_class::objectModelArrayTable)) \
		{ \
			return &_class::objectModelArrayTable[index]; \
		} \
		return ObjectModel::GetObjectModelArrayEntry(index); \
	}

#define DEFINE_GET_OBJECT_MODEL_ARRAY_TABLE_WITH_PARENT(_class,_parent,_offset) \
	constexpr unsigned int ArrayIndexOffset = _offset; \
	const ObjectModelArrayTableEntry *_ecv_null _class::GetObjectModelArrayEntry(unsigned int index) const noexcept \
	{ \
		if (index >= _offset && index < _offset + ARRAY_SIZE(_class::objectModelArrayTable)) \
		{ \
			return &_class::objectModelArrayTable[index - _offset]; \
		} \
		return _parent::GetObjectModelArrayEntry(index); \
	}

#define OBJECT_MODEL_FUNC_BODY(_class,...) [] (const ObjectModel *_ecv_from arg, ObjectExplorationContext& context) noexcept \
	{ const _class * const self = static_cast<const _class*>(arg); return ExpressionValue(__VA_ARGS__); }
#define OBJECT_MODEL_FUNC_IF_BODY(_class,_condition,...) [] (const ObjectModel *_ecv_from arg, ObjectExplorationContext& context) noexcept \
	{ const _class * const self = static_cast<const _class*>(arg); return (_condition) ? ExpressionValue(__VA_ARGS__) : ExpressionValue(nullptr); }
#define OBJECT_MODEL_FUNC_ARRAY(_index) [] (const ObjectModel *_ecv_from arg, ObjectExplorationContext& context) noexcept \
	{ \
		static_assert((unsigned int)_index >= ArrayIndexOffset); \
		static_assert((unsigned int)_index < sizeof(objectModelArrayTable)/sizeof(ObjectModelArrayTableEntry) + ArrayIndexOffset); \
		return ExpressionValue(arg, _index, true); \
	}
#define OBJECT_MODEL_FUNC_ARRAY_IF_BODY(_class,_condition,_index) [] (const ObjectModel *_ecv_from arg, ObjectExplorationContext& context) noexcept \
	{ \
		static_assert((unsigned int)_index >= ArrayIndexOffset); \
		static_assert((unsigned int)_index < sizeof(objectModelArrayTable)/sizeof(ObjectModelArrayTableEntry) + ArrayIndexOffset); \
		const _class * const self = static_cast<const _class*>(arg); \
		return (_condition) ? ExpressionValue(arg, _index, true) : ExpressionValue(nullptr); \
	}
#define OBJECT_MODEL_FUNC_NOSELF(...) [] (const ObjectModel *_ecv_from arg, ObjectExplorationContext& context) noexcept { return ExpressionValue(__VA_ARGS__); }
#define OBJECT_MODEL_FUNC_IF_NOSELF(_condition,...) [] (const ObjectModel *_ecv_from arg, ObjectExplorationContext& context) noexcept \
	{ return (_condition) ? ExpressionValue(__VA_ARGS__) : ExpressionValue(nullptr); }

#endif /* SRC_OBJECTMODEL_OBJECTMODEL_H_ */
