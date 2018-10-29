/*
 * ObjectModel.h
 *
 *  Created on: 27 Aug 2018
 *      Author: David
 */

#ifndef SRC_OBJECTMODEL_OBJECTMODEL_H_
#define SRC_OBJECTMODEL_OBJECTMODEL_H_

#include "RepRapFirmware.h"
#include <General/IPAddress.h>

#if SUPPORT_OBJECT_MODEL

typedef uint32_t ObjectModelFilterFlags;
typedef uint8_t TypeCode;
constexpr TypeCode IsArray = 128;						// this is or'ed in to a type code to indicate an array
constexpr TypeCode NoType = 0;							// code for an invalid or unknown type

// Forward declarations
class ObjectModelTableEntry;
class ObjectModel;

union ExpressionValue
{
	bool bVal;
	float fVal;
	int32_t iVal;
	uint32_t uVal;
	const char *sVal;
	const ObjectModel *omVal;
};

// Dummy types, used to define type codes
class Bitmap32;
class Enum32;
class Float2;			// float printed to 2 decimal places instead of 1
class Float3;			// float printed to 3 decimal places instead of 1

class ObjectModel
{
public:
	enum ReportFlags : uint16_t
	{
		flagsNone,
		flagShortForm = 1
	};

	ObjectModel();

	// Construct a JSON representation of those parts of the object model requested by the user
	bool ReportAsJson(OutputBuffer *buf, const char *filter, ReportFlags rflags);

	// Return the type of an object
	TypeCode GetObjectType(const char *idString);

	// Get the value of an object when we don't know what its type is
	TypeCode GetObjectValue(ExpressionValue& val, const char *idString);

	// Specialisation of above for float, allowing conversion from integer to float
	bool GetObjectValue(float& val, const char *idString);

	// Specialisation of above for int, allowing conversion from unsigned to signed
	bool GetObjectValue(int32_t& val, const char *idString);

	// Get the object model table entry for the current level object in the query
	const ObjectModelTableEntry *FindObjectModelTableEntry(const char *idString);

	// Skip the current element in the ID or filter string
	static const char* GetNextElement(const char *id);

protected:
	virtual const ObjectModelTableEntry *GetObjectModelTable(size_t& numEntries) const = 0;

private:
	// Get pointers to various types from the object model, returning null if failed
	template<class T> T* GetObjectPointer(const char* idString);

	const char **GetStringObjectPointer(const char *idString);
	uint32_t *GetShortEnumObjectPointer(const char *idString);
	uint32_t *GetBitmapObjectPointer(const char *idString);
};

// Function template used to get constexpr type IDs
// Each type must return a unique type code in the range 1 to 127
template<class T> constexpr TypeCode TypeOf();

template<> constexpr TypeCode TypeOf<bool> () { return 1; }
template<> constexpr TypeCode TypeOf<uint32_t> () { return 2; }
template<> constexpr TypeCode TypeOf<int32_t>() { return 3; }
template<> constexpr TypeCode TypeOf<float>() { return 4; }
template<> constexpr TypeCode TypeOf<Float2>() { return 5; }
template<> constexpr TypeCode TypeOf<Float3>() { return 6; }
template<> constexpr TypeCode TypeOf<Bitmap32>() { return 7; }
template<> constexpr TypeCode TypeOf<Enum32>() { return 8; }
template<> constexpr TypeCode TypeOf<ObjectModel>() { return 9; }
template<> constexpr TypeCode TypeOf<const char *>() { return 10; }
template<> constexpr TypeCode TypeOf<IPAddress>() { return 11; }

#define TYPE_OF(_t) (TypeOf<_t>())

// Entry to describe an array
class ObjectModelArrayDescriptor
{
public:
	size_t (*GetNumElements)(ObjectModel*);
	void * (*GetElement)(ObjectModel*, size_t);
};

// Object model table entry
// It must be possible to construct these in the form of initialised data in flash memory, to avoid using large amounts of RAM.
// Therefore we can't use a class hierarchy to represent different types of entry. Instead we use a type code and a void* parameter.
class ObjectModelTableEntry
{
public:
	// Type declarations
	// Flags field of a table entry
	enum ObjectModelEntryFlags : uint16_t
	{
		none = 0,				// nothing special
		live = 1,				// fast changing data, included in common status response
		canAlter = 2,			// we can alter this value
	};

	// Type of the function pointer in the table entry, that returns a pointer to the data
	typedef void *(*ParamFuncPtr_t)(ObjectModel*);

	// Member data. This must be public so that we can brace-initialise table entries.
	const char * name;				// name of this field
	ParamFuncPtr_t param;			// function that yields a pointer to this value
	TypeCode type;					// code for the type of this value
	ObjectModelEntryFlags flags;	// information about this value

	// Member functions. These must all be 'const'.

	// Return true if this object table entry matches a filter or query
	bool Matches(const char *filter, ObjectModelFilterFlags flags) const;

	// See whether we should add the value of this element to the buffer, returning true if it matched the filter and we did add it
	bool ReportAsJson(OutputBuffer* buf, ObjectModel *self, const char* filter, ObjectModel::ReportFlags flags) const;

	// Return the name of this field
	const char* GetName() const { return name; }

	// Compare the name of this field with the filter string that we are trying to match
	int IdCompare(const char *id) const;

	// Private function to report a value of primitive type
	static void ReportItemAsJson(OutputBuffer *buf, const char *filter, ObjectModel::ReportFlags flags, void *nParam, TypeCode type);
};

// Use this macro to inherit form ObjectModel
#define INHERIT_OBJECT_MODEL	: public ObjectModel

// Use this macro in the 'protected' section of every class declaration that derived from ObjectModel
#define DECLARE_OBJECT_MODEL \
	const ObjectModelTableEntry *GetObjectModelTable(size_t& numEntries) const override; \
	static const ObjectModelTableEntry objectModelTable[];

#define DEFINE_GET_OBJECT_MODEL_TABLE(_class) \
	const ObjectModelTableEntry *_class::GetObjectModelTable(size_t& numEntries) const \
	{ \
		numEntries = ARRAY_SIZE(objectModelTable); \
		return objectModelTable; \
	}

#define OBJECT_MODEL_FUNC_BODY(_class,_ret) [] (ObjectModel* arg) { _class * const self = static_cast<_class*>(arg); return (void *)(_ret); }
#define OBJECT_MODEL_FUNC_NOSELF(_ret) [] (ObjectModel* arg) { return (void *)(_ret); }

#else

#define INHERIT_OBJECT_MODEL			// nothing
#define DECLARE_OBJECT_MODEL			// nothing
#define DEFINE_GET_OBJECT_MODEL_TABLE	// nothing

#endif

#endif /* SRC_OBJECTMODEL_OBJECTMODEL_H_ */
