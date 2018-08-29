/*
 * ObjectModel.h
 *
 *  Created on: 27 Aug 2018
 *      Author: David
 */

#ifndef SRC_OBJECTMODEL_OBJECTMODEL_H_
#define SRC_OBJECTMODEL_OBJECTMODEL_H_

#include "RepRapFirmware.h"

#ifdef SUPPORT_OBJECT_MODEL

typedef uint32_t ObjectModelFilterFlags;
typedef uint8_t TypeCode;

// Dummy types, used to define type codes
class Bitmap32 { };
class Enum32 { };

// Forward declarations
class ObjectModelTableEntry;

class ObjectModel
{
public:
	enum ReportFlags : uint16_t
	{
		shortForm = 1
	};

	ObjectModel();

	// Construct a JSON representation of those parts of the object model requested by the user
	bool ReportAsJson(OutputBuffer *buf, const char *filter, ReportFlags rflags);

	// Get values of various types from the object model, returning true if successful
	template<class T> bool GetObjectValue(T& val, const char *idString);

	bool GetStringObjectValue(const StringRef& str, const char* idString) const;
	bool GetLongEnumObjectValue(const StringRef& str, const char *idString) const;
	bool GetShortEnumObjectValue(uint32_t &val, const char *idString) const;
	bool GetBitmapObjectValue(uint32_t &val, const char *idString) const;

	// Try to set values of various types from the object model, returning true if successful
	bool SetFloatObjectValue(float val, const char *idString);
	bool SetUnsignedObjectValue(uint32_t val, const char *idString);
	bool SetSignedObjectValue(int32_t val, const char *idString);
	bool SetStringObjectValue(const StringRef& str, const char *idString);
	bool SetLongEnumObjectValue(const StringRef& str, const char *idString);
	bool SetShortEnumObjectValue(uint32_t val, const char *idString);
	bool SetBitmapObjectValue(uint32_t val, const char *idString);
	bool SetBoolObjectValue(bool val, const char *idString);

	// Try to adjust values of various types from the object model, returning true if successful
	bool AdjustFloatObjectValue(float val, const char *idString);
	bool AdjustUnsignedObjectValue(int32_t val, const char *idString);
	bool AdjustSignedObjectValue(int32_t val, const char *idString);
	bool ToggleBoolObjectValue(const char *idString);

	// Get the object model table entry for the current level object in the query
	const ObjectModelTableEntry *FindObjectModelTableEntry(const char *idString);

	// Get the object model table entry for the leaf object in the query
	const ObjectModelTableEntry *FindObjectModelLeafEntry(const char *idString);
	// Skip the current element in the ID or filter string
	static const char* GetNextElement(const char *id);

protected:
	virtual const char *GetModuleName() const = 0;
	virtual const ObjectModelTableEntry *GetObjectModelTable(size_t& numEntries) const = 0;

private:
	// Get pointers to various types from the object model, returning null if failed
	template<class T> T* GetObjectPointer(const char* idString);

	const char **GetStringObjectPointer(const char *idString);
	uint32_t *GetShortEnumObjectPointer(const char *idString);
	uint32_t *GetBitmapObjectPointer(const char *idString);
};

// Function template used to get constexpr type IDs
template<class T> constexpr TypeCode TypeOf();

template<> constexpr TypeCode TypeOf<bool> () { return 1; }
template<> constexpr TypeCode TypeOf<uint32_t> () { return 2; }
template<> constexpr TypeCode TypeOf<int32_t>() { return 3; }
template<> constexpr TypeCode TypeOf<float>() { return 4; }
template<> constexpr TypeCode TypeOf<Bitmap32>() { return 5; }
template<> constexpr TypeCode TypeOf<Enum32>() { return 6; }
template<> constexpr TypeCode TypeOf<ObjectModel>() { return 7; }

#define TYPE_OF(_t) (TypeOf<_t>())

// Object model table entry
// It must be possible to construct these in the form of initialised data in flash memory, to avoid using large amounts of RAM.
// Therefore we can't use a class hierarchy to represent different types of entry. Instead we use a type code and a void* parameter.
// Only const member functions are allowed in this class
class ObjectModelTableEntry
{
public:
	enum ObjectModelEntryFlags : uint16_t
	{
		none = 0,
		live = 1,				// fast changing data, included in common status response
		canAlter = 2,			// we can alter this value
		isArray = 4			// value is an array of the basic type
	};

	typedef void *(*ParamFuncPtr_t)(ObjectModel*);

	// Return true if this object table entry matches a filter or query
	bool Matches(const char *filter, ObjectModelFilterFlags flags) const;

	// Add the value of this element to the buffer, returning true if it matched and we did
	bool ReportAsJson(OutputBuffer* buf, ObjectModel *self, const char* filter, ObjectModel::ReportFlags flags) const;

	const char* GetName() const { return name; }

	int IdCompare(const char *id) const;

	bool IsObject() const { return type == TYPE_OF(ObjectModel); }
	const ObjectModelTableEntry *FindLeafEntry(ObjectModel *self, const char *idString) const;

	// Check the type is correct, call the function if necessary and return the pointer
	void* GetValuePointer(ObjectModel *self, TypeCode t) const;

	// Note: all data members must be public so that we can brace-initialise these. This doesn't matter because they are always 'const'.
	const char * name;
	ParamFuncPtr_t param;
	TypeCode type;
	ObjectModelEntryFlags flags;
};

template<class T> bool ObjectModel::GetObjectValue(T& val, const char *idString)
{
	const ObjectModelTableEntry *e = FindObjectModelLeafEntry(idString);
	if (e == nullptr)
	{
		return false;
	}
	const T *p = (float*)(e->GetValuePointer(this, TYPE_OF(T)));
	if (p == nullptr)
	{
		return false;
	}
	val = *p;
	return true;
}

template<class T> T* ObjectModel::GetObjectPointer(const char* idString)
{
	const ObjectModelTableEntry *e = FindObjectModelLeafEntry(idString);
	return (e == nullptr) ? nullptr : (T*)(e->GetValuePointer(this, TYPE_OF(T)));
}

#endif

#endif /* SRC_OBJECTMODEL_OBJECTMODEL_H_ */
