/*
 * ObjectModel.cpp
 *
 *  Created on: 27 Aug 2018
 *      Author: David
 */

#include "ObjectModel.h"

#if SUPPORT_OBJECT_MODEL

#include "OutputMemory.h"
#include <cstring>

// Constructor
ObjectModel::ObjectModel()
{
}

// Report this object
bool ObjectModel::ReportAsJson(OutputBuffer* buf, const char* filter, ReportFlags flags)
{
	buf->cat('{');
	size_t numEntries;
	const ObjectModelTableEntry *omte = GetObjectModelTable(numEntries);
	bool added = false;
	while (numEntries != 0)
	{
		if (omte->Matches(filter, flags))
		{
			if (added)
			{
				buf->cat(',');
			}
			omte->ReportAsJson(buf, this, filter, flags);
			added = true;
		}
		--numEntries;
		++omte;
	}
	buf->cat('}');
	return true;
}

// Find the requested entry
const ObjectModelTableEntry* ObjectModel::FindObjectModelTableEntry(const char* idString)
{
	size_t numElems;
	const ObjectModelTableEntry *tbl = GetObjectModelTable(numElems);
	size_t low = 0, high = numElems;
	while (high > low)
	{
		const size_t mid = (high - low)/2 + low;
		const int t = tbl[mid].IdCompare(idString);
		if (t == 0)
		{
			return &tbl[mid];
		}
		if (t > 0)
		{
			low = mid + 1u;
		}
		else
		{
			high = mid;
		}
	}
	return (low < numElems && tbl[low].IdCompare(idString) == 0) ? &tbl[low] : nullptr;
}

// Get the object model table entry for the leaf object in the query
const ObjectModelTableEntry *ObjectModel::FindObjectModelLeafEntry(const char *idString)
{
	const ObjectModelTableEntry *e = FindObjectModelTableEntry(idString);
	return (e == nullptr) ? e : e->FindLeafEntry(this, idString);
}

#if 0	// not implemented yet
bool ObjectModel::GetStringObjectValue(const StringRef& str, const char* idString) const
{
}

bool ObjectModel::GetLongEnumObjectValue(const StringRef& str, const char* idString) const
{
}

bool ObjectModel::GetShortEnumObjectValue(uint32_t& val, const char* idString) const
{
}

bool ObjectModel::GetBitmapObjectValue(uint32_t& val, const char* idString) const
{
}
#endif

#if 0
bool ObjectModel::SetFloatObjectValue(float val, const char* idString)
{
}

bool ObjectModel::SetUnsignedObjectValue(uint32_t val, const char* idString)
{
}

bool ObjectModel::SetSignedObjectValue(int32_t val, const char* idString)
{
}

bool ObjectModel::SetStringObjectValue(const StringRef& str, const char* idString)
{
}

bool ObjectModel::SetLongEnumObjectValue(const StringRef& str, const char* idString)
{
}

bool ObjectModel::SetShortEnumObjectValue(uint32_t val, const char* idString)
{
}

bool ObjectModel::SetBitmapObjectValue(uint32_t val, const char* idString)
{
}

bool ObjectModel::SetBoolObjectValue(bool val, const char* idString)
{
}

bool ObjectModel::AdjustFloatObjectValue(float val, const char* idString)
{
}

bool ObjectModel::AdjustUnsignedObjectValue(int32_t val, const char* idString)
{
}

bool ObjectModel::AdjustSignedObjectValue(int32_t val, const char* idString)
{
}

bool ObjectModel::ToggleBoolObjectValue(const char* idString)
{
}
#endif

const char** ObjectModel::GetStringObjectPointer(const char* idString)
{
	//TODO
	return nullptr;
}

uint32_t* ObjectModel::GetShortEnumObjectPointer(const char* idString)
{
	const ObjectModelTableEntry *e = FindObjectModelLeafEntry(idString);
	return (e == nullptr) ? nullptr : (uint32_t*)(e->GetValuePointer(this, TYPE_OF(Enum32)));
}

uint32_t* ObjectModel::GetBitmapObjectPointer(const char* idString)
{
	const ObjectModelTableEntry *e = FindObjectModelLeafEntry(idString);
	return (e == nullptr) ? nullptr : (uint32_t*)(e->GetValuePointer(this, TYPE_OF(Bitmap32)));
}

/*static*/ const char* ObjectModel::GetNextElement(const char *id)
{
	while (*id != 0 && *id != '.' && *id != '[')
	{
		++id;
	}
	if (*id == '.')
	{
		++id;
	}
	return id;
}

bool ObjectModelTableEntry::Matches(const char* filterString, ObjectModelFilterFlags filterFlags) const
{
	return IdCompare(filterString) == 0 && (flags & filterFlags) == filterFlags;
}

const ObjectModelTableEntry *ObjectModelTableEntry::FindLeafEntry(ObjectModel *self, const char *idString) const
{
	if (!IsObject())
	{
		return this;
	}

	return ((ObjectModel*)param(self))->FindObjectModelLeafEntry(ObjectModel::GetNextElement(idString));
}

// Private function to report a value of primitive type
void ObjectModelTableEntry::ReportItemAsJson(OutputBuffer *buf, const char *filter, ObjectModel::ReportFlags flags, void *nParam, TypeCode type)
{
	switch (type)
	{
	case TYPE_OF(ObjectModel):
		((ObjectModel*)nParam)->ReportAsJson(buf, filter, flags);
		break;

	case TYPE_OF(float):
		buf->catf("%.1f", (double)*(const float *)nParam);		//TODO different parameters need different number of decimal places
		break;

	case TYPE_OF(uint32_t):
		buf->catf("%" PRIu32, *(const uint32_t *)nParam);
		break;

	case TYPE_OF(int32_t):
		buf->catf("%" PRIi32, *(const int32_t *)nParam);
		break;

	case TYPE_OF(Bitmap32):
		if (flags & ObjectModel::flagShortForm)
		{
			buf->catf("%" PRIu32, *(const uint32_t *)nParam);
		}
		else
		{
			buf->cat('[');
			// TODO list the bits that are set
			buf->cat(']');
		}
		break;

	case TYPE_OF(Enum32):
		if (flags & ObjectModel::flagShortForm)
		{
			buf->catf("%" PRIu32, *(const uint32_t *)nParam);
		}
		else
		{
			buf->cat("\"unimplemented\"");
			// TODO append the real name
		}
		break;

	case TYPE_OF(bool):
		{
			const bool bVal = *(const bool *)nParam;
			if (flags & ObjectModel::flagShortForm)
			{
				buf->cat((bVal) ? '1' : '0');
			}
			else
			{
				buf->cat((bVal) ? "yes" : "no");
			}
		}
		break;

	case TYPE_OF(IPAddress):
		{
			const IPAddress ipVal = *(const IPAddress *)nParam;
			char sep = '"';
			for (unsigned int q = 0; q < 4; ++q)
			{
				buf->catf("%c%u", sep, ipVal.GetQuad(q));
				sep = '.';
			}
			buf->cat('"');
		}
		break;
	}
}

// Add the value of this element to the buffer, returning true if it matched and we did
bool ObjectModelTableEntry::ReportAsJson(OutputBuffer* buf, ObjectModel *self, const char* filter, ObjectModel::ReportFlags flags) const
{
	buf->cat(name);
	buf->cat(':');

	if ((type & IsArray) != 0)
	{
		// TODO match array indices
		buf->cat('[');
		const ObjectModelArrayDescriptor *arr = (const ObjectModelArrayDescriptor*)param(self);
		for (size_t i = 0; i < arr->GetNumElements(self); ++i)
		{
			if (i != 0)
			{
				buf->cat(',');
			}
			ReportItemAsJson(buf, filter, flags, arr->GetElement(self, i), type & ~IsArray);
		}
		buf->cat(']');
	}
	else
	{
		ReportItemAsJson(buf, filter, flags, param(self), type);
	}
	return true;
}

// Compare an ID with the name of this object
int ObjectModelTableEntry::IdCompare(const char *id) const
{
	if (id[0] == 0 || id[0] == '*')
	{
		return 0;
	}

	const char *n = name;
	while (*id == *n && *n != 0)
	{
		++id;
		++n;
	}
	return (*n == 0 && (*id == 0 || *id == '.' || *id == '[')) ? 0
		: (*id > *n) ? 1
			: -1;
}

// Check the type is correct, call the function if necessary and return the pointer
void* ObjectModelTableEntry::GetValuePointer(ObjectModel *self, TypeCode t) const
{
	if (t != type)
	{
		return nullptr;
	}
	return param(self);
}

// Return the type of an object
TypeCode ObjectModel::GetObjectType(const char *idString)
{
	const ObjectModelTableEntry * const e = FindObjectModelLeafEntry(idString);
	return (e == nullptr) ? NoType : e->type;
}

// Get the value of an object when we don't know what its type is
TypeCode ObjectModel::GetObjectValue(ExpressionValue& val, const char *idString)
{
	const ObjectModelTableEntry * const e = FindObjectModelLeafEntry(idString);
	if (e == nullptr)
	{
		return NoType;
	}
	switch (e->type)
	{
	case TYPE_OF(float):
		val.fVal = *((const float*)e->param(this));
		break;

	case TYPE_OF(uint32_t):
	case TYPE_OF(Bitmap32):
	case TYPE_OF(Enum32):
		val.uVal = *((const uint32_t*)e->param(this));
		break;

	case TYPE_OF(int32_t):
		val.iVal = *((const int32_t*)e->param(this));
		break;

	case TYPE_OF(const char*):
		val.sVal = *((const char* const *)e->param(this));
		break;

	default:
		break;
	}
	return e->type;
}

// Template specialisations
template<> bool ObjectModel::GetObjectValue(float& val, const char *idString)
{
	const ObjectModelTableEntry * const e = FindObjectModelLeafEntry(idString);
	if (e == nullptr)
	{
		return false;
	}

	switch (e->type)
	{
	case TYPE_OF(float):
		val = *((const float*)e->param(this));
		return true;

	case TYPE_OF(uint32_t):
		val = (float)*((const uint32_t*)e->param(this));
		return true;

	case TYPE_OF(int32_t):
		val = (float)*((const int32_t*)e->param(this));
		return true;

	default:
		return false;
	}
}

// Specialisation of above for int, allowing conversion from unsigned to signed
template<> bool ObjectModel::GetObjectValue(int32_t& val, const char *idString)
{
	const ObjectModelTableEntry * const e = FindObjectModelLeafEntry(idString);
	if (e == nullptr)
	{
		return false;
	}

	switch (e->type)
	{
	case TYPE_OF(int32_t):
		val = *((const int32_t*)e->param(this));
		return true;

	case TYPE_OF(uint32_t):
		val = (int32_t)*((const uint32_t*)e->param(this));
		return true;

	default:
		return false;
	}
}

#endif

// End
