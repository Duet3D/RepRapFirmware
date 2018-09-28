/*
 * ObjectModel.cpp
 *
 *  Created on: 27 Aug 2018
 *      Author: David
 */

#include "ObjectModel.h"

#ifdef SUPPORT_OBJECT_MODEL

#include "OutputMemory.h"
#include <cstring>

// Constructor
ObjectModel::ObjectModel()
{
}

// Report this object
bool ObjectModel::ReportAsJson(OutputBuffer* buf, const char* filter, ReportFlags flags)
{
	size_t numEntries;
	const ObjectModelTableEntry *tbl = GetObjectModelTable(numEntries);
	const char *moduleName = GetModuleName();
	if (moduleName != nullptr)							// if we are not the root object
	{
		buf->cat(moduleName);
		buf->cat(':');
		filter = GetNextElement(filter);				// skip the bit that matches the name of this module
	}

	buf->cat('{');
	bool added = false;
	while (numEntries != 0)
	{
		if (added)
		{
			buf->cat(',');
		}
		if (filter[0] == 0 || tbl->Matches(filter, flags))
		{
			tbl->ReportAsJson(buf, this, filter, flags);
			added = true;
		}
		--numEntries;
		++tbl;
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

// Add the value of this element to the buffer, returning true if it matched and we did
bool ObjectModelTableEntry::ReportAsJson(OutputBuffer* buf, ObjectModel *self, const char* filter, ObjectModel::ReportFlags flags) const
{
	//TODO handle arrays
	buf->cat(name);
	buf->cat(':');

	if ((type & isArray) != 0)
	{
		//TODO
		buf->cat("[]");
	}
	else
	{
		void *nParam = param(self);
		switch (type & 15)
		{
		case TYPE_OF(ObjectModel):
			((ObjectModel*)nParam)->ReportAsJson(buf, filter, flags);
			break;

		case TYPE_OF(float):
			buf->cat("%.1f", (double)*(const float *)nParam);		//TODO different parameters need different number of decimal places
			break;

		case TYPE_OF(uint32_t):
			buf->cat("%" PRIu32, *(const uint32_t *)nParam);
			break;

		case TYPE_OF(int32_t):
			buf->cat("%" PRIi32, *(const int32_t *)nParam);
			break;

		case TYPE_OF(Bitmap32):
			if (flags & ObjectModel::shortForm)
			{
				buf->cat("%" PRIu32, *(const uint32_t *)nParam);
			}
			else
			{
				buf->cat('[');
				// TODO list the bits that are set
				buf->cat(']');
			}
			break;

		case TYPE_OF(Enum32):
			if (flags & ObjectModel::shortForm)
			{
				buf->cat("%" PRIu32, *(const uint32_t *)nParam);
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
				if (flags & ObjectModel::shortForm)
				{
					buf->cat((bVal) ? '1' : '0');
				}
				else
				{
					buf->cat((bVal) ? "yes" : "no");
				}
			}
			break;
		}
	}

	return true;
}

// Compare and ID with the name of this object
int ObjectModelTableEntry::IdCompare(const char *id) const
{
	if (id[0] == '*')
	{
		return true;
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

#endif

// End
