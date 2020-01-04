/*
 * ObjectModel.cpp
 *
 *  Created on: 27 Aug 2018
 *      Author: David
 */

#include "ObjectModel.h"

#if SUPPORT_OBJECT_MODEL

#include "OutputMemory.h"
#include <GCodes/GCodeBuffer/StringParser.h>
#include <cstring>
#include <General/SafeStrtod.h>

// Constructor
ObjectModel::ObjectModel() noexcept
{
}

// Report this object
bool ObjectModel::ReportAsJson(OutputBuffer* buf, const char* filter, ReportFlags flags) noexcept
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
			omte->ReportAsJson(buf, this, GetNextElement(filter), flags);
			added = true;
		}
		--numEntries;
		++omte;
	}
	buf->cat('}');
	return true;
}

// Find the requested entry
const ObjectModelTableEntry* ObjectModel::FindObjectModelTableEntry(const char* idString) noexcept
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

/*static*/ const char* ObjectModel::GetNextElement(const char *id) noexcept
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

bool ObjectModelTableEntry::Matches(const char* filterString, ObjectModelFilterFlags filterFlags) const noexcept
{
	return IdCompare(filterString) == 0 && (flags & filterFlags) == filterFlags;
}

// Private function to report a value of primitive type
void ObjectModelTableEntry::ReportItemAsJson(OutputBuffer *buf, const char *filter, ObjectModel::ReportFlags flags, void *nParam, TypeCode type) noexcept
{
	switch (type)
	{
	case TYPE_OF(ObjectModel):
		if (*filter == '.')
		{
			++filter;
		}
		((ObjectModel*)nParam)->ReportAsJson(buf, filter, flags);
		break;

	case TYPE_OF(float):
		buf->catf("%.1f", (double)*(const float *)nParam);
		break;

	case TYPE_OF(Float2):
		buf->catf("%.2f", (double)*(const float *)nParam);
		break;

	case TYPE_OF(Float3):
		buf->catf("%.3f", (double)*(const float *)nParam);
		break;

	case TYPE_OF(uint32_t):
		buf->catf("%" PRIu32, *(const uint32_t *)nParam);
		break;

	case TYPE_OF(int32_t):
		buf->catf("%" PRIi32, *(const int32_t *)nParam);
		break;

	case TYPE_OF(const char*):
		buf->EncodeString((const char*)nParam, true);
		break;

	case TYPE_OF(Bitmap32):
		if (flags & ObjectModel::flagShortForm)
		{
			buf->catf("%" PRIu32, *(const uint32_t *)nParam);
		}
		else
		{
			uint32_t v = *(const uint32_t *)nParam;
			buf->cat('[');
			buf->cat((v & 1) ? '1' : '0');
			for (unsigned int i = 1; i < 32; ++i)
			{
				v >>= 1;
				buf->cat(',');
				buf->cat((v & 1) ? '1' : '0');
			}
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
				buf->cat((bVal) ? "\"yes\"" : "\"no\"");
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
bool ObjectModelTableEntry::ReportAsJson(OutputBuffer* buf, ObjectModel *self, const char* filter, ObjectModel::ReportFlags flags) const noexcept
{
	buf->cat(name);
	buf->cat(':');

	if ((type & IsArray) != 0)
	{
		const bool isEmptyBrackets = (*filter == '[' && *(filter + 1) == ']');
		const ObjectModelArrayDescriptor *arr = (const ObjectModelArrayDescriptor*)param(self);
		if (*filter == 0 || isEmptyBrackets)
		{
			// Report entire array
			if (isEmptyBrackets)
			{
				filter += 2;
			}
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
		else if (*filter == '[')
		{
			++filter;
			const char *endptr;
			const unsigned long val = SafeStrtoul(filter, &endptr);
			if (endptr == filter || *endptr != ']' || val >= arr->GetNumElements(self))
			{
				buf->cat("[]");						// avoid returning badly-formed JSON
				return false;						// invalid syntax, or index out of range
			}
			buf->cat('[');
			ReportItemAsJson(buf, endptr + 1, flags, arr->GetElement(self, val), type & ~IsArray);
			buf->cat(']');
		}
	}
	else
	{
		ReportItemAsJson(buf, filter, flags, param(self), type);
	}
	return true;
}

// Compare an ID with the name of this object
int ObjectModelTableEntry::IdCompare(const char *id) const noexcept
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

// Get the value of an object when we don't know what its type is
ExpressionValue ObjectModel::GetObjectValue(const StringParser& sp, const char *idString)
{
	ExpressionValue val;

	const ObjectModelTableEntry *e = FindObjectModelTableEntry(idString);
	if (e == nullptr)
	{
		throw sp.ConstructParseException("unknown variable %s", idString);
	}

	idString = GetNextElement(idString);
	void * param = e->param(this);
	val.type = e->type;
	if ((val.type & IsArray) != 0)
	{
		if (*idString != '[')
		{
			throw sp.ConstructParseException("can't use whole array");
		}
		const char *endptr;
		const unsigned long index = SafeStrtoul(idString + 1, &endptr);
		if (endptr == idString + 1 || *endptr != ']')
		{
			throw sp.ConstructParseException("expected ']'");
		}
		const ObjectModelArrayDescriptor *arr = (const ObjectModelArrayDescriptor*)param;
		if (index >= arr->GetNumElements(this))
		{
			throw sp.ConstructParseException("array index out of bounds");
		}

		idString = endptr + 1;					// skip past the ']'
		if (*idString == '.')
		{
			++idString;							// skip any '.' after it because it could be an array of objects
		}
		val.type &= ~IsArray;					// clear the array flag
		param = arr->GetElement(this, index);	// fetch the pointer to the array element
	}

	switch (val.type)
	{
	case TYPE_OF(ObjectModel):
		return ((ObjectModel*)param)->GetObjectValue(sp, idString);

	case TYPE_OF(float):
	case TYPE_OF(Float2):
	case TYPE_OF(Float3):
		val.fVal = *((const float*)param);
		break;

	case TYPE_OF(uint32_t):
	case TYPE_OF(Bitmap32):
	case TYPE_OF(Enum32):
		val.uVal = *((const uint32_t*)param);
		break;

	case TYPE_OF(int32_t):
		val.iVal = *((const int32_t*)param);
		break;

	case TYPE_OF(const char*):
		val.sVal = (const char*)param;
		break;

	case TYPE_OF(bool):
		val.bVal = *((const bool*)param);
		break;

	case TYPE_OF(IPAddress):
		val.uVal = ((const IPAddress *)param)->GetV4LittleEndian();
		break;

	default:
		throw sp.ConstructParseException("unknown type");
	}
	return val;
}

// Template specialisations
bool ObjectModel::GetObjectValue(float& val, const char *idString)
{
	const ObjectModelTableEntry *e = FindObjectModelTableEntry(idString);
	if (e == nullptr)
	{
		return NoType;
	}

	if ((e->type & IsArray) != 0)
	{
		//TODO handle arrays
		return NoType;
	}

	switch (e->type)
	{
	case TYPE_OF(ObjectModel):
		return ((ObjectModel*)e->param(this))->GetObjectValue(val, GetNextElement(idString));

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
bool ObjectModel::GetObjectValue(int32_t& val, const char *idString)
{
	const ObjectModelTableEntry *e = FindObjectModelTableEntry(idString);
	if (e == nullptr)
	{
		return NoType;
	}

	if ((e->type & IsArray) != 0)
	{
		//TODO handle arrays
		return NoType;
	}

	switch (e->type)
	{
	case TYPE_OF(ObjectModel):
		return ((ObjectModel*)e->param(this))->GetObjectValue(val, GetNextElement(idString));

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
