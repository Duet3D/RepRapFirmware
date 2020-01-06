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
bool ObjectModel::ReportAsJson(OutputBuffer* buf, uint8_t tableNumber, const char* filter, ReportFlags flags) const noexcept
{
	buf->cat('{');
	bool added = false;
	const uint8_t *descriptor;
	const ObjectModelTableEntry *tbl = GetObjectModelTable(descriptor);
	if (tableNumber < descriptor[0])
	{
		size_t numEntries = descriptor[tableNumber + 1];
		while (tableNumber != 0)
		{
			--tableNumber;
			tbl += descriptor[tableNumber + 1];
		}

		while (numEntries != 0)
		{
			if (tbl->Matches(filter, flags))
			{
				if (added)
				{
					buf->cat(',');
				}
				tbl->ReportAsJson(buf, this, GetNextElement(filter), flags);
				added = true;
			}
			--numEntries;
			++tbl;
		}
	}
	buf->cat('}');
	return added;
}

// Function to report a value or object as JSON
bool ObjectModel::ReportItemAsJson(ExpressionValue val, OutputBuffer *buf, const char *filter, ObjectModel::ReportFlags flags) const noexcept
{
	switch (val.type)
	{
	case TYPE_OF(const ObjectModelArrayDescriptor*):
		if (*filter == '[')
		{
			++filter;
			if (*filter == ']')						// if reporting on [parts of] all elements in the array
			{
				return ReportArrayAsJson(buf, val.omadVal, filter + 1, flags);
			}

			const char *endptr;
			const long index = SafeStrtol(filter, &endptr);
			if (endptr == filter || *endptr != ']' || index < 0 || (size_t)index >= val.omadVal->GetNumElements(this))
			{
				buf->cat("[]");						// avoid returning badly-formed JSON
				return false;						// invalid syntax, or index out of range
			}
			buf->cat('[');
			const bool ret = ReportItemAsJson(val.omadVal->GetElement(this, index), buf, endptr + 1, flags);
			buf->cat(']');
			return ret;
		}
		if (*filter == 0)							// else reporting on all subparts of all elements in the array
		{
			return ReportArrayAsJson(buf, val.omadVal, filter, flags);
		}
		return false;

	case TYPE_OF(const ObjectModel*):
		if (*filter == '.')
		{
			++filter;
		}
		return val.omVal->ReportAsJson(buf, val.param, filter, flags);
		break;

	case TYPE_OF(float):
		buf->catf((val.param == 3) ? "%.3f" : (val.param == 2) ? "%.2f" : "%.1f", (double)val.fVal);
		break;

	case TYPE_OF(uint32_t):
		buf->catf("%" PRIu32, val.uVal);
		break;

	case TYPE_OF(int32_t):
		buf->catf("%" PRIi32, val.iVal);
		break;

	case TYPE_OF(const char*):
		buf->EncodeString(val.sVal, true);
		break;

	case TYPE_OF(Bitmap32):
		if (flags & ObjectModel::flagShortForm)
		{
			buf->catf("%" PRIu32, val.uVal);
		}
		else
		{
			uint32_t v = val.uVal;
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
			buf->catf("%" PRIu32, val.uVal);
		}
		else
		{
			buf->cat("\"unimplemented\"");
			// TODO append the real name
		}
		break;

	case TYPE_OF(bool):
		if (flags & ObjectModel::flagShortForm)
		{
			buf->cat((val.bVal) ? '1' : '0');
		}
		else
		{
			buf->cat((val.bVal) ? "\"yes\"" : "\"no\"");
		}
		break;

	case TYPE_OF(IPAddress):
		{
			const IPAddress ipVal(val.uVal);
			char sep = '"';
			for (unsigned int q = 0; q < 4; ++q)
			{
				buf->catf("%c%u", sep, ipVal.GetQuad(q));
				sep = '.';
			}
			buf->cat('"');
		}
		break;

	case NoType:
		buf->cat("null");
		break;
	}
	return true;
}

// Report an entire array as JSON
bool ObjectModel::ReportArrayAsJson(OutputBuffer *buf, const ObjectModelArrayDescriptor *omad, const char *filter, ReportFlags rflags) const noexcept
{
	buf->cat('[');
	bool ret = true;
	const size_t count = omad->GetNumElements(this);
	for (size_t i = 0; i < count && ret; ++i)
	{
		if (i != 0)
		{
			buf->cat(',');
		}
		ret = ReportItemAsJson(omad->GetElement(this, i), buf, filter, rflags);
	}
	buf->cat(']');
	return ret;
}

// Find the requested entry
const ObjectModelTableEntry* ObjectModel::FindObjectModelTableEntry(uint8_t tableNumber, const char* idString) const noexcept
{
	const uint8_t *descriptor;
	const ObjectModelTableEntry *tbl = GetObjectModelTable(descriptor);
	if (tableNumber >= descriptor[0])
	{
		return nullptr;
	}

	const size_t numEntries = descriptor[tableNumber + 1];
	while (tableNumber != 0)
	{
		--tableNumber;
		tbl += descriptor[tableNumber + 1];
	}

	size_t low = 0, high = numEntries;
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
	return (low < numEntries && tbl[low].IdCompare(idString) == 0) ? &tbl[low] : nullptr;
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
	return IdCompare(filterString) == 0 && ((uint16_t)flags & (uint16_t)filterFlags) == (uint16_t)filterFlags;
}

// Add the value of this element to the buffer, returning true if it matched and we did
bool ObjectModelTableEntry::ReportAsJson(OutputBuffer* buf, const ObjectModel *self, const char* filter, ObjectModel::ReportFlags flags) const noexcept
{
	buf->cat(name);
	buf->cat(':');
	return self->ReportItemAsJson(func(self), buf, filter, flags);
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

// Get the value of an object. It must have a primitive type.
ExpressionValue ObjectModel::GetObjectValue(const StringParser& sp, uint8_t tableNumber, const char *idString) const
{
	const ObjectModelTableEntry *e = FindObjectModelTableEntry(tableNumber, idString);
	if (e == nullptr)
	{
		throw sp.ConstructParseException("unknown value %s", idString);
	}

	idString = GetNextElement(idString);
	ExpressionValue val = e->func(this);
	while (val.type == TYPE_OF(const ObjectModelArrayDescriptor*))
	{
		if (*idString != '[')
		{
			throw sp.ConstructParseException("can't use whole array");
		}
		const char *endptr;
		const long index = SafeStrtol(idString + 1, &endptr);
		if (endptr == idString + 1 || *endptr != ']')
		{
			throw sp.ConstructParseException("expected ']'");
		}
		if (index < 0 || (size_t)index >= val.omadVal->GetNumElements(this))
		{
			throw sp.ConstructParseException("array index out of bounds");
		}

		idString = endptr + 1;							// skip past the ']'
		val = val.omadVal->GetElement(this, index);		// fetch the array element
	}

	if (val.type == TYPE_OF(const ObjectModel*))
	{
		if (*idString == '.')
		{
			return val.omVal->GetObjectValue(sp, val.param, idString + 1);
		}
		throw sp.ConstructParseException((*idString == 0) ? "selected value has non-primitive type" : "syntax error in value selector string");
	}

	if (*idString == 0)
	{
		return val;
	}

	throw sp.ConstructParseException("reached primive type beforee end of selector string");
}

#endif

// End
