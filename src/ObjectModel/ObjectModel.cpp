/*
 * ObjectModel.cpp
 *
 *  Created on: 27 Aug 2018
 *      Author: David
 */

#include "ObjectModel.h"

#if SUPPORT_OBJECT_MODEL

#include <OutputMemory.h>
#include <GCodes/GCodeBuffer/StringParser.h>
#include <cstring>
#include <General/SafeStrtod.h>

void ObjectExplorationContext::AddIndex(unsigned int index)
{
	if (numIndices == MaxIndices)
	{
		THROW_INTERNAL_ERROR;
	}
	indices[numIndices] = index;
	++numIndices;
}

void ObjectExplorationContext::RemoveIndex()
{
	if (numIndices == 0)
	{
		THROW_INTERNAL_ERROR;
	}
	--numIndices;
}

// Constructor
ObjectModel::ObjectModel() noexcept
{
}

unsigned int ObjectExplorationContext::GetIndex(size_t n) const
{
	if (n < numIndices)
	{
		return indices[numIndices - n - 1];
	}
	THROW_INTERNAL_ERROR;
}

// Report this object
bool ObjectModel::ReportAsJson(OutputBuffer* buf, ObjectExplorationContext& context, uint8_t tableNumber, const char* filter) const
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
			if (tbl->Matches(filter, context))
			{
				if (added)
				{
					buf->cat(',');
				}
				tbl->ReportAsJson(buf, context, this, GetNextElement(filter));
				added = true;
			}
			--numEntries;
			++tbl;
		}
	}
	buf->cat('}');
	return added;
}

// Construct a JSON representation of those parts of the object model requested by the user. This version is called on the root of the tree.
bool ObjectModel::ReportAsJson(OutputBuffer *buf, const char *filter, ObjectModelReportFlags rf, ObjectModelEntryFlags ff) const
{
	ObjectExplorationContext context(rf, ff);
	return ReportAsJson(buf, context, 0, filter);
}

// Function to report a value or object as JSON
bool ObjectModel::ReportItemAsJson(OutputBuffer *buf, ObjectExplorationContext& context, ExpressionValue val, const char *filter) const
{
	switch (val.type)
	{
	case TYPE_OF(const ObjectModelArrayDescriptor*):
		if (*filter == '[')
		{
			++filter;
			if (*filter == ']')						// if reporting on [parts of] all elements in the array
			{
				return ReportArrayAsJson(buf, context, val.omadVal, filter + 1);
			}

			const char *endptr;
			const long index = SafeStrtol(filter, &endptr);
			if (endptr == filter || *endptr != ']' || index < 0 || (size_t)index >= val.omadVal->GetNumElements(this, context))
			{
				buf->cat("[]");						// avoid returning badly-formed JSON
				return false;						// invalid syntax, or index out of range
			}
			buf->cat('[');
			context.AddIndex(index);
			const bool ret = ReportItemAsJson(buf, context, val.omadVal->GetElement(this, context), endptr + 1);
			context.RemoveIndex();
			buf->cat(']');
			return ret;
		}
		if (*filter == 0)							// else reporting on all subparts of all elements in the array
		{
			return ReportArrayAsJson(buf, context, val.omadVal, filter);
		}
		return false;

	case TYPE_OF(const ObjectModel*):
		if (*filter == '.')
		{
			++filter;
		}
		return val.omVal->ReportAsJson(buf, context, val.param, filter);
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
		if (context.ShortFormReport())
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
		if (context.ShortFormReport())
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
		buf->cat((val.bVal) ? "true" : "false");
		break;

	case TYPE_OF(char):
		buf->cat('"');
		buf->EncodeChar(val.cVal);
		buf->cat('"');
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
bool ObjectModel::ReportArrayAsJson(OutputBuffer *buf, ObjectExplorationContext& context, const ObjectModelArrayDescriptor *omad, const char *filter) const
{
	buf->cat('[');
	bool ret = true;
	const size_t count = omad->GetNumElements(this, context);
	for (size_t i = 0; i < count && ret; ++i)
	{
		if (i != 0)
		{
			buf->cat(',');
		}
		context.AddIndex(i);
		ret = ReportItemAsJson(buf, context, omad->GetElement(this, context), filter);
		context.RemoveIndex();
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

bool ObjectModelTableEntry::Matches(const char* filterString, const ObjectExplorationContext& context) const noexcept
{
	return IdCompare(filterString) == 0 && ((uint8_t)flags & (uint8_t)context.GetFilterFlags()) == (uint8_t)context.GetFilterFlags();
}

// Add the value of this element to the buffer, returning true if it matched and we did
bool ObjectModelTableEntry::ReportAsJson(OutputBuffer* buf, ObjectExplorationContext& context, const ObjectModel *self, const char* filter) const noexcept
{
	buf->cat(name);
	buf->cat(':');
	return self->ReportItemAsJson(buf, context, func(self, context), filter);
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

// Get the value of an object. This version is called on the root of the tree.
ExpressionValue ObjectModel::GetObjectValue(const StringParser& sp, const char *idString) const
{
	ObjectExplorationContext context(ObjectModelReportFlags::none, ObjectModelEntryFlags::none);
	return GetObjectValue(sp, context, 0, idString);
}

// Get the value of an object
ExpressionValue ObjectModel::GetObjectValue(const StringParser& sp, ObjectExplorationContext& context, uint8_t tableNumber, const char *idString) const
{
	const ObjectModelTableEntry *e = FindObjectModelTableEntry(tableNumber, idString);
	if (e == nullptr)
	{
		throw sp.ConstructParseException("unknown value %s", idString);
	}

	idString = GetNextElement(idString);
	ExpressionValue val = e->func(this, context);
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
		if (index < 0 || (size_t)index >= val.omadVal->GetNumElements(this, context))
		{
			throw sp.ConstructParseException("array index out of bounds");
		}

		idString = endptr + 1;							// skip past the ']'
		context.AddIndex(index);
		val = val.omadVal->GetElement(this, context);		// fetch the array element
		context.RemoveIndex();
	}

	if (val.type == TYPE_OF(const ObjectModel*))
	{
		if (*idString == '.')
		{
			return val.omVal->GetObjectValue(sp, context, val.param, idString + 1);
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
