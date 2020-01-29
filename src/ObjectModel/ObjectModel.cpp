/*
 * ObjectModel.cpp
 *
 *  Created on: 27 Aug 2018
 *      Author: David
 */

#include "ObjectModel.h"

#if SUPPORT_OBJECT_MODEL

#include <OutputMemory.h>
#include <cstring>
#include <General/SafeStrtod.h>

ExpressionValue::ExpressionValue(const MacAddress& mac) noexcept : type(TYPE_OF(MacAddress)), param(mac.HighWord()), uVal(mac.LowWord())
{
}

void ObjectExplorationContext::AddIndex(int32_t index)
{
	if (numIndicesCounted == MaxIndices)
	{
		throw GCodeException(-1, -1, "Too many indices");
	}
	indices[numIndicesCounted] = index;
	++numIndicesCounted;
}

void ObjectExplorationContext::AddIndex()
{
	if (numIndicesCounted == numIndicesProvided)
	{
		THROW_INTERNAL_ERROR;
	}
	++numIndicesCounted;
}

void ObjectExplorationContext::RemoveIndex()
{
	if (numIndicesCounted == 0)
	{
		THROW_INTERNAL_ERROR;
	}
	--numIndicesCounted;
}

void ObjectExplorationContext::ProvideIndex(int32_t index)
{
	if (numIndicesProvided == MaxIndices)
	{
		throw GCodeException(-1, -1, "Too many indices");
	}
	indices[numIndicesProvided] = index;
	++numIndicesProvided;
}

// Constructor
ObjectModel::ObjectModel() noexcept
{
}

// ObjectExplorationContext members

ObjectExplorationContext::ObjectExplorationContext(const char *reportFlags, bool wal, unsigned int initialMaxDepth, int p_line, int p_col) noexcept
	: maxDepth(initialMaxDepth), currentDepth(0), numIndicesProvided(0), numIndicesCounted(0),
	  line(p_line), column(p_col),
	  shortForm(false), onlyLive(false), includeVerbose(false), wantArrayLength(wal), includeNulls(false)
{
	while (true)
	{
		switch (*reportFlags++)
		{
		case '\0':
			return;
		case 'v':
			includeVerbose = true;
			break;
		case 's':
			shortForm = true;
			break;
		case 'f':
			onlyLive = true;
			break;
		case 'n':
			includeNulls = true;
			break;
		case 'd':
			maxDepth = 0;
			while (isdigit(*reportFlags))
			{
				maxDepth = (10 * maxDepth) + (*reportFlags - '0');
				++reportFlags;
			}
			break;
		case ' ':
		case ',':
			break;
		default:
			// We could report an error here
			break;
		}
	}
}

int32_t ObjectExplorationContext::GetIndex(size_t n) const
{
	if (n < numIndicesCounted)
	{
		return indices[numIndicesCounted - n - 1];
	}
	THROW_INTERNAL_ERROR;
}

int32_t ObjectExplorationContext::GetLastIndex() const
{
	if (numIndicesCounted != 0)
	{
		return indices[numIndicesCounted - 1];
	}
	THROW_INTERNAL_ERROR;
}

bool ObjectExplorationContext::ShouldReport(const ObjectModelEntryFlags f) const noexcept
{
	return (!onlyLive || ((uint8_t)f & (uint8_t)ObjectModelEntryFlags::live) != 0)
		&& (includeVerbose || ((uint8_t)f & (uint8_t)ObjectModelEntryFlags::verbose) == 0);
}

GCodeException ObjectExplorationContext::ConstructParseException(const char *msg) const noexcept
{
	return GCodeException(line, column, msg);
}

// Report this object
void ObjectModel::ReportAsJson(OutputBuffer* buf, ObjectExplorationContext& context, uint8_t tableNumber, const char* filter) const
{
	if (context.IncreaseDepth())
	{
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
					if (tbl->ReportAsJson(buf, context, this, filter, !added))
					{
						added = true;
					}
				}
				--numEntries;
				++tbl;
			}
			if (added && *filter == 0)
			{
				buf->cat('}');
			}
		}
		if (!added)
		{
			buf->cat("null");
		}
		context.DecreaseDepth();
	}
	else
	{
		buf->cat("{}");
	}
}

// Construct a JSON representation of those parts of the object model requested by the user. This version is called on the root of the tree.
void ObjectModel::ReportAsJson(OutputBuffer *buf, const char *filter, const char *reportFlags, bool wantArrayLength) const
{
	const unsigned int defaultMaxDepth = (wantArrayLength) ? 99 : (filter[0] == 0) ? 1 : 99;
	ObjectExplorationContext context(reportFlags, wantArrayLength, defaultMaxDepth);
	ReportAsJson(buf, context, 0, filter);
}

// Function to report a value or object as JSON
void ObjectModel::ReportItemAsJson(OutputBuffer *buf, ObjectExplorationContext& context, ExpressionValue val, const char *filter) const
{
	if (context.WantArrayLength() && *filter == 0)
	{
		// We have been asked for the length of an array and we have reached the end of the filter, so the value should be an array
		switch (val.type)
		{
		case TYPE_OF(const ObjectModelArrayDescriptor*):
			buf->catf("%u", val.omadVal->GetNumElements(this, context));
			break;

		case TYPE_OF(Bitmap<uint16_t>):
		case TYPE_OF(Bitmap<uint32_t>):
			buf->catf("%u", Bitmap<uint32_t>::MakeFromRaw(val.uVal).CountSetBits());
			break;

		case TYPE_OF(Bitmap<uint64_t>):
			buf->catf("%u", Bitmap<uint64_t>::MakeFromRaw(val.Get56BitValue()).CountSetBits());
			break;

		default:
			buf->cat("null");
			break;
		}
	}
	else
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
					buf->cat("null");					// avoid returning badly-formed JSON
					break;								// invalid syntax, or index out of range
				}
				if (*filter == 0)
				{
					buf->cat('[');
				}
				context.AddIndex(index);
				{
					ReadLocker lock(val.omadVal->lockPointer);
					ReportItemAsJson(buf, context, val.omadVal->GetElement(this, context), endptr + 1);
				}
				context.RemoveIndex();
				if (*filter == 0)
				{
					buf->cat(']');
				}
			}
			else if (*filter == 0)						// else reporting on all subparts of all elements in the array, or just the length
			{
				ReportArrayAsJson(buf, context, val.omadVal, filter);
			}
			break;

		case TYPE_OF(const ObjectModel*):
			if (val.omVal != nullptr)
			{
				if (*filter == '.')
				{
					++filter;
				}
				return val.omVal->ReportAsJson(buf, context, val.param, filter);
			}
			buf->cat("null");
			break;

		case TYPE_OF(float):
			if (val.fVal == 0.0)
			{
				buf->cat('0');				// replace 0.000... in JSON by 0. This is mostly to save space when writing workplace coordinates.
			}
			else
			{
				buf->catf(val.GetFloatFormatString(), (double)val.fVal);
			}
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

		case TYPE_OF(Bitmap<uint16_t>):
		case TYPE_OF(Bitmap<uint32_t>):
			if (*filter == '[')
			{
				++filter;
				if (*filter == ']')						// if reporting on all elements in the array
				{
					++filter;
				}
				else
				{
					const char *endptr;
					const long index = SafeStrtol(filter, &endptr);
					if (endptr == filter || *endptr != ']' || index < 0 || (size_t)index >= val.omadVal->GetNumElements(this, context))
					{
						buf->cat("null");				// avoid returning badly-formed JSON
						break;							// invalid syntax, or index out of range
					}
					const auto bm = Bitmap<uint32_t>::MakeFromRaw(val.uVal);
					buf->catf("%u", bm.GetSetBitNumber(index));
					break;
				}
			}
			else if (context.ShortFormReport())
			{
				buf->catf("%" PRIu32, val.uVal);
				break;
			}

			// If we get here then we want a long form report
			{
				const auto bm = Bitmap<uint32_t>::MakeFromRaw(val.uVal);
				buf->cat('[');
				bm.Iterate
					([buf](unsigned int bn, bool first) noexcept
						{
							if (!first)
							{
								buf->cat(',');
							}
							buf->catf("%u", bn);
						}
					);
				buf->cat(']');
			}
			break;

		case TYPE_OF(Bitmap<uint64_t>):
			if (*filter == '[')
			{
				++filter;
				if (*filter == ']')						// if reporting on all elements in the array
				{
					++filter;
				}
				else
				{
					const char *endptr;
					const long index = SafeStrtol(filter, &endptr);
					if (endptr == filter || *endptr != ']' || index < 0 || (size_t)index >= val.omadVal->GetNumElements(this, context))
					{
						buf->cat("null");				// avoid returning badly-formed JSON
						break;							// invalid syntax, or index out of range
					}
					const auto bm = Bitmap<uint64_t>::MakeFromRaw(val.uVal);
					buf->catf("%u", bm.GetSetBitNumber(index));
					break;
				}
			}
			else if (context.ShortFormReport())
			{
				buf->catf("%" PRIu64, val.Get56BitValue());
				break;
			}

			// If we get here then we want a long form report
			{
				const auto bm = Bitmap<uint64_t>::MakeFromRaw(val.Get56BitValue());
				buf->cat('[');
				bm.Iterate
					([buf](unsigned int bn, bool first) noexcept
						{
							if (!first)
							{
								buf->cat(',');
							}
							buf->catf("%u", bn);
						}
					);
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

		case TYPE_OF(DateTime):
			{
				const time_t time = val.Get56BitValue();
				tm timeInfo;
				gmtime_r(&time, &timeInfo);
				buf->catf("\"%04u-%02u-%02uT%02u:%02u:%02u\"",
							timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
			}
			break;

		case TYPE_OF(DriverId):
#if SUPPORT_CAN_EXPANSION
			buf->catf("\"%u.%u\"", (unsigned int)(val.uVal >> 8), (unsigned int)(val.uVal & 0xFF));
#else
			buf->catf("\"%u\"", (unsigned int)val.uVal);
#endif
			break;

		case TYPE_OF(MacAddress):
			buf->catf("\"%02x:%02x:%02x:%02x:%02x:%02x\"",
						(unsigned int)(val.uVal & 0xFF), (unsigned int)((val.uVal >> 8) & 0xFF), (unsigned int)((val.uVal >> 16) & 0xFF), (unsigned int)((val.uVal >> 24) & 0xFF),
						(unsigned int)(val.param & 0xFF), (unsigned int)((val.param >> 8) & 0xFF));
			break;

		case NoType:
			buf->cat("null");
			break;
		}
	}
}

// Report an entire array as JSON
void ObjectModel::ReportArrayAsJson(OutputBuffer *buf, ObjectExplorationContext& context, const ObjectModelArrayDescriptor *omad, const char *filter) const
{
	ReadLocker lock(omad->lockPointer);

	buf->cat('[');
	const size_t count = omad->GetNumElements(this, context);
	for (size_t i = 0; i < count; ++i)
	{
		if (i != 0)
		{
			buf->cat(',');
		}
		context.AddIndex(i);
		ReportItemAsJson(buf, context, omad->GetElement(this, context), filter);
		context.RemoveIndex();
	}
	buf->cat(']');
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
	while (*id != 0 && *id != '.' && *id != '[' && *id != '^')
	{
		++id;
	}
	return id;
}

bool ObjectModelTableEntry::Matches(const char* filterString, const ObjectExplorationContext& context) const noexcept
{
	return IdCompare(filterString) == 0 && context.ShouldReport(flags);
}

// Add the value of this element to the buffer, returning true if it matched and we did
bool ObjectModelTableEntry::ReportAsJson(OutputBuffer* buf, ObjectExplorationContext& context, const ObjectModel *self, const char* filter, bool first) const noexcept
{
	const char * nextElement = ObjectModel::GetNextElement(filter);
	if (*nextElement == '.')
	{
		++nextElement;
	}
	const ExpressionValue val = func(self, context);
	if (val.type != NoType || context.ShouldIncludeNulls())
	{
		if (*filter == 0)
		{
			buf->cat((first) ? "{\"" : ",\"");
			buf->cat(name);
			buf->cat("\":");
		}
		self->ReportItemAsJson(buf, context, val, nextElement);
		return true;
	}
	return false;
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
	return (*n == 0 && (*id == 0 || *id == '.' || *id == '[' || *id == '^')) ? 0
		: (*id > *n) ? 1
			: -1;
}

// Get the value of an object
ExpressionValue ObjectModel::GetObjectValue(ObjectExplorationContext& context, const char *idString, uint8_t tableNumber) const
{
	const ObjectModelTableEntry *const e = FindObjectModelTableEntry(tableNumber, idString);
	if (e == nullptr)
	{
		throw context.ConstructParseException("unknown value");		// idString will have gone out of scope by the time the exception is caught
	}

	idString = GetNextElement(idString);
	ExpressionValue val = e->func(this, context);
	return GetObjectValue(context, val, idString);
}

ExpressionValue ObjectModel::GetObjectValue(ObjectExplorationContext& context, ExpressionValue val, const char *idString) const
{
	switch (val.type)
	{
	case TYPE_OF(const ObjectModelArrayDescriptor*):
		{
			if (*idString == 0 && context.WantArrayLength())
			{
				ReadLocker lock(val.omadVal->lockPointer);
				return ExpressionValue((int32_t)val.omadVal->GetNumElements(this, context));
			}
			if (*idString != '^')
			{
				throw context.ConstructParseException("missing array index");
			}

			context.AddIndex();
			ReadLocker lock(val.omadVal->lockPointer);

			if (context.GetLastIndex() < 0 || (size_t)context.GetLastIndex() >= val.omadVal->GetNumElements(this, context))
			{
				throw context.ConstructParseException("array index out of bounds");
			}

			const ExpressionValue arrayElement = val.omadVal->GetElement(this, context);
			return GetObjectValue(context, arrayElement, idString + 1);
		}

	case TYPE_OF(const ObjectModel*):
		if (*idString == '.')
		{
			return val.omVal->GetObjectValue(context, idString + 1, val.param);
		}
		throw context.ConstructParseException((*idString == 0) ? "selected value has non-primitive type" : "syntax error in value selector string");

	case TYPE_OF(Bitmap<uint16_t>):
	case TYPE_OF(Bitmap<uint32_t>):
		if (context.WantArrayLength())
		{
			if (*idString != 0)
			{
				break;
			}
			const auto bm = Bitmap<uint32_t>::MakeFromRaw(val.uVal);
			return ExpressionValue((int32_t)bm.CountSetBits());
		}
		if (*idString == '^')
		{
			++idString;
			if (*idString != 0)
			{
				break;
			}
			const auto bm = Bitmap<uint32_t>::MakeFromRaw(val.uVal);
			return ExpressionValue((int32_t)bm.GetSetBitNumber(context.GetLastIndex()));
		}
		if (*idString != 0)
		{
			break;
		}
		if (val.uVal > 0x7FFFFFFF)
		{
			throw context.ConstructParseException("bitmap too large to convert to integer");
		}
		return ExpressionValue((int32_t)val.uVal);

	case TYPE_OF(Bitmap<uint64_t>):
		if (context.WantArrayLength())
		{
			if (*idString != 0)
			{
				break;
			}
			const auto bm = Bitmap<uint64_t>::MakeFromRaw(val.Get56BitValue());
			return ExpressionValue((int32_t)bm.CountSetBits());
		}
		if (*idString == '^')
		{
			++idString;
			if (*idString != 0)
			{
				break;
			}
			const auto bm = Bitmap<uint64_t>::MakeFromRaw(val.Get56BitValue());
			return ExpressionValue((int32_t)bm.GetSetBitNumber(context.GetLastIndex()));
		}
		if (*idString != 0)
		{
			break;
		}
		if (val.Get56BitValue() > 0x7FFFFFFF)
		{
			throw context.ConstructParseException("bitmap too large to convert to integer");
		}
		return ExpressionValue((int32_t)val.uVal);

	case TYPE_OF(MacAddress):
		if (*idString == 0)
		{
			return (context.WantArrayLength()) ? ExpressionValue((int32_t)17) : val;
		}
		break;

	case TYPE_OF(const char*):
		if (*idString == 0 && context.WantArrayLength())
		{
			return ExpressionValue((int32_t)strlen(val.sVal));
		}
		// no break
	default:
		if (*idString == 0)
		{
			return val;
		}
		break;
	}

	throw context.ConstructParseException("reached primitive type before end of selector string");
}

#endif

// End
