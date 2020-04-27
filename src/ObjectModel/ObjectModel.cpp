/*
 * ObjectModel.cpp
 *
 *  Created on: 27 Aug 2018
 *      Author: David
 */

#include "ObjectModel.h"

#if SUPPORT_OBJECT_MODEL

#include <RepRap.h>
#include <Platform.h>
#include <OutputMemory.h>
#include <cstring>
#include <General/SafeStrtod.h>
#include <General/IP4String.h>

ExpressionValue::ExpressionValue(const MacAddress& mac) noexcept : type((uint32_t)TypeCode::MacAddress), param(mac.HighWord()), uVal(mac.LowWord())
{
}

// Append a string representation of this value to a string
void ExpressionValue::AppendAsString(const StringRef& str) const noexcept
{
	switch (GetType())
	{
	case TypeCode::Char:
		str.cat(cVal);
		break;

	case TypeCode::CString:
		str.cat(sVal);
		break;

	case TypeCode::Float:
		str.catf(GetFloatFormatString(), (double)fVal);
		break;

	case TypeCode::Uint32:
		str.catf("%" PRIu32, uVal);			// convert unsigned integer to string
		break;

	case TypeCode::Uint64:
		str.catf("%" PRIu64, ((uint64_t)param << 32) | uVal);	// convert unsigned integer to string
		break;

	case TypeCode::Int32:
		str.catf("%" PRIi32, uVal);			// convert signed integer to string
		break;

	case TypeCode::Bool:
		str.cat((bVal) ? "true" : "false");	// convert bool to string
		break;

	case TypeCode::IPAddress:
		str.cat(IP4String(uVal).c_str());
		break;

	case TypeCode::None:
		str.cat("null");
		break;

	case TypeCode::DateTime:
		{
			const time_t time = Get56BitValue();
			tm timeInfo;
			gmtime_r(&time, &timeInfo);
			str.catf("%04u-%02u-%02u %02u:%02u:%02u",
						timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
		}
		break;

	case TypeCode::DriverId:
#if SUPPORT_CAN_EXPANSION
		str.catf("%u.%u", (unsigned int)(uVal >> 8), (unsigned int)(uVal & 0xFF));
#else
		str.catf("%u", (unsigned int)uVal);
#endif
		break;

	case TypeCode::MacAddress:
		str.catf("%02x:%02x:%02x:%02x:%02x:%02x",
					(unsigned int)(uVal & 0xFF), (unsigned int)((uVal >> 8) & 0xFF), (unsigned int)((uVal >> 16) & 0xFF), (unsigned int)((uVal >> 24) & 0xFF),
					(unsigned int)(param & 0xFF), (unsigned int)((param >> 8) & 0xFF));
		break;

#if SUPPORT_CAN_EXPANSION
	case TypeCode::CanExpansionBoardDetails:
		ExtractRequestedPart(str);
		break;
#endif

	case TypeCode::Special:
#if HAS_MASS_STORAGE
		switch ((SpecialType)param)
		{
		case SpecialType::sysDir:
			reprap.GetPlatform().AppendSysDir(str);
			break;
		}
#endif
		break;

	// We don't fully handle the remaining types
	case TypeCode::ObjectModel:
		str.cat("{object}");
		break;

	case TypeCode::Array:
		str.cat("[array]");
		break;

	case TypeCode::Bitmap16:
	case TypeCode::Bitmap32:
	case TypeCode::Bitmap64:
		str.cat("(Bitmap)");
		break;

	case TypeCode::Enum32:
		str.cat("(enumeration)");
		break;
	}
}

#if SUPPORT_CAN_EXPANSION

// Given that this is a CanExpansionBoardDetails value, extract the part requested according to the parameter
// sVal is a string of the form shortName|version
void ExpressionValue::ExtractRequestedPart(const StringRef& rslt) const noexcept
{
	const char *const p = strchr(sVal, '|');
	const size_t indexOfDivider = (p == nullptr) ? strlen(sVal) : p - sVal;

	switch((ExpansionDetail)param)
	{
	case ExpansionDetail::shortName:
		rslt.copy(sVal, indexOfDivider);
		break;

	case ExpansionDetail::firmwareVersion:
		if (p == nullptr)
		{
			rslt.Clear();
		}
		else
		{
			rslt.copy(sVal + indexOfDivider + 1);
		}
		break;

	case ExpansionDetail::firmwareFileName:
		rslt.copy("Duet3Firmware_");
		rslt.catn(sVal, indexOfDivider);
		rslt.cat(".bin");
		break;

	default:
		break;
	}
}

#endif

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
			buf->cat((*filter == 0) ? "{}" : "null");
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
// This function is recursive, so keep its stack usage low
void ObjectModel::ReportItemAsJson(OutputBuffer *buf, ObjectExplorationContext& context, ExpressionValue val, const char *filter) const
{
	if (context.WantArrayLength() && *filter == 0)
	{
		// We have been asked for the length of an array and we have reached the end of the filter, so the value should be an array
		switch (val.GetType())
		{
		case TypeCode::Array:
			buf->catf("%u", val.omadVal->GetNumElements(this, context));
			break;

		case TypeCode::Bitmap16:
		case TypeCode::Bitmap32:
			buf->catf("%u", Bitmap<uint32_t>::MakeFromRaw(val.uVal).CountSetBits());
			break;

		case TypeCode::Bitmap64:
			buf->catf("%u", Bitmap<uint64_t>::MakeFromRaw(val.Get56BitValue()).CountSetBits());
			break;

		default:
			buf->cat("null");
			break;
		}
	}
	else
	{
		switch (val.GetType())
		{
		case TypeCode::Array:
			if (*filter == '[')
			{
				++filter;
				if (*filter == ']')						// if reporting on [parts of] all elements in the array
				{
					ReportArrayAsJson(buf, context, val.omadVal, filter + 1);
				}
				else
				{
					const char *endptr;
					const int32_t index = StrToI32(filter, &endptr);
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
			}
			else if (*filter == 0)						// else reporting on all subparts of all elements in the array, or just the length
			{
				ReportArrayAsJson(buf, context, val.omadVal, filter);
			}
			else
			{
				buf->cat("null");
			}
			break;

		case TypeCode::ObjectModel:
			if (*filter == '.')
			{
				++filter;
			}
			else if (*filter != 0)
			{
				buf->cat("null");						// error, should have reached the end of the filter or a '.'
				break;
			}
			val.omVal->ReportAsJson(buf, context, val.param, filter);
			break;

		case TypeCode::Float:
			if (val.fVal == 0.0)
			{
				buf->cat('0');							// replace 0.000... in JSON by 0. This is mostly to save space when writing workplace coordinates.
			}
			else if (isnan(val.fVal) || isinf(val.fVal))
			{
				buf->cat("null");						// avoid generating bad JSON if the value is a NaN or infinity
			}
			else
			{
				buf->catf(val.GetFloatFormatString(), (double)val.fVal);
			}
			break;

		case TypeCode::Uint32:
			buf->catf("%" PRIu32, val.uVal);
			break;

		case TypeCode::Uint64:
			buf->catf("%" PRIu64, ((uint64_t)val.param << 32) | val.uVal);	// convert unsigned integer to string
			break;

		case TypeCode::Int32:
			buf->catf("%" PRIi32, val.iVal);
			break;

		case TypeCode::CString:
			buf->EncodeString(val.sVal, true);
			break;

#ifdef DUET3
		case TypeCode::CanExpansionBoardDetails:
			ReportExpansionBoardDetail(buf, val);
			break;
#endif

		case TypeCode::Bitmap16:
		case TypeCode::Bitmap32:
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
					const int32_t index = StrToI32(filter, &endptr);
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

		case TypeCode::Bitmap64:
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
					const int32_t index = StrToI32(filter, &endptr);
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

		case TypeCode::Enum32:
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

		case TypeCode::Bool:
			buf->cat((val.bVal) ? "true" : "false");
			break;

		case TypeCode::Char:
			buf->cat('"');
			buf->EncodeChar(val.cVal);
			buf->cat('"');
			break;

		case TypeCode::IPAddress:
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

		case TypeCode::DateTime:
			ReportDateTime(buf, val);
			break;

		case TypeCode::DriverId:
#if SUPPORT_CAN_EXPANSION
			buf->catf("\"%u.%u\"", (unsigned int)(val.uVal >> 8), (unsigned int)(val.uVal & 0xFF));
#else
			buf->catf("\"%u\"", (unsigned int)val.uVal);
#endif
			break;

		case TypeCode::MacAddress:
			buf->catf("\"%02x:%02x:%02x:%02x:%02x:%02x\"",
						(unsigned int)(val.uVal & 0xFF), (unsigned int)((val.uVal >> 8) & 0xFF), (unsigned int)((val.uVal >> 16) & 0xFF), (unsigned int)((val.uVal >> 24) & 0xFF),
						(unsigned int)(val.param & 0xFF), (unsigned int)((val.param >> 8) & 0xFF));
			break;

		case TypeCode::Special:
#if HAS_MASS_STORAGE
			switch ((ExpressionValue::SpecialType)val.param)
			{
			case ExpressionValue::SpecialType::sysDir:
				reprap.GetPlatform().EncodeSysDir(buf);
				break;
			}
#endif
			break;

		case TypeCode::None:
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
	const ExpressionValue val = func(self, context);
	if (val.GetType() != TypeCode::None || context.ShouldIncludeNulls())
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
	switch (val.GetType())
	{
	case TypeCode::Array:
		{
			if (*idString == 0)
			{
				if (context.WantArrayLength())
				{
					ReadLocker lock(val.omadVal->lockPointer);
					return ExpressionValue((int32_t)val.omadVal->GetNumElements(this, context));
				}
				return ExpressionValue(static_cast<const ObjectModelArrayDescriptor*>(nullptr));	// return a dummy array so that caller can report "[array]" or compare it with null
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

	case TypeCode::ObjectModel:
		switch (*idString)
		{
		case 0:
			return val;
		case '.':
			return val.omVal->GetObjectValue(context, idString + 1, val.param);
		case '^':
			throw context.ConstructParseException("object is not an array");
		default:
			throw context.ConstructParseException("syntax error in object model path");
		}
		break;

	case TypeCode::None:
		if (*idString == 0)
		{
			return val;				// a null value can be compared to null
		}
		throw context.ConstructParseException("reached null object before end of selector string");

	case TypeCode::Bitmap16:
	case TypeCode::Bitmap32:
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

	case TypeCode::Bitmap64:
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

	case TypeCode::MacAddress:
		if (*idString == 0)
		{
			return (context.WantArrayLength()) ? ExpressionValue((int32_t)17) : val;
		}
		break;

#ifdef DUET3
	case TypeCode::CanExpansionBoardDetails:
		if (*idString == 0)
		{
			if (context.WantArrayLength())
			{
				return GetExpansionBoardDetailLength(val);
			}
			return val;
		}
		break;
#endif

	case TypeCode::CString:
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

// Separate function to avoid the tm object (44 bytes) being allocated on the stack frame of a recursive function
void ObjectModel::ReportDateTime(OutputBuffer *buf, ExpressionValue val) noexcept
{
	const time_t time = val.Get56BitValue();
	tm timeInfo;
	gmtime_r(&time, &timeInfo);
	buf->catf("\"%04u-%02u-%02uT%02u:%02u:%02u\"",
				timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
}

#ifdef DUET3

// Separate functions to avoid the string being allocated on the stack frame of a recursive function
void ObjectModel::ReportExpansionBoardDetail(OutputBuffer *buf, ExpressionValue val) noexcept
{
	String<StringLength50> rslt;
	val.ExtractRequestedPart(rslt.GetRef());
	buf->EncodeString(rslt.c_str(), true);
}

ExpressionValue ObjectModel::GetExpansionBoardDetailLength(ExpressionValue val) noexcept
{
	String<StringLength50> rslt;
	val.ExtractRequestedPart(rslt.GetRef());
	return ExpressionValue((int32_t)rslt.strlen());
}

#endif

#endif

// End
