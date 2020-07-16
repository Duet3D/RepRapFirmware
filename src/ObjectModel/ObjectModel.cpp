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
			str.catf("%04u-%02u-%02uT%02u:%02u:%02u",
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
	// While updating firmware on expansion/tool boards we sometimes get a null board type string here, so allow for that
	if (sVal != nullptr)
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
}

#endif

void ObjectExplorationContext::AddIndex(int32_t index) THROWS(GCodeException)
{
	if (numIndicesCounted == MaxIndices)
	{
		throw GCodeException(-1, -1, "Too many indices");
	}
	indices[numIndicesCounted] = index;
	++numIndicesCounted;
}

void ObjectExplorationContext::AddIndex() THROWS(GCodeException)
{
	if (numIndicesCounted == numIndicesProvided)
	{
		THROW_INTERNAL_ERROR;
	}
	++numIndicesCounted;
}

void ObjectExplorationContext::RemoveIndex() THROWS(GCodeException)
{
	if (numIndicesCounted == 0)
	{
		THROW_INTERNAL_ERROR;
	}
	--numIndicesCounted;
}

void ObjectExplorationContext::ProvideIndex(int32_t index) THROWS(GCodeException)
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

int32_t ObjectExplorationContext::GetIndex(size_t n) const THROWS(GCodeException)
{
	if (n < numIndicesCounted)
	{
		return indices[numIndicesCounted - n - 1];
	}
	THROW_INTERNAL_ERROR;
}

int32_t ObjectExplorationContext::GetLastIndex() const THROWS(GCodeException)
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

GCodeException ObjectExplorationContext::ConstructParseException(const char *msg, const char *sparam) const noexcept
{
	return GCodeException(line, column, msg, sparam);
}

// Report this object
void ObjectModel::ReportAsJson(OutputBuffer* buf, ObjectExplorationContext& context, const ObjectModelClassDescriptor * null classDescriptor,
								uint8_t tableNumber, const char* filter) const THROWS(GCodeException)
{
	if (context.IncreaseDepth())
	{
		bool added = false;
		if (classDescriptor == nullptr)
		{
			classDescriptor = GetObjectModelClassDescriptor();
		}

		while (classDescriptor != nullptr)
		{
			const uint8_t * const descriptor = classDescriptor->omd;
			if (tableNumber < descriptor[0])
			{
				const ObjectModelTableEntry *tbl = classDescriptor->omt;
				for (size_t i = 0; i < tableNumber; ++i)
				{
					tbl += descriptor[i + 1];
				}

				size_t numEntries = descriptor[tableNumber + 1];
				while (numEntries != 0)
				{
					if (tbl->Matches(filter, context))
					{
						if (tbl->ReportAsJson(buf, context, classDescriptor, this, filter, !added))
						{
							added = true;
						}
					}
					--numEntries;
					++tbl;
				}
			}
			if (tableNumber != 0)
			{
				break;
			}
			classDescriptor = classDescriptor->parent;			// do parent table too
		}

		if (added)
		{
			if (*filter == 0)
			{
				buf->cat('}');
			}
		}
		else
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
void ObjectModel::ReportAsJson(OutputBuffer *buf, const char *filter, const char *reportFlags, bool wantArrayLength) const THROWS(GCodeException)
{
	const unsigned int defaultMaxDepth = (wantArrayLength) ? 99 : (filter[0] == 0) ? 1 : 99;
	ObjectExplorationContext context(reportFlags, wantArrayLength, defaultMaxDepth);
	ReportAsJson(buf, context, nullptr, 0, filter);
}

// Function to report a value or object as JSON
// This function is recursive, so keep its stack usage low.
// Most recursive calls are for non-array object values, so handle object values inline to reduce stack usage.
// This saves about 240 bytes of stack space but costs 272 bytes of flash memory.
inline void ObjectModel::ReportItemAsJson(OutputBuffer *buf, ObjectExplorationContext& context, const ObjectModelClassDescriptor *classDescriptor,
											const ExpressionValue& val, const char *filter) const THROWS(GCodeException)
{
	if (context.WantArrayLength() && *filter == 0)
	{
		ReportArrayLengthAsJson(buf, context, val);
	}
	else if (val.GetType() == TypeCode::ObjectModel)
	{
		if (*filter == '.')
		{
			++filter;
		}
		else if (*filter != 0)
		{
			buf->cat("null");						// error, should have reached the end of the filter or a '.'
			return;
		}
		val.omVal->ReportAsJson(buf, context, (val.omVal == this) ? classDescriptor : nullptr, val.param, filter);
	}
	else
	{
		ReportItemAsJsonFull(buf, context, classDescriptor, val, filter);
	}
}

void ObjectModel::ReportArrayLengthAsJson(OutputBuffer *buf, ObjectExplorationContext& context, const ExpressionValue& val) const noexcept
{
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

	case TypeCode::CString:
		buf->catf("%u", strlen(val.sVal));
		break;

	default:
		buf->cat("null");
		break;
	}
}

// Function to report a value or object as JSON
// This function is recursive, so keep its stack usage low
void ObjectModel::ReportItemAsJsonFull(OutputBuffer *buf, ObjectExplorationContext& context, const ObjectModelClassDescriptor *classDescriptor,
										const ExpressionValue& val, const char *filter) const THROWS(GCodeException)
{
	switch (val.GetType())
	{
	case TypeCode::Array:
		if (*filter == '[')
		{
			++filter;
			if (*filter == ']')						// if reporting on [parts of] all elements in the array
			{
				ReportArrayAsJson(buf, context, classDescriptor, val.omadVal, filter + 1);
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
					// As at release 3.1.1 this next block uses the most stack of this entire function
					ReadLocker lock(val.omadVal->lockPointer);
					const ExpressionValue element = val.omadVal->GetElement(this, context);
					ReportItemAsJson(buf, context, classDescriptor, element, endptr + 1);
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
			ReportArrayAsJson(buf, context, classDescriptor, val.omadVal, filter);
		}
		else
		{
			buf->cat("null");
		}
		break;

	case TypeCode::Float:
		ReportFloat(buf, val);
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
		ReportBitmap1632Long(buf, val);
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
		ReportBitmap64Long(buf, val);
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

	case TypeCode::ObjectModel:
		break;							// we already handled this case in the inline part
	}
}

// Report an entire array as JSON
void ObjectModel::ReportArrayAsJson(OutputBuffer *buf, ObjectExplorationContext& context, const ObjectModelClassDescriptor *classDescriptor,
										const ObjectModelArrayDescriptor *omad, const char *filter) const THROWS(GCodeException)
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
		const ExpressionValue element = omad->GetElement(this, context);
		ReportItemAsJson(buf, context, classDescriptor, element, filter);
		context.RemoveIndex();
	}
	buf->cat(']');
}

// Find the requested entry
const ObjectModelTableEntry* ObjectModel::FindObjectModelTableEntry(const ObjectModelClassDescriptor *classDescriptor, uint8_t tableNumber, const char* idString) const noexcept
{
	const uint8_t * const descriptor = classDescriptor->omd;
	if (tableNumber >= descriptor[0])
	{
		return nullptr;
	}

	const ObjectModelTableEntry *tbl = classDescriptor->omt;
	for (size_t i = 0; i < tableNumber; ++i)
	{
		tbl += descriptor[i + 1];
	}

	const size_t numEntries = descriptor[tableNumber + 1];
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
	if (low < numEntries && tbl[low].IdCompare(idString) == 0)
	{
		return &tbl[low];
	}
	return nullptr;
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
bool ObjectModelTableEntry::ReportAsJson(OutputBuffer* buf, ObjectExplorationContext& context, const ObjectModelClassDescriptor *classDescriptor, const ObjectModel *self, const char* filter, bool first) const noexcept
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
		self->ReportItemAsJson(buf, context, classDescriptor, val, nextElement);
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
ExpressionValue ObjectModel::GetObjectValue(ObjectExplorationContext& context, const ObjectModelClassDescriptor * null classDescriptor, const char *idString, uint8_t tableNumber) const
{
	if (classDescriptor == nullptr)
	{
		classDescriptor = GetObjectModelClassDescriptor();
	}

	while (classDescriptor != nullptr)
	{
		const ObjectModelTableEntry * const e = FindObjectModelTableEntry(classDescriptor, tableNumber, idString);
		if (e != nullptr)
		{
			idString = GetNextElement(idString);
			const ExpressionValue val = e->func(this, context);
			return GetObjectValue(context, classDescriptor, val, idString);
		}
		if (tableNumber != 0)
		{
			break;
		}
		classDescriptor = classDescriptor->parent;			// search parent class object model too
	}

	throw context.ConstructParseException("unknown value '%s'", idString);
}

ExpressionValue ObjectModel::GetObjectValue(ObjectExplorationContext& context, const ObjectModelClassDescriptor *classDescriptor, const ExpressionValue& val, const char *idString) const
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
			return GetObjectValue(context, classDescriptor, arrayElement, idString + 1);
		}

	case TypeCode::ObjectModel:
		switch (*idString)
		{
		case 0:
			return val;
		case '.':
			return val.omVal->GetObjectValue(context, (val.omVal == this) ? classDescriptor : nullptr, idString + 1, val.param);
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
void ObjectModel::ReportDateTime(OutputBuffer *buf, const ExpressionValue& val) noexcept
{
	const time_t time = val.Get56BitValue();
	tm timeInfo;
	gmtime_r(&time, &timeInfo);
	buf->catf("\"%04u-%02u-%02uT%02u:%02u:%02u\"",
				timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
}

// Separate function to avoid a recursive function saving all the FP registers
void ObjectModel::ReportFloat(OutputBuffer *buf, const ExpressionValue& val) noexcept
{
	if (val.fVal == 0.0)
	{
		buf->cat('0');							// replace 0.000... in JSON by 0. This is mostly to save space when writing workplace coordinates.
	}
	else if (std::isnan(val.fVal) || std::isinf(val.fVal))
	{
		buf->cat("null");						// avoid generating bad JSON if the value is a NaN or infinity
	}
	else
	{
		buf->catf(val.GetFloatFormatString(), (double)val.fVal);
	}
}

void ObjectModel::ReportBitmap1632Long(OutputBuffer *buf, const ExpressionValue& val) noexcept
{
	const auto bm = Bitmap<uint32_t>::MakeFromRaw(val.uVal);
	buf->cat('[');
	bm.Iterate
		([buf](unsigned int bn, unsigned int count) noexcept
			{
				if (count != 0)
				{
					buf->cat(',');
				}
				buf->catf("%u", bn);
			}
		);
	buf->cat(']');
}

void ObjectModel::ReportBitmap64Long(OutputBuffer *buf, const ExpressionValue& val) noexcept
{
	const auto bm = Bitmap<uint64_t>::MakeFromRaw(val.Get56BitValue());
	buf->cat('[');
	bm.Iterate
		([buf](unsigned int bn, unsigned int count) noexcept
			{
				if (count != 0)
				{
					buf->cat(',');
				}
				buf->catf("%u", bn);
			}
		);
	buf->cat(']');
}

#ifdef DUET3

// Separate functions to avoid the string being allocated on the stack frame of a recursive function
void ObjectModel::ReportExpansionBoardDetail(OutputBuffer *buf, const ExpressionValue& val) noexcept
{
	String<StringLength50> rslt;
	val.ExtractRequestedPart(rslt.GetRef());
	buf->EncodeString(rslt.c_str(), true);
}

ExpressionValue ObjectModel::GetExpansionBoardDetailLength(const ExpressionValue& val) noexcept
{
	String<StringLength50> rslt;
	val.ExtractRequestedPart(rslt.GetRef());
	return ExpressionValue((int32_t)rslt.strlen());
}

#endif

#endif

// End
