/*
 * ObjectModel.cpp
 *
 *  Created on: 27 Aug 2018
 *      Author: David
 */

#include "ObjectModel.h"

#if SUPPORT_OBJECT_MODEL

#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <Platform/OutputMemory.h>
#include <cstring>
#include <General/SafeStrtod.h>
#include <General/IP4String.h>
#include <Hardware/ExceptionHandlers.h>
#include <Hardware/IoPorts.h>

namespace StackUsage
{
	constexpr uint32_t GetObjectValue_noTable = 56;
	constexpr uint32_t GetObjectValue_withTable = 48;
}

ExpressionValue::ExpressionValue(const MacAddress& mac) noexcept : type((uint32_t)TypeCode::MacAddress_tc), param(mac.HighWord()), uVal(mac.LowWord())
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

	case TypeCode::HeapString:
		str.cat(shVal.Get().Ptr());
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

	case TypeCode::IPAddress_tc:
		str.cat(IP4String(uVal).c_str());
		break;

	case TypeCode::None:
		str.cat("null");
		break;

	case TypeCode::DateTime_tc:
		{
			const time_t time = Get56BitValue();
			tm timeInfo;
			gmtime_r(&time, &timeInfo);
			str.catf("%04u-%02u-%02uT%02u:%02u:%02u",
						timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
		}
		break;

	case TypeCode::Duration:
		{
			unsigned int hours = uVal/3600,
				minutes = (uVal / 60) % 60,
				seconds = uVal % 60;
			str.catf("%u:%02u:%02u", hours, minutes, seconds);
		}
		break;

	case TypeCode::DriverId_tc:
#if SUPPORT_CAN_EXPANSION
		str.catf("%u.%u", (unsigned int)param, (unsigned int)uVal);
#else
		str.catf("%u", (unsigned int)uVal);
#endif
		break;

	case TypeCode::MacAddress_tc:
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
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
		switch ((SpecialType)param)
		{
		case SpecialType::sysDir:
			reprap.GetPlatform().AppendSysDir(str);
			break;
		}
#endif
		break;

	// We don't fully handle the remaining types
	case TypeCode::ObjectModel_tc:
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

	case TypeCode::Port:
		iopVal->AppendPinName(str);
		break;

	case TypeCode::UniqueId_tc:
		uniqueIdVal->AppendCharsToString(str);
		break;
	}
}

// Compare two values that are assumed to be represented in the same way
bool ExpressionValue::operator==(const ExpressionValue& other) const noexcept
{
	if (type == other.type)
	{
		switch (GetType())
		{
		case TypeCode::None:
			return true;

		case TypeCode::Bool:
			return bVal == other.bVal;

		case TypeCode::Char:
			return cVal == other.cVal;

		case TypeCode::CString:
			return strcmp(sVal, other.sVal) == 0;

		case TypeCode::HeapString:
			return strcmp(shVal.Get().Ptr(), other.shVal.Get().Ptr()) == 0;

		case TypeCode::Float:
			return fVal == other.fVal;

		case TypeCode::Uint32:
		case TypeCode::IPAddress_tc:
		case TypeCode::Int32:
		case TypeCode::Bitmap16:
		case TypeCode::Bitmap32:
		case TypeCode::Enum32:
		case TypeCode::Duration:
			return uVal == other.uVal;

		case TypeCode::Uint64:
		case TypeCode::Bitmap64:
		case TypeCode::DateTime_tc:
		case TypeCode::MacAddress_tc:
		case TypeCode::DriverId_tc:
			return uVal == other.uVal && param == other.param;

		case TypeCode::Port:
			return iopVal == other.iopVal;

		// We don't handle the remaining types
		case TypeCode::Special:
		case TypeCode::ObjectModel_tc:
		case TypeCode::Array:
		case TypeCode::UniqueId_tc:
#if SUPPORT_CAN_EXPANSION
		case TypeCode::CanExpansionBoardDetails:
#endif
			return false;
		}
	}
	return false;
}

ExpressionValue::ExpressionValue(const ExpressionValue& other) noexcept
{
	type = other.type;
	param = other.param;
	whole = other.whole;
	if (type == (uint32_t)TypeCode::HeapString)
	{
		shVal.IncreaseRefCount();
	}
}

ExpressionValue::ExpressionValue(ExpressionValue&& other) noexcept
{
	type = other.type;
	param = other.param;
	whole = other.whole;
	other.type = (uint32_t)TypeCode::None;
}

ExpressionValue::~ExpressionValue()
{
	Release();
}

ExpressionValue& ExpressionValue::operator=(const ExpressionValue& other) noexcept
{
	if (&other != this)
	{
		Release();
		type = other.type;
		param = other.param;
		whole = other.whole;
		if (type == (uint32_t)TypeCode::HeapString)
		{
			shVal.IncreaseRefCount();
		}
	}
	return *this;
}

// Release any associated storage
void ExpressionValue::Release() noexcept
{
	if (type == (uint32_t)TypeCode::HeapString)
	{
		shVal.Delete();
		type = (uint32_t)TypeCode::None;
	}
}

#if SUPPORT_CAN_EXPANSION

// Given that this is a CanExpansionBoardDetails value, extract the part requested according to the parameter and append it to the string
// sVal is a string of the form shortName|version
void ExpressionValue::ExtractRequestedPart(const StringRef& rslt) const noexcept
{
	// While updating firmware on expansion/tool boards we sometimes get a null board type string here, so allow for that
	if (sVal != nullptr)
	{
		// Split the string into three field separate by vertical bar. These are board short name, firmware version, and firmware date.
		const char * p = strchr(sVal, '|');
		const size_t indexOfDivider1 = (p == nullptr) ? strlen(sVal) : p - sVal;
		if (p != nullptr)
		{
			p = strchr(p + 1, '|');
		}
		const size_t indexOfDivider2 = (p == nullptr) ? strlen(sVal) : p - sVal;

		switch((ExpansionDetail)param)
		{
		case ExpansionDetail::longName:
			rslt.cat("Duet 3 Expansion ");
			// no break
		case ExpansionDetail::shortName:
			rslt.catn(sVal, indexOfDivider1);
			break;

		case ExpansionDetail::firmwareVersion:
			if (indexOfDivider2 > indexOfDivider1)
			{
				rslt.catn(sVal + indexOfDivider1 + 1, indexOfDivider2 - indexOfDivider1 - 1);
			}
			break;

		case ExpansionDetail::firmwareFileName:
			rslt.cat("Duet3Firmware_");
			rslt.catn(sVal, indexOfDivider1);
			rslt.cat(".bin");
			break;

		case ExpansionDetail::firmwareDate:
			if (strlen(sVal) > indexOfDivider2)
			{
				rslt.cat(sVal + indexOfDivider2 + 1);
			}
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

// Constructor used when reporting the OM as JSON
ObjectExplorationContext::ObjectExplorationContext(const GCodeBuffer *_ecv_null gbp, bool wal, const char *reportFlags, unsigned int initialMaxDepth, size_t initialBufferOffset) noexcept
	: startMillis(millis()), initialBufOffset(initialBufferOffset), maxDepth(initialMaxDepth), currentDepth(0), startElement(0), nextElement(-1), numIndicesProvided(0), numIndicesCounted(0),
	  line(-1), column(-1), gb(gbp),
	  shortForm(false), wantArrayLength(wal), wantExists(false),
	  includeNonLive(true), includeImportant(false), includeNulls(false),
	  excludeVerbose(true), excludeObsolete(true),
	  obsoleteFieldQueried(false)
{
	while (true)
	{
		switch (*reportFlags++)
		{
		case '\0':
			return;
		case 'v':
			excludeVerbose = false;
			break;
		case 's':
			shortForm = true;
			break;
		case 'f':
			includeNonLive = false;
			break;
		case 'i':
			includeImportant = true;
			break;
		case 'n':
			includeNulls = true;
			break;
		case 'o':
			excludeObsolete = false;
			break;
		case 'd':
			maxDepth = 0;
			while (isdigit(*reportFlags))
			{
				maxDepth = (10 * maxDepth) + (*reportFlags - '0');
				++reportFlags;
			}
			break;
		case 'a':
			startElement = 0;
			while (isdigit(*reportFlags))
			{
				startElement = (10 * startElement) + (*reportFlags - '0');
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

// Constructor when evaluating expressions
ObjectExplorationContext::ObjectExplorationContext(const GCodeBuffer *_ecv_null gbp, bool wal, bool wex, int p_line, int p_col) noexcept
	: startMillis(millis()), initialBufOffset(0), maxDepth(99), currentDepth(0), startElement(0), nextElement(-1), numIndicesProvided(0), numIndicesCounted(0),
	  line(p_line), column(p_col), gb(gbp),
	  shortForm(false), wantArrayLength(wal), wantExists(wex),
	  includeNonLive(true), includeImportant(false), includeNulls(false),
	  excludeVerbose(false), excludeObsolete(false),
	  obsoleteFieldQueried(false)
{
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
	const bool wanted = includeNonLive
					 || ((uint8_t)f & (uint8_t)ObjectModelEntryFlags::live) != 0
					 || (includeImportant && ((uint8_t)f & (uint8_t)ObjectModelEntryFlags::important) != 0);
	return wanted
		&& (!excludeVerbose  || ((uint8_t)f & (uint8_t)ObjectModelEntryFlags::verbose) == 0)
		&& (!excludeObsolete || ((uint8_t)f & (uint8_t)ObjectModelEntryFlags::obsolete) == 0);
}

GCodeException ObjectExplorationContext::ConstructParseException(const char *msg) const noexcept
{
	return GCodeException(line, column, msg);
}

GCodeException ObjectExplorationContext::ConstructParseException(const char *msg, const char *sparam) const noexcept
{
	return GCodeException(line, column, msg, sparam);
}

// Call this before making a recursive call, or before calling a function that needs a lot of stack from a recursive function
void ObjectExplorationContext::CheckStack(uint32_t calledFunctionStackUsage) const THROWS(GCodeException)
{
	const char *_ecv_array stackPtr = (const char*_ecv_array)GetStackPointer();
	const char *_ecv_array stackLimit = (const char*_ecv_array)TaskBase::GetCallerTaskHandle() + sizeof(TaskBase);
	if (stackLimit + calledFunctionStackUsage + (StackUsage::Throw + StackUsage::Margin) < stackPtr)
	{
		return;
	}

	// The stack is in danger of overflowing. Throw an exception if we have enough stack to do so (ideally, this should always be the case)
	if (stackLimit + StackUsage::Throw <= stackPtr)
	{
		throw GCodeException(line, column, "Expression nesting too deep");
	}

	// Not enough stack left to throw an exception
	SoftwareReset(SoftwareResetReason::stackOverflow, (const uint32_t *)stackPtr);
}

// Report this object
void ObjectModel::ReportAsJson(OutputBuffer* buf, ObjectExplorationContext& context, const ObjectModelClassDescriptor * null classDescriptor,
								uint8_t tableNumber, const char *_ecv_array filter) const THROWS(GCodeException)
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
void ObjectModel::ReportAsJson(const GCodeBuffer *_ecv_null gb, OutputBuffer *buf, const char *_ecv_array filter, const char *_ecv_array reportFlags, bool wantArrayLength) const THROWS(GCodeException)
{
	const unsigned int defaultMaxDepth = (wantArrayLength) ? 99 : (filter[0] == 0) ? 1 : 99;
	ObjectExplorationContext context(gb, wantArrayLength, reportFlags, defaultMaxDepth, buf->Length());
	ReportAsJson(buf, context, nullptr, 0, filter);
	if (context.GetNextElement() >= 0)
	{
		buf->catf(",\"next\":%d", context.GetNextElement());
	}
}

// Function to report a value or object as JSON
// This function is recursive, so keep its stack usage low.
// Most recursive calls are for non-array object values, so handle object values inline to reduce stack usage.
// This saves about 240 bytes of stack space but costs 272 bytes of flash memory.
inline void ObjectModel::ReportItemAsJson(OutputBuffer *buf, ObjectExplorationContext& context, const ObjectModelClassDescriptor *classDescriptor,
											const ExpressionValue& val, const char *_ecv_array filter) const THROWS(GCodeException)
{
	if (context.WantArrayLength() && *filter == 0)
	{
		ReportArrayLengthAsJson(buf, context, val);
	}
	else if (val.GetType() == TypeCode::ObjectModel_tc)
	{
		if (  (*filter != '.' && *filter != 0)		// we should have reached the end of the filter or a '.', error if not
			|| val.omVal == nullptr					// OM arrays may contain null entries, so we need to handle them here
		   )
		{
			buf->cat("null");
		}
		else
		{
			if (*filter == '.')
			{
				++filter;
			}
			val.omVal->ReportAsJson(buf, context, (val.omVal == this) ? classDescriptor : nullptr, val.param, filter);
		}
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

	case TypeCode::HeapString:
		buf->catf("%u", val.shVal.GetLength());
		break;

	default:
		buf->cat("null");
		break;
	}
}

// Function to report a value or object as JSON
// This function is recursive, so keep its stack usage low
void ObjectModel::ReportItemAsJsonFull(OutputBuffer *buf, ObjectExplorationContext& context, const ObjectModelClassDescriptor *null classDescriptor,
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
		buf->catf("\"%.s\"", val.sVal);
		break;

	case TypeCode::HeapString:
		buf->catf("\"%.s\"", val.shVal.Get().Ptr());
		break;

#if SUPPORT_CAN_EXPANSION
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
				const auto bm = Bitmap<uint32_t>::MakeFromRaw(val.uVal);
				int bitNumber;
				if (endptr == filter || *endptr != ']' || index < 0 || (bitNumber = bm.GetSetBitNumber(index)) < 0)
				{
					buf->cat("null");				// avoid returning badly-formed JSON
					break;							// invalid syntax, or index out of range
				}
				buf->catf("%d", bitNumber);
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
				const auto bm = Bitmap<uint64_t>::MakeFromRaw(val.uVal);
				int bitNumber;
				if (endptr == filter || *endptr != ']' || index < 0 || (bitNumber = bm.GetSetBitNumber(index)) < 0)
				{
					buf->cat("null");				// avoid returning badly-formed JSON
					break;							// invalid syntax, or index out of range
				}
				buf->catf("%d", bitNumber);
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

	case TypeCode::IPAddress_tc:
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

	case TypeCode::DateTime_tc:
		ReportDateTime(buf, val);
		break;

	case TypeCode::Duration:
		{
			unsigned int hours = val.uVal/3600,
				minutes = (val.uVal / 60) % 60,
				seconds = val.uVal % 60;
			buf->catf("%u:%02u:%02u", hours, minutes, seconds);
		}
		break;

	case TypeCode::DriverId_tc:
#if SUPPORT_CAN_EXPANSION
		buf->catf("\"%u.%u\"", (unsigned int)val.param, (unsigned int)val.uVal);
#else
		buf->catf("\"%u\"", (unsigned int)val.uVal);
#endif
		break;

	case TypeCode::MacAddress_tc:
		buf->catf("\"%02x:%02x:%02x:%02x:%02x:%02x\"",
					(unsigned int)(val.uVal & 0xFF), (unsigned int)((val.uVal >> 8) & 0xFF), (unsigned int)((val.uVal >> 16) & 0xFF), (unsigned int)((val.uVal >> 24) & 0xFF),
					(unsigned int)(val.param & 0xFF), (unsigned int)((val.param >> 8) & 0xFF));
		break;

	case TypeCode::Special:
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES || HAS_SBC_INTERFACE
		switch ((ExpressionValue::SpecialType)val.param)
		{
		case ExpressionValue::SpecialType::sysDir:
			buf->catf("\"%.s\"", reprap.GetPlatform().GetSysDir().Ptr());
			break;
		}
#endif
		break;

	case TypeCode::None:
		buf->cat("null");
		break;

	case TypeCode::Port:
		ReportPinNameAsJson(buf, val);
		break;

	case TypeCode::UniqueId_tc:
		buf->cat('"');
		val.uniqueIdVal->AppendCharsToBuffer(buf);
		buf->cat('"');
		break;

	case TypeCode::ObjectModel_tc:
		break;											// we already handled this case in the inline part
	}
}

// This is a separate function to avoid having a string buffer on the stack of a recursive function
void ObjectModel::ReportPinNameAsJson(OutputBuffer *buf, const ExpressionValue& val) noexcept
{
	buf->cat('"');
	String<StringLength50> portName;
	val.iopVal->AppendPinName(portName.GetRef());
	buf->catf("%.0s", portName.c_str());				// the %.0s format specifier forces JSON escaping
	buf->cat('"');
}

// Report an entire array as JSON
void ObjectModel::ReportArrayAsJson(OutputBuffer *buf, ObjectExplorationContext& context, const ObjectModelClassDescriptor *null classDescriptor,
										const ObjectModelArrayDescriptor *omad, const char *_ecv_array filter) const THROWS(GCodeException)
{
	const bool isRootArray = (buf->Length() == context.GetInitialBufferOffset());		// it's a root array if we haven't started writing to the buffer yet
	ReadLocker lock(omad->lockPointer);

	buf->cat('[');
	const size_t count = omad->GetNumElements(this, context);
	const size_t startElement = (isRootArray) ? context.GetStartElement() : 0;
	for (size_t i = startElement; i < count; ++i)
	{
		// Support retrieving just part of the array in case it is too large to write all of it to the buffer
		if (i != startElement)
		{
			if (isRootArray && buf->Length() >= (OUTPUT_BUFFER_SIZE * (OUTPUT_BUFFER_COUNT - RESERVED_OUTPUT_BUFFERS))/2)
			{
				// We've used half the buffer space already, so stop reporting
				context.SetNextElement(i);
				break;
			}
			buf->cat(',');
		}
		context.AddIndex(i);
		const ExpressionValue element = omad->GetElement(this, context);
		ReportItemAsJson(buf, context, classDescriptor, element, filter);
		context.RemoveIndex();
	}
	if (isRootArray && context.GetNextElement() < 0)
	{
		context.SetNextElement(0);
	}
	buf->cat(']');
}

// Find the requested entry
const ObjectModelTableEntry* ObjectModel::FindObjectModelTableEntry(const ObjectModelClassDescriptor *classDescriptor, uint8_t tableNumber, const char *_ecv_array idString) const noexcept
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
	// We include nulls if either the "include nulls" flag is set or the "include important" flag is set and the field is flagged important.
	// The latter is so that field state.messageBox gets reported to PanelDue even if null when the "important" flag is set, so that PanelDue knows when a message has been cleared.
	if (val.GetType() != TypeCode::None || context.ShouldIncludeNulls() || (context.ShouldIncludeImportant() && ((uint8_t)flags & (uint8_t)ObjectModelEntryFlags::important)))
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
		: (*id > *n && *id != '^') ? 1
			: -1;
}

// Get the value of an object
ExpressionValue ObjectModel::GetObjectValueUsingTableNumber(ObjectExplorationContext& context, const ObjectModelClassDescriptor * null classDescriptor, const char *_ecv_array idString, uint8_t tableNumber) const THROWS(GCodeException)
decrease(strlen(idString))	// recursion variant
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
			if (e->IsObsolete())
			{
				context.SetObsoleteFieldQueried();
			}
			idString = GetNextElement(idString);
			const ExpressionValue val = e->func(this, context);
			context.CheckStack(StackUsage::GetObjectValue_noTable);
			return GetObjectValue(context, classDescriptor, val, idString);
		}
		if (tableNumber != 0)
		{
			break;
		}
		classDescriptor = classDescriptor->parent;			// search parent class object model too
	}

	if (context.WantExists())
	{
		return ExpressionValue(false);
	}

	throw context.ConstructParseException("unknown value '%s'", idString);
}

ExpressionValue ObjectModel::GetObjectValue(ObjectExplorationContext& context, const ObjectModelClassDescriptor *classDescriptor, const ExpressionValue& val, const char *_ecv_array idString) const THROWS(GCodeException)
decrease(strlen(idString))	// recursion variant
{
	if (*idString == 0 && context.WantExists() && val.GetType() != TypeCode::None)
	{
		return ExpressionValue(true);
	}

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
				if (context.WantExists())
				{
					return ExpressionValue(false);
				}
				throw context.ConstructParseException("array index out of bounds");
			}

			const ExpressionValue arrayElement = val.omadVal->GetElement(this, context);
			context.CheckStack(StackUsage::GetObjectValue_noTable);
			return GetObjectValue(context, classDescriptor, arrayElement, idString + 1);
		}

	case TypeCode::ObjectModel_tc:
		switch (*idString)
		{
		case 0:
			return val;
		case '.':
			context.CheckStack(StackUsage::GetObjectValue_withTable);
			return val.omVal->GetObjectValueUsingTableNumber(context, (val.omVal == this) ? classDescriptor : nullptr, idString + 1, val.param);
		case '^':
			throw context.ConstructParseException("object is not an array");
		default:
			throw context.ConstructParseException("syntax error in object model path");
		}
		break;

	case TypeCode::None:
		if (context.WantExists())
		{
			return ExpressionValue(false);
		}
		if (*idString == 0)
		{
			return val;				// a null value can be compared to null
		}
		throw context.ConstructParseException("reached null object before end of selector string");

	case TypeCode::Bitmap16:
	case TypeCode::Bitmap32:
		{
			const int numSetBits = Bitmap<uint32_t>::MakeFromRaw(val.uVal).CountSetBits();
			if (context.WantArrayLength())
			{
				if (*idString != 0)
				{
					break;
				}
				return ExpressionValue((int32_t)numSetBits);
			}

			if (*idString == '^')
			{
				++idString;
				if (*idString != 0)
				{
					break;
				}
				context.AddIndex();
				const bool inBounds = (context.GetLastIndex() >= 0 && context.GetLastIndex() < numSetBits);
				if (context.WantExists())
				{
					return ExpressionValue(inBounds);
				}

				if (!inBounds)
				{
					throw context.ConstructParseException("array index out of bounds");
				}

				if (context.WantExists())
				{
					return ExpressionValue(true);
				}
				return ExpressionValue((int32_t)(Bitmap<uint32_t>::MakeFromRaw(val.uVal).GetSetBitNumber(context.GetLastIndex())));
			}
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
		{
			const int numSetBits = Bitmap<uint64_t>::MakeFromRaw(val.Get56BitValue()).CountSetBits();
			if (context.WantArrayLength())
			{
				if (*idString != 0)
				{
					break;
				}
				return ExpressionValue((int32_t)numSetBits);
			}

			if (*idString == '^')
			{
				++idString;
				if (*idString != 0)
				{
					break;
				}
				context.AddIndex();
				const bool inBounds = (context.GetLastIndex() >= 0 && context.GetLastIndex() < numSetBits);
				if (context.WantExists())
				{
					return ExpressionValue(inBounds);
				}

				if (!inBounds)
				{
					throw context.ConstructParseException("array index out of bounds");
				}

				if (context.WantExists())
				{
					return ExpressionValue(true);
				}

				return ExpressionValue((int32_t)(Bitmap<uint64_t>::MakeFromRaw(val.Get56BitValue()).GetSetBitNumber(context.GetLastIndex())));
			}
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

	case TypeCode::MacAddress_tc:
		if (*idString == 0)
		{
			return (context.WantArrayLength()) ? ExpressionValue((int32_t)17) : val;
		}
		break;

#if SUPPORT_CAN_EXPANSION
	case TypeCode::CanExpansionBoardDetails:
		if (*idString == 0)
		{
			return (context.WantArrayLength()) ? GetExpansionBoardDetailLength(val) : val;
		}
		break;
#endif

	case TypeCode::HeapString:
		if (*idString == 0)
		{
			return (context.WantArrayLength()) ? ExpressionValue((int32_t)val.shVal.GetLength()) : val;
		}
		break;

	case TypeCode::CString:
		if (*idString == 0)
		{
			return (context.WantArrayLength()) ? ExpressionValue((int32_t)strlen(val.sVal)) : val;
		}
		break;

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

#if SUPPORT_CAN_EXPANSION

// Separate functions to avoid the string being allocated on the stack frame of a recursive function
void ObjectModel::ReportExpansionBoardDetail(OutputBuffer *buf, const ExpressionValue& val) noexcept
{
	String<StringLength50> rslt;
	val.ExtractRequestedPart(rslt.GetRef());
	buf->catf("\"%.s\"", rslt.c_str());
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
