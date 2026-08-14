/*
 * ParseException.cpp
 *
 *  Created on: 21 Dec 2019
 *      Author: David
 */

#include "GCodeException.h"

#include <General/StringRef.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Platform/RepRap.h>
#include <Platform/Tasks.h>

GCodeException::GCodeException(const GCodeBuffer *_ecv_null gb, int col, const char *_ecv_array msg) noexcept : line(-1), column(col), message(msg)
{
	if (   gb != nullptr
		&& (gb->IsDoingFileMacro() || gb->IsDoingFile())
#if HAS_SBC_INTERFACE
		&& (!reprap.UsingSbcInterface() || RTOSIface::GetCurrentTask() == Tasks::GetMainTask())		// don't output the filename or line number if this exception is thrown from a SBC evaluation request, they are prepended by DSF
#endif
	   )
	{
		line = gb->GetLineNumber();
		file = gb->GetFileName();
	}
}

GCodeException::GCodeException(const GCodeBuffer *_ecv_null gb, int col, const char *_ecv_array msg, uint32_t uparam) noexcept : GCodeException(gb, col, msg)
{
	param.u = uparam;
}

GCodeException::GCodeException(const GCodeBuffer *_ecv_null gb, int col, const char *_ecv_array msg, const char *_ecv_array sparam) noexcept : GCodeException(gb, col, msg)
{
	stringParam.copy(sparam);
}

// Construct the error message. This will be prefixed with "Error: " when it is returned to the user.
void GCodeException::GetMessage(const StringRef &reply, const GCodeBuffer *_ecv_null gb) const noexcept
{
	// Print the file location, if possible
	if (!file.IsNull())
	{
		const auto text = file.Get();
		reply.printf("in file %s", text.Ptr());
	}

	if (line >= 0)
	{
		reply.catf(" line %d", line);
		if (column >= 0)
		{
			reply.catf(" column %d", column + 1);
		}
		reply.cat(": ");
	}
	else if (column >= 0)
	{
		reply.catf(" at column %d: ", column + 1);
	}
	else
	{
		reply.Clear();
	}

	// Print the command letter/number, if possible
	if (gb != nullptr)
	{
		switch (gb->GetCommandLetter())
		{
		case 'E':
			reply.cat("meta command: ");
			break;

		case 'G':
		case 'M':
		case 'T':
			if (gb->HasCommandNumber())
			{
				if (gb->GetCommandFraction() >= 0)
				{
					reply.catf("%c%d.%d: ", gb->GetCommandLetter(), gb->GetCommandNumber(), gb->GetCommandFraction());
				}
				else
				{
					reply.catf("%c%d: ", gb->GetCommandLetter(), gb->GetCommandNumber());
				}
			}
			break;

		case 'Q':		// we use Q0 for comments
		default:
			break;
		}
	}

	// Print the message and any parameter
	if (message == nullptr)
	{
		reply.cat("<null error message>");					// should not happen
	}
	else if (strstr(message, "%s") != nullptr)
	{
		reply.catf(message, stringParam.c_str());
	}
	else if (strstr(message, "%u") != nullptr || strstr(message, "%c") != nullptr)
	{
		reply.catf(message, param.u);
	}
	else
	{
		reply.catf(message, param.i);
	}
}

// Print basic details for debugging. Currently called only from FileInfoParser, so don't bother printing line and column.
void GCodeException::DebugPrint() const noexcept
{
	if (message == nullptr)
	{
		debugPrintf("<null error message>");					// should not happen
	}
	else if (strstr(message, "%s") != nullptr)
	{
		debugPrintf(message, stringParam.c_str());
	}
	else if (strstr(message, "%u") != nullptr || strstr(message, "%c") != nullptr)
	{
		debugPrintf(message, param.u);
	}
	else
	{
		debugPrintf(message, param.i);
	}
	debugPrintf("\n");
}

[[noreturn]] void ThrowGCodeException(const char *_ecv_array errMsg) THROWS(GCodeException)
{
	throw GCodeException(-1, -1, errMsg);
}

[[noreturn]] void ThrowGCodeException(const char *_ecv_array errMsg, uint32_t param) THROWS(GCodeException)
{
	throw GCodeException(-1, -1, errMsg, param);
}

// End
