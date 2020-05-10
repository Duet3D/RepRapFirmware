/*
 * ParseException.cpp
 *
 *  Created on: 21 Dec 2019
 *      Author: David
 */

#include "GCodeException.h"

#include <General/StringRef.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

// Construct the error message. This will be prefixed with "Error: " when it is returned to the user.
void GCodeException::GetMessage(const StringRef &reply, const GCodeBuffer *gb) const noexcept
{
	// Print the file location, if possible
	const bool inFile = gb != nullptr && gb->IsDoingFile();
	if (inFile)
	{
		reply.copy((gb->IsDoingFileMacro()) ? "in file macro": "in GCode file");
		if (line >= 0)
		{
			reply.catf(" line %d", line);
			if (column >= 0)
			{
				reply.catf(" column %d", column + 1);
			}
		}
		reply.cat(": ");
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
				reply.catf("%c%u: ", gb->GetCommandLetter(), gb->GetCommandNumber());
			}
			break;

		case 'Q':		// we use Q0 for comments
		default:
			break;
		}
	}

	// Print the message and any parameter
	if (haveStringParam)
	{
		reply.catf(message, stringParam.c_str());
	}
	else
	{
		reply.catf(message, param.u);
	}
}

// End
